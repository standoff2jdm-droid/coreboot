/* SPDX-License-Identifier: GPL-2.0-only */

#include <acpi/acpi.h>
#include <acpi/acpi_device.h>
#include <acpi/acpigen.h>
#include <acpi/acpigen_pci.h>
#include <commonlib/helpers.h>
#include <console/console.h>
#include <delay.h>
#include <device/device.h>
#include <device/pci.h>
#include <device/pci_ids.h>
#include <device/pci_ops.h>
#include <device/pciexp.h>
#include <stdio.h>
#include <stdlib.h>
#include <timer.h>
#include "chip.h"

#define IIO_PCI_EXP_LNKCTL2	0x30
#define  IIO_LNKCTL2_TLS		0x000f
#define  IIO_LNKSTA_CLS		0x000f
#define  IIO_LNKSTA_NLW		0x03f0
#define  IIO_RTCAP_CRSSV		0x0001
/*
 * Discovery windows shared by all ports, counted from the first scan. The links
 * need up to 400 ms to train after bifurcation is started and another 200 ms
 * for Gen3 equalization. Bifurcation is started in romstage, so the window
 * covers both even when ramstage reaches the ports early. A port that has not
 * left Detect by the shorter window has no receiver on the other end; giving
 * up on it early keeps empty ports from using up the training window. An
 * explicit retrain gets the longer timeout below.
 */
#define IIO_LINK_DETECT_TIMEOUT_MS	100
#define IIO_LINK_DISCOVERY_TIMEOUT_MS	600
#define IIO_LINK_TIMEOUT_MS		1000

static struct device_operations iio_root_port_ops;
static struct stopwatch iio_initial_detect_deadline;
static struct stopwatch iio_initial_link_deadline;
static bool iio_initial_link_deadline_set;

static bool iio_link_ready(struct device *rp, unsigned int cap)
{
	const u16 lnksta = pci_read_config16(rp, cap + PCI_EXP_LNKSTA);

	return lnksta != 0xffff && !(lnksta & PCI_EXP_LNKSTA_LT) &&
		(lnksta & IIO_LNKSTA_CLS) && (lnksta & IIO_LNKSTA_NLW);
}

static bool iio_wait_for_link(struct device *rp, unsigned int cap, struct stopwatch *sw)
{
	do {
		if (iio_link_ready(rp, cap))
			return true;
		udelay(100);
	} while (!stopwatch_expired(sw));

	printk(BIOS_DEBUG, "%s: link not ready, LnkSta=%04x after %ld ms\n",
	       dev_path(rp), pci_read_config16(rp, cap + PCI_EXP_LNKSTA),
	       (long)stopwatch_duration_msecs(sw));
	return false;
}

/* Wait for a link brought up by bifurcation, giving up early on an empty port. */
static bool iio_wait_for_initial_link(struct device *rp, unsigned int cap)
{
	u16 lnksta;

	do {
		if (iio_link_ready(rp, cap)) {
			stopwatch_tick(&iio_initial_link_deadline);
			printk(BIOS_DEBUG, "%s: link up, LnkSta=%04x at %ld ms\n", dev_path(rp),
			       pci_read_config16(rp, cap + PCI_EXP_LNKSTA),
			       (long)stopwatch_duration_msecs(&iio_initial_link_deadline));
			return true;
		}

		lnksta = pci_read_config16(rp, cap + PCI_EXP_LNKSTA);
		if (!(lnksta & (PCI_EXP_LNKSTA_LT | IIO_LNKSTA_NLW)) &&
		    stopwatch_expired(&iio_initial_detect_deadline)) {
			printk(BIOS_DEBUG, "%s: no device detected, LnkSta=%04x\n",
			       dev_path(rp), lnksta);
			return false;
		}
		udelay(100);
	} while (!stopwatch_expired(&iio_initial_link_deadline));

	printk(BIOS_DEBUG, "%s: link not ready, LnkSta=%04x after %ld ms\n",
	       dev_path(rp), pci_read_config16(rp, cap + PCI_EXP_LNKSTA),
	       (long)stopwatch_duration_msecs(&iio_initial_link_deadline));
	return false;
}

static void iio_train_link(struct device *rp, unsigned int cap, u32 lnkcap)
{
	static const unsigned int link_speed[] = { 0, 2500, 5000, 8000, 16000 };
	const unsigned int target = lnkcap & PCI_EXP_LNKCAP_MLS;
	u16 lnksta = pci_read_config16(rp, cap + PCI_EXP_LNKSTA);
	struct stopwatch sw;

	if (!iio_link_ready(rp, cap) || (lnksta & IIO_LNKSTA_CLS) >= target)
		return;

	/* Change only Link Control 2, not the adjacent Link Status 2 register. */
	const u16 lnkctl2 = pci_read_config16(rp, cap + IIO_PCI_EXP_LNKCTL2);
	pci_write_config16(rp, cap + IIO_PCI_EXP_LNKCTL2, (lnkctl2 & ~IIO_LNKCTL2_TLS) | target);
	pci_or_config16(rp, cap + PCI_EXP_LNKCTL, PCI_EXP_LNKCTL_RL);
	stopwatch_init_msecs_expire(&sw, IIO_LINK_TIMEOUT_MS);
	if (!iio_wait_for_link(rp, cap, &sw)) {
		pci_write_config16(rp, cap + IIO_PCI_EXP_LNKCTL2, lnkctl2);
		printk(BIOS_WARNING, "%s: retrain timed out, restored Link Control 2\n",
		       dev_path(rp));
		return;
	}

	lnksta = pci_read_config16(rp, cap + PCI_EXP_LNKSTA);
	const unsigned int cls = lnksta & IIO_LNKSTA_CLS;
	printk(BIOS_INFO, "%s: link at %u MT/s x%u, target %u MT/s\n", dev_path(rp),
	       cls < ARRAY_SIZE(link_speed) ? link_speed[cls] : 0,
	       (lnksta & IIO_LNKSTA_NLW) >> 4,
	       target < ARRAY_SIZE(link_speed) ? link_speed[target] : 0);
}

static bool iio_wait_for_endpoint(struct bus *bus, unsigned int cap, u32 lnkcap)
{
	struct device *rp = bus->dev;
	const pci_devfn_t endpoint = PCI_DEV((bus->segment_group << 8) | bus->secondary, 0, 0);
	const u16 rootctl = pci_read_config16(rp, cap + PCI_EXP_RTCTL);
	struct stopwatch sw;
	u16 vendor = 0xffff;
	bool ready = false;

	/* Expose CRS to the polling loop instead of letting hardware retry it. */
	if (pci_read_config16(rp, cap + PCI_EXP_RTCAP) & IIO_RTCAP_CRSSV)
		pci_or_config16(rp, cap + PCI_EXP_RTCTL, PCI_EXP_RTCTL_CRSSVE);

	stopwatch_init_msecs_expire(&sw, IIO_LINK_TIMEOUT_MS);
	do {
		if (iio_link_ready(rp, cap)) {
			vendor = pci_s_read_config16(endpoint, PCI_VENDOR_ID);
			/* Vendor ID 1 is Configuration Request Retry Status, not a device. */
			if (vendor != 0 && vendor != 0xffff && vendor != 1) {
				ready = true;
				break;
			}
		}
		udelay(100);
	} while (!stopwatch_expired(&sw));

	pci_write_config16(rp, cap + PCI_EXP_RTCTL, rootctl);
	if (ready && (lnkcap & PCI_EXP_LNKCAP_DLLLARC) &&
	    !(pci_read_config16(rp, cap + PCI_EXP_LNKSTA) & PCI_EXP_LNKSTA_DLLLA))
		printk(BIOS_DEBUG, "%s: endpoint responds despite DLLLA being clear\n",
		       dev_path(rp));
	stopwatch_tick(&sw);
	printk(ready ? BIOS_DEBUG : BIOS_WARNING,
	       "%s: endpoint %s, vendor %04x after %ld ms\n", dev_path(rp),
	       ready ? "ready" : "not ready", vendor, (long)stopwatch_duration_msecs(&sw));
	return ready;
}

static void iio_scan_bus(struct bus *bus, unsigned int min_devfn, unsigned int max_devfn)
{
	struct device *rp = bus->dev;
	const unsigned int cap = pci_find_capability(rp, PCI_CAP_ID_PCIE);

	if (!cap)
		return;
	const u32 lnkcap = pci_read_config32(rp, cap + PCI_EXP_LNKCAP);
	if (pci_read_config16(rp, cap + PCI_EXP_LNKCTL) & PCI_EXP_LNKCTL_LD)
		return;
	if (!iio_wait_for_initial_link(rp, cap))
		return;

	if (!iio_wait_for_endpoint(bus, cap, lnkcap))
		return;

	pciexp_scan_bus(bus, min_devfn, max_devfn);

	/* Do not let an optional speed upgrade decide whether a device is discovered. */
	iio_train_link(rp, cap, lnkcap);
	iio_wait_for_endpoint(bus, cap, lnkcap);
}

static void iio_scan_bridge(struct device *dev)
{
	const struct device *domain = dev_get_domain(dev);
	const struct northbridge_intel_grantley_config *cfg = domain ? domain->chip_info : NULL;

	/*
	 * A board that starts the links has to wait for them somewhere; boards that
	 * leave training to the Uncore PEIM only need this when the readiness quirk
	 * is opted into.
	 */
	if (!cfg || !(cfg->start_iio_links || cfg->iio_readiness_quirk)) {
		pciexp_scan_bridge(dev);
		return;
	}

	/*
	 * grantley_iio_init() started all stacks in romstage. Share the deadlines
	 * rather than restarting the discovery windows for every empty port.
	 */
	if (!iio_initial_link_deadline_set) {
		stopwatch_init_msecs_expire(&iio_initial_detect_deadline,
					    IIO_LINK_DETECT_TIMEOUT_MS);
		stopwatch_init_msecs_expire(&iio_initial_link_deadline,
					    IIO_LINK_DISCOVERY_TIMEOUT_MS);
		iio_initial_link_deadline_set = true;
	}

	/* The callback runs after bus routing is installed, before child probing. */
	do_pci_scan_bridge(dev, iio_scan_bus);
}

static const char *iio_root_port_acpi_name(const struct device *dev)
{
	if (!is_pci(dev) || !dev->upstream)
		return NULL;

	/* Only direct children inherit this port's naming callback. */
	if (dev->upstream->dev && dev->upstream->dev->ops == &iio_root_port_ops)
		return dev->name;

	if (dev->upstream->secondary != 0)
		return NULL;

	/* Keep CPU IIO ports separate from the PCH's RP01..RP08 namespace. */
	switch (dev->path.pci.devfn) {
	case PCI_DEVFN(1, 0): return "IIO0";
	case PCI_DEVFN(1, 1): return "IIO1";
	case PCI_DEVFN(2, 0): return "IIO2";
	case PCI_DEVFN(2, 1): return "IIO3";
	case PCI_DEVFN(2, 2): return "IIO4";
	case PCI_DEVFN(2, 3): return "IIO5";
	case PCI_DEVFN(3, 0): return "IIO6";
	case PCI_DEVFN(3, 1): return "IIO7";
	case PCI_DEVFN(3, 2): return "IIO8";
	case PCI_DEVFN(3, 3): return "IIO9";
	default: return NULL;
	}
}

static void iio_root_port_init(struct device *dev)
{
	struct device *child = NULL;
	const unsigned int cap = pci_find_capability(dev, PCI_CAP_ID_PCIE);

	/* Clear ASPM (bits 1:0) on IIO root port to prevent L1 substate latency lockup */
	if (cap) {
		u16 lnkctl = pci_read_config16(dev, cap + PCI_EXP_LNKCTL);
		lnkctl &= ~0x0003;
		pci_write_config16(dev, cap + PCI_EXP_LNKCTL, lnkctl);
	}

	while ((child = dev_bus_each_child(dev->downstream, child))) {
		if (!is_pci(child) || !child->enabled)
			continue;

		/* As on Xeon SP, give each PCI function a distinct ACPI name. */
		char *name = xmalloc(ACPI_NAME_BUFFER_SIZE);
		snprintf(name, ACPI_NAME_BUFFER_SIZE, "DC%02X", child->path.pci.devfn);
		child->name = name;
	}
}

static void iio_root_port_fill_ssdt(const struct device *dev)
{
	if (dev->enabled)
		acpigen_pci_root_port_fill_ssdt(dev, -1, true);
}

/* Preserve the generic PCIe bridge's lack of subsystem-ID programming. */
static struct pci_operations iio_root_port_pci_ops = {
	.set_subsystem = NULL,
};

static struct device_operations iio_root_port_ops = {
	.read_resources = pci_bus_read_resources,
	.set_resources = pci_dev_set_resources,
	.enable_resources = pci_bus_enable_resources,
	.init = iio_root_port_init,
	.scan_bus = iio_scan_bridge,
	.reset_bus = pci_bus_reset,
	.acpi_name = iio_root_port_acpi_name,
	.acpi_fill_ssdt = iio_root_port_fill_ssdt,
	.ops_pci = &iio_root_port_pci_ops,
};

static const unsigned short iio_root_port_ids[] = {
	/* Haswell-EP: ports 1A..1B, 2A..2D and 3A..3D. */
	0x2f02, 0x2f03, 0x2f04, 0x2f05, 0x2f06,
	0x2f07, 0x2f08, 0x2f09, 0x2f0a, 0x2f0b,
	/* Broadwell-EP: the same IIO port layout. */
	0x6f02, 0x6f03, 0x6f04, 0x6f05, 0x6f06,
	0x6f07, 0x6f08, 0x6f09, 0x6f0a, 0x6f0b,
	0
};

static const struct pci_driver iio_root_port_driver __pci_driver = {
	.ops = &iio_root_port_ops,
	.vendor = PCI_VID_INTEL,
	.devices = iio_root_port_ids,
};
