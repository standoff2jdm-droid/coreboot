/* SPDX-License-Identifier: GPL-2.0-or-later */

#include <bootstate.h>
#include <console/console.h>
#include <cpu/intel/grantley/grantley.h>
#include <cpu/intel/grantley/grantley_routing.h>
#include <delay.h>
#include <device/pci_def.h>
#include <device/pci_ids.h>
#include <device/pci_ops.h>
#include <timer.h>

#define PCU_BIOS_MAILBOX_DATA	0x8c
#define PCU_BIOS_MAILBOX_INTERFACE 0x90
#define PCU_MAILBOX_RUN_BUSY	BIT(31)
#define PCU_MAILBOX_READ_MISC	0x05
#define PCU_MAILBOX_WRITE_MISC	0x06
#define PCU_MAILBOX_UNSUPPORTED	0x01
#define PCU_MISC_PL2_SAFETY_NET	BIT(1)
#define PCU_MISC_PKGC_RCOMP_DISABLE BIT(23)
#define PCU_BIOS_RESET_CPL	0x94
#define PCU_RST_CPL1		BIT(1)
#define PCU_RST_CPL2		BIT(2)
#define PCU_RST_CPL3		BIT(3)
#define PCU_SAPMCTL		0xb0
#define PCU_SAPM_MPLL_OFF_ENABLE	BIT(1)
#define PCU_SAPM_IIO_L1_ENABLE		BIT(2)
#define PCU_SAPM_QPI_L1_ENABLE		BIT(3)
#define PCU_SAPM_ASPM_MASK		(BIT(1) | BIT(2) | BIT(3))
#define PCU_SAPM_LOCK		BIT(31)
#define PCU_MCA_ERR_SRC_LOG	0xec /* Function 2. */

/* Adapted from soc/intel/xeon_sp/pcu1.c. */
static u32 wait_for_pcu(pci_devfn_t pcu, unsigned int reg, u32 mask, u32 expected)
{
	struct stopwatch sw;
	u32 value;

	stopwatch_init_msecs_expire(&sw, 5000);
	for (;;) {
		value = pci_s_read_config32(pcu, reg);
		if (value != UINT32_MAX && (value & mask) == expected)
			return value;
		if (stopwatch_expired(&sw)) {
			printk(BIOS_ERR, "Grantley: PCU reg %02x=%08x, expected %08x/%08x\n",
			       reg, value, expected, mask);
			die("Grantley: PCU operation timed out\n");
		}
		udelay(50);
	}
}

static u8 pcu_mailbox_command(pci_devfn_t pcu, u8 command, u32 *data)
{
	u32 status;

	wait_for_pcu(pcu, PCU_BIOS_MAILBOX_INTERFACE, PCU_MAILBOX_RUN_BUSY, 0);
	pci_s_write_config32(pcu, PCU_BIOS_MAILBOX_DATA, *data);
	pci_s_write_config32(pcu, PCU_BIOS_MAILBOX_INTERFACE, PCU_MAILBOX_RUN_BUSY | command);
	status = wait_for_pcu(pcu, PCU_BIOS_MAILBOX_INTERFACE, PCU_MAILBOX_RUN_BUSY, 0) & 0xff;
	if (!status)
		*data = pci_s_read_config32(pcu, PCU_BIOS_MAILBOX_DATA);
	return status;
}

static void configure_sapm(pci_devfn_t pcu)
{
	const u32 before = pci_s_read_config32(pcu, PCU_SAPMCTL);
	/* Enforce hardware ASPM disable: clear MPLL and IIO/QPI L1 sleep states */
	const u32 mask = PCU_SAPM_ASPM_MASK;
	const u32 desired = before & ~PCU_SAPM_ASPM_MASK;

	/* Production Haswell-EP and Broadwell-EP keep the memory PLL on. */
	if (before == UINT32_MAX)
		die("Grantley: SAPMCTL is inaccessible\n");
	if (before != desired) {
		if (before & PCU_SAPM_LOCK)
			die("Grantley: incompatible SAPMCTL is already locked\n");
		pci_s_write_config32(pcu, PCU_SAPMCTL, desired);
		if ((pci_s_read_config32(pcu, PCU_SAPMCTL) & mask) != (desired & mask))
			die("Grantley: SAPMCTL configuration did not stick\n");
	}
	printk(BIOS_DEBUG, "Grantley: SAPMCTL=%08x -> %08x\n", before, desired);
}

static void configure_pcu_misc(pci_devfn_t pcu)
{
	const u32 required = PCU_MISC_PL2_SAFETY_NET | PCU_MISC_PKGC_RCOMP_DISABLE;
	u32 data = 0;
	u32 desired;
	u8 status = pcu_mailbox_command(pcu, PCU_MAILBOX_READ_MISC, &data);

	/* An older PCU may not implement the command; that is not fatal here. */
	if (status == PCU_MAILBOX_UNSUPPORTED) {
		printk(BIOS_WARNING, "Grantley: PCU_MISC_CONFIG command is unsupported\n");
		return;
	}
	if (status) {
		printk(BIOS_ERR, "Grantley: PCU_MISC_CONFIG read status=%02x\n", status);
		die("Grantley: cannot read PCU PM configuration\n");
	}

	/*
	 * Enable the PL2 safety net and disable the package C-state RCOMP path.
	 * Production HSX/BDX parts need both while the memory PLL keeps running.
	 */
	desired = data | required;
	printk(BIOS_DEBUG, "Grantley: PCU_MISC_CONFIG=%08x -> %08x\n", data, desired);
	if (data == desired)
		return;
	data = desired;
	status = pcu_mailbox_command(pcu, PCU_MAILBOX_WRITE_MISC, &data);
	if (status == PCU_MAILBOX_UNSUPPORTED) {
		printk(BIOS_WARNING, "Grantley: PCU_MISC_CONFIG write unsupported; policy not applied\n");
		return;
	}
	if (status) {
		printk(BIOS_ERR, "Grantley: PCU_MISC_CONFIG write status=%02x\n", status);
		die("Grantley: cannot configure PCU power management\n");
	}
	data = 0;
	status = pcu_mailbox_command(pcu, PCU_MAILBOX_READ_MISC, &data);
	if (status || (data & required) != required)
		die("Grantley: PCU PM configuration readback failed\n");
}

static void set_bios_reset_cpl(pci_devfn_t pcu, unsigned int phase)
{
	const u32 request = BIT(phase);
	const u32 ack = BIT(phase + 8);
	u32 value = pci_s_read_config32(pcu, PCU_BIOS_RESET_CPL);

	if (value == UINT32_MAX)
		die("Grantley: PCU is inaccessible\n");
	pci_s_write_config32(pcu, PCU_BIOS_RESET_CPL, value | request);
	wait_for_pcu(pcu, PCU_BIOS_RESET_CPL, ack, ack);
}

static void pm_post_phase3(u8 bus)
{
	const pci_devfn_t pcu2 = PCI_DEV(bus, 30, 2);
	const u32 id = pci_s_read_config32(pcu2, PCI_VENDOR_ID);
	u32 value;

	if (id != ((0x2f9aU << 16) | PCI_VID_INTEL) &&
	    id != ((0x6f9aU << 16) | PCI_VID_INTEL))
		die("Grantley: PCU function 2 is inaccessible\n");
	/*
	 * Clear the PCU MCA source status only after ACK3: doing it any earlier
	 * drives the package into a shutdown loop. Log the source before clearing
	 * it and leave the MCI banks alone.
	 */
	value = pci_s_read_config32(pcu2, PCU_MCA_ERR_SRC_LOG);
	if (value == UINT32_MAX)
		die("Grantley: PCU MCA source status is inaccessible\n");
	if (value)
		printk(BIOS_WARNING, "Grantley: pre-clear PCU MCA_ERR_SRC_LOG=%08x\n", value);
	pci_s_write_config32(pcu2, PCU_MCA_ERR_SRC_LOG, 0);
	value = pci_s_read_config32(pcu2, PCU_MCA_ERR_SRC_LOG);
	if (value)
		printk(BIOS_WARNING, "Grantley: PCU MCA_ERR_SRC_LOG remains %08x\n", value);
}

static void grantley_bios_init_completion(void *unused)
{
	const u8 bus = grantley_uncore_bus();
	const pci_devfn_t pcu = PCI_DEV(bus, 30, 1);
	const u32 id = pci_s_read_config32(pcu, PCI_VENDOR_ID);
	bool prepare_pm;
	u32 value;

	if (id != ((0x2f99U << 16) | PCI_VID_INTEL) &&
	    id != ((0x6f99U << 16) | PCI_VID_INTEL))
		die("Grantley: PCU not found on the active uncore bus\n");

	value = pci_s_read_config32(pcu, PCU_BIOS_RESET_CPL);
	printk(BIOS_DEBUG, "Grantley: %02x:1e.1 BIOS_RESET_CPL=%08x\n", bus, value);
	if (value == UINT32_MAX ||
	    (value & (PCU_RST_CPL1 | PCU_RST_CPL2)) != (PCU_RST_CPL1 | PCU_RST_CPL2))
		die("Grantley: Uncore initialization is incomplete\n");
	/* Memory calibration must be acknowledged before the late PM commands. */
	wait_for_pcu(pcu, PCU_BIOS_RESET_CPL,
		     (PCU_RST_CPL1 | PCU_RST_CPL2) << 8,
		     (PCU_RST_CPL1 | PCU_RST_CPL2) << 8);

	prepare_pm = !(value & PCU_RST_CPL3) && grantley_cpu_pm_supported();
	if (prepare_pm) {
		grantley_configure_package_power_limit();
		configure_sapm(pcu);
		configure_pcu_misc(pcu);
	} else if (!(value & PCU_RST_CPL3)) {
		printk(BIOS_WARNING, "Grantley: native PM is not implemented for this CPU stepping\n");
	}
	set_bios_reset_cpl(pcu, 3);
	if (prepare_pm)
		pm_post_phase3(bus);
	/* CPL4 locks the BIOS mailbox; issue it only after PM configuration. */
	set_bios_reset_cpl(pcu, 4);

	printk(BIOS_DEBUG, "Grantley: BIOS_RESET_CPL=%08x after PM handoff\n",
	       pci_s_read_config32(pcu, PCU_BIOS_RESET_CPL));
}

BOOT_STATE_INIT_ENTRY(BS_WRITE_TABLES, BS_ON_ENTRY, grantley_bios_init_completion, NULL);
BOOT_STATE_INIT_ENTRY(BS_OS_RESUME, BS_ON_ENTRY, grantley_bios_init_completion, NULL);
