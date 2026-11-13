/* SPDX-License-Identifier: GPL-2.0-only */

#include <console/console.h>
#include <cpu/cpu.h>
#include <cpu/intel/common/common.h>
#include <cpu/intel/speedstep.h>
#include <cpu/x86/msr.h>
#include <drivers/intel/oc_mailbox/oc_mailbox.h>
#include "grantley.h"

#define OC_MBOX_PLANE_CORE		0
#define OC_MBOX_PLANE_CACHE		2
#define OC_MBOX_CMD_OVERRIDE_WRITE	0x11

static void set_voltage_offset(uint8_t plane, int16_t offset_mv)
{
	const union oc_mailbox_interface cmd = {
		.cmd_code = OC_MBOX_CMD_OVERRIDE_WRITE,
		.param1   = plane,
	};
	/* FIVR offset is 11-bit signed fixed point in units of 1/1024 V */
	int32_t val = ((int32_t)offset_mv * 1024) / 1000;
	uint32_t data = ((uint32_t)val & 0x7ff) << 21;

	printk(BIOS_INFO,
	       "Grantley: Setting plane %u voltage offset %d mV (data 0x%08x)\n",
	       plane, offset_mv, data);
	if (oc_mailbox_write(cmd, data) < 0)
		printk(BIOS_ERR,
		       "Grantley: Failed to write OC mailbox for plane %u\n",
		       plane);
}

void grantley_apply_cpu_overrides(const struct grantley_cpu_params *params)
{
	if (!params)
		return;

	if (params->core_voltage_offset_mv != 0)
		set_voltage_offset(OC_MBOX_PLANE_CORE,
				   params->core_voltage_offset_mv);

	if (params->uncore_voltage_offset_mv != 0)
		set_voltage_offset(OC_MBOX_PLANE_CACHE,
				   params->uncore_voltage_offset_mv);
}

#define MSR_PLATFORM_INFO		0xce
#define MSR_IA32_PERF_CTL		0x199
#define MSR_MISC_PWR_MGMT		0x1aa
#define MSR_POWER_CTL			0x1fc
#define MSR_PKG_POWER_LIMIT		0x610
#define MSR_PKG_POWER_SKU		0x614

#define MONITOR_FSM_ENABLE		BIT(18)
#define CST_IO_MWAIT_REDIRECTION		BIT(10)
#define CST_CONFIG_LOCK			BIT(15)
#define SINGLE_PCTL_ENABLE		BIT(0)
#define MISC_PWR_MGMT_LOCK		BIT(13)
#define POWER_CTL_SAPM_IMC_C2_POLICY	BIT(2)
#define POWER_CTL_EPB_ENABLE		BIT(18)
#define PLATFORM_INFO_TDP_PROGRAMMABLE	BIT(29)
#define PKG_POWER_LIMIT_MASK		0x7fff
#define PKG_POWER_LIMIT_ENABLE		BIT(15)
#define PKG_POWER_LIMIT_LOCK		BIT(31)

/*
 * Keep the Xeon SP split between per-CPU setup and package handoff, but use
 * the C-state MSR layout that this platform's firmware uses.
 */
static void configure_c_states(uint32_t features)
{
	msr_t msr;

	if (features & BIT(3)) {
		msr = rdmsr(IA32_MISC_ENABLE);
		msr.lo |= MONITOR_FSM_ENABLE;
		wrmsr(IA32_MISC_ENABLE, msr);
	}

	msr = rdmsr(MSR_PKG_CST_CONFIG_CONTROL);
	if (!(msr.lo & CST_CONFIG_LOCK)) {
		/* _CST uses native MWAIT, not I/O-to-MWAIT redirection. */
		if (msr.lo & CST_IO_MWAIT_REDIRECTION) {
			msr.lo &= ~CST_IO_MWAIT_REDIRECTION;
			wrmsr(MSR_PKG_CST_CONFIG_CONTROL, msr);
		}
		/* Clamp package C-state limit to C0/C1 to eliminate deep C-state wake latency */
		msr.lo = (msr.lo & ~0x7) | 0x0;
		wrmsr(MSR_PKG_CST_CONFIG_CONTROL, msr);
	}

	msr = rdmsr(MSR_POWER_CTL);
	/* Leave the IMC C2 policy disabled: the memory PLL remains on. */
	/* Disable hardware automatic C1E (bit 1) to eliminate transient FIVR droop during AP wakeups */
	msr.lo &= ~BIT(1);
	msr.lo &= ~POWER_CTL_SAPM_IMC_C2_POLICY;
	/* Expose IA32_ENERGY_PERF_BIAS and its CPUID bit, as on Haswell. */
	msr.lo |= POWER_CTL_EPB_ENABLE;
	wrmsr(MSR_POWER_CTL, msr);
}

static void configure_p_states(uint32_t features)
{
	msr_t msr;
	uint32_t ratio;

	if (!(features & BIT(7)))
		return;

	msr = rdmsr(MSR_MISC_PWR_MGMT);
	if (!(msr.lo & MISC_PWR_MGMT_LOCK)) {
		/* Hardware coordinates package requests; _PSD reads this actual bit. */
		msr.lo &= ~SINGLE_PCTL_ENABLE;
		wrmsr(MSR_MISC_PWR_MGMT, msr);
	}

	/* Start at the fused non-turbo ratio, without changing turbo limits. */
	ratio = rdmsr(MSR_PLATFORM_INFO).lo & 0xff00;
	if (ratio) {
		msr = rdmsr(MSR_IA32_PERF_CTL);
		msr.lo = (msr.lo & ~0xff00) | ratio;
		wrmsr(MSR_IA32_PERF_CTL, msr);
	}
}

bool grantley_cpu_pm_supported(void)
{
	const uint32_t signature = cpuid_eax(1);

	switch (grantley_cpu_model(signature)) {
	case GRANTLEY_HASWELL_EP:
		/* Pre-production HSX steppings lack the DDR crossover policy. */
		return (signature & 0xf) >= 2;
	case GRANTLEY_BROADWELL_EP:
		/*
		 * Broadwell-EP exposes the same C/P-state MSRs and PCU mailboxes.
		 * Keep the Haswell-EP memory PLL policy (SAPM MPLL off stays
		 * disabled) until a Broadwell-EP DDR power policy is validated.
		 */
		return true;
	default:
		return false;
	}
}

void grantley_configure_cpu_pm(void)
{
	const uint32_t features = cpu_get_feature_flags_ecx();

	if (!grantley_cpu_pm_supported())
		return;
	configure_c_states(features);
	configure_p_states(features);
	printk(BIOS_DEBUG, "Grantley: CPU%u PM CST=%08x POWER_CTL=%08x MISC_PWR=%08x\n",
	       (unsigned int)cpu_index(), rdmsr(MSR_PKG_CST_CONFIG_CONTROL).lo,
	       rdmsr(MSR_POWER_CTL).lo, rdmsr(MSR_MISC_PWR_MGMT).lo);
}

/* Called once on the BSP before CPL3; preserve active PL1 and all PL2 fields. */
void grantley_configure_package_power_limit(void)
{
	msr_t limit = rdmsr(MSR_PKG_POWER_LIMIT);
	const uint32_t tdp = rdmsr(MSR_PKG_POWER_SKU).lo & PKG_POWER_LIMIT_MASK;

	printk(BIOS_DEBUG, "Grantley: PKG_POWER_LIMIT=%08x:%08x, SKU TDP raw=%u\n",
	       limit.hi, limit.lo, tdp);
	if ((limit.lo & PKG_POWER_LIMIT_ENABLE) && (limit.lo & PKG_POWER_LIMIT_MASK))
		return;

	if ((limit.hi & PKG_POWER_LIMIT_LOCK) ||
	    !(rdmsr(MSR_PLATFORM_INFO).lo & PLATFORM_INFO_TDP_PROGRAMMABLE) || !tdp) {
		printk(BIOS_WARNING, "Grantley: cannot initialize missing PL1; preserving RAPL\n");
		return;
	}

	/* SKU and limit share RAPL units. Preserve the time window, clamp and PL2. */
	limit.lo &= ~PKG_POWER_LIMIT_MASK;
	/* Enforce PL1 clamping (bit 16) alongside TDP limit */
	limit.lo |= tdp | PKG_POWER_LIMIT_ENABLE | BIT(16);
	wrmsr(MSR_PKG_POWER_LIMIT, limit);
	limit = rdmsr(MSR_PKG_POWER_LIMIT);
	if ((limit.lo & (PKG_POWER_LIMIT_MASK | PKG_POWER_LIMIT_ENABLE)) !=
	    (tdp | PKG_POWER_LIMIT_ENABLE)) {
		printk(BIOS_WARNING, "Grantley: PL1 readback differs, PKG_POWER_LIMIT=%08x:%08x\n",
		       limit.hi, limit.lo);
		return;
	}
	printk(BIOS_DEBUG, "Grantley: initialized PL1 from SKU, PKG_POWER_LIMIT=%08x:%08x\n",
	       limit.hi, limit.lo);
}
