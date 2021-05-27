/*
 * Copyright (c) 2015, ARM Limited and Contributors. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this
 * list of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * Neither the name of ARM nor the names of its contributors may be used
 * to endorse or promote products derived from this software without specific
 * prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <arch_helpers.h>
#include <assert.h>
#include <bl_common.h>
#include <context.h>
#include <context_mgmt.h>
#include <debug.h>
#include <mmio.h>
#include <platform.h>
#include <platform_def.h>
#include <psci.h>
#include <plat_private.h>
#include <console.h>

extern uint64_t hisi_bl31_phys_base;
extern uint64_t hisi_sec_entry_point;

static struct hisi_pm_ops_cb *hisi_ops;

/*******************************************************************************
 * This handler is called by the PSCI implementation during the `SYSTEM_SUSPEND`
 * call to get the `power_state` parameter. This allows the platform to encode
 * the appropriate State-ID field within the `power_state` parameter which can
 * be utilized in `pwr_domain_suspend()` to suspend to system affinity level.
******************************************************************************/
void hisi_get_sys_suspend_power_state(psci_power_state_t *req_state)
{
	/* lower affinities use PLAT_MAX_OFF_STATE */
	for (int i = MPIDR_AFFLVL0; i <= PLAT_MAX_PWR_LVL; i++)
		req_state->pwr_domain_state[i] = PLAT_MAX_OFF_STATE;

}

/*******************************************************************************
 * Handler called when an affinity instance is about to enter standby.
 ******************************************************************************/
void hisi_cpu_standby(plat_local_state_t cpu_state)
{
	/*
	 * Enter standby state
	 * dsb is good practice before using wfi to enter low power states
	 */
	dsb();
	wfi();
}

/*******************************************************************************
 * Handler called when an affinity instance is about to be turned on. The
 * level and mpidr determine the affinity instance.
 ******************************************************************************/
int hisi_pwr_domain_on(u_register_t mpidr)
{
	if (hisi_ops && hisi_ops->cores_pwr_dm_on)
		hisi_ops->cores_pwr_dm_on(mpidr, hisi_sec_entry_point);

	return PSCI_E_SUCCESS;
}

/*******************************************************************************
 * Handler called when a power domain is about to be turned off. The
 * target_state encodes the power state that each level should transition to.
 ******************************************************************************/
void hisi_pwr_domain_off(const psci_power_state_t *target_state)
{
	if (hisi_ops && hisi_ops->cores_pwr_dm_off)
		hisi_ops->cores_pwr_dm_off();
}

/*******************************************************************************
 * Handler called when called when a power domain is about to be suspended. The
 * target_state encodes the power state that each level should transition to.
 ******************************************************************************/
void hisi_pwr_domain_suspend(const psci_power_state_t *target_state)
{
	if (hisi_ops && hisi_ops->cores_pwr_dm_suspend)
		hisi_ops->cores_pwr_dm_suspend(target_state, hisi_sec_entry_point);
}

/*******************************************************************************
 * Handler called when a power domain has just been powered on after
 * being turned off earlier. The target_state encodes the low power state that
 * each level has woken up from.
 ******************************************************************************/
void hisi_pwr_domain_on_finish(const psci_power_state_t *target_state)
{
		__asm volatile("mrs     x0, cnthctl_el2\n"
			"orr     x0, x0, #0x3\n"
			"msr     cnthctl_el2, x0\n"
			"msr	cntvoff_el2, xzr\n"
			"msr     cntvoff_el2, x0\n"
			"orr     x0, x0, #0x3\n"
			"msr     cntkctl_el1, x0\n");
	/* Program the gic per-cpu distributor or re-distributor interface */
	plat_hisi_gic_cpuif_enable();
}

/*******************************************************************************
 * Handler called when a power domain has just been powered on after
 * having been suspended earlier. The target_state encodes the low power state
 * that each level has woken up from.
 ******************************************************************************/
void hisi_pwr_domain_suspend_finish(const psci_power_state_t *target_state)
{
    if (hisi_ops && hisi_ops->cores_pwr_dm_suspend_finish)
		hisi_ops->cores_pwr_dm_suspend_finish(target_state);
}

void hisi_pwr_domain_pwr_down_wfi(const psci_power_state_t *target_state)
{
    while(1)
		wfi();
}

/*******************************************************************************
 * Handler called when the system wants to be powered off
 ******************************************************************************/
__dead2 void hisi_system_off(void)
{
	if (hisi_ops && hisi_ops->sys_gbl_soft_reset)
		hisi_ops->sys_gbl_soft_reset();
}

/*******************************************************************************
 * Handler called when the system wants to be restarted.
 ******************************************************************************/
__dead2 void hisi_system_reset(void)
{
	if (hisi_ops && hisi_ops->sys_gbl_soft_reset)
		hisi_ops->sys_gbl_soft_reset();
}

/*******************************************************************************
 * Handler called to check the validity of the power state parameter.
 ******************************************************************************/
int32_t hisi_validate_power_state(unsigned int power_state,
				   psci_power_state_t *req_state)
{
	int pwr_lvl = psci_get_pstate_pwrlvl(power_state);
	int i;

	assert(req_state);

	if (pwr_lvl > PLAT_MAX_PWR_LVL)
		return PSCI_E_INVALID_PARAMS;

	for (i = 0; i <= pwr_lvl; i++)
		req_state->pwr_domain_state[i] =
			PLAT_MAX_OFF_STATE;

	return PSCI_E_SUCCESS;
}

/*******************************************************************************
 * Platform handler called to check the validity of the non secure entrypoint.
 ******************************************************************************/
int hisi_validate_ns_entrypoint(uintptr_t entrypoint)
{
	/*
	 * Check if the non secure entrypoint lies within the non
	 * secure DRAM.
	 */
	return PSCI_E_SUCCESS;

}

/*******************************************************************************
 * Export the platform handlers to enable psci to invoke them
 ******************************************************************************/
static const plat_psci_ops_t hisi_plat_psci_ops = {
	.cpu_standby			= hisi_cpu_standby,
	.pwr_domain_on			= hisi_pwr_domain_on,
	.pwr_domain_off			= hisi_pwr_domain_off,
	.pwr_domain_suspend		= hisi_pwr_domain_suspend,
	.pwr_domain_on_finish		= hisi_pwr_domain_on_finish,
	.pwr_domain_suspend_finish	= hisi_pwr_domain_suspend_finish,
	.pwr_domain_pwr_down_wfi	= hisi_pwr_domain_pwr_down_wfi,
	.system_off			= hisi_system_off,
	.system_reset			= hisi_system_reset,
	.validate_power_state		= hisi_validate_power_state,
	.validate_ns_entrypoint		= hisi_validate_ns_entrypoint,
	.get_sys_suspend_power_state	= hisi_get_sys_suspend_power_state,
};

/*******************************************************************************
 * Export the platform specific power ops and initialize Power Controller
 ******************************************************************************/
int plat_setup_psci_ops(uintptr_t sec_entrypoint,
			const plat_psci_ops_t **psci_ops)
{

	/*
	 * Flush entrypoint variable to PoC since it will be
	 * accessed after a reset with the caches turned off.
	 */
	hisi_sec_entry_point = sec_entrypoint;
	flush_dcache_range((uint64_t)&hisi_sec_entry_point, sizeof(uint64_t));

	/*
	 * Initialize PSCI ops struct
	 */
	*psci_ops = &hisi_plat_psci_ops;

	return 0;
}

void plat_setup_hisi_pm_ops(struct hisi_pm_ops_cb *ops)
{
	hisi_ops = ops;
}
