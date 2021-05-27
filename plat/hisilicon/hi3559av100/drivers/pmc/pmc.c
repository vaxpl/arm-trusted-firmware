/*
 * Copyright (c) 2016, ARM Limited and Contributors. All rights reserved.
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
#include <debug.h>
#include <delay_timer.h>
#include <errno.h>
#include <mmio.h>
#include <platform.h>
#include <platform_def.h>
#include <plat_private.h>
#include <psci.h>

#define writel(val, addr)               mmio_write_32((uintptr_t)addr, (uint32_t)(val))
#define readl(addr)                             mmio_read_32((uintptr_t)addr)

static int cores_pwr_domain_on(unsigned long mpidr, uint64_t entrypoint)
{
	uint32_t cpu, cluster;
	cluster = MPIDR_AFFLVL1_VAL(mpidr);
	cpu = MPIDR_AFFLVL0_VAL(mpidr) + MPIDR_AFFLVL1_VAL(mpidr) * 2;

	/* Program the Slave core bootmode(aarch64) and bootaddr */
	writel(readl(REG_PERI_CPU_AARCH_MODE) | (0x1 << cpu), REG_PERI_CPU_AARCH_MODE);
	writel(entrypoint >> 2, REG_PERI_CPU_RVBARADDR);	 /* psci_entrypoint */

	unsigned long tmp;
	/* [0] cpu0 cken
	 * [1] cpu0 po_srst_req
	 * [2] cpu0 srst_req
	 * [4] cpu1 cken
	 * [5] cpu1 po_srst_req
	 * [6] cpu1 srst_req
	 * [8] cpu2 cken
	 * [9] cpu2 po_srst_req
	 * [10] cpu2 srst_req
	 * [12] cpu3 cken
	 * [13] cpu3 po_srst_req
	 * [14] cpu3 srst_req
	 * [20] cluster0 cken
	 * [21] cluster0 glb_srst_req
	 * [24] cluster1 cken
	 * [25] cluster1 glb_srst_req
	 */
	
	tmp = readl(CPU_REG_BASE_RST);

	if (1 == cluster) {
		/* release isolation*/
		writel((readl(PMC_PWR_CLUSTER1) & (~(0x1 << 3))), PMC_PWR_CLUSTER1);

		tmp |= 0x1 << 24; /* enable cluster cken */
		tmp &= ~(0x1 << 25); /* clear cluster reset */
	}
	if (1 == cpu) {
		tmp |= 0x1 << 4; /* enable cpu cken */
		tmp &= ~(0x6 << 4); /* clear cpu po_srst_req and  srst_req */
	} else if (2 == cpu) {
		tmp |= 0x1 << 8; /* enable cpu cken */
		tmp &= ~(0x6 << 8); /* clear cpu po_srst_req and  srst_req */
	} else if (3 == cpu) {
		tmp |= 0x1 << 12; /* enable cpu cken */
		tmp &= ~(0x6 << 12); /* clear cpu po_srst_req and  srst_req */
	} else {
		ERROR("[%s][%d] invalid cpu[0x%x]\n", __func__, __LINE__, cpu);
	}

	writel(tmp, CPU_REG_BASE_RST);

	return 0;
}

static int cores_pwr_domain_on_finish(void)
{
	INFO("[%s][%d]\n", __func__, __LINE__);
	return 0;
}

static int cores_pwr_domain_suspend(const psci_power_state_t *target_state, uint64_t entrypoint)
{
	unsigned int cpu = plat_my_core_pos();

	/* Prevent interrupts from spuriously waking up this cpu */
	plat_hisi_gic_cpuif_disable();

	/* Program the Slave core bootmode(aarch64) and bootaddr */
	writel(readl(REG_PERI_CPU_AARCH_MODE) | (0x1 << cpu), REG_PERI_CPU_AARCH_MODE);
	writel(entrypoint >> 2, REG_PERI_CPU_RVBARADDR);  /* psci_entrypoint */

	writel(0x1, CPU_PMC_BASE + PMC_CORE_PWRDN_CNT_EN(cpu)); /* enable cpu power on/down counter */
	writel(0x1, CPU_PMC_BASE + PMC_CORE_PWRDN_REQ(cpu)); /* cpu power down request */

	/* psci_cpu_suspend()
	 *  ->psci_cpu_suspend_start()
	 *    ->psci_plat_pm_ops->pwr_domain_suspend(state_info);
	 *    ->psci_suspend_to_pwrdown_start()
	 *      ->psci_do_pwrdown_cache_maintenance()
	 *    
     *
	 * psci_do_pwrdown_cache_maintenance() call cortex_a73_core_pwr_dwn() and cortex_a53_core_pwr_dwn() to do:
	 * disable d-cached
	 * clean & invalidate l1
	 * disable SMPEN
	 */

	return 0;
}

static int cores_pwr_domain_suspend_finish(const psci_power_state_t *target_state)
{
	plat_hisi_gic_cpuif_enable();
	return 0;
}

static int sys_pwr_domain_resume(void)
{
	INFO("[%s][%d]\n", __func__, __LINE__);
	return 0;
}

static int sys_pwr_domain_suspend(void)
{
	INFO("[%s][%d]\n", __func__, __LINE__);
	return 0;
}

static int sys_system_reset(void)
{
	/* Any value to this reg will reset the cpu */
	writel(0x12345678, SYS_CTRL_BASE + REG_SC_SYSRES);
	return 0;
}

static struct hisi_pm_ops_cb pm_ops = {
	.cores_pwr_dm_on = cores_pwr_domain_on,
	.cores_pwr_dm_on_finish = cores_pwr_domain_on_finish,
	.cores_pwr_dm_suspend = cores_pwr_domain_suspend,
	.cores_pwr_dm_suspend_finish = cores_pwr_domain_suspend_finish,
	.sys_pwr_dm_suspend = sys_pwr_domain_suspend,
	.sys_pwr_dm_resume = sys_pwr_domain_resume,
	.sys_gbl_soft_reset = sys_system_reset,
};

void plat_hisi_pmc_init(void)
{
	plat_setup_hisi_pm_ops(&pm_ops);
}
