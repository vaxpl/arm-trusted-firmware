/*
 * Copyright (c) 2014-2015, ARM Limited and Contributors. All rights reserved.
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

#ifndef __PLAT_PRIVATE_H__
#define __PLAT_PRIVATE_H__

#include <psci.h>

/*******************************************************************************
 * For hisi socs pm ops
 ******************************************************************************/
struct hisi_pm_ops_cb {
	int (*cores_pwr_dm_on)(unsigned long mpidr, uint64_t entrypoint);
	int (*cores_pwr_dm_off)(void);
	int (*cores_pwr_dm_on_finish)(void);
	int (*cores_pwr_dm_suspend)(const psci_power_state_t *target_state, uint64_t entrypoint);
	int (*cores_pwr_dm_suspend_finish)(const psci_power_state_t *target_state);
	int (*cores_pwr_dm_resume)(void);
	int (*sys_pwr_dm_suspend)(void);
	int (*sys_pwr_dm_resume)(void);
	int (*sys_gbl_soft_reset)(void);
};

/*******************************************************************************
 * Function and variable prototypes
 ******************************************************************************/
void configure_mmu_el3(unsigned long total_base,
			    unsigned long total_size,
			    unsigned long,
			    unsigned long,
			    unsigned long,
			    unsigned long);


void plat_delay_timer_init(void);

void plat_hisi_gic_driver_init(void);
void plat_hisi_gic_init(void);
void plat_hisi_gic_cpuif_enable(void);
void plat_hisi_gic_cpuif_disable(void);
void plat_hisi_gic_pcpu_init(void);

void plat_hisi_pmc_init(void);
void plat_setup_hisi_pm_ops(struct hisi_pm_ops_cb *ops);
void plat_hisi_cci_enable(void);

#endif /* __PLAT_PRIVATE_H__ */
