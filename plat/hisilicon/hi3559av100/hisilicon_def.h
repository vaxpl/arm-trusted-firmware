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

#ifndef __PLAT_DEF_H__
#define __PLAT_DEF_H__

/* Special value used to verify platform parameters from BL2 to BL3-1 */
#define HISI_BL31_PLAT_PARAM_VAL	0x0f1e2d3c4b5a6978ULL

/*******************************************************************************
 * Platform memory map related constants
 ******************************************************************************/
#define DEVICE_BASE             0x00000000
#define DEVICE_SIZE             0x40000000

#define DRAM_NS_BASE            0x80000000
#define DRAM_NS_SIZE            0x40000000

#define TEE_SEC_MEM_BASE	0x90000000
#define TEE_SEC_MEM_SIZE	0x10000000

/*******************************************************************************
 * UART related constants
 ******************************************************************************/
#define HISILICON_UART0_BASE		(0x12100000)
#define HISILICON_BAUDRATE		(115200)
#define HISILICON_UART_CLOCK		(24000000)

/*******************************************************************************
 * System counter frequency related constants
 ******************************************************************************/
#define SYS_COUNTER_FREQ_IN_TICKS	24000000
#define SYS_COUNTER_FREQ_IN_MHZ		24

/* Base MTK_platform compatible GIC memory map */
#define HISILICON_GICD_BASE		(0x1F101000) /*  */
#define HISILICON_GICC_BASE		(0x1F102000) /*  */

/* HISILICON no secure IRQS */
#define HISILICON_IRQ_RESERVED	(156 + 32)
/*
 * Define a list of Group 1 Secure and Group 0 interrupts as per GICv3
 * terminology. On a GICv2 system or mode, the lists will be merged and treated
 * as Group 0 interrupts.
 */
#define HISI_G1S_IRQS  HISILICON_IRQ_RESERVED

/* cpu pmc register */
#define PMC_REG_BASE	        0x1D820000  /* 0x1C000000 + 0x01820000 = 0x1B820000 */
#define PMC_PWR_CLUSTER0        (PMC_REG_BASE + 0x1004)
#define PMC_STATUS_CLUSTER0     (PMC_REG_BASE + 0x100c)
#define PMC_PWR_CLUSTER1        (PMC_REG_BASE + 0x1104)
#define PMC_STATUS_CLUSTER1     (PMC_REG_BASE + 0x110c)

#define MISC_REG_BASE	        0x12030000
#define MISC_CTRL0              (MISC_REG_BASE + 0x0)

#define REG_PERI_CPU_RVBARADDR		0x1D830020 /* 0x1D830020*/
#define REG_PERI_CPU_AARCH_MODE		0x1D830024 /* 0x1D830024 */

#define SYS_CTRL_BASE	0x12020000
#define REG_SC_SYSRES	0x0004
#define REG_SYSSTAT	0x008c 

#define CPU_REG_BASE_RST                0x120100cc
#define CPU_PMC_BASE                    (PMC_REG_BASE)

#define PMC_CORE_PWRDN_REQ(cpu)   	 	((cpu) << 8) 		  /* core power down request */
#define PMC_CORE_PWRDN_MODE(cpu)   	 	(((cpu) << 8) + 0x4)  /* core power down mode */
#define PMC_CORE_PWRDN_PARAM(cpu)   	(((cpu) << 8) + 0x8)  /* core power down param */
#define PMC_CORE_PWRDN_CNT_EN(cpu)   	(((cpu) << 8) + 0xc)  /* core power down/on counter enable */
#define PMC_CORE_PWRDN_CNT(cpu)   	 	(((cpu) << 8) + 0x10) /* core power down time */
#define PMC_CORE_PWRON_CNT(cpu)   	 	(((cpu) << 8) + 0x14) /* core power on time */
#define PMC_CORE_PWRDN_DONE_CNT(cpu)   	(((cpu) << 8) + 0x14) /* core power off time */
#define PMC_CORE_PWR_STATUS(cpu)   	 	(((cpu) << 8) + 0x1c) /* core power status */

#define PMC_CLUSTER_PWRDN_REQ(cls)      (0x1000 + ((cls) << 8))         /* cluster power down request */
#define PMC_CLUSTER_PWRDN_MODE(cls)   	((0x1000 + ((cls) << 8)) + 0x4) /* cluster power down mode */
#define PMC_CLUSTER_PWRDN_PARAM(cls)   	((0x1000 + ((cls) << 8)) + 0x8) /* cluster power down param */
#define PMC_CLUSTER_PWR_STATUS(cls)   	((0x1000 + ((cls) << 8)) + 0xc) /* cluster power status */
#define PMC_CLUSTER_PWRDN_CNT_EN(cls)   ((0x1000 + ((cls) << 8)) + 0x10) /* cluster power down/on counter enable */
#define PMC_CLUSTER_PWRDN_CNT(cls)      ((0x1000 + ((cls) << 8)) + 0x14) /* cluster power down time */
#define PMC_CLUSTER_PWRON_CNT(cls)      ((0x1000 + ((cls) << 8)) + 0x18) /* cluster power on time */
#define PMC_CLUSTER_PWRDN_DONE_CNT(cls) ((0x1000 + ((cls) << 8)) + 0x1c) /* cluster power off time */

#endif /* __PLAT_DEF_H__ */
