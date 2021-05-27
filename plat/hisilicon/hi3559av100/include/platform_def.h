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

#ifndef __PLATFORM_DEF_H__
#define __PLATFORM_DEF_H__

#include <common_def.h>
#include <arch.h>
#include <hisilicon_def.h>

#define DEBUG_XLAT_TABLE 0

/*******************************************************************************
 * Platform binary types for linking
 ******************************************************************************/
#define PLATFORM_LINKER_FORMAT          "elf64-littleaarch64"
#define PLATFORM_LINKER_ARCH            aarch64

/* Size of cacheable stacks, May need fix */
#if DEBUG_XLAT_TABLE
#define PLATFORM_STACK_SIZE 0x800
#elif IMAGE_BL1
#define PLATFORM_STACK_SIZE 0x440
#elif IMAGE_BL2
#define PLATFORM_STACK_SIZE 0x400
#elif IMAGE_BL31
#define PLATFORM_STACK_SIZE 0x800
#elif IMAGE_BL32
#define PLATFORM_STACK_SIZE 0x440
#endif

/*******************************************************************************
 * Declarations and constants to access the mailboxes safely. Each mailbox is
 * aligned on the biggest cache line size in the platform. This is known only
 * to the platform as it might have a combination of integrated and external
 * caches. Such alignment ensures that two maiboxes do not sit on the same cache
 * line at any cache level. They could belong to different cpus/clusters &
 * get written while being protected by different locks causing corruption of
 * a valid mailbox address.
 ******************************************************************************/
#define CACHE_WRITEBACK_SHIFT   6
#define CACHE_WRITEBACK_GRANULE (1 << CACHE_WRITEBACK_SHIFT)

#define FIRMWARE_WELCOME_STR            "Booting Trusted Firmware\n"

#define PLATFORM_CLUSTER_COUNT		3
#define PLATFORM_MAX_CPUS_PER_CLUSTER	2

#define PLAT_MAX_PWR_LVL                MPIDR_AFFLVL1
#define PLATFORM_CORE_COUNT             (PLATFORM_CLUSTER_COUNT * \
                                         PLATFORM_MAX_CPUS_PER_CLUSTER)
#define PLAT_NUM_PWR_DOMAINS            (PLATFORM_CORE_COUNT + \
                                         PLATFORM_CLUSTER_COUNT + 1)
/*******************************************************************************
 * Platform power states
 ******************************************************************************/
#define PLAT_MAX_OFF_STATE	2
#define PLAT_MAX_RET_STATE	1

/*******************************************************************************
 * BL31 specific defines.
 ******************************************************************************/
#define BL31_SIZE                       0x20000

//
//Modify this configuration according to the system framework
#define BL31_BASE (0x47000000)

#define BL31_LIMIT                      (BL31_BASE + BL31_SIZE - 1)

#define ADDR_SPACE_SIZE         (1ull << 32)
#define MAX_XLAT_TABLES         4
#define MAX_MMAP_REGIONS        16


/*
 * Define GICD and GICC and GICR base
 */
#define PLAT_HISI_GICD_BASE	HISILICON_GICD_BASE
#define PLAT_HISI_GICC_BASE	HISILICON_GICC_BASE

/*
 * Define a list of Group 1 Secure and Group 0 interrupts as per GICv3
 * terminology. On a GICv2 system or mode, the lists will be merged and treated
 * as Group 0 interrupts.
 */
#define PLAT_HISI_G1S_IRQS	HISI_G1S_IRQS

#define PLAT_HISI_UART_BASE	HISILICON_UART0_BASE
#define PLAT_HISI_UART_CLOCK	HISILICON_UART_CLOCK
#define PLAT_HISI_UART_BAUDRATE	HISILICON_BAUDRATE

#define PLAT_HISI_PRIMARY_CPU	0x0

#define REG_SYS_CNT_BASE                (0x1c000000 + 0x01800000) /* 0x1D800000 */

#endif /* __PLATFORM_DEF_H__ */
