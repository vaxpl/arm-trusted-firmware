#
# Copyright (c) 2015, ARM Limited and Contributors. All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# Redistributions of source code must retain the above copyright notice, this
# list of conditions and the following disclaimer.
#
# Redistributions in binary form must reproduce the above copyright notice,
# this list of conditions and the following disclaimer in the documentation
# and/or other materials provided with the distribution.
#
# Neither the name of ARM nor the names of its contributors may be used
# to endorse or promote products derived from this software without specific
# prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#

HISI_PLAT		:=	plat/hisilicon
HISI_PLAT_SOC		:=	${HISI_PLAT}/${PLAT}

PLAT_INCLUDES		:=	-I${HISI_PLAT}/				\
				-I${HISI_PLAT_SOC}/				\
				-I${HISI_PLAT_SOC}/include/

HISI_GIC_SOURCES	:= 	drivers/arm/gic/common/gic_common.c		\
				drivers/arm/gic/v2/gicv2_main.c			\
				drivers/arm/gic/v2/gicv2_helpers.c		\
				plat/common/plat_gicv2.c			\
				$(HISI_PLAT)/common/hisilicon_gicv2.c

PLAT_BL_COMMON_SOURCES	:=	lib/aarch64/xlat_tables.c			\
				plat/common/aarch64/plat_common.c		\
				plat/common/aarch64/plat_psci_common.c

BL31_SOURCES		+=	$(HISI_GIC_SOURCES)				\
				drivers/arm/pl011/pl011_console.S		\
				drivers/delay_timer/delay_timer.c		\
				lib/cpus/aarch64/aem_generic.S			\
				lib/cpus/aarch64/cortex_a53.S			\
				lib/cpus/aarch64/cortex_a73.S			\
				plat/common/aarch64/platform_mp_stack.S		\
				${HISI_PLAT_SOC}/aarch64/plat_helpers.S		\
				${HISI_PLAT_SOC}/aarch64/platform_common.c	\
				${HISI_PLAT_SOC}/drivers/pmc/pmc.c		\
				${HISI_PLAT_SOC}/bl31_plat_setup.c		\
				${HISI_PLAT_SOC}/plat_pm.c			\
				${HISI_PLAT_SOC}/plat_topology.c		\
				${HISI_PLAT_SOC}/plat_delay_timer.c			
			

ENABLE_PLAT_COMPAT      := 0
CTX_INCLUDE_FPREGS	:= 1
NEED_BL33		:= yes
