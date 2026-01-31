# SPDX-License-Identifier: GPL-2.0-only
################################################################################
#
# r8125 is the Linux device driver released for Realtek 2.5 Gigabit Ethernet
# controllers with PCI-Express interface.
#
# Copyright(c) 2024 Realtek Semiconductor Corp. All rights reserved.
#
# This program is free software; you can redistribute it and/or modify it
# under the terms of the GNU General Public License as published by the Free
# Software Foundation; either version 2 of the License, or (at your option)
# any later version.
#
# This program is distributed in the hope that it will be useful, but WITHOUT
# ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
# FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
# more details.
#
# You should have received a copy of the GNU General Public License along with
# this program; if not, see <http://www.gnu.org/licenses/>.
#
# Author:
# Realtek NIC software team <nicfae@realtek.com>
# No. 2, Innovation Road II, Hsinchu Science Park, Hsinchu 300, Taiwan
#
################################################################################

################################################################################
#  This product is covered by one or more of the following patents:
#  US6,570,884, US6,115,776, and US6,327,625.
################################################################################

# Modify by flippy(flippy@sina.com) 2026-01-31

ENABLE_LIB_SUPPORT = n
ENABLE_RX_PACKET_FRAGMENT = n

obj-$(CONFIG_R8125) := r8125.o
r8125-objs := r8125_n.o rtl_eeprom.o rtltool.o

# 使用 ccflags-y 替代 EXTRA_CFLAGS
ifeq ($(CONFIG_R8125_SOC_LAN), y)
        ccflags-y += -DCONFIG_SOC_LAN
endif

ifeq ($(CONFIG_R8125_REALWOW_SUPPORT), y)
        r8125-objs += r8125_realwow.o
        ccflags-y += -DENABLE_REALWOW_SUPPORT
endif

ifeq ($(CONFIG_R8125_DASH_SUPPORT), y)
        r8125-objs += r8125_dash.o
        ccflags-y += -DENABLE_DASH_SUPPORT
        ifeq ($(CONFIG_R8125_DASH_PRINT_SUPPORT), y)
                r8125-objs += r8125_dash.o
                ccflags-y += -DENABLE_DASH_SUPPORT -DENABLE_DASH_PRINTER_SUPPORT
        endif
endif

ccflags-y += -DCONFIG_R8125_NAPI
ccflags-y += -DCONFIG_R8125_VLAN

ifeq ($(CONFIG_R8125_DOWN_SPEED_100), y)
        ccflags-y += -DCONFIG_DOWN_SPEED_100
endif

ifeq ($(CONFIG_R8125_ASPM), y)
        ccflags-y += -DCONFIG_ASPM
endif

ifeq ($(CONFIG_R8125_WOL_SUPPORT), y)
        ifeq ($(CONFIG_R8125_S5WOL), y)
                ccflags-y += -DENABLE_S5WOL
        endif

        ifeq ($(CONFIG_R8125_S5_KEEP_CURR_MAC), y)
                ccflags-y += -DENABLE_S5_KEEP_CURR_MAC
        endif
else
        ccflags-y += -DDISABLE_PM_SUPPORT
endif

ifeq ($(CONFIG_R8125_EEE), y)
        ccflags-y += -DENABLE_EEE
endif

ifeq ($(CONFIG_R8125_S0_MAGIC_PACKET), y)
        ccflags-y += -DENABLE_S0_MAGIC_PACKET
endif

ifeq ($(CONFIG_R8125_TX_NO_CLOSE), y)
        ccflags-y += -DENABLE_TX_NO_CLOSE
endif

ifeq ($(CONFIG_R8125_MULTI_MSIX_VECTOR), y)
        ifeq ($(CONFIG_R8125_RSS_SUPPORT), y)
                r8125-objs += r8125_rss.o
                ccflags-y += -DENABLE_RSS_SUPPORT
        endif
        ifeq ($(CONFIG_R8125_MULTIPLE_TX_QUEUE), y)
                ccflags-y += -DENABLE_MULTIPLE_TX_QUEUE
        endif
else
        ccflags-y += -DDISABLE_MULTI_MSIX_VECTOR
endif

ifeq ($(CONFIG_R8125_PTP_SUPPORT), y)
        r8125-objs += r8125_ptp.o
        ccflags-y += -DENABLE_PTP_SUPPORT
        ifeq ($(CONFIG_R8125_PTP_MASTER_MODE), y)
                ccflags-y += -DENABLE_PTP_MASTER_MODE
        endif
endif

ifeq ($(ENABLE_LIB_SUPPORT), y)
        r8125-objs += r8125_lib.o
        ccflags-y += -DENABLE_LIB_SUPPORT
endif

ifeq ($(CONFIG_R8125_FIBER_SUPPORT), y)
        r8125-objs += r8125_fiber.o
        ccflags-y += -DENABLE_FIBER_SUPPORT
endif

ifeq ($(CONFIG_R8125_USE_FIRMWARE_FILE), y)
        r8125-objs += r8125_firmware.o
        ccflags-y += -DENABLE_USE_FIRMWARE_FILE
endif

ifeq ($(CONFIG_R8125_DOUBLE_VLAN), y)
        ccflags-y += -DENABLE_DOUBLE_VLAN
endif

ifeq ($(CONFIG_R8125_PAGE_REUSE), y)
        ccflags-y += -DENABLE_PAGE_REUSE
endif

ifeq ($(CONFIG_R8125_GIGA_LITE), y)
        ccflags-y += -DENABLE_GIGA_LITE
endif

ifeq ($(ENABLE_RX_PACKET_FRAGMENT), y)
        ccflags-y += -DENABLE_RX_PACKET_FRAGMENT
endif
