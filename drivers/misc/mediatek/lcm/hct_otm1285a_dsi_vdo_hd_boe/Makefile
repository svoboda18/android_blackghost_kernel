#
# Makefile for misc devices that really don't fit anywhere else.
#

obj-y += hct_otm1285a_dsi_vdo_hd_boe.o

ccflags-y += -Wno-error=unused-variable \
	     -Wno-error=unused-function
ccflags-$(CONFIG_MTK_LCM) += -I$(srctree)/drivers/misc/mediatek/lcm/inc

