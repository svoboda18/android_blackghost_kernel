#
# Makefile for misc devices that really don't fit anywhere else.
#

MTK_PLATFORM := $(subst ",,$(CONFIG_MTK_PLATFORM))
ARCH_MTK_PROJECT := $(subst ",,$(CONFIG_ARCH_MTK_PROJECT))

MTK_CUSTOM_PATH  := $(srctree)/drivers/misc/mediatek

subdir-ccflags-y += -I$(MTK_CUSTOM_PATH)/lens/main/inc

obj-$(CONFIG_MTK_LENS)    += main_lens.o

obj-$(CONFIG_MTK_LENS_DW9714AF_SUPPORT)      += common/dw9714af/
