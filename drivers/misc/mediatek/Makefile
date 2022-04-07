#  SPDX-License-Identifier: GPL-2.0 */


MTK_PLATFORM := $(subst ",,$(CONFIG_MTK_PLATFORM))
subdir-ccflags-y += -Werror -Wunused-function
subdir-ccflags-y += -I$(srctree)/drivers/misc/mediatek/include
subdir-ccflags-y += -I$(srctree)/drivers/misc/mediatek/include/mt-plat/$(MTK_PLATFORM)/include
subdir-ccflags-y += -I$(srctree)/drivers/misc/mediatek/include/mt-plat/
subdir-ccflags-y += -I$(srctree)/drivers/mmc/host/mediatek/$(MTK_PLATFORM)

obj-$(CONFIG_MTK_MET_CORE) += met_drv/
obj-$(CONFIG_MTK_ACCDET) += accdet/
obj-$(CONFIG_MTK_COMBO) += connectivity/
obj-$(CONFIG_MTK_CONN_MD) += conn_md/
obj-$(CONFIG_MTK_BTIF) += btif/
obj-$(CONFIG_MTK_SERIAL)	+= uart/
obj-$(CONFIG_MTK_PWM)	    += pwm/
ifeq ($(CONFIG_MTK_I2C_EXTENSION),y)
obj-$(CONFIG_MTK_I2C)  += i2c/
endif
obj-$(CONFIG_MTK_EMI_MPU) += emi_mpu/
obj-$(CONFIG_MTK_EMI_BWL) += emi_bwl/
obj-$(CONFIG_MTK_DEVAPC_DRIVER) += devapc/
obj-$(CONFIG_MTK_PMIC_WRAP_HAL) += pmic_wrap/
obj-$(CONFIG_MTK_BOOT) += boot/
obj-$(CONFIG_MTPROF) += mtprof/
obj-$(CONFIG_MTK_IRQ) += irq/
obj-$(CONFIG_MTK_BOOT_REASON) += boot_reason/
obj-$(CONFIG_MTK_CHIP) += chip/
obj-$(CONFIG_MTK_MT6306_GPIO_SUPPORT) += gpio/
obj-$(CONFIG_MTK_GPIO) += gpio/
obj-$(CONFIG_MTK_DRAMC_LEGACY) += dramc/
ifeq ($(subst ",,$(CONFIG_MTK_SENSOR_ARCHITECTURE)),1.0)
obj-y += sensors-1.0/
endif
obj-$(CONFIG_MTK_DEVINFO_DRIVER) += devinfo/
obj-$(CONFIG_MTK_DEVINFO_DRIVER_V2) += devinfo/
obj-$(CONFIG_MTK_GPU_SUPPORT) += gpu/
obj-$(CONFIG_MTK_PMIC)  += power/
obj-$(CONFIG_MMPROFILE) += mmp/
obj-$(CONFIG_MTK_BASE_POWER) += base/
obj-$(CONFIG_MTK_POWER_GS) += power_gs/
obj-$(CONFIG_MTK_FREQ_HOPPING) += freqhopping/
obj-$(CONFIG_MTK_RTC)	+= rtc/
obj-$(CONFIG_MTK_BLOCK_TAG) += blocktag/
obj-$(CONFIG_MTK_IMGSENSOR) += imgsensor/src/
obj-$(CONFIG_MTK_AEE_FEATURE)	+= aee/
obj-$(CONFIG_MEDIATEK_SOLUTION) += sched/
obj-y += cameraisp/
obj-$(CONFIG_MTK_CMDQ) += cmdq/
obj-$(CONFIG_MTK_CMDQ) += mdp/
obj-$(CONFIG_MTK_CMDQ_TAB) += cmdq/
obj-$(CONFIG_MTK_M4U) += m4u/
obj-$(CONFIG_MEDIATEK_SOLUTION) += cmo/
obj-$(CONFIG_MTK_LCM) += lcm/
obj-$(CONFIG_MTK_FB) += video/
obj-$(CONFIG_MTK_SYNC) += sync/
obj-$(CONFIG_MTK_VIBRATOR) += vibrator/
obj-$(CONFIG_PRINTK_MTK) += mtprintk/
obj-$(CONFIG_MTK_LEDS) += leds/
obj-$(CONFIG_MTK_FLASHLIGHT) += flashlight/
obj-$(CONFIG_RT_FLASHLIGHT) += flashlight/richtek/
obj-$(CONFIG_USB_MTK_HDRC) += usb20/
obj-$(CONFIG_MTK_USB2JTAG_SUPPORT) += usb2jtag/
obj-$(CONFIG_MTK_VIDEOCODEC_DRIVER) += videocodec/
obj-$(CONFIG_MTK_JPEG)	+= jpeg/
obj-$(CONFIG_MTK_SMI) += smi/
obj-$(CONFIG_MTK_AUXADC)	+= auxadc/
obj-$(CONFIG_MTK_CCCI_EXT)  +=  dual_ccci/
obj-$(CONFIG_MTK_CCCI_DEVICES)	+=  ccci_util/
obj-$(CONFIG_MTK_NET_CCMNI) +=  ccmni/
ifeq ($(CONFIG_MACH_MT6735M),y)
obj-$(CONFIG_MTK_ECCCI_DRIVER) +=  eccci1/
else
obj-$(CONFIG_MTK_ECCCI_DRIVER) +=  eccci/
endif
obj-$(CONFIG_MTK_V4L2_LENS) += lens/
obj-$(CONFIG_MTK_LENS)      += lens/
obj-$(CONFIG_THERMAL) +=  thermal/
obj-y += performance/
obj-$(CONFIG_MTK_SYSTRACKER)   += systracker/
obj-$(CONFIG_MTK_BTCVSD) += btcvsd/
obj-$(CONFIG_MTK_SECURITY_SW_SUPPORT) += masp/
obj-$(CONFIG_MTK_HANG_DETECT) += monitor_hang/
obj-$(CONFIG_MTK_FINGERPRINT_SUPPORT) += fingerprint/
obj-$(CONFIG_MTK_HARDWAREINFO) += hardwareinfo/
obj-$(CONFIG_NFC_NXP_SUPPORT) += nfc/
