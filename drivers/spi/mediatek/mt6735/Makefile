subdir-ccflags-y += -I$(srctree)/drivers/misc/mediatek/include/mt-plat/$(MTK_PLATFORM)/include \
					-I$(srctree)/drivers/misc/mediatek/include/mt-plat

ifeq ($(CONFIG_TRUSTKERNEL_TEE_SUPPORT),y)
 subdir-ccflags-y += -I$(srctree)/drivers/tee/tkcore/include/linux
endif

obj-$(CONFIG_MTK_SPI) += spi.o spi-dev.o
