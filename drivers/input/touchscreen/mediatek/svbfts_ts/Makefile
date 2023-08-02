# Linux driver folder
ccflags-y += -I$(srctree)/drivers/input/touchscreen/mediatek/
ccflags-y += -I$(srctree)/drivers/input/keyboard/mediatek/
ccflags-y += -I$(srctree)/drivers/misc/mediatek/include/mt-plat/
ccflags-y += -I$(srctree)/drivers/misc/mediatek/include/mt-plat/$(MTK_PLATFORM)/include/
ccflags-y += -I$(srctree)/drivers/base
ccflags-y += -I$(srctree)/drivers/misc/mediatek/hwmon/include

obj-$(CONFIG_TOUCHSCREEN_MTK_SVB_FOCALTECH_TS)    +=  svbfocaltech_gesture.o
obj-$(CONFIG_TOUCHSCREEN_MTK_SVB_FOCALTECH_TS)    +=  svbfocaltech_gesture.a
obj-$(CONFIG_TOUCHSCREEN_MTK_SVB_FOCALTECH_TS)    +=  svbfocaltech_core.o
obj-$(CONFIG_TOUCHSCREEN_MTK_SVB_FOCALTECH_TS)    +=  svbfocaltech_i2c.o
