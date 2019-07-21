# In case the platform does NOT support this type of sensors
ccflags-y += -I$(srctree)/drivers/misc/mediatek/sensors-1.0/hwmon/include
ccflags-y += -I$(srctree)/drivers/misc/mediatek/sensors-1.0/include

obj-y   +=  accel.o accel_factory.o

obj-$(CONFIG_MTK_MXC400X)   +=  mxc400x/
obj-$(CONFIG_MTK_KXTJ2_1009_NEW)   +=  KXTJ2_1009-new/
