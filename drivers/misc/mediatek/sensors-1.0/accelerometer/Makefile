# In case the platform does NOT support this type of sensors
ccflags-y += -I$(srctree)/drivers/misc/mediatek/sensors-1.0/hwmon/include
ccflags-y += -I$(srctree)/drivers/misc/mediatek/sensors-1.0/include

obj-y   +=  accel.o accel_factory.o

obj-$(CONFIG_MTK_ACCELHUB) += accelhub/
obj-$(CONFIG_MTK_MC3410_I2C) += mc3410-i2c/
obj-$(CONFIG_MTK_BMA222E) += bma222e/
obj-$(CONFIG_MTK_MXC400X_NEW)   +=  mxc400x-new/
obj-$(CONFIG_MTK_STK8BAXX_NEW)	+= stk8baxx-new/
obj-$(CONFIG_MTK_MIR3DA)	+= mir3da/
obj-$(CONFIG_MTK_MXC6655)	+= mxc6655/