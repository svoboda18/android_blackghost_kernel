# In case the platform does NOT support this type of sensors
ccflags-y += -I$(srctree)/drivers/misc/mediatek/sensors-1.0/hwmon/include
obj-y   +=  alsps.o
obj-y   +=  alsps_factory.o
obj-y   +=  aal_control.o

obj-$(CONFIG_MTK_ALSPSHUB)      +=  alspshub/
obj-$(CONFIG_MTK_CM36652)	+=  cm36652/
obj-$(CONFIG_MTK_EPL2182_NEW)	+=  EPL2182-new/
