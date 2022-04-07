/* this file function is display all devices name */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/types.h>
#include <linux/wait.h>
#include <linux/slab.h>
#include <linux/fs.h>
#include <linux/sched.h>
#include <linux/poll.h>
#include <linux/device.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/cdev.h>
#include <linux/errno.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/nand.h>
#include <asm/io.h>
#include <asm/uaccess.h>
#include <linux/proc_fs.h>

#include "../../../misc/mediatek/lcm/inc/lcm_drv.h"
#include "../../../input/touchscreen/mediatek/tpd.h"


#define HARDWARE_INFO_VERSION	0x02




extern struct LCM_DRIVER  *lcm_drv_name; 

extern char *main_camera;
extern char *sub_camera;
#if defined(CONFIG_CUSTOM_KERNEL_ALSPS) || defined(CONFIG_CUSTOM_KERNEL_PS)
extern char *alsps_name;
#endif

#if defined(CONFIG_CUSTOM_KERNEL_ACCELEROMETER)
extern char *gsensor_name;
#endif

#if defined(CONFIG_CUSTOM_KERNEL_MAGNETOMETER)
extern char *msensor_name;
#endif

#if defined(CONFIG_CUSTOM_KERNEL_GYROSCOPE)
extern char *gyro_name;
#endif
#if defined(CONFIG_MTK_FINGERPRINT_SUPPORT)
char *fingerprint_name_info = NULL;
#endif
extern struct tpd_device  *tpd;
extern struct tpd_driver_t *g_tpd_drv;

int g_ctp_fwvr_info = 0; 
int g_ctp_vendor_info = 0;



/****************************************************************************** 
 * Function Configuration
******************************************************************************/


/****************************************************************************** 
 * Debug configuration
******************************************************************************/

static ssize_t show_version(struct device *dev,struct device_attribute *attr, char *buf)
{
    int ret_value = 1;
	
    ret_value = sprintf(buf, "Version     : 0x%x\n", HARDWARE_INFO_VERSION); 	

    return ret_value;
}

static ssize_t show_lcm(struct device *dev,struct device_attribute *attr, char *buf)
{
    int ret_value = 1;

    if(lcm_drv_name->name)
    	ret_value = sprintf(buf, "lcd  name   : %s\n", lcm_drv_name->name); 	
    else
    	ret_value = sprintf(buf, "lcd not found!!!\n");	

    return ret_value;
}

static ssize_t show_ctp(struct device *dev,struct device_attribute *attr, char *buf)
{
    int count = 0;
    int ret_value = 1;

    if(g_tpd_drv->tpd_device_name)
	{
    	count += sprintf(buf + count, "ctp  driver : %s\n", g_tpd_drv->tpd_device_name);
		count += sprintf(buf + count, "ctp  vendor : 0x%x fw: %d\n", g_ctp_vendor_info, g_ctp_fwvr_info);
	}
    else
	{
    	count += sprintf(buf + count, "ctp id read fail\n");
	}
	
	ret_value = count;
	
    return ret_value;
}

static ssize_t show_main_camera(struct device *dev,struct device_attribute *attr, char *buf)
{
    int ret_value = 1;

    if(main_camera)
    	ret_value = sprintf(buf, "main camera : %s\n", main_camera);
    else
        ret_value = sprintf(buf, "main camera : can not  get camera id\n");
	
    return ret_value;
}

static ssize_t show_sub_camera(struct device *dev,struct device_attribute *attr, char *buf)
{
    int ret_value = 1;

    if(sub_camera)
    	ret_value = sprintf(buf, "sub  camera : %s\n", sub_camera);
    else
        ret_value = sprintf(buf, "sub  camera : can not  get camera id\n");
	
    return ret_value;
}
#if defined(CONFIG_CUSTOM_KERNEL_ALSPS) || defined(CONFIG_CUSTOM_KERNEL_PS)
static ssize_t show_alsps(struct device *dev,struct device_attribute *attr, char *buf)
{
    int ret_value = 1;
	
    if(alsps_name)
        ret_value = sprintf(buf, "AlSPS name  : %s\n", alsps_name);	
    else
        ret_value = sprintf(buf, "AlSPS not found!!!\n");    

    return ret_value;
}
#endif
#if defined(CONFIG_CUSTOM_KERNEL_ACCELEROMETER)
static ssize_t show_gsensor(struct device *dev,struct device_attribute *attr, char *buf)
{
    int ret_value = 1;

    if(gsensor_name)
        ret_value = sprintf(buf, "GSensor name: %s\n", gsensor_name); 	
    else
        ret_value = sprintf(buf, "GSensor not found!!!\n");    
   
    return ret_value;
}
#endif
#if defined(CONFIG_CUSTOM_KERNEL_MAGNETOMETER)
static ssize_t show_msensor(struct device *dev,struct device_attribute *attr, char *buf)
{
    int ret_value = 1;
    
    if(msensor_name)
        ret_value = sprintf(buf, "MSensor name: %s\n", msensor_name);
    else
        ret_value = sprintf(buf, "MSensor not found!!!\n");
		
    return ret_value;
}
#endif
#if defined(CONFIG_CUSTOM_KERNEL_GYROSCOPE)
static ssize_t show_gyro(struct device *dev,struct device_attribute *attr, char *buf)
{
    int ret_value = 1;

    if(gyro_name)
        ret_value = sprintf(buf, "Gyro  name  : %s\n", gyro_name);
    else
        ret_value = sprintf(buf, "Gyro not found!!!\n");

    return ret_value;
}
#endif
#if 0
static ssize_t show_wifi(struct device *dev,struct device_attribute *attr, char *buf)
{
    int ret_value = 1;
	
    ret_value = sprintf(buf, "WiFi  name  : MT6625L\n");
	
    return ret_value;
}

static ssize_t show_bt(struct device *dev,struct device_attribute *attr, char *buf)
{
    int ret_value = 1;
	
    ret_value = sprintf(buf, "BT    name  : MT6625L\n");
	
    return ret_value;
}

static ssize_t show_gps(struct device *dev,struct device_attribute *attr, char *buf)
{
    int ret_value = 1;
	
    ret_value = sprintf(buf, "GPS   name  : MT6625L\n");
	
    return ret_value;
}

static ssize_t show_fm(struct device *dev,struct device_attribute *attr, char *buf)
{
    int ret_value = 1;
	
    ret_value = sprintf(buf, "FM    name  : MT6625L\n");
	
    return ret_value;
}
#endif
#if defined(CONFIG_MTK_FINGERPRINT_SUPPORT)
static ssize_t show_fingerprint(struct device *dev,struct device_attribute *attr, char *buf)
{
    int ret_value = 1;
    
	if(fingerprint_name_info)
	{
		ret_value = sprintf(buf, "fingerprint : %s\n",fingerprint_name_info);
	}	
	else	
		ret_value = sprintf(buf, "fingerprint name:Not found\n");		
	
	return ret_value;
}
#endif
static DEVICE_ATTR(99_version, 0444, show_version, NULL);
static DEVICE_ATTR(00_lcm, 0444, show_lcm, NULL);
static DEVICE_ATTR(01_ctp, 0444, show_ctp, NULL);
static DEVICE_ATTR(02_main_camera, 0444, show_main_camera, NULL);
static DEVICE_ATTR(03_sub_camera, 0444, show_sub_camera, NULL);
#if defined(CONFIG_CUSTOM_KERNEL_ACCELEROMETER)
static DEVICE_ATTR(05_gsensor, 0444, show_gsensor, NULL);
#endif
#if defined(CONFIG_CUSTOM_KERNEL_MAGNETOMETER)
static DEVICE_ATTR(06_msensor, 0444, show_msensor, NULL);
#endif
#if defined(CONFIG_CUSTOM_KERNEL_ALSPS) || defined(CONFIG_CUSTOM_KERNEL_PS)
static DEVICE_ATTR(07_alsps, 0444, show_alsps, NULL);
#endif
#if defined(CONFIG_CUSTOM_KERNEL_GYROSCOPE)
static DEVICE_ATTR(08_gyro, 0444, show_gyro, NULL);
#endif
#if 0	
static DEVICE_ATTR(09_wifi, 0444, show_wifi, NULL);
static DEVICE_ATTR(10_bt, 0444, show_bt, NULL);
static DEVICE_ATTR(11_gps, 0444, show_gps, NULL);
static DEVICE_ATTR(12_fm, 0444, show_fm, NULL);
#endif
#if defined(CONFIG_MTK_FINGERPRINT_SUPPORT)
static DEVICE_ATTR(13_fingerprint, 0444, show_fingerprint, NULL);
#endif

///////////////////////////////////////////////////////////////////////////////////////////
//// platform_driver API 
///////////////////////////////////////////////////////////////////////////////////////////
static int HardwareInfo_driver_probe(struct platform_device *dev)	
{	
	int ret_device_file = 0;

    printk("*** HardwareInfo_driver_probe!!! ***\n" );
    if((ret_device_file = device_create_file(&(dev->dev), &dev_attr_00_lcm)) != 0) goto exit_error;
    if((ret_device_file = device_create_file(&(dev->dev), &dev_attr_01_ctp)) != 0) goto exit_error;
    if((ret_device_file = device_create_file(&(dev->dev), &dev_attr_02_main_camera)) != 0) goto exit_error;
    if((ret_device_file = device_create_file(&(dev->dev), &dev_attr_03_sub_camera)) != 0) goto exit_error;
#if defined(CONFIG_CUSTOM_KERNEL_ACCELEROMETER)	
    if((ret_device_file = device_create_file(&(dev->dev), &dev_attr_05_gsensor)) != 0) goto exit_error;
#endif	
#if defined(CONFIG_CUSTOM_KERNEL_MAGNETOMETER)	
    if((ret_device_file = device_create_file(&(dev->dev), &dev_attr_06_msensor)) != 0) goto exit_error;
#endif
#if defined(CONFIG_CUSTOM_KERNEL_ALSPS) || defined(CONFIG_CUSTOM_KERNEL_PS)
	if((ret_device_file = device_create_file(&(dev->dev), &dev_attr_07_alsps)) != 0) goto exit_error;
#endif	
#if defined(CONFIG_CUSTOM_KERNEL_GYROSCOPE)	
    if((ret_device_file = device_create_file(&(dev->dev), &dev_attr_08_gyro)) != 0) goto exit_error;
#endif
#if 0	
    if((ret_device_file = device_create_file(&(dev->dev), &dev_attr_09_wifi)) != 0) goto exit_error;
    if((ret_device_file = device_create_file(&(dev->dev), &dev_attr_10_bt)) != 0) goto exit_error;
    if((ret_device_file = device_create_file(&(dev->dev), &dev_attr_11_gps)) != 0) goto exit_error;
    if((ret_device_file = device_create_file(&(dev->dev), &dev_attr_12_fm)) != 0) goto exit_error;
#endif
#if defined(CONFIG_MTK_FINGERPRINT_SUPPORT)	
    if((ret_device_file = device_create_file(&(dev->dev), &dev_attr_13_fingerprint)) != 0) goto exit_error;
#endif
	if((ret_device_file = device_create_file(&(dev->dev), &dev_attr_99_version)) != 0) goto exit_error;   
    

exit_error:	

    return ret_device_file;
}

static int HardwareInfo_driver_remove(struct platform_device *dev)
{
	printk("*** HardwareInfo_drvier_remove!!! ***");

    device_remove_file(&(dev->dev), &dev_attr_00_lcm);
    device_remove_file(&(dev->dev), &dev_attr_01_ctp);
    device_remove_file(&(dev->dev), &dev_attr_02_main_camera);
    device_remove_file(&(dev->dev), &dev_attr_03_sub_camera);
#if defined(CONFIG_CUSTOM_KERNEL_ACCELEROMETER)	
    device_remove_file(&(dev->dev), &dev_attr_05_gsensor);
#endif	
#if defined(CONFIG_CUSTOM_KERNEL_MAGNETOMETER)	
	device_remove_file(&(dev->dev), &dev_attr_06_msensor);
#endif
#if defined(CONFIG_CUSTOM_KERNEL_ALSPS) || defined(CONFIG_CUSTOM_KERNEL_PS)	
	device_remove_file(&(dev->dev), &dev_attr_07_alsps);
#endif	
#if defined(CONFIG_CUSTOM_KERNEL_GYROSCOPE)	
    device_remove_file(&(dev->dev), &dev_attr_08_gyro);
#endif
#if 0	
    device_remove_file(&(dev->dev), &dev_attr_09_wifi);
    device_remove_file(&(dev->dev), &dev_attr_10_bt);
    device_remove_file(&(dev->dev), &dev_attr_11_gps);
    device_remove_file(&(dev->dev), &dev_attr_12_fm);
#endif	
#if defined(CONFIG_MTK_FINGERPRINT_SUPPORT)	
    device_remove_file(&(dev->dev), &dev_attr_13_fingerprint);
#endif
    device_remove_file(&(dev->dev), &dev_attr_99_version);
	
    return 0;
}



static struct platform_driver HardwareInfo_driver = {
    .probe	= HardwareInfo_driver_probe,
    .remove = HardwareInfo_driver_remove,
    .driver = {
        .name = "HardwareInfo",
    },
};

static struct platform_device HardwareInfo_device = {
    .name   = "HardwareInfo",
    .id	    = -1,
};



static int __init HardwareInfo_mod_init(void)
{
    int ret = 0;

    ret = platform_device_register(&HardwareInfo_device);
    if (ret) {
        printk("*** HardwareInfo_mod_init  Unable to driver register(%d)!!!\n", ret);
        goto fail_2;
    } 

    ret = platform_driver_register(&HardwareInfo_driver);
    if (ret) {
        printk("*** HardwareInfo_mod_init  Unable to driver register(%d)!!!\n", ret);
        goto fail_1;
    }

    goto ok_result;

    
fail_1:
	platform_driver_unregister(&HardwareInfo_driver);
	
fail_2:
	platform_device_unregister(&HardwareInfo_device);
	
ok_result:

    return ret;
}


static void __exit HardwareInfo_mod_exit(void)
{
    platform_driver_unregister(&HardwareInfo_driver);
	platform_device_unregister(&HardwareInfo_device);
}

/*****************************************************************************/
module_init(HardwareInfo_mod_init);
module_exit(HardwareInfo_mod_exit);
/*****************************************************************************/

MODULE_AUTHOR("<hello@world.com>");
MODULE_DESCRIPTION("MTK Hareware Info driver");
MODULE_LICENSE("GPL");



