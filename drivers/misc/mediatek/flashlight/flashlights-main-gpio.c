/*
 * Copyright (C) 2015 MediaTek Inc.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 */

// #define pr_fmt(fmt) KBUILD_MODNAME ": %s: " fmt, __func__
#define LOG_INFO(fmt,arg...)    printk("FL_MAIN [%s:%d] "fmt"\n", __func__, __LINE__, ##arg)

#include <linux/types.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/hrtimer.h>
#include <linux/ktime.h>
#include <linux/workqueue.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/list.h>
#include <linux/delay.h>
#include <linux/pinctrl/consumer.h>

#include "flashlight-core.h"
#include "flashlight-dt.h"

#include <mt-plat/upmu_common.h>
/* define device tree */
/* TODO: modify temp device tree name */
#ifndef FLMAIN_DTNAME
#define FLMAIN_DTNAME "mediatek,flashlights_main_gpio"
#endif

/* TODO: define driver name */
#define FLMAIN_NAME "flashlights-main-gpio"

/* define registers */
/* TODO: define register */


static int g_duty = -1;

/* define mutex and work queue */
static DEFINE_MUTEX(flmain_mutex);
static struct work_struct flmain_work;

/* define pinctrl */
/* TODO: define pinctrl */
#define FLMAIN_PINCTRL_PIN_GPIO 0
#define FLMAIN_PINCTRL_PIN_TORCH_GPIO 1
#define FLMAIN_PINCTRL_PINSTATE_LOW 0
#define FLMAIN_PINCTRL_PINSTATE_HIGH 1
#define FLMAIN_PINCTRL_STATE_GPIO_HIGH "pin_fl_en1"
#define FLMAIN_PINCTRL_STATE_GPIO_LOW  "pin_fl_en0"
#define FLMAIN_PINCTRL_STATE_GPIO_TORCH_HIGH "pin_fl_sb1"
#define FLMAIN_PINCTRL_STATE_GPIO_TORCH_LOW  "pin_fl_sb0"
static struct pinctrl *flmain_pinctrl;
static struct pinctrl_state *flmain_gpio_high;
static struct pinctrl_state *flmain_gpio_low;
static struct pinctrl_state *gpio_torch_high;
static struct pinctrl_state *gpio_torch_low;

/* define usage count */
static int use_count;

/* platform data */
struct flmain_platform_data {
	int channel_num;
	struct flashlight_device_id *dev_id;
};


/******************************************************************************
 * Pinctrl configuration
 *****************************************************************************/
static int flmain_pinctrl_init(struct platform_device *pdev)
{
	int ret = 0;

	LOG_INFO("flmain pinctrl init start.\n");

	/* get pinctrl */
	flmain_pinctrl = devm_pinctrl_get(&pdev->dev);
	if (IS_ERR(flmain_pinctrl)) {
		LOG_INFO("Failed to get flashlight pinctrl.\n");
		ret = PTR_ERR(flmain_pinctrl);
	}

	/* TODO: Flashlight GPIO pin initialization */
	flmain_gpio_high = pinctrl_lookup_state(flmain_pinctrl, FLMAIN_PINCTRL_STATE_GPIO_HIGH);
	if (IS_ERR(flmain_gpio_high)) {
		LOG_INFO("Failed to init (%s)\n", FLMAIN_PINCTRL_STATE_GPIO_HIGH);
		ret = PTR_ERR(flmain_gpio_high);
	}

	flmain_gpio_low = pinctrl_lookup_state(flmain_pinctrl, FLMAIN_PINCTRL_STATE_GPIO_LOW);
	if (IS_ERR(flmain_gpio_low)) {
		LOG_INFO("Failed to init (%s)\n", FLMAIN_PINCTRL_STATE_GPIO_LOW);
		ret = PTR_ERR(flmain_gpio_low);
	}

	gpio_torch_high = pinctrl_lookup_state(flmain_pinctrl, FLMAIN_PINCTRL_STATE_GPIO_TORCH_HIGH);
	if (IS_ERR(gpio_torch_high)) {
		LOG_INFO("Failed to init (%s)\n", FLMAIN_PINCTRL_STATE_GPIO_TORCH_HIGH);
		ret = PTR_ERR(gpio_torch_high);
	}

	gpio_torch_low = pinctrl_lookup_state(flmain_pinctrl, FLMAIN_PINCTRL_STATE_GPIO_TORCH_LOW);
	if (IS_ERR(gpio_torch_low)) {
		LOG_INFO("Failed to init (%s)\n", FLMAIN_PINCTRL_STATE_GPIO_TORCH_LOW);
		ret = PTR_ERR(gpio_torch_low);
	}

	LOG_INFO("flmain pinctrl init end.\n");

	return ret;
}

static int flmain_pinctrl_set(int pin, int state)
{
	int ret = 0;

	if (IS_ERR(flmain_pinctrl)) {
		LOG_INFO("pinctrl is not available\n");
		return -1;
	}

	switch (pin) {
	case FLMAIN_PINCTRL_PIN_TORCH_GPIO:
		if (IS_ERR(gpio_torch_low) || IS_ERR(gpio_torch_high)) {
			LOG_INFO("pinctrl state is error! pin(%d) state(%d)\n", pin, state);
			break;
		}
		LOG_INFO("set torch gpio, pin(%d) state(%d)\n", pin, state);
		if (state == FLMAIN_PINCTRL_PINSTATE_HIGH)
			pinctrl_select_state(flmain_pinctrl, gpio_torch_high);
		else
			pinctrl_select_state(flmain_pinctrl, gpio_torch_low);
		break;
	case FLMAIN_PINCTRL_PIN_GPIO:
		LOG_INFO("set enable gpio, pin(%d) state(%d)\n", pin, state);
		if (state == FLMAIN_PINCTRL_PINSTATE_LOW && !IS_ERR(flmain_gpio_low))
			pinctrl_select_state(flmain_pinctrl, flmain_gpio_low);
		else if (state == FLMAIN_PINCTRL_PINSTATE_HIGH && !IS_ERR(flmain_gpio_high))
			pinctrl_select_state(flmain_pinctrl, flmain_gpio_high);
		else
			LOG_INFO("set err, pin(%d) state(%d)\n", pin, state);
		break;
	default:
		LOG_INFO("set err, pin(%d) state(%d)\n", pin, state);
		break;
	}
	LOG_INFO("pin(%d) state(%d)\n", pin, state);

	return ret;
}


/******************************************************************************
 * flmain operations
 *****************************************************************************/
/* flashlight enable function */
static int flmain_enable(void)
{
	printk("flmain_enable g_duty = %d\n",g_duty);
	if (g_duty < 0)
		g_duty = 0;
	
	if (g_duty > 1)
		g_duty = 1;
	
	if (g_duty < 1) {
		#ifndef CONFIG_MTK_ISINK_FLASHLIGHT
		flmain_pinctrl_set(FLMAIN_PINCTRL_PIN_TORCH_GPIO, 0);
		flmain_pinctrl_set(FLMAIN_PINCTRL_PIN_GPIO, 1);
		#else
		// ISINK1
		pmic_set_register_value(PMIC_RG_DRV_32K_CK_PDN,0x0); // Disable power down  
		pmic_set_register_value(PMIC_RG_ISINK0_CK_PDN,0);
		pmic_set_register_value(PMIC_RG_ISINK0_CK_SEL,0);
		pmic_set_register_value(PMIC_ISINK_CH0_MODE,0);
		pmic_set_register_value(PMIC_ISINK_CH0_STEP,3);//16mA
		pmic_set_register_value(PMIC_ISINK_DIM0_DUTY,15);
		pmic_set_register_value(PMIC_ISINK_DIM0_FSEL,0);//1KHz
		pmic_set_register_value(PMIC_ISINK_CH0_EN,1);

		pmic_set_register_value(PMIC_RG_DRV_32K_CK_PDN,0x0); // Disable power down 
		pmic_set_register_value(PMIC_RG_ISINK1_CK_PDN, 0x0);	/* Disable power down */
		pmic_set_register_value(PMIC_RG_ISINK1_CK_SEL, 0x1);	/* Freq = 1Mhz for Backlight */
		pmic_set_register_value(PMIC_ISINK_CH1_MODE, 0x0);
		pmic_set_register_value(PMIC_ISINK_CH1_STEP, 0x3);	/* 24mA */
		pmic_set_register_value(PMIC_ISINK_DIM1_DUTY,15);
		pmic_set_register_value(PMIC_ISINK_DIM1_FSEL,0);//1KHz
		pmic_set_register_value(PMIC_ISINK_CH1_EN,1);				
		#endif
	} else {
		#ifndef CONFIG_MTK_ISINK_FLASHLIGHT
		flmain_pinctrl_set(FLMAIN_PINCTRL_PIN_TORCH_GPIO, 1);
		flmain_pinctrl_set(FLMAIN_PINCTRL_PIN_GPIO, 1);
		#else
		// ISINK1
		pmic_set_register_value(PMIC_RG_DRV_32K_CK_PDN,0x0); // Disable power down  
		pmic_set_register_value(PMIC_RG_ISINK0_CK_PDN,0);
		pmic_set_register_value(PMIC_RG_ISINK0_CK_SEL,0);
		pmic_set_register_value(PMIC_ISINK_CH0_MODE,0);
		pmic_set_register_value(PMIC_ISINK_CH0_STEP,3);//16mA
		pmic_set_register_value(PMIC_ISINK_DIM0_DUTY,15);
		pmic_set_register_value(PMIC_ISINK_DIM0_FSEL,0);//1KHz
		pmic_set_register_value(PMIC_ISINK_CH0_EN,1);  	

		pmic_set_register_value(PMIC_RG_DRV_32K_CK_PDN,0x0); // Disable power down 
		pmic_set_register_value(PMIC_RG_ISINK1_CK_PDN, 0x0);	/* Disable power down */
		pmic_set_register_value(PMIC_RG_ISINK1_CK_SEL, 0x1);	/* Freq = 1Mhz for Backlight */
		pmic_set_register_value(PMIC_ISINK_CH1_MODE, 0x0);
		pmic_set_register_value(PMIC_ISINK_CH1_STEP, 0x3);	/* 24mA */
		pmic_set_register_value(PMIC_ISINK_DIM1_DUTY,15);
		pmic_set_register_value(PMIC_ISINK_DIM1_FSEL,0);//1KHz
		pmic_set_register_value(PMIC_ISINK_CH1_EN,1);		
		#endif
	}

	return 0;
}

/* flashlight disable function */
static int flmain_disable(void)
{
	#ifndef CONFIG_MTK_ISINK_FLASHLIGHT
	flmain_pinctrl_set(FLMAIN_PINCTRL_PIN_TORCH_GPIO, 0);
	flmain_pinctrl_set(FLMAIN_PINCTRL_PIN_GPIO, 0);
	#else
		pmic_set_register_value(PMIC_ISINK_CH0_EN,0); // Enable CHOP clk
	#endif

	return 0;
}

/* set flashlight level */
static int flmain_set_level(int level)
{
	g_duty = level;

	return 0;
}

/* flashlight init */
static int flmain_init(void)
{
#ifndef CONFIG_MTK_ISINK_FLASHLIGHT
	int pin = 0, state = 0;

	/* TODO: wrap init function */

	return flmain_pinctrl_set(pin, state);
#else
	pmic_set_register_value(PMIC_RG_DRV_32K_CK_PDN,0x0); // Disable power down  
	pmic_set_register_value(PMIC_RG_ISINK0_CK_PDN,0);
	pmic_set_register_value(PMIC_RG_ISINK0_CK_SEL,0);
	pmic_set_register_value(PMIC_ISINK_CH0_MODE,0);
	pmic_set_register_value(PMIC_ISINK_CH0_STEP,3);//16mA
	pmic_set_register_value(PMIC_ISINK_DIM0_DUTY,15);
	pmic_set_register_value(PMIC_ISINK_DIM0_FSEL,0);//1KHz
	pmic_set_register_value(PMIC_ISINK_CH0_EN,0);	

	pmic_set_register_value(PMIC_RG_DRV_32K_CK_PDN,0x0); // Disable power down 
	pmic_set_register_value(PMIC_RG_ISINK1_CK_PDN, 0x0);	/* Disable power down */
	pmic_set_register_value(PMIC_RG_ISINK1_CK_SEL, 0x1);	/* Freq = 1Mhz for Backlight */
	pmic_set_register_value(PMIC_ISINK_CH1_MODE, 0x0);
	pmic_set_register_value(PMIC_ISINK_CH1_STEP, 0x3);	/* 24mA */
	pmic_set_register_value(PMIC_ISINK_DIM1_DUTY,15);
	pmic_set_register_value(PMIC_ISINK_DIM1_FSEL,0);//1KHz
	pmic_set_register_value(PMIC_ISINK_CH1_EN,0);	
#endif

	return 0;
}

/* flashlight uninit */
static int flmain_uninit(void)
{
	int pin = 0, state = 0;

	/* TODO: wrap uninit function */

	return flmain_pinctrl_set(pin, state);
}

/******************************************************************************
 * Timer and work queue
 *****************************************************************************/
static struct hrtimer flmain_timer;
static unsigned int flmain_timeout_ms;

static void flmain_work_disable(struct work_struct *data)
{
	LOG_INFO("work queue callback\n");
	flmain_disable();
}

static enum hrtimer_restart flmain_timer_func(struct hrtimer *timer)
{
	schedule_work(&flmain_work);
	return HRTIMER_NORESTART;
}


/******************************************************************************
 * Flashlight operations
 *****************************************************************************/
static int flmain_ioctl(unsigned int cmd, unsigned long arg)
{
	struct flashlight_dev_arg *fl_arg;
	int channel;
	ktime_t ktime;

	fl_arg = (struct flashlight_dev_arg *)arg;
	channel = fl_arg->channel;

	switch (cmd) {
	case FLASH_IOC_SET_TIME_OUT_TIME_MS:
		LOG_INFO("FLASH_IOC_SET_TIME_OUT_TIME_MS(%d): %d\n",
				channel, (int)fl_arg->arg);
		flmain_timeout_ms = fl_arg->arg;
		break;

	case FLASH_IOC_SET_DUTY:
		LOG_INFO("FLASH_IOC_SET_DUTY(%d): %d\n",
				channel, (int)fl_arg->arg);
		flmain_set_level(fl_arg->arg);
		break;

	case FLASH_IOC_SET_ONOFF:
		LOG_INFO("FLASH_IOC_SET_ONOFF: channel = %d; state = %d\n",
				channel, (int)fl_arg->arg);
		if (fl_arg->arg == 1) {
			if (flmain_timeout_ms) {
				ktime = ktime_set(flmain_timeout_ms / 1000,
						(flmain_timeout_ms % 1000) * 1000000);
				hrtimer_start(&flmain_timer, ktime, HRTIMER_MODE_REL);
			}
			flmain_enable();
		} else {
			flmain_disable();
			hrtimer_cancel(&flmain_timer);
		}
		break;
	default:
		LOG_INFO("No such command and arg(%d): (%d, %d)\n",
				channel, _IOC_NR(cmd), (int)fl_arg->arg);
		return -ENOTTY;
	}

	return 0;
}

static int flmain_open(void)
{
	/* Move to set driver for saving power */
	return 0;
}

static int flmain_release(void)
{
	/* Move to set driver for saving power */
	return 0;
}

static int flmain_set_driver(int set)
{
	int ret = 0;

	/* set chip and usage count */
	mutex_lock(&flmain_mutex);
	if (set) {
		if (!use_count)
			ret = flmain_init();
		use_count++;
		LOG_INFO("Set driver: %d\n", use_count);
	} else {
		use_count--;
		if (!use_count)
			ret = flmain_uninit();
		if (use_count < 0)
			use_count = 0;
		LOG_INFO("Unset driver: %d\n", use_count);
	}
	mutex_unlock(&flmain_mutex);

	return ret;
}

static ssize_t flmain_strobe_store(struct flashlight_arg arg)
{
	flmain_set_driver(1);
	flmain_set_level(arg.level);
	flmain_timeout_ms = 0;
	flmain_enable();
	msleep(arg.dur);
	flmain_disable();
	flmain_set_driver(0);

	return 0;
}

static struct flashlight_operations flmain_ops = {
	flmain_open,
	flmain_release,
	flmain_ioctl,
	flmain_strobe_store,
	flmain_set_driver
};


/******************************************************************************
 * Platform device and driver
 *****************************************************************************/
static int flmain_chip_init(void)
{
	/* NOTE: Chip initialication move to "set driver" operation for power saving issue.
	 * flmain_init();
	 */

	return 0;
}

static int flmain_parse_dt(struct device *dev,
		struct flmain_platform_data *pdata)
{
	struct device_node *np, *cnp;
	u32 decouple = 0;
	int i = 0;

	if (!dev || !dev->of_node || !pdata)
		return -ENODEV;

	np = dev->of_node;

	pdata->channel_num = of_get_child_count(np);
	if (!pdata->channel_num) {
		LOG_INFO("Parse no dt, node.\n");
		return 0;
	}
	LOG_INFO("Channel number(%d).\n", pdata->channel_num);

	if (of_property_read_u32(np, "decouple", &decouple))
		LOG_INFO("Parse no dt, decouple.\n");

	pdata->dev_id = devm_kzalloc(dev,
			pdata->channel_num * sizeof(struct flashlight_device_id),
			GFP_KERNEL);
	if (!pdata->dev_id)
		return -ENOMEM;

	for_each_child_of_node(np, cnp) {
		if (of_property_read_u32(cnp, "type", &pdata->dev_id[i].type))
			goto err_node_put;
		if (of_property_read_u32(cnp, "ct", &pdata->dev_id[i].ct))
			goto err_node_put;
		if (of_property_read_u32(cnp, "part", &pdata->dev_id[i].part))
			goto err_node_put;
		snprintf(pdata->dev_id[i].name, FLASHLIGHT_NAME_SIZE, FLMAIN_NAME);
		pdata->dev_id[i].channel = i;
		pdata->dev_id[i].decouple = decouple;

		LOG_INFO("Parse dt (type,ct,part,name,channel,decouple)=(%d,%d,%d,%s,%d,%d).\n",
				pdata->dev_id[i].type, pdata->dev_id[i].ct,
				pdata->dev_id[i].part, pdata->dev_id[i].name,
				pdata->dev_id[i].channel, pdata->dev_id[i].decouple);
		i++;
	}

	return 0;

err_node_put:
	of_node_put(cnp);
	return -EINVAL;
}

static int flmain_probe(struct platform_device *pdev)
{
	struct flmain_platform_data *pdata = dev_get_platdata(&pdev->dev);
	int err;
	int i;

	LOG_INFO("Probe start.\n");

	/* init pinctrl */
	if (flmain_pinctrl_init(pdev)) {
		LOG_INFO("Failed to init pinctrl.\n");
		err = -EFAULT;
		goto err;
	}

	/* init platform data */
	if (!pdata) {
		pdata = devm_kzalloc(&pdev->dev, sizeof(*pdata), GFP_KERNEL);
		if (!pdata) {
			err = -ENOMEM;
			goto err;
		}
		pdev->dev.platform_data = pdata;
		err = flmain_parse_dt(&pdev->dev, pdata);
		if (err)
			goto err;
	}

	/* init work queue */
	INIT_WORK(&flmain_work, flmain_work_disable);

	/* init timer */
	hrtimer_init(&flmain_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	flmain_timer.function = flmain_timer_func;
	flmain_timeout_ms = 100;

	/* init chip hw */
	flmain_chip_init();

	/* clear usage count */
	use_count = 0;

	/* register flashlight device */
	if (pdata->channel_num) {
		for (i = 0; i < pdata->channel_num; i++)
			if (flashlight_dev_register_by_device_id(&pdata->dev_id[i], &flmain_ops)) {
				err = -EFAULT;
				goto err;
			}
	} else {
		if (flashlight_dev_register(FLMAIN_NAME, &flmain_ops)) {
			err = -EFAULT;
			goto err;
		}
	}

	LOG_INFO("Probe done.\n");

	return 0;
err:
	return err;
}

static int flmain_remove(struct platform_device *pdev)
{
	struct flmain_platform_data *pdata = dev_get_platdata(&pdev->dev);
	int i;

	LOG_INFO("Remove start.\n");

	pdev->dev.platform_data = NULL;

	/* unregister flashlight device */
	if (pdata && pdata->channel_num)
		for (i = 0; i < pdata->channel_num; i++)
			flashlight_dev_unregister_by_device_id(&pdata->dev_id[i]);
	else
		flashlight_dev_unregister(FLMAIN_NAME);

	/* flush work queue */
	flush_work(&flmain_work);

	LOG_INFO("Remove done.\n");

	return 0;
}

#ifdef CONFIG_OF
static const struct of_device_id flmain_gpio_of_match[] = {
	{.compatible = FLMAIN_DTNAME},
	{},
};
MODULE_DEVICE_TABLE(of, flmain_gpio_of_match);
#else
static struct platform_device flmain_gpio_platform_device[] = {
	{
		.name = FLMAIN_NAME,
		.id = 0,
		.dev = {}
	},
	{}
};
MODULE_DEVICE_TABLE(platform, flmain_gpio_platform_device);
#endif

static struct platform_driver flmain_platform_driver = {
	.probe = flmain_probe,
	.remove = flmain_remove,
	.driver = {
		.name = FLMAIN_NAME,
		.owner = THIS_MODULE,
#ifdef CONFIG_OF
		.of_match_table = flmain_gpio_of_match,
#endif
	},
};

static int __init flashlight_flmain_init(void)
{
	int ret;

	LOG_INFO("Init start.\n");

#ifndef CONFIG_OF
	ret = platform_device_register(&flmain_gpio_platform_device);
	if (ret) {
		LOG_INFO("Failed to register platform device\n");
		return ret;
	}
#endif

	ret = platform_driver_register(&flmain_platform_driver);
	if (ret) {
		LOG_INFO("Failed to register platform driver\n");
		return ret;
	}

	LOG_INFO("Init done.\n");

	return 0;
}

static void __exit flashlight_flmain_exit(void)
{
	LOG_INFO("Exit start.\n");

	platform_driver_unregister(&flmain_platform_driver);

	LOG_INFO("Exit done.\n");
}

module_init(flashlight_flmain_init);
module_exit(flashlight_flmain_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Simon Wang <Simon-TCH.Wang@mediatek.com>");
MODULE_DESCRIPTION("MTK Flashlight FLMAIN GPIO Driver");

