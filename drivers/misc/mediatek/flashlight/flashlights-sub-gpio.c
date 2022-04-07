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
#define LOG_INFO(fmt,arg...)    printk("FL_SUB [%s:%d] "fmt"\n", __func__, __LINE__, ##arg)

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

/* define device tree */
/* TODO: modify temp device tree name */
#ifndef FLSUB_DTNAME
#define FLSUB_DTNAME "mediatek,flashlights_sub_gpio"
#endif

/* TODO: define driver name */
#define FLSUB_NAME "flashlights-sub-gpio"

/* define registers */
/* TODO: define register */

/* define mutex and work queue */
static DEFINE_MUTEX(flsub_mutex);
static struct work_struct flsub_work;

/* define pinctrl */
/* TODO: define pinctrl */
#define FLSUB_PINCTRL_PIN_GPIO 0
#define FLSUB_PINCTRL_PINSTATE_LOW 0
#define FLSUB_PINCTRL_PINSTATE_HIGH 1
#define FLSUB_PINCTRL_STATE_GPIO_HIGH "gpio_sub_high"
#define FLSUB_PINCTRL_STATE_GPIO_LOW  "gpio_sub_low"
static struct pinctrl *flsub_pinctrl;
static struct pinctrl_state *flsub_gpio_high;
static struct pinctrl_state *flsub_gpio_low;

/* define usage count */
static int use_count;

/* platform data */
struct flsub_platform_data {
	int channel_num;
	struct flashlight_device_id *dev_id;
};


/******************************************************************************
 * Pinctrl configuration
 *****************************************************************************/
static int flsub_pinctrl_init(struct platform_device *pdev)
{
	int ret = 0;

	/* get pinctrl */
	flsub_pinctrl = devm_pinctrl_get(&pdev->dev);
	if (IS_ERR(flsub_pinctrl)) {
		LOG_INFO("Failed to get flashlight pinctrl.\n");
		ret = PTR_ERR(flsub_pinctrl);
	} else {
		LOG_INFO("Success to get flashlight pinctrl.\n");
	}

	/* TODO: Flashlight GPIO pin initialization */
	flsub_gpio_high = pinctrl_lookup_state(flsub_pinctrl, FLSUB_PINCTRL_STATE_GPIO_HIGH);
	if (IS_ERR(flsub_gpio_high)) {
		LOG_INFO("Failed to init (%s)\n", FLSUB_PINCTRL_STATE_GPIO_HIGH);
		ret = PTR_ERR(flsub_gpio_high);
	} else {
		LOG_INFO("Success to init (%s)\n", FLSUB_PINCTRL_STATE_GPIO_HIGH);
	}
	flsub_gpio_low = pinctrl_lookup_state(flsub_pinctrl, FLSUB_PINCTRL_STATE_GPIO_LOW);
	if (IS_ERR(flsub_gpio_low)) {
		LOG_INFO("Failed to init (%s)\n", FLSUB_PINCTRL_STATE_GPIO_LOW);
		ret = PTR_ERR(flsub_gpio_low);
	} else {
		LOG_INFO("Success to init (%s)\n", FLSUB_PINCTRL_STATE_GPIO_LOW);
	}

	return ret;
}

static int flsub_pinctrl_set(int pin, int state)
{
	int ret = 0;

	if (IS_ERR(flsub_pinctrl)) {
		LOG_INFO("pinctrl is not available\n");
		return -1;
	}

	switch (pin) {
	case FLSUB_PINCTRL_PIN_GPIO:
		if (state == FLSUB_PINCTRL_PINSTATE_LOW && !IS_ERR(flsub_gpio_low))
			pinctrl_select_state(flsub_pinctrl, flsub_gpio_low);
		else if (state == FLSUB_PINCTRL_PINSTATE_HIGH && !IS_ERR(flsub_gpio_high))
			pinctrl_select_state(flsub_pinctrl, flsub_gpio_high);
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
 * flsub operations
 *****************************************************************************/
/* flashlight enable function */
static int flsub_enable(void)
{
	int pin = 0, state = 1;

	/* TODO: wrap enable function */

	return flsub_pinctrl_set(pin, state);
}

/* flashlight disable function */
static int flsub_disable(void)
{
	int pin = 0, state = 0;

	/* TODO: wrap disable function */

	return flsub_pinctrl_set(pin, state);
}

/* set flashlight level */
static int flsub_set_level(int level)
{
	int pin = 0, state = 0;

	/* TODO: wrap set level function */

	return flsub_pinctrl_set(pin, state);
}

/* flashlight init */
static int flsub_init(void)
{
	int pin = 0, state = 0;

	/* TODO: wrap init function */

	return flsub_pinctrl_set(pin, state);
}

/* flashlight uninit */
static int flsub_uninit(void)
{
	int pin = 0, state = 0;

	/* TODO: wrap uninit function */

	return flsub_pinctrl_set(pin, state);
}

/******************************************************************************
 * Timer and work queue
 *****************************************************************************/
static struct hrtimer flsub_timer;
static unsigned int flsub_timeout_ms;

static void flsub_work_disable(struct work_struct *data)
{
	LOG_INFO("work queue callback\n");
	flsub_disable();
}

static enum hrtimer_restart flsub_timer_func(struct hrtimer *timer)
{
	schedule_work(&flsub_work);
	return HRTIMER_NORESTART;
}


/******************************************************************************
 * Flashlight operations
 *****************************************************************************/
static int flsub_ioctl(unsigned int cmd, unsigned long arg)
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
		flsub_timeout_ms = fl_arg->arg;
		break;

	case FLASH_IOC_SET_DUTY:
		LOG_INFO("FLASH_IOC_SET_DUTY(%d): %d\n",
				channel, (int)fl_arg->arg);
		flsub_set_level(fl_arg->arg);
		break;

	case FLASH_IOC_SET_ONOFF:
		LOG_INFO("FLASH_IOC_SET_ONOFF: channel = %d; state = %d\n",
				channel, (int)fl_arg->arg);
		if (fl_arg->arg == 1) {
			if (flsub_timeout_ms) {
				ktime = ktime_set(flsub_timeout_ms / 1000,
						(flsub_timeout_ms % 1000) * 1000000);
				hrtimer_start(&flsub_timer, ktime, HRTIMER_MODE_REL);
			}
			flsub_enable();
		} else {
			flsub_disable();
			hrtimer_cancel(&flsub_timer);
		}
		break;
	default:
		LOG_INFO("No such command and arg(%d): (%d, %d)\n",
				channel, _IOC_NR(cmd), (int)fl_arg->arg);
		return -ENOTTY;
	}

	return 0;
}

static int flsub_open(void)
{
	/* Move to set driver for saving power */
	return 0;
}

static int flsub_release(void)
{
	/* Move to set driver for saving power */
	return 0;
}

static int flsub_set_driver(int set)
{
	int ret = 0;

	/* set chip and usage count */
	mutex_lock(&flsub_mutex);
	if (set) {
		if (!use_count)
			ret = flsub_init();
		use_count++;
		LOG_INFO("Set driver: %d\n", use_count);
	} else {
		use_count--;
		if (!use_count)
			ret = flsub_uninit();
		if (use_count < 0)
			use_count = 0;
		LOG_INFO("Unset driver: %d\n", use_count);
	}
	mutex_unlock(&flsub_mutex);

	return ret;
}

static ssize_t flsub_strobe_store(struct flashlight_arg arg)
{
	flsub_set_driver(1);
	flsub_set_level(arg.level);
	flsub_timeout_ms = 0;
	flsub_enable();
	msleep(arg.dur);
	flsub_disable();
	flsub_set_driver(0);

	return 0;
}

static struct flashlight_operations flsub_ops = {
	flsub_open,
	flsub_release,
	flsub_ioctl,
	flsub_strobe_store,
	flsub_set_driver
};


/******************************************************************************
 * Platform device and driver
 *****************************************************************************/
static int flsub_chip_init(void)
{
	/* NOTE: Chip initialication move to "set driver" operation for power saving issue.
	 * flsub_init();
	 */

	return 0;
}

static int flsub_parse_dt(struct device *dev,
		struct flsub_platform_data *pdata)
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
		snprintf(pdata->dev_id[i].name, FLASHLIGHT_NAME_SIZE, FLSUB_NAME);
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

static int flsub_probe(struct platform_device *pdev)
{
	struct flsub_platform_data *pdata = dev_get_platdata(&pdev->dev);
	int err;
	int i;

	LOG_INFO("Probe start.\n");

	/* init pinctrl */
	if (flsub_pinctrl_init(pdev)) {
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
		err = flsub_parse_dt(&pdev->dev, pdata);
		if (err)
			goto err;
	}

	/* init work queue */
	INIT_WORK(&flsub_work, flsub_work_disable);

	/* init timer */
	hrtimer_init(&flsub_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	flsub_timer.function = flsub_timer_func;
	flsub_timeout_ms = 100;

	/* init chip hw */
	flsub_chip_init();

	/* clear usage count */
	use_count = 0;

	/* register flashlight device */
	if (pdata->channel_num) {
		for (i = 0; i < pdata->channel_num; i++)
			if (flashlight_dev_register_by_device_id(&pdata->dev_id[i], &flsub_ops)) {
				err = -EFAULT;
				goto err;
			}
	} else {
		if (flashlight_dev_register(FLSUB_NAME, &flsub_ops)) {
			err = -EFAULT;
			goto err;
		}
	}

	LOG_INFO("Probe done.\n");

	return 0;
err:
	return err;
}

static int flsub_remove(struct platform_device *pdev)
{
	struct flsub_platform_data *pdata = dev_get_platdata(&pdev->dev);
	int i;

	LOG_INFO("Remove start.\n");

	pdev->dev.platform_data = NULL;

	/* unregister flashlight device */
	if (pdata && pdata->channel_num)
		for (i = 0; i < pdata->channel_num; i++)
			flashlight_dev_unregister_by_device_id(&pdata->dev_id[i]);
	else
		flashlight_dev_unregister(FLSUB_NAME);

	/* flush work queue */
	flush_work(&flsub_work);

	LOG_INFO("Remove done.\n");

	return 0;
}

#ifdef CONFIG_OF
static const struct of_device_id flsub_gpio_of_match[] = {
	{.compatible = FLSUB_DTNAME},
	{},
};
MODULE_DEVICE_TABLE(of, flsub_gpio_of_match);
#else
static struct platform_device flsub_gpio_platform_device[] = {
	{
		.name = FLSUB_NAME,
		.id = 0,
		.dev = {}
	},
	{}
};
MODULE_DEVICE_TABLE(platform, flsub_gpio_platform_device);
#endif

static struct platform_driver flsub_platform_driver = {
	.probe = flsub_probe,
	.remove = flsub_remove,
	.driver = {
		.name = FLSUB_NAME,
		.owner = THIS_MODULE,
#ifdef CONFIG_OF
		.of_match_table = flsub_gpio_of_match,
#endif
	},
};

static int __init flashlight_flsub_init(void)
{
	int ret;

	LOG_INFO("Init start.\n");

#ifndef CONFIG_OF
	ret = platform_device_register(&flsub_gpio_platform_device);
	if (ret) {
		LOG_INFO("Failed to register platform device\n");
		return ret;
	}
#endif

	ret = platform_driver_register(&flsub_platform_driver);
	if (ret) {
		LOG_INFO("Failed to register platform driver\n");
		return ret;
	} else {
		LOG_INFO("Success to register platform driver\n");
	}

	LOG_INFO("Init done.\n");

	return 0;
}

static void __exit flashlight_flsub_exit(void)
{
	LOG_INFO("Exit start.\n");

	platform_driver_unregister(&flsub_platform_driver);

	LOG_INFO("Exit done.\n");
}

module_init(flashlight_flsub_init);
module_exit(flashlight_flsub_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Simon Wang <Simon-TCH.Wang@mediatek.com>");
MODULE_DESCRIPTION("MTK Flashlight FLSUB GPIO Driver");

