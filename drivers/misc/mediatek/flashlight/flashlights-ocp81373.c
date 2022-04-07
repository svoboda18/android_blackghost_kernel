// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2020 MediaTek Inc.
 */

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
#include <linux/err.h>
#include <linux/errno.h>
#include <linux/time.h>
#include <linux/io.h>
#include <linux/uaccess.h>
#include <linux/hrtimer.h>
#include <linux/ktime.h>
#include <linux/version.h>
#include <linux/mutex.h>
#include <linux/i2c.h>
#include <linux/leds.h>

#include <linux/pinctrl/consumer.h>

#include "flashlight-core.h"
#include "flashlight-dt.h"
//ITD:modify by camera driver team start
// #include "flashlight_current_config.h"
//ITD:modify by camera driver team end
/* define device tree */
/* TODO: modify temp device tree name */
#ifndef OCP81373_DTNAME_I2C
#define OCP81373_DTNAME_I2C "mediatek,strobe_main"
#endif
/* define device tree */
/* TODO: modify temp device tree name */
#ifndef OCP81373_DTNAME
#define OCP81373_DTNAME "mediatek,flashlights_ocp81373"
#endif
/* TODO: define driver name */
#define OCP81373_NAME "flashlights-ocp81373"

/* define registers */
#define OCP81373_REG_ENABLE			0x01
#define OCP81373_REG_IVFM				0x02
#define OCP81373_REG_FLASH_LEVEL_LED1	0x03
#define OCP81373_REG_FLASH_LEVEL_LED2 0x04
#define OCP81373_REG_TORCH_LEVEL_LED1	0x05
#define OCP81373_REG_TORCH_LEVEL_LED2	0x06
#define OCP81373_REG_BOOST_CONFIG		0x07
#define OCP81373_REG_TIMING_CONFIG	0x08
#define OCP81373_REG_TEMP				0x09
#define OCP81373_REG_FLAG1			0x0A
#define OCP81373_REG_FLAG2			0x0B
#define OCP81373_REG_DEVICE_ID		0x0C


#define OCP81373_PINCTRL_PIN_HWEN 0
#define OCP81373_PINCTRL_PINSTATE_LOW 0
#define OCP81373_PINCTRL_PINSTATE_HIGH 1
#define OCP81373_PINCTRL_STATE_HWEN_HIGH "hwen_high"
#define OCP81373_PINCTRL_STATE_HWEN_LOW  "hwen_low"
static struct pinctrl *ocp81373_pinctrl;
static struct pinctrl_state *ocp81373_hwen_high;
static struct pinctrl_state *ocp81373_hwen_low;

/* define channel, level */
#define OCP81373_CHANNEL_NUM 2
#define OCP81373_CHANNEL_CH1 0
#define OCP81373_CHANNEL_CH2 1

#define OCP81373_NONE (-1)
#define OCP81373_DISABLE 0
#define OCP81373_ENABLE 1
#define OCP81373_ENABLE_TORCH 1
#define OCP81373_ENABLE_FLASH 2
#define OCP81373_WAIT_TIME 3
#define OCP81373_RETRY_TIMES 3

/* TODO: define register */
struct flashlight_dev_para_struct {
	int min_flash_duty;//min duty supported by IC flash mode(auto generate)
	int flashmode;//if this time flashlight operate use flash mode(auto generate)
	int max_torch_current;//max duty supported by IC torch mode(static, config by developer)
	int max_flash_current;//max duty supported by IC flash mode(static, config by developer)
	int duty_num;//save duty num(auto generate)
	int duty_reg_code[30];//save IC reg code by duty(auto generate)
};

//project para struct: static, config by developer
struct project_current_config_struct {
	int hw_limit[2];//hardware limit, [0]torch current limit, [1]flash current limit
	int sysui_torch[2];//sysui torch, [0]low1 level current, [1]low2 level current
	int fac_flash;//factory mode current
	int faceid_torch;//face id current
	int torch_360;//torch_360 current
	int phonecall;//phonecall reminder current
	int torch_duty_range[2];//slide control torch current, [0]min, [1]max
	int duty_num;//app duty num
	int app_duty_current[30];//app duty current
};

static struct project_current_config_struct g_project_current_config[2] = {
	{
		{372,1500},
		{72,350},
		421,
		22,
		72,
		22,
		{3,92},
		30,{22,72,124,174,223,273,322,372,
			421,468,515,573,620,667,714,773,
			820,866,913,972,1019,1066,1124,1171,
			1218,1265,1323,1370,1417,1488}
	},
	{
		{450,900},
		{182,182},
		235,
		182,
		182,
		182,
		{100,350},
		2,{350,850}
	}
};

/* define mutex, work queue and timer */
static DEFINE_MUTEX(ocp81373_mutex);
static struct work_struct ocp81373_work_ch1;
static struct work_struct ocp81373_work_ch2;
static struct hrtimer ocp81373_timer_ch1;
static struct hrtimer ocp81373_timer_ch2;
static unsigned int ocp81373_timeout_ms[OCP81373_CHANNEL_NUM];
//ITD:modify by camera driver team start
static struct flashlight_dev_para_struct ocp81373_para[OCP81373_CHANNEL_NUM];
//ITD:modify by camera driver team end
/* define usage count */
static int use_count;
/* define i2c */
static struct i2c_client *OCP81373_i2c_client;

/* platform data */
struct ocp81373_platform_data {
	u8 torch_pin_enable;
	u8 pam_sync_pin_enable;
	u8 thermal_comp_mode_enable;
	u8 strobe_pin_disable;
	u8 vout_mode_enable;
};

/* ocp81373 chip data */
struct ocp81373_chip_data {
	struct i2c_client *client;
	struct ocp81373_platform_data *pdata;
	struct mutex lock;
	u8 last_flag;
	u8 no_pdata;
};

static int ocp81373_flash_read(struct i2c_client *client, u8 reg)
{
	int ret;
	//char data = 0;
	struct ocp81373_chip_data *chip = i2c_get_clientdata(client);

	mutex_lock(&chip->lock);
	ret = i2c_smbus_read_byte_data(client, reg);
	mutex_unlock(&chip->lock);
	pr_info("%s reg:0x%x val:0x%x\n", __func__, reg, ret);
	if (ret < 0)
		pr_err("failed reading at 0x%02x\n", reg);

	return ret;
}

/******************************************************************************
 * ocp81373 operations
 *****************************************************************************/

/* i2c wrapper function */
static int ocp81373_flash_write(struct i2c_client *client, u8 reg, u8 val)
{

	int ret;
	struct ocp81373_chip_data *chip = i2c_get_clientdata(client);

	mutex_lock(&chip->lock);
	ret = i2c_smbus_write_byte_data(client, reg, val);
	mutex_unlock(&chip->lock);

	if (ret < 0)
		pr_err("failed writing at 0x%02x\n", reg);

	return ret;

}


/******************************************************************************
 * Pinctrl configuration
 *****************************************************************************/
static int ocp81373_pinctrl_init(struct platform_device *pdev)
{
	int ret = 0;
	pr_info("%s in\n", __func__);
	//return 1;
	/* get pinctrl */
	ocp81373_pinctrl = devm_pinctrl_get(&pdev->dev);
	if (IS_ERR(ocp81373_pinctrl)) {
		pr_err("Failed to get flashlight pinctrl.\n");
		ret = PTR_ERR(ocp81373_pinctrl);
	}

	/* Flashlight HWEN pin initialization */
	ocp81373_hwen_high = pinctrl_lookup_state(ocp81373_pinctrl,
		OCP81373_PINCTRL_STATE_HWEN_HIGH);
	if (IS_ERR(ocp81373_hwen_high)) {
		pr_err("Failed to init (%s)\n",
			OCP81373_PINCTRL_STATE_HWEN_HIGH);
		ret = PTR_ERR(ocp81373_hwen_high);
	}
	ocp81373_hwen_low = pinctrl_lookup_state(ocp81373_pinctrl,
		OCP81373_PINCTRL_STATE_HWEN_LOW);
	if (IS_ERR(ocp81373_hwen_low)) {
		pr_err("Failed to init (%s)\n", OCP81373_PINCTRL_STATE_HWEN_LOW);
		ret = PTR_ERR(ocp81373_hwen_low);
	}
	pr_info("%s out\n", __func__);
	return ret;
}

static int ocp81373_pinctrl_set(int pin, int state)
{
	int ret = 0;
	pr_info("%s in\n", __func__);
	//return 0;

	if (IS_ERR(ocp81373_pinctrl)) {
		pr_err("pinctrl is not available\n");
		return -1;
	}

	switch (pin) {
	case OCP81373_PINCTRL_PIN_HWEN:
		if (state == OCP81373_PINCTRL_PINSTATE_LOW &&
			!IS_ERR(ocp81373_hwen_low))
			pinctrl_select_state(ocp81373_pinctrl, ocp81373_hwen_low);
		else if (state == OCP81373_PINCTRL_PINSTATE_HIGH &&
			!IS_ERR(ocp81373_hwen_high))
			pinctrl_select_state(ocp81373_pinctrl, ocp81373_hwen_high);
		else
			pr_err("set err, pin(%d) state(%d)\n", pin, state);
		break;
	default:
		pr_err("set err, pin(%d) state(%d)\n", pin, state);
		break;
	}
	pr_info("pin(%d) state(%d)\n", pin, state);
	pr_info("%s out\n", __func__);
	return ret;
}

static int ocp81373_decouple_mode;
static int ocp81373_keepstate_decouple_mode;
static int ocp81373_en_ch1;
static int ocp81373_en_ch2;
//ITD:modify by camera driver team start
static int ocp81373_set_torch_brightness(int channel,int regval)
{
	int led1regval = 0;
	if(channel == OCP81373_CHANNEL_CH1){
		ocp81373_flash_write(OCP81373_i2c_client, OCP81373_REG_TORCH_LEVEL_LED1,regval&0x7f);
		ocp81373_flash_write(OCP81373_i2c_client, OCP81373_REG_TORCH_LEVEL_LED2,0x00);
	}else if(channel == OCP81373_CHANNEL_CH2){
		/*OCP81373_REG_TORCH_LEVEL_LED1:
		bit7 default=1,LED2 Torch Current is set to LED1 Torch Current,
		bit7 =0,Torch Current is not set to LED1 Torch Current
		before set led2 torch,clear OCP81373_REG_TORCH_LEVEL_LED1 bit6 first*/
		if(ocp81373_keepstate_decouple_mode == 1){//360 torch
			led1regval = ocp81373_flash_read(OCP81373_i2c_client, OCP81373_REG_TORCH_LEVEL_LED1);
			ocp81373_flash_write(OCP81373_i2c_client, OCP81373_REG_TORCH_LEVEL_LED2,0x00);
			ocp81373_flash_write(OCP81373_i2c_client, OCP81373_REG_TORCH_LEVEL_LED1,(0x7f&led1regval));
		}else{
			ocp81373_flash_write(OCP81373_i2c_client, OCP81373_REG_TORCH_LEVEL_LED1,0);
		}
		ocp81373_flash_write(OCP81373_i2c_client, OCP81373_REG_TORCH_LEVEL_LED2,regval);
	}else{
		pr_err("Error channel\n");
		return -1;
	}
	mdelay(OCP81373_WAIT_TIME);
	return 0;
}

static int ocp81373_set_strobe_brightness(int channel,int regval)
{
	int led1regval = 0;
	if(channel == OCP81373_CHANNEL_CH1){
		ocp81373_flash_write(OCP81373_i2c_client, OCP81373_REG_FLASH_LEVEL_LED1,regval&0x7f);
		ocp81373_flash_write(OCP81373_i2c_client, OCP81373_REG_FLASH_LEVEL_LED2,0x00);
	}else if(channel == OCP81373_CHANNEL_CH2){
		/*OCP81373_REG_FLASH_LEVEL_LED1:
		bit7 default=1,LED2 Flash Current is set to LED1 Flash Current,
		bit7 =0,Flash Current is not set to LED1 Flash Current
		before set led2 flash,clear OCP81373_REG_FLASH_LEVEL_LED1 bit6 first*/
		if(ocp81373_keepstate_decouple_mode == 1){//360 torch
			led1regval = ocp81373_flash_read(OCP81373_i2c_client, OCP81373_REG_FLASH_LEVEL_LED1);
			ocp81373_flash_write(OCP81373_i2c_client, OCP81373_REG_FLASH_LEVEL_LED1,(0x7f&led1regval));
		}else{
			ocp81373_flash_write(OCP81373_i2c_client, OCP81373_REG_FLASH_LEVEL_LED1,0);
		}
		ocp81373_flash_write(OCP81373_i2c_client, OCP81373_REG_FLASH_LEVEL_LED2,regval);
	}else{
		pr_err("Error channel\n");
		return -1;
	}
	mdelay(OCP81373_WAIT_TIME);
	return 0;
}

static int ocp81373_is_torch(int channel,int level)
{
	if (level >= ocp81373_para[channel].min_flash_duty)
		return -1;

	return 0;
}

static int ocp81373_verify_level(int channel, int level)
{
	if (level < 0)
		level = 0;
	else if (level >= ocp81373_para[channel].duty_num)
		level = ocp81373_para[channel].duty_num - 1;

	return level;
}

/* flashlight enable function */
static int ocp81373_enable(int channel)
{
	int enableregval = 0;

	pr_info("channel:%d ocp81373_en_ch1:%d", channel, ocp81373_en_ch1);
	if (channel == OCP81373_CHANNEL_CH1) {
		if (ocp81373_en_ch1 == OCP81373_ENABLE_FLASH) {
			ocp81373_flash_write(OCP81373_i2c_client,
				OCP81373_REG_ENABLE, 0x0D);
		} else{
			if (ocp81373_keepstate_decouple_mode == 1) {//360 torch
				enableregval =
					ocp81373_flash_read(OCP81373_i2c_client,
						OCP81373_REG_ENABLE);
				ocp81373_flash_write(OCP81373_i2c_client,
					OCP81373_REG_ENABLE, (0x09|enableregval));
			} else{
				ocp81373_flash_write(OCP81373_i2c_client,
					OCP81373_REG_ENABLE, 0x09);
			}
		}
	} else{
		if (ocp81373_en_ch2 == OCP81373_ENABLE_FLASH) {
			ocp81373_flash_write(OCP81373_i2c_client,
				OCP81373_REG_ENABLE, 0x0E);
		} else{
			if (ocp81373_keepstate_decouple_mode == 1) {//360 torch
				enableregval =
					ocp81373_flash_read(OCP81373_i2c_client,
						OCP81373_REG_ENABLE);
				ocp81373_flash_write(OCP81373_i2c_client,
					OCP81373_REG_ENABLE, (0x0A|enableregval));
			} else{
				ocp81373_flash_write(OCP81373_i2c_client,
					OCP81373_REG_ENABLE, 0x0A);
			}
		}
	}

	return 0;
}

/* flashlight disable function */
static int ocp81373_disable(void)
{
	pr_info("%s\n", __func__);
	ocp81373_flash_write(OCP81373_i2c_client, OCP81373_REG_ENABLE, 0x00);
	return 0;
}


/* set flashlight level */
static int ocp81373_set_level(int channel,int lel)
{
	int level = 0;
	level = ocp81373_verify_level(channel,lel);
	ocp81373_para[channel].flashmode = 0;
	if (!ocp81373_is_torch(channel,level)){
		ocp81373_set_torch_brightness(
			channel,  ocp81373_para[channel].duty_reg_code[level]);
	}else{
		ocp81373_para[channel].flashmode = 1;
		ocp81373_set_strobe_brightness(
			channel, ocp81373_para[channel].duty_reg_code[level]);
	}
	return 0;
}

//map current to regcode, implement according to datasheet
static unsigned char mapcurrent2regcode(int channel, unsigned int duty_current)
{
	if(duty_current > ocp81373_para[channel].max_flash_current){
		return 0x7F;
	}else if(duty_current < 3){//decause for 0, 1, 2  ((duty_current*100 - 255)/291)&0x7F will be abnormal
		return 0;
	}else if(duty_current > ocp81373_para[channel].max_torch_current){
		return ((duty_current*100 - 1172)/1172)&0xFF;
	}else{
		return ((duty_current*100 - 292)/292)&0x7F;
	}
}

//generate register code by duty
static void generate_dev_para(int channel, int type)
{
	int i = 0;
	int duty_num = g_project_current_config[type].duty_num;
	ocp81373_para[channel].duty_num = g_project_current_config[type].duty_num;
	for(;i<30;i++){
		int duty_current = g_project_current_config[type].app_duty_current[i];
		if(i >= duty_num){
			ocp81373_para[channel].duty_reg_code[i] = ocp81373_para[channel].duty_reg_code[duty_num -1];
		}else{
			if(duty_current > g_project_current_config[type].hw_limit[1])
				duty_current = g_project_current_config[type].hw_limit[1];
			if((duty_current > ocp81373_para[channel].max_torch_current) && (i < ocp81373_para[channel].min_flash_duty))//get min_flash_duty
				ocp81373_para[channel].min_flash_duty = i;
			ocp81373_para[channel].duty_reg_code[i] = mapcurrent2regcode(channel, duty_current);
		}
		pr_info("generate_dev_para = 0x%x\n",ocp81373_para[channel].duty_reg_code[i]);
	}
}

static int ocp81373_set_scenario(int scenario)
{
	/* set decouple mode */
	ocp81373_decouple_mode = scenario & FLASHLIGHT_SCENARIO_DECOUPLE_MASK;
#if 0
	ocp81373_keepstate_decouple_mode =
		scenario & FLASHLIGHT_SCENARIO_KEEPSTATE_DECOUPLE_MASK;
#endif
	return 0;
}

/* flashlight init */
static int ocp81373_init(void)
{
	pr_info("%s\n", __func__);
	/* clear flashlight state */
	ocp81373_en_ch1 = OCP81373_DISABLE;
	ocp81373_en_ch2 = OCP81373_DISABLE;
	/* clear decouple mode */
	ocp81373_decouple_mode = FLASHLIGHT_SCENARIO_COUPLE;
#if 0
	ocp81373_keepstate_decouple_mode = FLASHLIGHT_SCENARIO_KEEPSTATE_COUPLE;
#endif
	ocp81373_keepstate_decouple_mode = 0;
	ocp81373_pinctrl_set(OCP81373_PINCTRL_PIN_HWEN,
		OCP81373_PINCTRL_PINSTATE_HIGH);
	mdelay(OCP81373_WAIT_TIME);
	ocp81373_flash_write(OCP81373_i2c_client, OCP81373_REG_ENABLE, 0x00);
	ocp81373_flash_write(OCP81373_i2c_client, OCP81373_REG_BOOST_CONFIG, 0x09);
	ocp81373_flash_write(OCP81373_i2c_client, OCP81373_REG_TIMING_CONFIG, 0x1f);

	return 0;
}

/* flashlight uninit */
static int ocp81373_uninit(void)
{
	/* clear flashlight state */
	ocp81373_en_ch1 = OCP81373_NONE;
	ocp81373_en_ch2 = OCP81373_NONE;
	ocp81373_decouple_mode = FLASHLIGHT_SCENARIO_COUPLE;
#if 0
	ocp81373_keepstate_decouple_mode = FLASHLIGHT_SCENARIO_KEEPSTATE_COUPLE;
#endif
	ocp81373_keepstate_decouple_mode = 0;
	ocp81373_disable();
	ocp81373_pinctrl_set(OCP81373_PINCTRL_PIN_HWEN,
		OCP81373_PINCTRL_PINSTATE_LOW);

	return 0;
}
//ITD:modify by camera driver team end
/******************************************************************************
 * Timer and work queue
 *****************************************************************************/

/******************************************************************************
 * Timer and work queue
 *****************************************************************************/
static void ocp81373_work_disable_ch1(struct work_struct *data)
{
	pr_debug("ht work queue callback\n");
	ocp81373_disable();
}

static void ocp81373_work_disable_ch2(struct work_struct *data)
{
	pr_debug("lt work queue callback\n");
	ocp81373_disable();
}

static enum hrtimer_restart ocp81373_timer_func_ch1(struct hrtimer *timer)
{
	schedule_work(&ocp81373_work_ch1);
	return HRTIMER_NORESTART;
}

static enum hrtimer_restart ocp81373_timer_func_ch2(struct hrtimer *timer)
{
	schedule_work(&ocp81373_work_ch2);
	return HRTIMER_NORESTART;
}

static int ocp81373_timer_start(int channel, ktime_t ktime)
{
	if (channel == OCP81373_CHANNEL_CH1)
		hrtimer_start(&ocp81373_timer_ch1, ktime, HRTIMER_MODE_REL);
	else if (channel == OCP81373_CHANNEL_CH2)
		hrtimer_start(&ocp81373_timer_ch2, ktime, HRTIMER_MODE_REL);
	else {
		pr_err("Error channel\n");
		return -1;
	}

	return 0;
}

static int ocp81373_timer_cancel(int channel)
{
	if (channel == OCP81373_CHANNEL_CH1)
		hrtimer_cancel(&ocp81373_timer_ch1);
	else if (channel == OCP81373_CHANNEL_CH2)
		hrtimer_cancel(&ocp81373_timer_ch2);
	else {
		pr_err("Error channel\n");
		return -1;
	}

	return 0;
}

//ITD:modify by camera driver team start
static int ocp81373_operate(int channel, int enable)
{
	ktime_t ktime;
	unsigned int s;
	unsigned int ns;

	/* setup enable/disable */
	if (channel == OCP81373_CHANNEL_CH1) {
		ocp81373_en_ch1 = enable;
		if (ocp81373_en_ch1 && ocp81373_para[channel].flashmode)
			ocp81373_en_ch1 = OCP81373_ENABLE_FLASH;
	} else if (channel == OCP81373_CHANNEL_CH2) {
		ocp81373_en_ch2 = enable;
		if (ocp81373_en_ch2 && ocp81373_para[channel].flashmode)
			ocp81373_en_ch2 = OCP81373_ENABLE_FLASH;
	} else {
		pr_err("Error channel\n");
		return -1;
	}

	/* decouple mode */
	if (ocp81373_decouple_mode) {
		if (channel == OCP81373_CHANNEL_CH1) {
			ocp81373_en_ch2 = OCP81373_DISABLE;
			ocp81373_timeout_ms[OCP81373_CHANNEL_CH2] = 0;
		} else if (channel == OCP81373_CHANNEL_CH2) {
			ocp81373_en_ch1 = OCP81373_DISABLE;
			ocp81373_timeout_ms[OCP81373_CHANNEL_CH1] = 0;
		}
	}

	/* operate flashlight and setup timer */
	if ((ocp81373_en_ch1 != OCP81373_NONE) && (ocp81373_en_ch2 != OCP81373_NONE)) {
		if ((ocp81373_en_ch1 == OCP81373_DISABLE) &&
				(ocp81373_en_ch2 == OCP81373_DISABLE)) {
			ocp81373_disable();
			ocp81373_timer_cancel(OCP81373_CHANNEL_CH1);
			ocp81373_timer_cancel(OCP81373_CHANNEL_CH2);
		} else {
			if (ocp81373_timeout_ms[OCP81373_CHANNEL_CH1] &&
				ocp81373_en_ch1 != OCP81373_DISABLE) {
				s = ocp81373_timeout_ms[OCP81373_CHANNEL_CH1] /
					1000;
				ns = ocp81373_timeout_ms[OCP81373_CHANNEL_CH1] %
					1000 * 1000000;
				ktime = ktime_set(s, ns);
				ocp81373_timer_start(OCP81373_CHANNEL_CH1, ktime);
			}
			if (ocp81373_timeout_ms[OCP81373_CHANNEL_CH2] &&
				ocp81373_en_ch2 != OCP81373_DISABLE) {
				s = ocp81373_timeout_ms[OCP81373_CHANNEL_CH2] /
					1000;
				ns = ocp81373_timeout_ms[OCP81373_CHANNEL_CH2] %
					1000 * 1000000;
				ktime = ktime_set(s, ns);
				ocp81373_timer_start(OCP81373_CHANNEL_CH2, ktime);
			}
			ocp81373_enable(channel);
		}

		/* clear flashlight state */
//ITD:modify KBSHLES-43 by quan.chang 180919 start
		if ((ocp81373_keepstate_decouple_mode == 0)
				|| (channel != OCP81373_CHANNEL_CH1))
			ocp81373_en_ch1 = OCP81373_NONE;
		if ((ocp81373_keepstate_decouple_mode == 0)
				|| (channel != OCP81373_CHANNEL_CH2))
			ocp81373_en_ch2 = OCP81373_NONE;
//ITD:modify KBSHLES-43 by quan.chang 180919 end
	}

	return 0;
}
//ITD:modify by camera driver team end

/******************************************************************************
 * Flashlight operations
 *****************************************************************************/
static int ocp81373_ioctl(unsigned int cmd, unsigned long arg)
{
	struct flashlight_dev_arg *fl_arg;
	int channel;

	fl_arg = (struct flashlight_dev_arg *)arg;
	channel = fl_arg->channel;

	switch (cmd) {
	case FLASH_IOC_SET_TIME_OUT_TIME_MS:
		pr_debug("FLASH_IOC_SET_TIME_OUT_TIME_MS(%d): %d\n",
				channel, (int)fl_arg->arg);
		ocp81373_timeout_ms[channel] = fl_arg->arg;
		break;

	case FLASH_IOC_SET_DUTY:
		pr_debug("FLASH_IOC_SET_DUTY(%d): %d\n",
				channel, (int)fl_arg->arg);
		ocp81373_set_level(channel,fl_arg->arg);
		break;
#if 0
//ITD:modify by camera driver team start
	case FLASH_IOC_SET_CURRENT:
		pr_debug("FLASH_IOC_SET_CURRENT(%d): %d\n",
				channel, (int)fl_arg->arg);
		{
			int max_flash_current = ocp81373_para[channel].max_flash_current;
			fl_arg->arg = (fl_arg->arg > max_flash_current) ? max_flash_current : fl_arg->arg;
			if(fl_arg->arg >  ocp81373_para[channel].max_torch_current){
				ocp81373_para[channel].flashmode = 1;
				ocp81373_set_strobe_brightness(channel,mapcurrent2regcode(channel,fl_arg->arg));
			}else{
				ocp81373_para[channel].flashmode = 0;
				ocp81373_set_torch_brightness(channel,mapcurrent2regcode(channel,fl_arg->arg));
			}
		}
		break;
	case FLASH_IOC_GENERATE_DEV_PARA:
		pr_debug("FLASH_IOC_GENERATE_DEV_PARA(%d): %d\n",
				channel, (int)fl_arg->arg);
		generate_dev_para(channel,fl_arg->arg);
		break;
//ITD:modify by camera driver team end
#endif
	case FLASH_IOC_SET_SCENARIO:
		pr_debug("FLASH_IOC_SET_SCENARIO(%d): %d\n",
				channel, (int)fl_arg->arg);
		ocp81373_set_scenario(fl_arg->arg);
		break;
	case FLASH_IOC_SET_ONOFF:
		pr_debug("FLASH_IOC_SET_ONOFF(%d): %d\n",
				channel, (int)fl_arg->arg);
		ocp81373_operate(channel, fl_arg->arg);
		break;
	default:
		pr_info("No such command and arg(%d): (%d, %d)\n",
				channel, _IOC_NR(cmd), (int)fl_arg->arg);
		return -ENOTTY;
	}

	return 0;
}

static int ocp81373_open(void)
{
	return 0;
}

static int ocp81373_release(void)
{
	/* uninit chip and clear usage count */
	mutex_lock(&ocp81373_mutex);
	use_count--;
	if (!use_count)
		ocp81373_uninit();
	if (use_count < 0)
		use_count = 0;
	mutex_unlock(&ocp81373_mutex);

	pr_info("Release: %d\n", use_count);

	return 0;
}

static int ocp81373_set_driver(int set)
{
	int ret = 0;
	/* init chip and set usage count */
	mutex_lock(&ocp81373_mutex);
	if (set) {
		if (!use_count)
			ret = ocp81373_init();
		use_count++;
		pr_debug("Set driver: %d\n", use_count);
	} else {
		use_count--;
		if (!use_count)
			ret = ocp81373_uninit();
		if (use_count < 0)
			use_count = 0;
		pr_debug("Unset driver: %d\n", use_count);
	}
	mutex_unlock(&ocp81373_mutex);

	return 0;
}

static ssize_t ocp81373_strobe_store(struct flashlight_arg arg)
{
	ocp81373_set_driver(1);
	ocp81373_set_level(arg.channel, arg.level);
	ocp81373_enable(arg.channel);
	mdelay(arg.dur);
	ocp81373_disable();
	ocp81373_set_driver(0);
	// ocp81373_release();

	return 0;
}

static struct flashlight_operations ocp81373_ops = {
	ocp81373_open,
	ocp81373_release,
	ocp81373_ioctl,
	ocp81373_strobe_store,
	ocp81373_set_driver
};


/******************************************************************************
 * I2C device and driver
 *****************************************************************************/
static int ocp81373_chip_init(struct ocp81373_chip_data *chip)
{
	/* NOTE: Chip initialication move to "set driver" operation
	 * ocp81373_init();
	 */

	return 0;
}

static int ocp81373_i2c_probe(struct i2c_client *client,
	const struct i2c_device_id *id)
{
	struct ocp81373_chip_data *chip;
	struct ocp81373_platform_data *pdata = client->dev.platform_data;
	int err;

	pr_info("%s start.\n", __func__);
	/* check i2c */
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		pr_err("Failed to check i2c functionality.\n");
		err = -ENODEV;
		goto err_out;
	}
	/* init chip private data */
	chip = kzalloc(sizeof(struct ocp81373_chip_data), GFP_KERNEL);
	if (!chip) {
		err = -ENOMEM;
		goto err_out;
	}
	chip->client = client;

	/* init platform data */
	if (!pdata) {
		pr_debug("Platform data does not exist\n");
		pdata = kzalloc(sizeof(struct ocp81373_platform_data),
			GFP_KERNEL);
		chip->no_pdata = 1;
	}
	chip->pdata = pdata;
	i2c_set_clientdata(client, chip);
	OCP81373_i2c_client = client;

	/* init mutex and spinlock */
	mutex_init(&chip->lock);

	/* init work queue */
	INIT_WORK(&ocp81373_work_ch1, ocp81373_work_disable_ch1);
	INIT_WORK(&ocp81373_work_ch2, ocp81373_work_disable_ch2);

	/* init timer */
	hrtimer_init(&ocp81373_timer_ch1, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	ocp81373_timer_ch1.function = ocp81373_timer_func_ch1;
	hrtimer_init(&ocp81373_timer_ch2, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	ocp81373_timer_ch2.function = ocp81373_timer_func_ch2;
	ocp81373_timeout_ms[OCP81373_CHANNEL_CH1] = 100;
	ocp81373_timeout_ms[OCP81373_CHANNEL_CH2] = 100;
	/* init chip hw */
	ocp81373_chip_init(chip);
	/* register flashlight operations */
	if (flashlight_dev_register(OCP81373_NAME, &ocp81373_ops)) {
		err = -EFAULT;
		goto err_free;
	}
	/* clear usage count */
	use_count = 0;
	pr_info("Probe done.\n");

	return 0;

err_free:
	kfree(chip->pdata);
	i2c_set_clientdata(client, NULL);
	kfree(chip);
err_out:
	return err;
}

static int ocp81373_i2c_remove(struct i2c_client *client)
{
	struct ocp81373_chip_data *chip = i2c_get_clientdata(client);

	pr_info("Remove start.\n");

	/* flush work queue */
	flush_work(&ocp81373_work_ch1);
	flush_work(&ocp81373_work_ch2);
	/* unregister flashlight operations */
	flashlight_dev_unregister(OCP81373_NAME);

	/* free resource */
	if (chip->no_pdata)
		kfree(chip->pdata);
	kfree(chip);

	pr_info("Remove done.\n");

	return 0;
}

static const struct i2c_device_id ocp81373_i2c_id[] = {
	{OCP81373_NAME, 0},
	{}
};
MODULE_DEVICE_TABLE(i2c, ocp81373_i2c_id);

#ifdef CONFIG_OF
static const struct of_device_id ocp81373_i2c_of_match[] = {
	{.compatible = OCP81373_DTNAME_I2C},
	{},
};
MODULE_DEVICE_TABLE(of, ocp81373_i2c_of_match);
#endif

static struct i2c_driver ocp81373_i2c_driver = {
	.driver = {
		   .name = OCP81373_NAME,
#ifdef CONFIG_OF
		   .of_match_table = ocp81373_i2c_of_match,
#endif
		   },
	.probe = ocp81373_i2c_probe,
	.remove = ocp81373_i2c_remove,
	.id_table = ocp81373_i2c_id,
};

//ITD:modify by camera driver team start
/******************************************************************************
 * Platform device and driver
 *****************************************************************************/
static int ocp81373_probe(struct platform_device *dev)
{
	int i = 0;
	pr_info("ocp81373_platform_probe start.\n");
	/* init pinctrl */
	if (ocp81373_pinctrl_init(dev)) {
		pr_debug("Failed to init pinctrl.\n");
		return -1;
	}

	for(;i<OCP81373_CHANNEL_NUM;i++){
		ocp81373_para[i].min_flash_duty = 30;//must
		ocp81373_para[i].flashmode = 0;//must
		ocp81373_para[i].max_torch_current = 372;//config according to datasheet
		ocp81373_para[i].max_flash_current = 1500;//config according to datasheet
		ocp81373_para[i].duty_num = 0;//must
		memset(&ocp81373_para[i].duty_reg_code[0], 0, sizeof(ocp81373_para[i].duty_reg_code));
	}

	if (i2c_add_driver(&ocp81373_i2c_driver)) {
		pr_debug("Failed to add i2c driver.\n");
		return -1;
	}

	generate_dev_para(OCP81373_CHANNEL_CH1, 0);
	generate_dev_para(OCP81373_CHANNEL_CH2, 0);
	pr_info("%s done.\n", __func__);

	return 0;
}
//ITD:modify by camera driver team end

static int ocp81373_remove(struct platform_device *dev)
{
	pr_debug("Remove start.\n");

	i2c_del_driver(&ocp81373_i2c_driver);

	pr_debug("Remove done.\n");

	return 0;
}

#ifdef CONFIG_OF
static const struct of_device_id ocp81373_of_match[] = {
	{.compatible = OCP81373_DTNAME},
	{},
};
MODULE_DEVICE_TABLE(of, ocp81373_of_match);
#else
static struct platform_device ocp81373_platform_device = {

		.name = OCP81373_NAME,
		.id = 0,
		.dev = {}

};
MODULE_DEVICE_TABLE(platform, ocp81373_platform_device);
#endif

static struct platform_driver ocp81373_platform_driver = {
	.probe = ocp81373_probe,
	.remove = ocp81373_remove,
	.driver = {
		.name = OCP81373_NAME,
		.owner = THIS_MODULE,
#ifdef CONFIG_OF
		.of_match_table = ocp81373_of_match,
#endif
	},
};

static int __init flashlight_ocp81373_init(void)
{
	int ret;

	pr_info("flashlight_ocp81373_initInit start.\n");

#ifndef CONFIG_OF
	ret = platform_device_register(&ocp81373_platform_device);
	if (ret) {
		pr_info("Failed to register platform device\n");
		return ret;
	}
#endif

	ret = platform_driver_register(&ocp81373_platform_driver);
	if (ret) {
		pr_info("Failed to register platform driver\n");
		return ret;
	}

	pr_debug("Init done.\n");

	return 0;
}

static void __exit flashlight_ocp81373_exit(void)
{
	pr_debug("Exit start.\n");

	platform_driver_unregister(&ocp81373_platform_driver);

	pr_debug("Exit done.\n");
}

module_init(flashlight_ocp81373_init);
module_exit(flashlight_ocp81373_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Simon Wang <Simon-TCH.Wang@mediatek.com>");
MODULE_DESCRIPTION("MTK Flashlight OCP81373 Driver");
