/*
 * Author: yucong xiong <yucong.xion@mediatek.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/irq.h>
#include <linux/miscdevice.h>
#include <linux/uaccess.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/workqueue.h>
#include <linux/kobject.h>
#include <linux/platform_device.h>
#include <linux/atomic.h>

#include <linux/io.h>
#include "epl2182.h"
#include <linux/gpio.h>
#include <linux/of_irq.h>

#include <linux/sched.h>

#include <alsps.h>
/******************************************************************************
 * extern functions
 *******************************************************************************/

#define COMPATIABLE_NAME "mediatek,epl2182"

/* TODO: change ps/als integrationtime */
int PS_INTT = 4;
int ALS_INTT = 7;
unsigned int alsps_int_gpio_number;

#define TXBYTES 2
#define RXBYTES 2
#define PACKAGE_SIZE 2
#define I2C_RETRY_COUNT 3

/* TODO: change delay time */
#define PS_DELAY 15
#define ALS_DELAY 51

/* TODO: parameters for lux equation y = ax + b */
#define LUX_PER_COUNT 1100 /* 1100 = 1.1 * 1000 */

#define IPI_WAIT_RSP_TIMEOUT (HZ / 10) /* 100ms */

static DEFINE_MUTEX(epl2182_mutex);

struct epl_raw_data
{
	u8 raw_bytes[PACKAGE_SIZE];
	u16 ps_raw;
	u16 ps_state;
	u16 ps_int_state;
	u16 als_ch0_raw;
	u16 als_ch1_raw;
};

#define EPL2182_DEV_NAME "EPL2182_NEW"

/*----------------------------------------------------------------------------*/
#define APS_TAG "[EPL2182] "
#define APS_FUN(f)
#define APS_ERR(fmt, args...)
#define APS_LOG(fmt, args...)
#define APS_DBG(fmt, args...)
#define FTM_CUST_ALSPS "/data/epl2182"

static struct i2c_client *epl2182_i2c_client;

/*----------------------------------------------------------------------------*/
static const struct i2c_device_id epl2182_i2c_id[] = {{"EPL2182", 0}, {}};

/*----------------------------------------------------------------------------*/
static int epl2182_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id);
static int epl2182_i2c_remove(struct i2c_client *client);
static int epl2182_i2c_detect(struct i2c_client *client, struct i2c_board_info *info);
static int alsps_local_init(void);
static int alsps_remove(void);
/*----------------------------------------------------------------------------*/
static irqreturn_t epl2182_eint_func(int irq, void *desc);
static int set_psensor_intr_threshold(uint16_t low_thd, uint16_t high_thd);
static int set_psensor_threshold(struct i2c_client *client);

static struct epl2182_priv *g_epl2182_ptr;
static bool isInterrupt = true;

static long long int_top_time;
static int int_flag;

/*----------------------------------------------------------------------------*/
enum CMC_TRC
{
	CMC_TRC_ALS_DATA = 0x0001,
	CMC_TRC_PS_DATA = 0X0002,
	CMC_TRC_EINT = 0x0004,
	CMC_TRC_IOCTL = 0x0008,
	CMC_TRC_I2C = 0x0010,
	CMC_TRC_CVT_ALS = 0x0020,
	CMC_TRC_CVT_PS = 0x0040,
	CMC_TRC_DEBUG = 0x0800,
};

/*----------------------------------------------------------------------------*/
enum CMC_BIT
{
	CMC_BIT_ALS = 1,
	CMC_BIT_PS = 2,
};

/*----------------------------------------------------------------------------*/
struct epl2182_i2c_addr
{ /*define a series of i2c slave address */
	u8 write_addr;
	u8 ps_thd; /*PS INT threshold */
};

/*----------------------------------------------------------------------------*/
struct epl2182_priv
{
	struct alsps_hw hw;
	struct i2c_client *client;
	struct work_struct eint_work;
	struct work_struct data_work;
	/*i2c address group */
	struct epl2182_i2c_addr addr;

	int enable_pflag;
	int enable_lflag;
	struct device_node *irq_node;
	int irq;

	/*misc */
	atomic_t trace;
	atomic_t i2c_retry;
	atomic_t als_suspend;
	atomic_t als_debounce; /*debounce time after enabling als */
	atomic_t als_deb_on;   /*indicates if the debounce is on */
	atomic_t als_deb_end;  /*the jiffies representing the end of debounce */
	atomic_t ps_mask;	   /*mask ps: always return far away */
	atomic_t ps_debounce;  /*debounce time after enabling ps */
	atomic_t ps_deb_on;	   /*indicates if the debounce is on */
	atomic_t ps_deb_end;   /*the jiffies representing the end of debounce */
	atomic_t ps_suspend;

	/*data */
	u16 als;
	u16 ps;
	u16 lux_per_count;
	bool als_enable;	/*record current als status */
	bool ps_enable;		/*record current ps status */
	ulong enable;		/*record HAL enalbe status */
	ulong pending_intr; /*pending interrupt */
	/* ulong        first_read;   // record first read ps and als */

	/*data */
	u16 als_level_num;
	u16 als_value_num;
	u32 als_level[C_CUST_ALS_LEVEL - 1];
	u32 als_value[C_CUST_ALS_LEVEL];
	int ps_cali;

	atomic_t ps_thd_val_high; /*the cmd value can't be read, stored in ram */
	atomic_t ps_thd_val_low;  /*the cmd value can't be read, stored in ram */
};

#ifdef CONFIG_OF
static const struct of_device_id alsps_of_match[] = {
	{.compatible = "mediatek,alsps"},
	{},
};
#endif

/*----------------------------------------------------------------------------*/
static struct i2c_driver epl2182_i2c_driver = {
	.probe = epl2182_i2c_probe,
	.remove = epl2182_i2c_remove,
	.detect = epl2182_i2c_detect,
	.id_table = epl2182_i2c_id,
	.driver = {
		.name = EPL2182_DEV_NAME,
#ifdef CONFIG_OF
		.of_match_table = alsps_of_match,
#endif
	},
};

static struct epl2182_priv *epl2182_obj;
static struct epl_raw_data gRawData;

static int alsps_init_flag = -1; /* 0<==>OK -1 <==> fail */

static struct alsps_init_info epl2182_init_info = {
	.name = EPL2182_DEV_NAME,
	.init = alsps_local_init,
	.uninit = alsps_remove,

};

static DECLARE_WAIT_QUEUE_HEAD(wait_rsp_wq);

static int elan_epl2182_I2C_Write(struct i2c_client *client, uint8_t regaddr, uint8_t bytecount,
								  uint8_t txbyte, uint8_t data)
{
	uint8_t buffer[2];
	int ret = 0;
	int retry;

	/* APS_DBG("[ELAN epl2182] %s\n", __func__); */
	mutex_lock(&epl2182_mutex);
	buffer[0] = (regaddr << 3) | bytecount;
	buffer[1] = data;

	for (retry = 0; retry < I2C_RETRY_COUNT; retry++)
	{
		ret = i2c_master_send(client, buffer, txbyte);
		if (ret >= 0)
			break;

		APS_DBG("epl2182 i2c write error,TXBYTES %d\r\n", ret);
		mdelay(10);
	}

	if (retry >= I2C_RETRY_COUNT)
	{
		mutex_unlock(&epl2182_mutex);
		APS_DBG(KERN_ERR "[ELAN epl2182 error] %s i2c write retry over %d\n", __func__,
				I2C_RETRY_COUNT);
		return -EINVAL;
	}
	mutex_unlock(&epl2182_mutex);
	return ret;
}

/*
//====================I2C read operation===============//
*/
static int elan_epl2182_I2C_Read(struct i2c_client *client, uint8_t regaddr, uint8_t bytecount,
								 uint8_t rxbyte, uint8_t *data)
{
	uint8_t buffer[RXBYTES];
	int ret = 0, i = 0;
	int retry;

	/* APS_DBG("[ELAN epl2182] %s\n", __func__); */
	mutex_lock(&epl2182_mutex);
	buffer[0] = (regaddr << 3) | bytecount;

	for (retry = 0; retry < I2C_RETRY_COUNT; retry++)
	{
		ret = hwmsen_read_block(client, buffer[0], buffer, rxbyte);
		if (ret >= 0)
			break;

		APS_ERR("epl2182 i2c read error,RXBYTES %d\r\n", ret);
		mdelay(10);
	}

	if (retry >= I2C_RETRY_COUNT)
	{
		APS_ERR(KERN_ERR "[ELAN epl2182 error] %s i2c read retry over %d\n", __func__,
				I2C_RETRY_COUNT);
		mutex_unlock(&epl2182_mutex);
		return -EINVAL;
	}

	for (i = 0; i < PACKAGE_SIZE; i++)
		*data++ = buffer[i];
	mutex_unlock(&epl2182_mutex);

	return ret;
}

static int elan_epl2182_psensor_enable(struct epl2182_priv *epl_data, int enable)
{
	int ret = 0;

	uint8_t regdata;
	uint8_t read_data[2];
	int ps_state;
	struct i2c_client *client = epl_data->client;

	APS_DBG("[ELAN epl2182] %s enable = %d\n", __func__, enable);

	epl_data->enable_pflag = enable;
	ret = elan_epl2182_I2C_Write(client, REG_9, W_SINGLE_BYTE, 0x02,
								 EPL_INT_DISABLE | EPL_DRIVE_120MA);

	if (enable)
	{
		regdata = EPL_SENSING_2_TIME | EPL_PS_MODE | EPL_L_GAIN;
		regdata = regdata | (isInterrupt ? EPL_C_SENSING_MODE : EPL_S_SENSING_MODE);
		ret = elan_epl2182_I2C_Write(client, REG_0, W_SINGLE_BYTE, 0X02, regdata);

		regdata = PS_INTT << 4 | EPL_PST_1_TIME | EPL_10BIT_ADC;
		ret = elan_epl2182_I2C_Write(client, REG_1, W_SINGLE_BYTE, 0X02, regdata);

		ret = elan_epl2182_I2C_Write(client, REG_7, W_SINGLE_BYTE, 0X02, EPL_C_RESET);
		ret = elan_epl2182_I2C_Write(client, REG_7, W_SINGLE_BYTE, 0x02, EPL_C_START_RUN);

		msleep(PS_DELAY);

		set_psensor_threshold(client);

		elan_epl2182_I2C_Write(client, REG_13, R_SINGLE_BYTE, 0x01, 0);
		ret = elan_epl2182_I2C_Read(client, REG_13, R_SINGLE_BYTE, 0x01, read_data);

		ps_state = !((read_data[0] & 0x04) >> 2);

		APS_DBG("epl2182 ps state = %d, gRawData.ps_state = %d, %s\n", ps_state,
				gRawData.ps_state, __func__);

		int_flag = ps_state;

		schedule_work(&epl_data->data_work);

		gRawData.ps_state = ps_state; /* update ps state */

		ret = elan_epl2182_I2C_Write(client, REG_9, W_SINGLE_BYTE, 0x02, EPL_INT_ACTIVE_LOW | EPL_DRIVE_120MA);
	}
	else
	{
		regdata = EPL_SENSING_2_TIME | EPL_PS_MODE | EPL_L_GAIN;
		regdata |= EPL_S_SENSING_MODE;
		ret = elan_epl2182_I2C_Write(client, REG_0, W_SINGLE_BYTE, 0X02, regdata);
		ret = elan_epl2182_I2C_Write(client, REG_9, W_SINGLE_BYTE, 0x02, EPL_INT_DISABLE | EPL_DRIVE_120MA);
	}

	if (ret < 0)
		APS_ERR("[ELAN epl2182 error]%s: ps enable %d fail\n", __func__, ret);
	else
		ret = 0;

	return ret;
}

static int elan_epl2182_lsensor_enable(struct epl2182_priv *epl_data, int enable)
{
	int ret = 0;
	uint8_t regdata;
	struct i2c_client *client = epl_data->client;

	APS_LOG("[ELAN epl2182] %s enable = %d\n", __func__, enable);
	epl_data->enable_lflag = enable;

	if (enable)
	{
		regdata = EPL_INT_DISABLE;
		ret = elan_epl2182_I2C_Write(client, REG_9, W_SINGLE_BYTE, 0x02, regdata);

		regdata = EPL_S_SENSING_MODE | EPL_SENSING_8_TIME | EPL_ALS_MODE | EPL_AUTO_GAIN;
		ret = elan_epl2182_I2C_Write(client, REG_0, W_SINGLE_BYTE, 0X02, regdata);

		regdata = ALS_INTT << 4 | EPL_PST_1_TIME | EPL_10BIT_ADC;
		ret = elan_epl2182_I2C_Write(client, REG_1, W_SINGLE_BYTE, 0X02, regdata);

		ret = elan_epl2182_I2C_Write(client, REG_10, W_SINGLE_BYTE, 0X02, 0x1e);
		ret = elan_epl2182_I2C_Write(client, REG_11, W_SINGLE_BYTE, 0x02, 0x1e);

		ret = elan_epl2182_I2C_Write(client, REG_7, W_SINGLE_BYTE, 0X02, EPL_C_RESET);
		ret = elan_epl2182_I2C_Write(client, REG_7, W_SINGLE_BYTE, 0x02, EPL_C_START_RUN);
		msleep(ALS_DELAY);
	}

	if (ret < 0)
		APS_ERR("[ELAN epl2182 error]%s: als_enable %d fail\n", __func__, ret);
	else
		ret = 0;

	return ret;
}

/* convert raw to lux */
static int epl2182_get_als_value(struct epl2182_priv *obj, u16 als)
{
	int idx;
	int invalid = 0;
	int lux = 0;

	if (als < 15)
		return 0;

	lux = (als * obj->lux_per_count) / 1000;

	for (idx = 0; idx < obj->als_level_num; idx++)
	{
		if (lux < obj->hw.als_level[idx])
			break;
	}

	if (idx >= obj->als_value_num)
	{
		APS_ERR("epl2182 exceed range\n");
		idx = obj->als_value_num - 1;
	}

	if (1 == atomic_read(&obj->als_deb_on))
	{
		unsigned long endt = atomic_read(&obj->als_deb_end);

		if (time_after(jiffies, endt))
			atomic_set(&obj->als_deb_on, 0);

		if (1 == atomic_read(&obj->als_deb_on))
			invalid = 1;
	}

	if (!invalid)
	{
#if defined(MTK_AAL_SUPPORT)
		int level_high = obj->hw.als_level[idx];
		int level_low = (idx > 0) ? obj->hw.als_level[idx - 1] : 0;
		int level_diff = level_high - level_low;
		int value_high = obj->hw.als_value[idx];
		int value_low = (idx > 0) ? obj->hw.als_value[idx - 1] : 0;
		int value_diff = value_high - value_low;
		int value = 0;

		if ((level_low >= level_high) || (value_low >= value_high))
			value = value_low;
		else
			value =
				(level_diff * value_low + (als - level_low) * value_diff +
				 ((level_diff + 1) >> 1)) /
				level_diff;

		return value;
#endif
		/* APS_DBG("ALS: %05d => %05d\n", als, obj->hw.als_value[idx]); */
		return obj->hw.als_value[idx];
	}
	APS_ERR("ALS: %05d => %05d (-1)\n", als, obj->hw.als_value[idx]);
	return -1;
}

static int set_psensor_intr_threshold(uint16_t low_thd, uint16_t high_thd)
{
	int ret = 0;
	struct epl2182_priv *epld = epl2182_obj;
	struct i2c_client *client = epld->client;
	uint8_t high_msb, high_lsb, low_msb, low_lsb;

	/* APS_LOG("epl2182 %s: low_thd = 0x%X, high_thd = 0x%x\n",__func__, low_thd, high_thd); */

	high_msb = (uint8_t)(high_thd >> 8);
	high_lsb = (uint8_t)(high_thd & 0x00ff);
	low_msb = (uint8_t)(low_thd >> 8);
	low_lsb = (uint8_t)(low_thd & 0x00ff);

	elan_epl2182_I2C_Write(client, REG_2, W_SINGLE_BYTE, 0x02, high_lsb);
	elan_epl2182_I2C_Write(client, REG_3, W_SINGLE_BYTE, 0x02, high_msb);
	elan_epl2182_I2C_Write(client, REG_4, W_SINGLE_BYTE, 0x02, low_lsb);
	elan_epl2182_I2C_Write(client, REG_5, W_SINGLE_BYTE, 0x02, low_msb);

	return ret;
}

/*----------------------------------------------------------------------------*/
int hw8k_init_device(struct i2c_client *client)
{
	APS_LOG("hw8k_init_device.........\r\n");

	epl2182_i2c_client = client;

	APS_LOG("epl2182 I2C Addr==[0x%x],line=%d\n", epl2182_i2c_client->addr, __LINE__);

	return 0;
}

/*----------------------------------------------------------------------------*/
int epl2182_get_addr(struct alsps_hw *hw, struct epl2182_i2c_addr *addr)
{
	if (!hw || !addr)
		return -EFAULT;
	addr->write_addr = hw->i2c_addr[0];
	return 0;
}

/*----------------------------------------------------------------------------*/

int epl2182_read_als(struct i2c_client *client, u16 *data)
{
	struct epl2182_priv *obj = i2c_get_clientdata(client);
	uint8_t read_data[2];

	if (client == NULL)
	{
		APS_DBG("CLIENT CANN'T EQUL NULL\n");
		return -1;
	}

	elan_epl2182_I2C_Read(obj->client, REG_14, R_TWO_BYTE, 0x02, read_data);
	gRawData.als_ch0_raw = (read_data[1] << 8) | read_data[0];
	elan_epl2182_I2C_Read(obj->client, REG_16, R_TWO_BYTE, 0x02, read_data);
	gRawData.als_ch1_raw = (read_data[1] << 8) | read_data[0];
	*data = gRawData.als_ch1_raw;

	/*APS_ERR("epl2182 read als raw data = %d\n", gRawData.als_ch1_raw); */

	return 0;
}

/*----------------------------------------------------------------------------*/
long epl2182_read_ps(struct i2c_client *client, u16 *data)
{
	struct epl2182_priv *obj = i2c_get_clientdata(client);

	uint8_t read_data[2];

	if (client == NULL)
	{
		APS_DBG("CLIENT CANN'T EQUL NULL\n");
		return -1;
	}
	/* elan_epl2182_I2C_Read(client,REG_13,R_SINGLE_BYTE,0x01,read_data); */
	/* APS_DBG("epl2182_read_als read REG_13 raw_bytes: 0x%x\n",read_data[0]); */
	/* setting = read_data[0]; */
	/* if((setting&(3<<4))!=0x10) */
	/* { */
	/* APS_ERR("epl2182 read ps data in wrong mode\n"); */
	/* } */

	/* gRawData.ps_state= !((read_data[0]&0x04)>>2); */
	/* APS_LOG("epl2182 ps state = %d, %s\n", gRawData.ps_state, __func__); */

	elan_epl2182_I2C_Read(obj->client, REG_16, R_TWO_BYTE, 0x02, read_data);
	gRawData.ps_raw = (read_data[1] << 8) | read_data[0];

	if (gRawData.ps_raw < obj->ps_cali)
		*data = 0;
	else
		*data = gRawData.ps_raw - obj->ps_cali;

	// INIT_WORK(&obj->data_work, epl2182_check_ps_data);

	/* APS_LOG("epl2182 read ps raw data = %d\n", gRawData.ps_raw); */
	/* APS_LOG("epl2182 read ps binary data = %d\n", gRawData.ps_state); */

	return 0;
}

static int epl2182_check_intr(struct i2c_client *client)
{
	struct epl2182_priv *obj = i2c_get_clientdata(client);
	uint8_t read_data[1];
	int mode;

	gpio_direction_input(alsps_int_gpio_number);
	if (gpio_get_value(alsps_int_gpio_number) == 1) /*skip if no interrupt */
		return 0;

	elan_epl2182_I2C_Write(obj->client, REG_13, R_SINGLE_BYTE, 0x01, 0);
	elan_epl2182_I2C_Read(obj->client, REG_13, R_SINGLE_BYTE, 0x01, read_data);
	mode = read_data[0] & (3 << 4);
	APS_LOG("mode %d\n", mode);

	if (mode == 0x10)
		set_bit(CMC_BIT_PS, &obj->pending_intr);
	else
		clear_bit(CMC_BIT_PS, &obj->pending_intr);

	APS_LOG("check intr: %lu\n", obj->pending_intr);
	return 0;
}

static irqreturn_t epl2182_eint_func(int irq, void *desc)
{
	struct epl2182_priv *obj = g_epl2182_ptr;

	int_top_time = sched_clock();

	if (!obj)
		return IRQ_HANDLED;

	disable_irq_nosync(epl2182_obj->irq);
	schedule_work(&obj->eint_work);

	return IRQ_HANDLED;
}

static void epl2182_eint_work(struct work_struct *work)
{
	struct epl2182_priv *epld = (struct epl2182_priv *)container_of(work, struct epl2182_priv, eint_work);
	int err;
	uint8_t read_data[2];
	int flag;

	if (NULL == epld)
	{
		APS_ERR("NULL Pointer\n");
		return;
	}

	APS_DBG("%s: epld->enable_pflag = %d\n", __func__, epld->enable_pflag);

	if (epld->enable_pflag == 0)
		goto exit;

	APS_LOG("epl2182 int top half time = %lld\n", int_top_time);

	elan_epl2182_I2C_Write(epld->client, REG_7, W_SINGLE_BYTE, 0x02, EPL_DATA_LOCK);

	elan_epl2182_I2C_Read(epld->client, REG_16, R_TWO_BYTE, 0x02, read_data);
	gRawData.ps_raw = (read_data[1] << 8) | read_data[0];
	APS_LOG("epl2182 ps raw_data = %d\n", gRawData.ps_raw);

	elan_epl2182_I2C_Read(epld->client, REG_13, R_SINGLE_BYTE, 0x01, read_data);
	flag = !((read_data[0] & 0x04) >> 2);
	if (flag != gRawData.ps_state)
	{
		APS_LOG("epl2182 eint work gRawData.ps_state = %d, flag = %d, %s\n",
				gRawData.ps_state, flag, __func__);

		gRawData.ps_state = flag; /* update ps state */

		/* let up layer to know */
		err = ps_report_interrupt_data(flag);
		if (err)
			APS_ERR("epl2182 call ps_report_interrupt_data fail = %d\n", err);
		/* APS_LOG("epl2182 xxxxx eint work\n"); */
		// re_enable ps so that it can report back
	}
	else
	{
		APS_LOG("epl2182 eint data won't update");
	}

exit:
	elan_epl2182_I2C_Write(epld->client, REG_7, W_SINGLE_BYTE, 0x02, EPL_DATA_UNLOCK);

	enable_irq(epl2182_obj->irq);
}

/*----------------------------------------------------------------------------*/
int epl2182_setup_eint(struct i2c_client *client)
{
	struct epl2182_priv *obj = i2c_get_clientdata(client);
	int ret;
	u32 ints[2] = {0, 0};

	APS_LOG("epl2182_setup_eint\n");

	g_epl2182_ptr = obj;

	if (epl2182_obj->irq_node)
	{
		of_property_read_u32_array(epl2182_obj->irq_node, "debounce", ints,
								   ARRAY_SIZE(ints));
		alsps_int_gpio_number = ints[0];
		ret = gpio_request_one(alsps_int_gpio_number, GPIOF_IN, "alsps_int");
		if (ret < 0)
		{
			APS_ERR("Unable to request gpio int_pin\n");
			return -1;
		}

		gpio_set_debounce(ints[0], ints[1]);

		epl2182_obj->irq = irq_of_parse_and_map(epl2182_obj->irq_node, 0);
		APS_LOG("epl2182_obj->irq = %d\n", epl2182_obj->irq);
		if (!epl2182_obj->irq)
		{
			APS_ERR("irq_of_parse_and_map fail!!\n");
			return -EINVAL;
		}

		gpio_direction_input(alsps_int_gpio_number);
		if (request_irq(epl2182_obj->irq, epl2182_eint_func, IRQF_TRIGGER_LOW, "als_ps", NULL))
		{
			APS_ERR("IRQ LINE NOT AVAILABLE!!\n");
			return -EINVAL;
		}
	}
	else
	{
		APS_ERR("null irq node!!\n");
		return -EINVAL;
	}

	enable_irq_wake(epl2182_obj->irq);

	return 0;
}

/*----------------------------------------------------------------------------*/
static int epl2182_init_client(struct i2c_client *client)
{
	int err = 0;

	APS_LOG("epl2182 [Agold spl] I2C Addr==[0x%x],line=%d\n", epl2182_i2c_client->addr,
			__LINE__);

	/*  interrupt mode */

	APS_FUN();

	if (isInterrupt)
	{
		err = epl2182_setup_eint(client);
		if (err)
		{
			APS_ERR("setup eint: %d\n", err);
			return err;
		}
		APS_LOG("epl2182 interrupt setup\n");
	}

	err = hw8k_init_device(client);
	if (err != 0)
	{
		APS_ERR("init dev: %d\n", err);
		return err;
	}

	err = epl2182_check_intr(client);
	if (err != 0)
	{
		APS_ERR("check/clear intr: %d\n", err);
		return err;
	}

	return err;
}

static void epl2182_check_ps_data(struct work_struct *work)
{
	int flag;
	uint8_t read_data[2];
	int err = 0;
	struct epl2182_priv *epld = epl2182_obj;

	APS_ERR("epl2182 check data\n");

	elan_epl2182_I2C_Write(epld->client, REG_7, W_SINGLE_BYTE, 0x02, EPL_DATA_LOCK);
	elan_epl2182_I2C_Read(epld->client, REG_13, R_SINGLE_BYTE, 0x01, read_data);

	flag = !((read_data[0] & 0x04) >> 2);
	APS_ERR("epl2182 %d %d\n", flag, int_flag);

	/* let up layer to know */
	APS_LOG("epl2182 int_flag state = %d, %s\n", int_flag, __func__);
	err = ps_report_interrupt_data(int_flag);
	if (err != 0)
	{
		APS_ERR("epl2182 call ps_report_interrupt_data fail = %d\n", err);
	}

	elan_epl2182_I2C_Write(epld->client, REG_7, W_SINGLE_BYTE, 0x02, EPL_DATA_UNLOCK);
	return;
}

static int set_psensor_threshold(struct i2c_client *client)
{
	struct epl2182_priv *obj = i2c_get_clientdata(client);

	int res = 0;
	int databuf[2];

	APS_LOG("set_psensor_threshold function high: 0x%x, low:0x%x\n",
			atomic_read(&obj->ps_thd_val_high), atomic_read(&obj->ps_thd_val_low));
	databuf[0] = atomic_read(&obj->ps_thd_val_low);
	databuf[1] = atomic_read(&obj->ps_thd_val_high); /* threshold value need to confirm */

	res = set_psensor_intr_threshold(databuf[0], databuf[1]);
	return res;
}

/*--------------------------------------------------------------------------------*/
static int als_open_report_data(int open)
{
	/* should queuq work to report event if  is_report_input_direct=true */
	return 0;
}

/*--------------------------------------------------------------------------------*/
/* if use  this typ of enable , Gsensor only enabled but not report inputEvent to HAL */
static int als_enable_nodata(int en)
{
	int res = 0;

	APS_LOG("epl2182_obj als enable value = %d\n", en);

	if (!epl2182_obj)
	{
		APS_ERR("epl2182_obj is null!!\n");
		return -1;
	}
	APS_LOG("epl2182_obj als enable value = %d\n", en);

	if (en)
	{
		set_bit(CMC_BIT_ALS, &epl2182_obj->enable);
	}
	else
	{
		clear_bit(CMC_BIT_ALS, &epl2182_obj->enable);
		if (epl2182_obj->enable_pflag && isInterrupt)
		{
			if (elan_epl2182_psensor_enable(epl2182_obj, 1) != 0)
			{
				APS_ERR("enable ps fail: \n");
				return -1;
			}
		}
	}

	if (res)
	{
		APS_ERR("als_enable_nodata is failed!!\n");
		return -1;
	}
	return 0;
}

/*--------------------------------------------------------------------------------*/
static int als_set_delay(u64 ns)
{
	return 0;
}

/*--------------------------------------------------------------------------------*/
static int als_get_data(int *value, int *status)
{
	int err = 0;
	if (!epl2182_obj)
	{
		APS_ERR("epl2182_obj is null!!\n");
		return -1;
	}

	if (0 == atomic_read(&epl2182_obj->als_suspend))
	{
		err = elan_epl2182_lsensor_enable(epl2182_obj, 1);
		if (err != 0)
		{
			APS_ERR("enable als fail: %d\n", err);
			return -1;
		}
		epl2182_read_als(epl2182_obj->client, &epl2182_obj->als);

		*value = epl2182_get_als_value(epl2182_obj, epl2182_obj->als);
		*status = SENSOR_STATUS_ACCURACY_MEDIUM;
		/* APS_LOG("get als data->values[0] = %d\n", sensor_data->values[0]); */
	}
	else
	{
		APS_LOG("epl2182 sensor in suspend!\n");
		return -1;
	}

	if (epl2182_obj->enable_pflag && isInterrupt)
	{
		err = elan_epl2182_psensor_enable(epl2182_obj, 1);
		if (err != 0)
		{
			APS_ERR("enable ps fail: %d\n", err);
			return -1;
		}
	}

	return err;
}

/*--------------------------------------------------------------------------------*/
/* if use  this typ of enable , Gsensor should report inputEvent(x, y, z ,stats, div) to HAL */
static int ps_open_report_data(int open)
{
	/* should queuq work to report event if  is_report_input_direct=true */
	return 0;
}

/*--------------------------------------------------------------------------------*/
/* if use  this typ of enable , Gsensor only enabled but not report inputEvent to HAL */
static int ps_enable_nodata(int en)
{
	int res = 0;
	APS_LOG("epl2182_obj als enable value = %d\n", en);

	if (!epl2182_obj)
	{
		APS_ERR("epl2182_obj is null!!\n");
		return -1;
	}
	APS_LOG("epl2182_obj als enable value = %d\n", en);

	if (en)
	{
		if (isInterrupt)
		{
			res = elan_epl2182_psensor_enable(epl2182_obj, 1);
			if (res != 0)
			{
				APS_ERR("enable ps fail: %d\n", res);
				return -1;
			}
		}
		set_bit(CMC_BIT_PS, &epl2182_obj->enable);
	}
	else
	{
		if (isInterrupt)
		{
			res = elan_epl2182_psensor_enable(epl2182_obj, 0);
			if (res != 0)
			{
				APS_ERR("disable ps fail: %d\n", res);
				return -1;
			}
		}
		clear_bit(CMC_BIT_PS, &epl2182_obj->enable);
	}

	if (res)
	{
		APS_ERR("als_enable_nodata is failed!!\n");
		return -1;
	}

	return 0;
}

/*--------------------------------------------------------------------------------*/
static int ps_set_delay(u64 ns)
{
	return 0;
}

/*--------------------------------------------------------------------------------*/
static int ps_get_data(int *value, int *status)
{
	int err = 0;

	if (!epl2182_obj)
	{
		APS_ERR("epl2182_obj is null!!\n");
		return -1;
	}

	APS_ERR("get data with no enable, done \n");

	err = elan_epl2182_psensor_enable(epl2182_obj, 1);
	if (err != 0)
	{
		APS_ERR("enable ps fail: %d\n", err);
		return -1;
	}

	err = epl2182_read_ps(epl2182_obj->client, &epl2182_obj->ps);
	if (err != 0)
	{
		APS_ERR("read ps fail: %d\n", err);
		return -1;
	}
	*value = epl2182_obj->ps;
	*status = SENSOR_STATUS_ACCURACY_HIGH;

	return err;
}

/*----------------------------------------------------------------------------*/

static int epl2182_i2c_detect(struct i2c_client *client, struct i2c_board_info *info)
{
	strcpy(info->type, EPL2182_DEV_NAME);
	return 0;
}

static int als_batch(int flag, int64_t samplingPeriodNs, int64_t maxBatchReportLatencyNs)
{
	int value = 0;

	value = (int)samplingPeriodNs / 1000 / 1000;
	/*FIX ME*/

	APS_LOG("epl2182 als set delay = (%d) ok.\n", value);
	return als_set_delay(samplingPeriodNs);
}

static int als_flush(void)
{
	return als_flush_report();
}

static int ps_batch(int flag, int64_t samplingPeriodNs, int64_t maxBatchReportLatencyNs)
{
	int value = 0;

	value = (int)samplingPeriodNs / 1000 / 1000;
	/*FIX ME*/

	APS_LOG("epl2182 ps setdelay = (%d) ok.\n", value);
	return 0;
}

static int ps_flush(void)
{
	return ps_flush_report();
}

static int epl2182_als_factory_enable_sensor(bool enable_disable, int64_t sample_periods_ms)
{
	int err = 0;

	err = als_enable_nodata(enable_disable ? 1 : 0);
	if (err)
	{
		APS_ERR("%s:%s failed\n", __func__, enable_disable ? "enable" : "disable");
		return -1;
	}
	err = als_batch(0, sample_periods_ms * 1000000, 0);
	if (err)
	{
		APS_ERR("%s set_batch failed\n", __func__);
		return -1;
	}
	return 0;
}
static int epl2182_als_factory_get_data(int32_t *data)
{
	int status = 1;

	return als_get_data(data, &status);
}
static int epl2182_als_factory_get_raw_data(int32_t *data)
{
	int err = 0;
	struct epl2182_priv *obj = epl2182_obj;

	if (!obj)
	{
		APS_ERR("obj is null!!\n");
		return -1;
	}

	err = epl2182_read_als(obj->client, &obj->als);
	if (err)
	{
		APS_ERR("%s failed\n", __func__);
		return -1;
	}
	*data = epl2182_obj->als;

	return 0;
}
static int epl2182_als_factory_enable_calibration(void)
{
	return 0;
}
static int epl2182_als_factory_clear_cali(void)
{
	return 0;
}
static int epl2182_als_factory_set_cali(int32_t offset)
{
	return 0;
}
static int epl2182_als_factory_get_cali(int32_t *offset)
{
	return 0;
}
static int epl2182_ps_factory_enable_sensor(bool enable_disable, int64_t sample_periods_ms)
{
	int err = 0;

	err = ps_enable_nodata(enable_disable ? 1 : 0);
	if (err)
	{
		APS_ERR("%s:%s failed\n", __func__, enable_disable ? "enable" : "disable");
		return -1;
	}
	err = ps_batch(0, sample_periods_ms * 1000000, 0);
	if (err)
	{
		APS_ERR("%s set_batch failed\n", __func__);
		return -1;
	}
	return err;
}
static int epl2182_ps_factory_get_data(int32_t *data)
{
	int status = 0;
	ps_get_data(data, &status);
	return 0;
}
static int epl2182_ps_factory_get_raw_data(int32_t *data)
{
	int err = 0;
	struct epl2182_priv *obj = epl2182_obj;

	err = epl2182_read_ps(obj->client, &obj->ps);
	if (err)
	{
		APS_ERR("%s failed\n", __func__);
		return -1;
	}
	*data = epl2182_obj->ps;
	return 0;
}
static int epl2182_ps_factory_enable_calibration(void)
{
	return 0;
}
static int epl2182_ps_factory_clear_cali(void)
{
	struct epl2182_priv *obj = epl2182_obj;

	obj->ps_cali = 0;
	return 0;
}
static int epl2182_ps_factory_set_cali(int32_t offset)
{
	struct epl2182_priv *obj = epl2182_obj;

	obj->ps_cali = offset;
	return 0;
}
static int epl2182_ps_factory_get_cali(int32_t *offset)
{
	struct epl2182_priv *obj = epl2182_obj;

	*offset = obj->ps_cali;
	return 0;
}
static int epl2182_ps_factory_set_threshold(int32_t threshold[2])
{
	int err = 0;
	struct epl2182_priv *obj = epl2182_obj;

	atomic_set(&obj->ps_thd_val_high, (threshold[0] + obj->ps_cali));
	atomic_set(&obj->ps_thd_val_low, (threshold[1] + obj->ps_cali));
	err = set_psensor_threshold(obj->client);

	if (err < 0)
	{
		APS_ERR("set_psensor_threshold fail\n");
		return -1;
	}
	return 0;
}
static int epl2182_ps_factory_get_threshold(int32_t threshold[2])
{
	struct epl2182_priv *obj = epl2182_obj;

	threshold[0] = atomic_read(&obj->ps_thd_val_high) - obj->ps_cali;
	threshold[1] = atomic_read(&obj->ps_thd_val_low) - obj->ps_cali;
	return 0;
}

static struct alsps_factory_fops epl2182_factory_fops = {
	.als_enable_sensor = epl2182_als_factory_enable_sensor,
	.als_get_data = epl2182_als_factory_get_data,
	.als_get_raw_data = epl2182_als_factory_get_raw_data,
	.als_enable_calibration = epl2182_als_factory_enable_calibration,
	.als_clear_cali = epl2182_als_factory_clear_cali,
	.als_set_cali = epl2182_als_factory_set_cali,
	.als_get_cali = epl2182_als_factory_get_cali,

	.ps_enable_sensor = epl2182_ps_factory_enable_sensor,
	.ps_get_data = epl2182_ps_factory_get_data,
	.ps_get_raw_data = epl2182_ps_factory_get_raw_data,
	.ps_enable_calibration = epl2182_ps_factory_enable_calibration,
	.ps_clear_cali = epl2182_ps_factory_clear_cali,
	.ps_set_cali = epl2182_ps_factory_set_cali,
	.ps_get_cali = epl2182_ps_factory_get_cali,
	.ps_set_threshold = epl2182_ps_factory_set_threshold,
	.ps_get_threshold = epl2182_ps_factory_get_threshold,
};

static struct alsps_factory_public epl2182_factory_device = {
	.gain = 1,
	.sensitivity = 1,
	.fops = &epl2182_factory_fops,
};

/*----------------------------------------------------------------------------*/
static int epl2182_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct epl2182_priv *obj = NULL;
	struct als_control_path als_ctl = {0};
	struct als_data_path als_data = {0};
	struct ps_control_path ps_ctl = {0};
	struct ps_data_path ps_data = {0};
	int err = 0;

	APS_FUN();

	/* epl2182_dumpReg(client); */
	obj = kzalloc(sizeof(*obj), GFP_KERNEL);
	if (!obj)
	{
		err = -ENOMEM;
		goto exit;
	}

	err = get_alsps_dts_func(client->dev.of_node, &obj->hw);
	if (err < 0)
	{
		APS_ERR("get dts info fail\n");
		goto exit_init_failed;
	}

	epl2182_obj = obj;

	epl2182_get_addr(&obj->hw, &obj->addr);

	APS_ERR("addr is 0x%x!\n", obj->addr.write_addr);

	epl2182_obj->als_level_num =
		sizeof(epl2182_obj->hw.als_level) / sizeof(epl2182_obj->hw.als_level[0]);
	epl2182_obj->als_value_num =
		sizeof(epl2182_obj->hw.als_value) / sizeof(epl2182_obj->hw.als_value[0]);
	BUG_ON(sizeof(epl2182_obj->als_level) != sizeof(epl2182_obj->hw.als_level));
	memcpy(epl2182_obj->als_level, epl2182_obj->hw.als_level, sizeof(epl2182_obj->als_level));
	BUG_ON(sizeof(epl2182_obj->als_value) != sizeof(epl2182_obj->hw.als_value));
	memcpy(epl2182_obj->als_value, epl2182_obj->hw.als_value, sizeof(epl2182_obj->als_value));

	INIT_WORK(&obj->eint_work, epl2182_eint_work);
	INIT_WORK(&obj->data_work, epl2182_check_ps_data);
	init_waitqueue_head(&wait_rsp_wq);

	obj->client = client;
	obj->client->timing = 400;

	i2c_set_clientdata(client, obj);

	atomic_set(&obj->als_debounce, 200);
	atomic_set(&obj->als_deb_on, 0);
	atomic_set(&obj->als_deb_end, 0);
	atomic_set(&obj->ps_debounce, 100);
	atomic_set(&obj->ps_deb_on, 0);
	atomic_set(&obj->ps_deb_end, 0);
	atomic_set(&obj->ps_mask, 0);
	atomic_set(&obj->trace, 0x00);
	atomic_set(&obj->als_suspend, 0);
	atomic_set(&obj->ps_thd_val_high, obj->hw.ps_threshold_high);
	atomic_set(&obj->ps_thd_val_low, obj->hw.ps_threshold_low);
	obj->irq_node = of_find_compatible_node(NULL, NULL, "mediatek, als-eint");

	obj->ps_cali = 0;
	obj->ps_enable = 0;
	obj->als_enable = 0;
	obj->lux_per_count = LUX_PER_COUNT;
	obj->enable = 0;
	obj->pending_intr = 0;

	gRawData.ps_state = -1;

	atomic_set(&obj->i2c_retry, 3);

	epl2182_i2c_client = client;

	elan_epl2182_I2C_Write(client, REG_0, W_SINGLE_BYTE, 0x02, EPL_S_SENSING_MODE);
	elan_epl2182_I2C_Write(client, REG_9, W_SINGLE_BYTE, 0x02, EPL_INT_DISABLE);

	err = epl2182_init_client(client);
	if (err)
		goto exit_init_failed;

	APS_ERR("epl2182_init_client OK!\n");

	err = alsps_factory_device_register(&epl2182_factory_device);
	if (err)
	{
		APS_ERR("epl2182_device register failed\n");
		goto exit_misc_device_register_failed;
	}

	err = epl2182_check_intr(client);
	if (err)
	{
		APS_ERR("check/clear intr: %d\n", err);
		goto exit_misc_device_register_failed;
	}

	als_ctl.is_use_common_factory = false;
	ps_ctl.is_use_common_factory = false;
	APS_LOG("epl2182_device misc_register OK!\n");

	isInterrupt = obj->hw.polling_mode_ps != 1;

	als_ctl.open_report_data = als_open_report_data;
	als_ctl.enable_nodata = als_enable_nodata;
	als_ctl.set_delay = als_set_delay;
	als_ctl.batch = als_batch;
	als_ctl.flush = als_flush;
	als_ctl.is_report_input_direct = false;
	als_ctl.is_support_batch = false;

	err = als_register_control_path(&als_ctl);
	if (err)
	{
		APS_ERR("register fail = %d\n", err);
		goto exit_sensor_obj_attach_fail;
	}

	als_data.get_data = als_get_data;
	als_data.vender_div = 100;
	err = als_register_data_path(&als_data);
	if (err)
	{
		APS_ERR("tregister fail = %d\n", err);
		goto exit_sensor_obj_attach_fail;
	}

	ps_ctl.open_report_data = ps_open_report_data;
	ps_ctl.enable_nodata = ps_enable_nodata;
	ps_ctl.set_delay = ps_set_delay;
	ps_ctl.batch = ps_batch;
	ps_ctl.flush = ps_flush;
	ps_ctl.is_report_input_direct = false;
	ps_ctl.is_support_batch = false;

	err = ps_register_control_path(&ps_ctl);
	if (err)
	{
		APS_ERR("register fail = %d\n", err);
		goto exit_sensor_obj_attach_fail;
	}

	ps_data.get_data = ps_get_data;
	ps_data.vender_div = 100;
	err = ps_register_data_path(&ps_data);
	if (err)
	{
		APS_ERR("tregister fail = %d\n", err);
		goto exit_sensor_obj_attach_fail;
	}

	alsps_init_flag = 0;
	APS_LOG("%s: OK\n", __func__);
	return 0;

exit_sensor_obj_attach_fail:
exit_misc_device_register_failed:
exit_init_failed:
	kfree(obj);
exit:
	obj = NULL;
	epl2182_i2c_client = NULL;
	APS_ERR("%s: err = %d\n", __func__, err);
	alsps_init_flag = -1;
	return err;
}

/*----------------------------------------------------------------------------*/
static int epl2182_i2c_remove(struct i2c_client *client)
{
	alsps_factory_device_deregister(&epl2182_factory_device);
	epl2182_i2c_client = NULL;
	i2c_unregister_device(client);
	kfree(i2c_get_clientdata(client));

	return 0;
}

/*----------------------------------------------------------------------------*/
static int alsps_local_init(void)
{
	APS_FUN();

	if (i2c_add_driver(&epl2182_i2c_driver))
	{
		APS_ERR("add driver error\n");
		return -1;
	}

	APS_ERR("add driver epl2182_i2c_driver ok!\n");

	if (-1 == alsps_init_flag)
		return -1;
	/* printk("fwq loccal init---\n"); */
	return 0;
}

/*----------------------------------------------------------------------------*/
static int alsps_remove(void)
{
	APS_FUN();

	APS_ERR("EPL2182 remove\n");
	i2c_del_driver(&epl2182_i2c_driver);
	return 0;
}

/*----------------------------------------------------------------------------*/
static int __init epl2182_init(void)
{
	alsps_driver_add(&epl2182_init_info);
	return 0;
}

/*----------------------------------------------------------------------------*/
static void __exit epl2182_exit(void)
{
	APS_FUN();
}

/*----------------------------------------------------------------------------*/
module_init(epl2182_init);
module_exit(epl2182_exit);
/*----------------------------------------------------------------------------*/
MODULE_AUTHOR("yucong.xiong@mediatek.com");
MODULE_DESCRIPTION("EPL2182 ALSPS driver");
MODULE_LICENSE("GPL");
