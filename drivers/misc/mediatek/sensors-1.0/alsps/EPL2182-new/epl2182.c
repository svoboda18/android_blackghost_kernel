/* drivers/hwmon/mt6516/amit/epl2182.c - EPL2182 ALS/PS driver
 *
 * Author: MingHsien Hsieh <minghsien.hsieh@mediatek.com>
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
#include <linux/uaccess.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/workqueue.h>
#include <linux/kobject.h>
#include <linux/platform_device.h>
#include <linux/atomic.h>
#include <linux/gpio.h>
#include <linux/of_irq.h>

#include <alsps.h>

#include "epl2182.h"

static long long int_top_time;

/*******************************************************************************/
#define LUX_PER_COUNT 1100 // 1100 = 1.1 * 1000
#define PS_DRIVE EPL_DRIVE_120MA
#define DYN_ENABLE 1

static int PS_INTT = EPL_INTT_PS_80;
static int ALS_INTT = 7;

#define PS_DELAY 20
#define ALS_DELAY 40

#if DYN_ENABLE
#define DYN_H_OFFSET 1000
#define DYN_L_OFFSET 700
#define DYN_PS_CONDITION 30000
#endif

/******************************************************************************
*******************************************************************************/

#define TXBYTES 2
#define RXBYTES 2

#define PACKAGE_SIZE 2
#define I2C_RETRY_COUNT 3

static DEFINE_MUTEX(epl2182_mutex);

#define EPL2182_DEV_NAME "EPL2182"
#define DRIVER_VERSION "v2.06"

typedef struct _epl2182_raw_data
{
    u8 raw_bytes[PACKAGE_SIZE];
    u16 renvo;
    u16 ps_state;
    u16 ps_raw;
    u16 ps_ch0_raw;
#if DYN_ENABLE
    u16 ps_min_raw;
    u16 ps_sta;
    u16 ps_dyn_high;
    u16 ps_dyn_low;
#endif
    u16 als_ch0_raw;
    u16 als_ch1_raw;
    u16 als_lux;
} epl2182_raw_data;

/*----------------------------------------------------------------------------*/
#define APS_TAG "[ALS/PS] "
#define APS_FUN(f)
#define APS_ERR(fmt, args...)
#define APS_LOG(fmt, args...)
#define APS_DBG(fmt, args...)

/*----------------------------------------------------------------------------*/
static struct i2c_client *epl2182_i2c_client = NULL;

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

static struct epl2182_priv *g_epl2182_ptr = NULL;

/*----------------------------------------------------------------------------*/
struct epl2182_i2c_addr /*define a series of i2c slave address*/
{
    u8 write_addr;
};

/*----------------------------------------------------------------------------*/
struct epl2182_priv
{
    struct alsps_hw hw;
    struct i2c_client *client;
    struct work_struct eint_work;
    struct device_node *irq_node;
    int irq;
    /*i2c address group*/
    struct epl2182_i2c_addr addr;

    /*misc*/
    atomic_t trace;
    /*data*/
    u16 lux_per_count;
    bool als_enable;
	bool ps_enable;
    bool pending_intr; /*pending interrupt*/

    int ps_cali;

    /*data*/
    u16 als_level_num;
    u16 als_value_num;
    u32 als_level[C_CUST_ALS_LEVEL - 1];
    u32 als_value[C_CUST_ALS_LEVEL];
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

static struct epl2182_priv *epl2182_obj = NULL;

static epl2182_raw_data gRawData;

static int alsps_init_flag = -1; // 0<==>OK -1 <==> fail
static int alsps_local_init(void);
static int alsps_remove(void);
static struct alsps_init_info epl2182_init_info = {
    .name = EPL2182_DEV_NAME,
    .init = alsps_local_init,
    .uninit = alsps_remove,

};

static int elan_epl2182_I2C_Write(struct i2c_client *client, uint8_t regaddr, uint8_t bytecount, uint8_t txbyte, uint8_t data)
{
    uint8_t buffer[2];
    int ret = 0;
    int retry;

	mutex_lock(&epl2182_mutex);

    buffer[0] = (regaddr << 3) | bytecount;
    buffer[1] = data;

    for (retry = 0; retry < I2C_RETRY_COUNT; retry++)
    {
        ret = i2c_master_send(client, buffer, txbyte);

        if (ret == txbyte)
        {
            break;
        }

        APS_ERR("i2c write error,TXBYTES %d\n", ret);
        mdelay(10);
    }

    if (retry >= I2C_RETRY_COUNT)
    {
        APS_ERR("i2c write retry over %d\n", I2C_RETRY_COUNT);
        mutex_unlock(&epl2182_mutex);
        return -EINVAL;
    }

    mutex_unlock(&epl2182_mutex);
    return ret;
}

static int elan_epl2182_I2C_Read(struct i2c_client *client)
{
    uint8_t buffer[RXBYTES];
    int ret = 0, i = 0;
    int retry;

    mutex_lock(&epl2182_mutex);

    for (retry = 0; retry < I2C_RETRY_COUNT; retry++)
    {
        ret = i2c_master_recv(client, buffer, RXBYTES);

        if (ret == RXBYTES)
            break;

        APS_ERR("i2c read error,RXBYTES %d\r\n", ret);
        mdelay(10);
    }

    if (retry >= I2C_RETRY_COUNT)
    {
        APS_ERR("i2c read retry over %d\n", I2C_RETRY_COUNT);
        mutex_unlock(&epl2182_mutex);
        return -EINVAL;
    }

    for (i = 0; i < PACKAGE_SIZE; i++)
        gRawData.raw_bytes[i] = buffer[i];

    mutex_unlock(&epl2182_mutex);
    return ret;
}

#if DYN_ENABLE
static void dyn_ps_cal(struct epl2182_priv *epl_data)
{
    if ((gRawData.ps_raw < gRawData.ps_min_raw) && (gRawData.ps_sta != 1) && (gRawData.ps_ch0_raw <= DYN_PS_CONDITION))
    {
        gRawData.ps_min_raw = gRawData.ps_raw;
        epl_data->hw.ps_threshold_low = gRawData.ps_raw + epl_data->ps_cali + DYN_L_OFFSET;
        epl_data->hw.ps_threshold_high = gRawData.ps_raw + epl_data->ps_cali + DYN_H_OFFSET;
        set_psensor_intr_threshold(epl_data->hw.ps_threshold_low, epl_data->hw.ps_threshold_high);
        APS_LOG("dyn ps raw = %d, min = %d, ch0 = %d\n dyn h_thre = %d, l_thre = %d, ps_state = %d",
                gRawData.ps_raw, gRawData.ps_min_raw, gRawData.ps_ch0_raw, epl_data->hw.ps_threshold_high, epl_data->hw.ps_threshold_low, gRawData.ps_state);
    }
}
#endif

static int elan_epl2182_psensor_enable(struct epl2182_priv *epl_data, int enable)
{
    int ret = 0;
    int ps_state = 0;
    uint8_t regdata;
    struct i2c_client *client = epl_data->client;
    u8 ps_state_tmp;

    APS_LOG("[ELAN epl2182] %s enable = %d\n", __func__, enable);

    ret = elan_epl2182_I2C_Write(client, REG_9, W_SINGLE_BYTE, 0x02, EPL_INT_DISABLE | PS_DRIVE);

    if (enable)
    {
        regdata = EPL_SENSING_2_TIME | EPL_PS_MODE | EPL_L_GAIN;
        regdata = regdata | (!epl_data->hw.polling_mode_ps ? EPL_C_SENSING_MODE : EPL_S_SENSING_MODE);
        ret = elan_epl2182_I2C_Write(client, REG_0, W_SINGLE_BYTE, 0x02, regdata);
        regdata = PS_INTT << 4 | EPL_PST_1_TIME | EPL_10BIT_ADC;
        ret = elan_epl2182_I2C_Write(client, REG_1, W_SINGLE_BYTE, 0x02, regdata);

#if !DYN_ENABLE
        set_psensor_intr_threshold(epl_data->hw.ps_threshold_low, epl_data->hw.ps_threshold_high);
#endif

        ret = elan_epl2182_I2C_Write(client, REG_7, W_SINGLE_BYTE, 0x02, EPL_C_RESET);
        ret = elan_epl2182_I2C_Write(client, REG_7, W_SINGLE_BYTE, 0x02, EPL_C_START_RUN);

        msleep(PS_DELAY);

        elan_epl2182_I2C_Write(client, REG_13, R_SINGLE_BYTE, 0x01, 0);
        elan_epl2182_I2C_Read(client);
        ps_state_tmp = !((gRawData.raw_bytes[0] & 0x04) >> 2);
        APS_LOG("[%s]:real ps_state = %d\n", __func__, ps_state_tmp);
#if DYN_ENABLE
        gRawData.ps_sta = ((gRawData.raw_bytes[0] & 0x02) >> 1);
#endif
        elan_epl2182_I2C_Write(client, REG_14, R_TWO_BYTE, 0x01, 0x00);
        elan_epl2182_I2C_Read(client);
        gRawData.ps_ch0_raw = ((gRawData.raw_bytes[1] << 8) | gRawData.raw_bytes[0]);

        elan_epl2182_I2C_Write(client, REG_16, R_TWO_BYTE, 0x01, 0x00);
        elan_epl2182_I2C_Read(client);
        gRawData.ps_raw = ((gRawData.raw_bytes[1] << 8) | gRawData.raw_bytes[0]);
#if DYN_ENABLE
        dyn_ps_cal(epl_data);
        APS_LOG("dyn k ps raw = %d, ch0 = %d\n, ps_state = %d", gRawData.ps_raw, gRawData.ps_ch0_raw, ps_state);
#endif
        APS_LOG("[%s]:gRawData.ps_raw=%d \r\n", __func__, gRawData.ps_raw);
        if (!epl_data->hw.polling_mode_ps)
        {
            if (epl_data->als_enable)
            {
                APS_LOG("[%s]: ALS+PS mode \r\n", __func__);
                if ((ps_state_tmp == 0 && gRawData.ps_raw > epl_data->hw.ps_threshold_high) ||
                    (ps_state_tmp == 1 && gRawData.ps_raw < epl_data->hw.ps_threshold_low))
                {
                    APS_LOG("change ps_state(ps_state_tmp=%d, gRawData.ps_state=%d) \r\n", ps_state_tmp, gRawData.ps_state);
                    ps_state = ps_state_tmp;
                }
                else
                {
                    ps_state = gRawData.ps_state;
                }
            }
            else
            {
                ps_state = ps_state_tmp;
                APS_LOG("[%s]: PS only \r\n", __func__);
            }

            if (gRawData.ps_state != ps_state)
            {
                gRawData.ps_state = ps_state;
            }
            else
            {
                APS_LOG("[%s]: EPL_INT_ACTIVE_LOW .............\r\n", __func__);
            }
            
            ps_report_interrupt_data(ps_state);

            elan_epl2182_I2C_Write(client, REG_9, W_SINGLE_BYTE, 0x02, EPL_INT_ACTIVE_LOW | PS_DRIVE);
        }
    }
    else
    {
        regdata = EPL_SENSING_2_TIME | EPL_PS_MODE | EPL_L_GAIN | EPL_S_SENSING_MODE;
        ret = elan_epl2182_I2C_Write(client, REG_0, W_SINGLE_BYTE, 0x02, regdata);
    }

    if (ret < 0)
    {
        APS_ERR("[ELAN epl2182 error]%s: ps enable %d fail\n", __func__, ret);
    }
    else
    {
        ret = 0;
    }

    return ret;
}

static int elan_epl2182_lsensor_enable(struct epl2182_priv *epl_data, int enable)
{
    int ret = 0;
    uint8_t regdata;
    struct i2c_client *client = epl_data->client;

    APS_LOG("[ELAN epl2182] %s enable = %d\n", __func__, enable);

    if (enable)
    {
        regdata = EPL_INT_DISABLE;
        ret = elan_epl2182_I2C_Write(client, REG_9, W_SINGLE_BYTE, 0x02, regdata);

        regdata = EPL_S_SENSING_MODE | EPL_SENSING_8_TIME | EPL_ALS_MODE | EPL_AUTO_GAIN;
        ret = elan_epl2182_I2C_Write(client, REG_0, W_SINGLE_BYTE, 0x02, regdata);

        regdata = ALS_INTT << 4 | EPL_PST_1_TIME | EPL_10BIT_ADC;
        ret = elan_epl2182_I2C_Write(client, REG_1, W_SINGLE_BYTE, 0x02, regdata);

        ret = elan_epl2182_I2C_Write(client, REG_10, W_SINGLE_BYTE, 0x02, EPL_GO_MID);
        ret = elan_epl2182_I2C_Write(client, REG_11, W_SINGLE_BYTE, 0x02, EPL_GO_LOW);

        ret = elan_epl2182_I2C_Write(client, REG_7, W_SINGLE_BYTE, 0x02, EPL_C_RESET);
        ret = elan_epl2182_I2C_Write(client, REG_7, W_SINGLE_BYTE, 0x02, EPL_C_START_RUN);

        msleep(ALS_DELAY);
    }

    if (ret < 0)
    {
        APS_ERR("[ELAN epl2182 error]%s: als_enable %d fail\n", __func__, ret);
    }
    else
    {
        ret = 0;
    }

    return ret;
}

static int epl2182_get_als_value(struct epl2182_priv *obj, u16 als)
{
    int idx;
    int lux = 0;

    lux = (als * obj->lux_per_count) / 1000;
    for (idx = 0; idx < obj->als_level_num; idx++)
    {
        if (lux < obj->hw.als_level[idx])
        {
            break;
        }
    }

    if (idx >= obj->als_value_num)
    {
        APS_ERR("exceed range\n");
        idx = obj->als_value_num - 1;
    }

    gRawData.als_lux = obj->hw.als_value[idx];
    APS_LOG("ALS: %05d => %05d\n", als, obj->hw.als_value[idx]);
    return gRawData.als_lux;
}

static int set_psensor_intr_threshold(uint16_t low_thd, uint16_t high_thd)
{
    int ret = 0;
    struct epl2182_priv *epld = epl2182_obj;
    struct i2c_client *client = epld->client;

    uint8_t high_msb, high_lsb, low_msb, low_lsb;

    APS_LOG("%s\n", __func__);

    high_msb = (uint8_t)(high_thd >> 8);
    high_lsb = (uint8_t)(high_thd & 0x00ff);
    low_msb = (uint8_t)(low_thd >> 8);
    low_lsb = (uint8_t)(low_thd & 0x00ff);

    APS_LOG("%s: low_thd = %d, high_thd = %d \n", __func__, low_thd, high_thd);

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

    APS_LOG(" I2C Addr==[0x%x],line=%d\n", epl2182_i2c_client->addr, __LINE__);

    return 0;
}

/*----------------------------------------------------------------------------*/
int epl2182_get_addr(struct alsps_hw *hw, struct epl2182_i2c_addr *addr)
{
    if (!hw || !addr)
    {
        return -EFAULT;
    }
    addr->write_addr = hw->i2c_addr[0];
    return 0;
}

/*----------------------------------------------------------------------------*/
static int epl2182_check_intr(struct i2c_client *client)
{
    struct epl2182_priv *obj = i2c_get_clientdata(client);
    int mode;

    elan_epl2182_I2C_Write(obj->client, REG_13, R_SINGLE_BYTE, 0x01, 0);
    elan_epl2182_I2C_Read(obj->client);
    mode = gRawData.raw_bytes[0] & (3 << 4);

    obj->pending_intr = mode == 0x10;

    return 0;
}

/*----------------------------------------------------------------------------*/
int epl2182_read_als(struct i2c_client *client)
{
    struct epl2182_priv *obj = i2c_get_clientdata(client);
    uint8_t setting;
    u16 ch1;

    if (client == NULL)
    {
        APS_DBG("CLIENT CANN'T EQUL NULL\n");
        return -1;
    }

    elan_epl2182_I2C_Write(client, REG_13, R_SINGLE_BYTE, 0x01, 0);
    elan_epl2182_I2C_Read(client);
    setting = gRawData.raw_bytes[0];
    if ((setting & (3 << 4)) != 0x00)
    {
        APS_ERR("read als data in wrong mode\n");
    }

    elan_epl2182_I2C_Write(obj->client, REG_16, R_TWO_BYTE, 0x01, 0x00);
    elan_epl2182_I2C_Read(obj->client);
    ch1 = (gRawData.raw_bytes[1] << 8) | gRawData.raw_bytes[0];

    // FIX: mid gain and low gain cannot report ff in auton gain
    if (setting >> 7 == 0 && ch1 == 65535)
    {
        APS_LOG("setting %d, gain %x, als %d\n", setting, setting >> 7, ch1);
        APS_LOG("skip FF in auto gain\n\n");
    }
    else
    {
        gRawData.als_ch1_raw = ch1;
        APS_LOG("read als raw data = %d\n", gRawData.als_ch1_raw);
    }

    return 0;
}

/*----------------------------------------------------------------------------*/
long epl2182_read_ps(struct i2c_client *client, u16 *data)
{
    struct epl2182_priv *obj = i2c_get_clientdata(client);

    if (client == NULL)
    {
        APS_DBG("CLIENT CANN'T EQUL NULL\n");
        return -1;
    }

    elan_epl2182_I2C_Write(obj->client, REG_16, R_TWO_BYTE, 0x01, 0x00);
    elan_epl2182_I2C_Read(obj->client);
    gRawData.ps_raw = (gRawData.raw_bytes[1] << 8) | gRawData.raw_bytes[0];

	if (gRawData.ps_raw < obj->ps_cali)
		*data = 0;
	else
		*data = gRawData.ps_raw - obj->ps_cali;

    return 0;
}

/*--------------------------------------------------------------------------------*/
static int als_open_report_data(int open)
{
    // should queuq work to report event if  is_report_input_direct=true
    return 0;
}
/*--------------------------------------------------------------------------------*/
// if use  this typ of enable , Gsensor only enabled but not report inputEvent to HAL
static int als_enable_nodata(int en)
{
    struct epl2182_priv *obj = epl2182_obj;

    if (!obj)
    {
        APS_ERR("obj is null!!\n");
        return -1;
    }
    APS_LOG("[%s] als enable en = %d\n", __func__, en);

    if (obj->als_enable != en)
    {
        elan_epl2182_lsensor_enable(obj, en);
        obj->als_enable = en;
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
    struct epl2182_priv *obj = epl2182_obj;
    if (!obj)
    {
        APS_ERR("obj is null!!\n");
        return -1;
    }

    err = elan_epl2182_lsensor_enable(epl2182_obj, 1);
    if (err != 0)
    {
        APS_ERR("enable als fail: %d\n", err);
        return -1;
    }
    err = epl2182_read_als(epl2182_obj->client);
    if (err != 0)
    {
        APS_ERR("read raw als fail: %d\n", err);
        return -1;
    }
    *value = epl2182_get_als_value(obj, gRawData.als_ch1_raw);
    *status = SENSOR_STATUS_ACCURACY_MEDIUM;
    APS_LOG("[%s]:*value = %d\n", __func__, *value);

    return err;
}
/*--------------------------------------------------------------------------------*/
// if use  this typ of enable , Gsensor should report inputEvent(x, y, z ,stats, div) to HAL
static int ps_open_report_data(int open)
{
    // should queuq work to report event if  is_report_input_direct=true
    return 0;
}
/*--------------------------------------------------------------------------------*/
// if use  this typ of enable , Gsensor only enabled but not report inputEvent to HAL
static int ps_enable_nodata(int en)
{
    struct epl2182_priv *obj = epl2182_obj;

    APS_LOG("ps enable = %d\n", en);
    if (obj->ps_enable != en)
    {
        if (en)
        {
#if DYN_ENABLE
            gRawData.ps_min_raw = 0xffff;
#endif
        }
        elan_epl2182_psensor_enable(obj, en);
        obj->ps_enable = en;
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

	err = epl2182_read_ps(epl2182_obj->client, &gRawData.ps_raw);
	if (err != 0)
	{
		APS_ERR("read ps fail: %d\n", err);
		return -1;
	}
	*value = gRawData.ps_raw;
	*status = SENSOR_STATUS_ACCURACY_HIGH;

	return err;
}
/*----------------------------------------------------------------------------*/

/*----------------------------------------------------------------------------*/
static irqreturn_t epl2182_eint_func(int irq, void *desc)
{
    struct epl2182_priv *obj = g_epl2182_ptr;

    int_top_time = sched_clock();

    if (!obj)
    {
        return IRQ_HANDLED;
    }

    disable_irq_nosync(epl2182_obj->irq);

    schedule_work(&obj->eint_work);

    return IRQ_HANDLED;
}

/*----------------------------------------------------------------------------*/
static void epl2182_eint_work(struct work_struct *work)
{
    struct epl2182_priv *epld = g_epl2182_ptr;
    int err;
    u8 ps_state;

    if (epld->ps_enable)
    {
        APS_LOG("xxxxx eint work\n");

        if ((err = epl2182_check_intr(epld->client)))
        {
            APS_ERR("check intrs: %d\n", err);
        }

        if (epld->pending_intr)
        {
            elan_epl2182_I2C_Write(epld->client, REG_13, R_SINGLE_BYTE, 0x01, 0);
            elan_epl2182_I2C_Read(epld->client);
            ps_state = !((gRawData.raw_bytes[0] & 0x04) >> 2);
            APS_LOG("real ps_state = %d\n", ps_state);

            elan_epl2182_I2C_Write(epld->client, REG_16, R_TWO_BYTE, 0x01, 0x00);
            elan_epl2182_I2C_Read(epld->client);
            gRawData.ps_raw = (gRawData.raw_bytes[1] << 8) | gRawData.raw_bytes[0];
            APS_LOG("ps raw_data = %d\n", gRawData.ps_raw);

            if (epld->als_enable)
            {
                APS_LOG("ALS+PS mode \r\n");
                if ((ps_state == 0 && gRawData.ps_raw > epld->hw.ps_threshold_high) ||
                    (ps_state == 1 && gRawData.ps_raw < epld->hw.ps_threshold_low))
                {
                    APS_LOG("change ps_state(ps_state=%d, gRawData.ps_state=%d) \r\n", ps_state, gRawData.ps_state);
                    gRawData.ps_state = ps_state;
                }
            }
            else
            {
                gRawData.ps_state = ps_state;
                APS_LOG("PS only \r\n");
            }
            err = ps_report_interrupt_data(gRawData.ps_state);
            if (err != 0)
            {
                APS_ERR("epl2182_eint_work err: %d\n", err);
            }
        }

        elan_epl2182_I2C_Write(epld->client, REG_9, W_SINGLE_BYTE, 0x02, EPL_INT_ACTIVE_LOW | PS_DRIVE);
        elan_epl2182_I2C_Write(epld->client, REG_7, W_SINGLE_BYTE, 0x02, EPL_DATA_UNLOCK);
    }

    enable_irq(epld->irq);
}

/*----------------------------------------------------------------------------*/
int epl2182_setup_eint(struct i2c_client *client)
{
    struct epl2182_priv *obj = i2c_get_clientdata(client);
    int ret;
    u32 ints[2] = {0, 0};
    struct pinctrl *pinctrl;
    struct pinctrl_state *pins_cfg;
    struct platform_device *alsps_pdev = get_alsps_platformdev();

    APS_LOG("epl2182_setup_eint\n");

    g_epl2182_ptr = obj;
    pinctrl = devm_pinctrl_get(&alsps_pdev->dev);
    if (IS_ERR(pinctrl))
    {
        ret = PTR_ERR(pinctrl);
        APS_ERR("Cannot find alsps pinctrl!\n");
    }

    pins_cfg = pinctrl_lookup_state(pinctrl, "pin_cfg");
    if (IS_ERR(pins_cfg))
    {
        ret = PTR_ERR(pins_cfg);
        APS_ERR("Cannot find alsps pinctrl pin_cfg!\n");
    }

    if (epl2182_obj->irq_node)
    {
        of_property_read_u32_array(epl2182_obj->irq_node, "debounce", ints,
                                   ARRAY_SIZE(ints));
        ret = gpio_request_one(ints[0], GPIOF_IN, "alsps_int");
        if (ret < 0)
        {
            APS_ERR("Unable to request gpio int_pin\n");
            return -1;
        }

        mt_gpio_set_debounce(ints[0], ints[1]);
        pinctrl_select_state(pinctrl, pins_cfg);

        epl2182_obj->irq = irq_of_parse_and_map(epl2182_obj->irq_node, 0);
        APS_LOG("epl2182_obj->irq = %d\n", epl2182_obj->irq);
        if (!epl2182_obj->irq)
        {
            APS_ERR("irq_of_parse_and_map fail!!\n");
            return -EINVAL;
        }
        if (request_irq(epl2182_obj->irq, epl2182_eint_func, IRQF_TRIGGER_LOW, "ALS-eint", NULL))
        {
            APS_ERR("IRQ LINE NOT AVAILABLE!!\n");
            return -EINVAL;
        }
        enable_irq_wake(epl2182_obj->irq);
    }
    else
    {
        APS_ERR("null irq node!!\n");
        return -EINVAL;
    }

    return 0;
}

/*----------------------------------------------------------------------------*/
static int epl2182_init_client(struct i2c_client *client)
{
    struct epl2182_priv *obj = i2c_get_clientdata(client);
    int err = 0;

    APS_LOG("[Agold spl] I2C Addr==[0x%x],line=%d\n", epl2182_i2c_client->addr, __LINE__);

    APS_FUN();

    if (obj->hw.polling_mode_ps == 0)
    {
        if ((err = epl2182_setup_eint(client)))
        {
            APS_ERR("setup eint: %d\n", err);
            return err;
        }
        APS_LOG("epl2182 interrupt setup\n");
    }

    if ((err = hw8k_init_device(client)) != 0)
    {
        APS_ERR("init dev: %d\n", err);
        return err;
    }

    if ((err = epl2182_check_intr(client)))
    {
        APS_ERR("check/clear intr: %d\n", err);
        return err;
    }

    return err;
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

	err = epl2182_read_als(obj->client);
	if (err)
	{
		APS_ERR("%s failed\n", __func__);
		return -1;
	}
    *data = gRawData.als_ch1_raw;

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
	int err = 0, status = 0;

	err = ps_get_data(data, &status);
	if (err < 0)
		return -1;
	return 0;
}

static int epl2182_ps_factory_get_raw_data(int32_t *data)
{
	int err = 0, status = 0;

	err = ps_get_data(data, &status);
	if (err < 0)
		return -1;
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
	struct epl2182_priv *obj = epl2182_obj;

    obj->hw.ps_threshold_low = threshold[1] + obj->ps_cali;
    obj->hw.ps_threshold_high = threshold[0] + obj->ps_cali;
    set_psensor_intr_threshold(obj->hw.ps_threshold_low, obj->hw.ps_threshold_high);

    return 0;
}

static int epl2182_ps_factory_get_threshold(int32_t threshold[2])
{
    struct epl2182_priv *obj = epl2182_obj;
    threshold[1] = obj->hw.ps_threshold_low - obj->ps_cali;
    threshold[0] = obj->hw.ps_threshold_high - obj->ps_cali;;
    return 0;
}

/*----------------------------------------------------------------------------*/
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
    struct epl2182_priv *obj;
    struct als_control_path als_ctl = {0};
    struct als_data_path als_data = {0};
    struct ps_control_path ps_ctl = {0};
    struct ps_data_path ps_data = {0};
    int err = 0;
    APS_FUN();

    if ((err = i2c_smbus_read_byte_data(client, 0x98)) != 0x68)
    { // check chip
        APS_ERR("elan ALS/PS sensor is failed. \n");
        err = -1;
        goto exit;
    }

    if (!(obj = kzalloc(sizeof(*obj), GFP_KERNEL)))
    {
        err = -ENOMEM;
        goto exit;
    }

    memset(obj, 0, sizeof(*obj));

    err = get_alsps_dts_func(client->dev.of_node, &obj->hw);
    if (err < 0)
    {
        APS_ERR("get dts info fail\n");
        goto exit_init_failed;
    }

    epl2182_obj = obj;

    epl2182_get_addr(&obj->hw, &obj->addr);

    obj->als_level_num = sizeof(obj->hw.als_level) / sizeof(obj->hw.als_level[0]);
    obj->als_value_num = sizeof(obj->hw.als_value) / sizeof(obj->hw.als_value[0]);
    BUG_ON(sizeof(obj->als_level) != sizeof(obj->hw.als_level));
    memcpy(obj->als_level, obj->hw.als_level, sizeof(obj->als_level));
    BUG_ON(sizeof(obj->als_value) != sizeof(obj->hw.als_value));
    memcpy(obj->als_value, obj->hw.als_value, sizeof(obj->als_value));

    INIT_WORK(&obj->eint_work, epl2182_eint_work);

    obj->client = client;

    i2c_set_clientdata(client, obj);

    atomic_set(&obj->trace, 0x00);

    obj->irq_node = of_find_compatible_node(NULL, NULL, "mediatek, als-eint");
    obj->lux_per_count = LUX_PER_COUNT;
    obj->ps_enable = 0;
    obj->als_enable = 0;
    obj->pending_intr = false;

    epl2182_i2c_client = client;

    elan_epl2182_I2C_Write(client, REG_0, W_SINGLE_BYTE, 0x02, EPL_S_SENSING_MODE);
    elan_epl2182_I2C_Write(client, REG_9, W_SINGLE_BYTE, 0x02, EPL_INT_DISABLE);

    elan_epl2182_I2C_Write(client, REG_19, R_TWO_BYTE, 0x01, 0x00);
    elan_epl2182_I2C_Read(client);
    gRawData.renvo = (gRawData.raw_bytes[1] << 8) | gRawData.raw_bytes[0];

    if ((err = epl2182_init_client(client)))
    {
        goto exit_init_failed;
    }

    err = alsps_factory_device_register(&epl2182_factory_device);
    if (err)
    {
        APS_ERR("epl2182_device register failed\n");
        goto exit_misc_device_register_failed;
    }

    als_ctl.is_use_common_factory = false;
    ps_ctl.is_use_common_factory = false;

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
        goto exit_register_path;
    }

    als_data.get_data = als_get_data;
    als_data.vender_div = 100;
    err = als_register_data_path(&als_data);
    if (err)
    {
        APS_ERR("tregister fail = %d\n", err);
        goto exit_register_path;
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
        goto exit_register_path;
    }

    ps_data.get_data = ps_get_data;
    ps_data.vender_div = 100;
    err = ps_register_data_path(&ps_data);
    if (err)
    {
        APS_ERR("tregister fail = %d\n", err);
        goto exit_register_path;
    }

    if (obj->hw.polling_mode_ps == 0 || obj->hw.polling_mode_als == 0)
        epl2182_setup_eint(client);

    alsps_init_flag = 0;
    APS_ERR("%s: OK\n", __func__);
    return 0;

exit_register_path:
    alsps_factory_device_deregister(&epl2182_factory_device);
exit_misc_device_register_failed:
exit_init_failed:
    kfree(obj);
exit:
    epl2182_i2c_client = NULL;
    alsps_init_flag = -1;

    APS_ERR("%s: err = %d\n", __func__, err);
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

static int epl2182_i2c_detect(struct i2c_client *client, struct i2c_board_info *info)
{
    strcpy(info->type, EPL2182_DEV_NAME);
    return 0;
}

/*----------------------------------------------------------------------------*/
static int alsps_local_init(void)
{
    if (i2c_add_driver(&epl2182_i2c_driver))
    {
        APS_ERR("add driver error\n");
        return -1;
    }

    if (-1 == alsps_init_flag)
    {
        return -1;
    }
    return 0;
}
/*----------------------------------------------------------------------------*/
static int alsps_remove(void)
{
    APS_FUN();
    APS_ERR("epl2182 remove \n");

    i2c_del_driver(&epl2182_i2c_driver);
    return 0;
}

/*----------------------------------------------------------------------------*/
static int __init epl2182_init(void)
{
    APS_FUN();
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
MODULE_AUTHOR("renato.pan@eminent-tek.com");
MODULE_DESCRIPTION("EPL2182 ALPsr driver");
MODULE_LICENSE("GPL");