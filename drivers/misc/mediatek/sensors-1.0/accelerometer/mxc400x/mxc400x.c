/******************** (C) COPYRIGHT 2016 MEMSIC ********************
 *
 * File Name          : mxc400x.c
 * Description        : MXC400X accelerometer sensor API
 *
 *******************************************************************************
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * THE PRESENT SOFTWARE IS PROVIDED ON AN "AS IS" BASIS, WITHOUT WARRANTIES
 * OR CONDITIONS OF ANY KIND, EITHER EXPRESS OR IMPLIED, FOR THE SOLE
 * PURPOSE TO SUPPORT YOUR APPLICATION DEVELOPMENT.
 * AS A RESULT, MEMSIC SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
 * INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
 * CONTENT OF SUCH SOFTWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
 * INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
 *
 * THIS SOFTWARE IS SPECIFICALLY DESIGNED FOR EXCLUSIVE USE WITH MEMSIC PARTS.
 *
 ******************************************************************************/

#include "accel.h"
#include "mxc400x.h"

#define DRIVER_VERSION "V60.97.05.01"
#define GSE_DEBUG_ON 1
#define GSE_DEBUG_FUNC_ON 0
/* Log define */
#define GSE_INFO(fmt, arg...) pr_warn("<<-GSE INFO->> " fmt "\n", ##arg)
#define GSE_ERR(fmt, arg...) pr_err("<<-GSE ERROR->> " fmt "\n", ##arg)
#define GSE_DEBUG(fmt, arg...)                                         \
    do                                                                 \
    {                                                                  \
        if (GSE_DEBUG_ON)                                              \
            pr_warn("<<-GSE DEBUG->> [%d]" fmt "\n", __LINE__, ##arg); \
    } while (0)
#define GSE_DEBUG_FUNC()                                                      \
    do                                                                        \
    {                                                                         \
        if (GSE_DEBUG_FUNC_ON)                                                \
            pr_debug("<<-GSE FUNC->> Func:%s@Line:%d\n", __func__, __LINE__); \
    } while (0)

#define MXC400X_AXIS_X 0
#define MXC400X_AXIS_Y 1
#define MXC400X_AXIS_Z 2

#define MXC400X_AXES_NUM 3
#define MXC400X_DATA_LEN 6
#define MXC400X_AXIS_Z_PAD 120

#define USE_DELAY

#ifdef USE_DELAY
static int delay_state = 0;
#endif
static const struct i2c_device_id mxc400x_i2c_id[] = {{MXC400X_DEV_NAME, 0},{}};

static int mxc400x_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id);
static int mxc400x_i2c_remove(struct i2c_client *client);

static int mxc400x_local_init(void);
static int mxc400x_remove(void);

struct scale_factor
{
    u8 whole;
    u8 fraction;
};

struct data_resolution
{
    struct scale_factor scalefactor;
    int sensitivity;
};

static int mxc400x_init_flag = -1;

static struct acc_init_info mxc400x_init_info = {
    .name = "mxc400x",
    .init = mxc400x_local_init,
    .uninit = mxc400x_remove,
};
struct mxc400x_i2c_data
{
    struct i2c_client *client;
    struct acc_hw hw;
    struct hwmsen_convert cvt;
    /*misc*/
    struct data_resolution *reso;
    atomic_t trace;

    int32_t cali_sw[MXC400X_AXES_NUM + 1];
    s16 data[MXC400X_AXES_NUM + 1];
};

#ifdef CONFIG_OF
static const struct of_device_id accel_of_match[] = {
    {.compatible = "mediatek,gsensor"},
    {},
};
#endif

static struct i2c_driver mxc400x_i2c_driver = {
    .driver = {
        .name = MXC400X_DEV_NAME,
#ifdef CONFIG_OF
        .of_match_table = accel_of_match,
#endif
    },
    .probe = mxc400x_i2c_probe,
    .remove = mxc400x_i2c_remove,
    .id_table = mxc400x_i2c_id,
};

struct i2c_client *mxc400x_i2c_client = NULL;
static struct mxc400x_i2c_data *obj_i2c_data = NULL;

static struct mutex mxc400x_mutex;

static bool enable_status = false;

static struct data_resolution mxc400x_data_resolution[] = {
    {{0, 9}, 1024}, /*+/-2g  in 12-bit resolution:  0.9 mg/LSB*/
    {{1, 9}, 512},	/*+/-4g  in 12-bit resolution:  1.9 mg/LSB*/
    {{3, 9}, 256},	/*+/-8g  in 12-bit resolution: 3.9 mg/LSB*/
};

static int mxc400x_i2c_read_block(struct i2c_client *client, u8 addr, u8 *data, u8 len)
{
    int err = 0;
    if (!client)
        return -EINVAL;
    else if (len > C_I2C_FIFO_SIZE)
    {
        pr_err("Length %d exceeds %d\n", len, C_I2C_FIFO_SIZE);
        return -EINVAL;
    }

    mutex_lock(&mxc400x_mutex);

    err = i2c_smbus_read_i2c_block_data(client, addr, len, data);
    if (err < 0)
    {
        GSE_ERR("i2c_smbus_read_i2c_block_data error: %d\n", err);
        return err;
    }

    mutex_unlock(&mxc400x_mutex);

    return MXC400X_SUCCESS;
}

static int MXC400X_ReadData(struct i2c_client *client, s16 data[MXC400X_AXES_NUM])
{
    u8 buf[MXC400X_DATA_LEN] = {0};
    int err = 0;

#ifdef USE_DELAY
    if (delay_state)
    {
        msleep(300);
        delay_state = 0;
    }
#endif

    if (NULL == client)
    {
        GSE_ERR("client is null\n");
        err = -EINVAL;
    }
    if ((err = mxc400x_i2c_read_block(client, MXC400X_REG_X, buf, MXC400X_DATA_LEN)) != 0)
    {
        GSE_ERR("error: %d\n", err);
    }
    else
    {
        data[MXC400X_AXIS_X] = (s16)(buf[0] << 8 | buf[1]) >> 4;
        data[MXC400X_AXIS_Y] = (s16)(buf[2] << 8 | buf[3]) >> 4;
        data[MXC400X_AXIS_Z] = (s16)(buf[4] << 8 | buf[5]) >> 4;
    
        GSE_DEBUG("intr x = %d y = %d z = %d\n", data[MXC400X_AXIS_X], data[MXC400X_AXIS_Y], data[MXC400X_AXIS_Z]);
    }
    return err;
}

static int MXC400X_ResetCalibration(struct i2c_client *client)
{
    struct mxc400x_i2c_data *obj = i2c_get_clientdata(client);
    int err;
    err = 0;

    memset(obj->cali_sw, 0x00, sizeof(obj->cali_sw));
    return err;
}

static int MXC400X_ReadCalibrationEx(struct i2c_client *client, int dat[MXC400X_AXES_NUM])
{
    /*raw: the raw calibration data; act: the actual calibration data*/
    struct mxc400x_i2c_data *obj = i2c_get_clientdata(client);

    dat[obj->cvt.map[MXC400X_AXIS_X]] = obj->cvt.sign[MXC400X_AXIS_X] * obj->cali_sw[MXC400X_AXIS_X];
    dat[obj->cvt.map[MXC400X_AXIS_Y]] = obj->cvt.sign[MXC400X_AXIS_Y] * obj->cali_sw[MXC400X_AXIS_Y];
    dat[obj->cvt.map[MXC400X_AXIS_Z]] = obj->cvt.sign[MXC400X_AXIS_Z] * obj->cali_sw[MXC400X_AXIS_Z];

    return 0;
}

static int MXC400X_WriteCalibration(struct i2c_client *client, int dat[MXC400X_AXES_NUM])
{
    struct mxc400x_i2c_data *obj = i2c_get_clientdata(client);
    int err = 0;

    GSE_DEBUG("UPDATE: (%+3d %+3d %+3d)\n",
              dat[MXC400X_AXIS_X], dat[MXC400X_AXIS_Y], dat[MXC400X_AXIS_Z]);

    obj->cali_sw[MXC400X_AXIS_X] = obj->cvt.sign[MXC400X_AXIS_X] * dat[obj->cvt.map[MXC400X_AXIS_X]];
    obj->cali_sw[MXC400X_AXIS_Y] = obj->cvt.sign[MXC400X_AXIS_Y] * dat[obj->cvt.map[MXC400X_AXIS_Y]];
    obj->cali_sw[MXC400X_AXIS_Z] = obj->cvt.sign[MXC400X_AXIS_Z] * dat[obj->cvt.map[MXC400X_AXIS_Z]];

    return err;
}

static int MXC400X_CheckDeviceID(struct i2c_client *client)
{
    u8 databuf[1];
    int res = 0;

    databuf[0] = MXC400X_REG_ID;
	res = i2c_master_send(client, databuf, 0x01);
	if(res <= 0)
    {
        GSE_ERR("MXC400X Device ID read faild\n");
        return MXC400X_ERR_I2C;
    }
	msleep(5);

	databuf[0] = 0x0;
	res = i2c_master_recv(client, databuf, 0x01);
	if(res <= 0)
    {
        GSE_ERR("MXC400X Device ID read faild\n");
        return MXC400X_ERR_I2C;
    }

    databuf[0] = (databuf[0] & 0x3f);
    if ((databuf[0] != MXC400X_ID_1) && (databuf[0] != MXC400X_ID_2))
    {
        return MXC400X_ERR_IDENTIFICATION;
    }

    GSE_INFO("MXC400X_CheckDeviceID %d done!\n ", databuf[0]);

    return MXC400X_SUCCESS;
}

static int MXC400X_SetPowerMode(struct i2c_client *client, bool enable)
{
    u8 databuf[2] = {0};
    int res = 0, i = 0;

    databuf[0] = MXC400X_REG_CTRL;
    databuf[1] = enable ? MXC400X_AWAKE : MXC400X_SLEEP;
    while (i++ < 3)
    {
        res = i2c_master_send(client, databuf, 0x2);
        msleep(5);
        if (res > 0)
            break;
    }

    if (res <= 0)
    {
        GSE_ERR("memsic set power mode failed!\n");
        return MXC400X_ERR_I2C;
    }
#ifdef USE_DELAY
    delay_state = enable;
#else
    msleep(300);
#endif
    return MXC400X_SUCCESS;
}

static int MXC400X_SetDataResolution(struct mxc400x_i2c_data *obj)
{
    obj->reso = &mxc400x_data_resolution[2];
    return MXC400X_SUCCESS;
}

static int MXC400X_SetDataFormat(struct i2c_client *client, u8 dataformat)
{
    struct mxc400x_i2c_data *obj = i2c_get_clientdata(client);
    u8 databuf[10];
    int res = 0;

    memset(databuf, 0, sizeof(u8) * 10);

    databuf[0] = MXC400X_REG_CTRL;
    databuf[1] = MXC400X_RANGE_8G;
    res = i2c_master_send(client, databuf, 0x2);
    if (res <= 0)
    {
        GSE_ERR("set power mode failed!\n");
        return MXC400X_ERR_I2C;
    }

    return MXC400X_SetDataResolution(obj);
}

static int mxc400x_init_client(struct i2c_client *client, int reset_cali)
{
    int res = 0;

    GSE_DEBUG_FUNC();
    res = MXC400X_SetPowerMode(client, true);
    if (res != MXC400X_SUCCESS)
    {
        return res;
    }

    res = MXC400X_CheckDeviceID(client);
    if (res != MXC400X_SUCCESS)
    {
        GSE_ERR("MXC400X check device id failed\n");
        return res;
    }

    res = MXC400X_SetDataFormat(client, MXC400X_RANGE_8G);
    if (res != MXC400X_SUCCESS)
    {
        return res;
    }

    if (reset_cali)
    {
        /*reset calibration only in power on*/
        res = MXC400X_ResetCalibration(client);
        if (res != MXC400X_SUCCESS)
        {
            return res;
        }
    }
    GSE_INFO("mxc400x_init_client OK!\n");

    return MXC400X_SUCCESS;
}

static int MXC400X_ReadSensorData(struct i2c_client *client, int buf[MXC400X_AXES_NUM])
{
    struct mxc400x_i2c_data *obj = i2c_get_clientdata(client);
    int acc[MXC400X_AXES_NUM] = {0};
    int res = 0;

    GSE_DEBUG_FUNC();

    if (NULL == buf)
    {
        GSE_ERR("buf is null !!!\n");
        return -1;
    }
    if (NULL == client)
    {
        *buf = 0;
        GSE_ERR("client is null !!!\n");
        return MXC400X_ERR_STATUS;
    }

    if (!enable_status)
    {
        GSE_DEBUG("sensor in suspend read not data!\n");
        return MXC400X_ERR_GETGSENSORDATA;
    }

    if ((res = MXC400X_ReadData(client, obj->data)))
    {
        GSE_ERR("I2C error: ret value=%d", res);
        return MXC400X_ERR_I2C;
    }
    else
    {
        GSE_DEBUG("cali x = %d y = %d z = %d\n", obj->cali_sw[MXC400X_AXIS_X], obj->cali_sw[MXC400X_AXIS_Y], obj->cali_sw[MXC400X_AXIS_Z]);

		/*remap coordinate*/
		acc[obj->cvt.map[MXC400X_AXIS_X]] = obj->cvt.sign[MXC400X_AXIS_X] * obj->data[MXC400X_AXIS_X];
		acc[obj->cvt.map[MXC400X_AXIS_Y]] = obj->cvt.sign[MXC400X_AXIS_Y] * obj->data[MXC400X_AXIS_Y];
		acc[obj->cvt.map[MXC400X_AXIS_Z]] = obj->cvt.sign[MXC400X_AXIS_Z] * obj->data[MXC400X_AXIS_Z];

		// Out put the mg
		acc[MXC400X_AXIS_X] = (obj->data[MXC400X_AXIS_X] * GRAVITY_EARTH_1000 + obj->cali_sw[MXC400X_AXIS_X]) / obj->reso->sensitivity;
		acc[MXC400X_AXIS_Y] = (obj->data[MXC400X_AXIS_Y] * GRAVITY_EARTH_1000 + obj->cali_sw[MXC400X_AXIS_Y]) / obj->reso->sensitivity;
		acc[MXC400X_AXIS_Z] = ((obj->data[MXC400X_AXIS_Z] + MXC400X_AXIS_Z_PAD) * GRAVITY_EARTH_1000 + obj->cali_sw[MXC400X_AXIS_Z]) / obj->reso->sensitivity;

        GSE_DEBUG("accel x = %d y = %d z = %d\n", acc[MXC400X_AXIS_X], acc[MXC400X_AXIS_Y], acc[MXC400X_AXIS_Z]);

		buf[0] = acc[MXC400X_AXIS_X];
		buf[1] = acc[MXC400X_AXIS_Y];
		buf[2] = acc[MXC400X_AXIS_Z];
    }

    return res;

}

static int MXC400X_ReadRawData(struct i2c_client *client, s16 *buf)
{
    struct mxc400x_i2c_data *obj = (struct mxc400x_i2c_data *)i2c_get_clientdata(client);
    int res = 0;

    if (!buf || !client)
    {
        GSE_ERR(" buf or client is null !!\n");
        return EINVAL;
    }

    if ((res = MXC400X_ReadData(client, obj->data)))
    {
        GSE_ERR("I2C error: ret value=%d\n", res);
        return EIO;
    }
    else
    {
        buf[0] = obj->data[MXC400X_AXIS_X];
        buf[1] = obj->data[MXC400X_AXIS_Y];
        buf[2] = obj->data[MXC400X_AXIS_Z];
    }

    return MXC400X_SUCCESS;
}

// if use  this typ of enable , Gsensor should report inputEvent(x, y, z ,stats, div) to HAL
static int mxc400x_open_report_data(int open)
{
    // should queuq work to report event if  is_report_input_direct=true
    return 0;
}

// if use  this typ of enable , Gsensor only enabled but not report inputEvent to HAL
static int mxc400x_enable_nodata(int en)
{
    int res = 0;
    bool power = 1 == en;

    res = MXC400X_SetPowerMode(obj_i2c_data->client, power);
    if (res != MXC400X_SUCCESS)
    {
        GSE_ERR("MXC400X_SetPowerMode fail!\n");
        return -1;
    }

    enable_status = power;
    return 0;
}

static int mxc400x_batch(int flag, int64_t samplingPeriodNs, int64_t maxBatchReportLatencyNs)
{
    int value = 0;

    value = (int)samplingPeriodNs / 1000 / 1000;
    /*Fix Me*/
    GSE_INFO("mxc400x set delay = (%d) OK!\n", value);

    return 0;
}

static int mxc400x_flush(void)
{
    return acc_flush_report();
}

static int mxc400x_set_delay(u64 ns)
{
    return 0;
}

static int mxc400x_get_data(int *x, int *y, int *z, int *status)
{
    int buf[MXC400X_AXES_NUM] = {0};

    int err = MXC400X_ReadSensorData(obj_i2c_data->client, buf);
    if (err != MXC400X_SUCCESS)
    {
        GSE_ERR("mxc400x_get_data fail!\n");
        return -1;
    }
    *x = buf[0];
    *y = buf[1];
    *z = buf[2];
    *status = SENSOR_STATUS_ACCURACY_MEDIUM;

    return err;
}

static int mxc400x_factory_enable_sensor(bool enabledisable, int64_t sample_periods_ms)
{
    int err;

    err = mxc400x_enable_nodata(enabledisable ? 1 : 0);
    if (err)
    {
        GSE_ERR("%s enable sensor failed!\n", __func__);
        return -1;
    }
    err = mxc400x_batch(0, sample_periods_ms * 1000000, 0);
    if (err)
    {
        GSE_ERR("%s enable set batch failed!\n", __func__);
        return -1;
    }
    return 0;
}

static int mxc400x_factory_get_data(int32_t data[3], int *status)
{
    return mxc400x_get_data(&data[0], &data[1], &data[2], status);
}

static int mxc400x_factory_get_raw_data(int32_t data[3])
{
	s16 buf[3];
    int res = MXC400X_ReadRawData(mxc400x_i2c_client, buf);
    if (res != MXC400X_SUCCESS)
    {
        GSE_ERR("MXC400X_ReadRawData fail!\n");
        return -1;
    }
	data[0] = buf[0];
	data[1] = buf[1];
	data[2] = buf[2];
	GSE_ERR("support mxc400x_factory_get_raw_data!\n");
    return 0;
}

static int mxc400x_factory_enable_calibration(void)
{
    return 0;
}

static int mxc400x_factory_clear_cali(void)
{
    int err = 0;

    err = MXC400X_ResetCalibration(mxc400x_i2c_client);
    if (err)
    {
        GSE_ERR("mxc400x_ResetCalibration failed!\n");
        return -1;
    }
    return 0;
}

static int mxc400x_factory_set_cali(int32_t data[3])
{
    if (MXC400X_WriteCalibration(mxc400x_i2c_client, data))
    {
        GSE_ERR("mxc400x_WriteCalibration failed!\n");
        return -1;
    }
    return 0;
}
static int mxc400x_factory_get_cali(int32_t data[3])
{
    int cali[MXC400X_AXES_NUM];
    int err = 0;
    if ((err = MXC400X_ReadCalibrationEx(mxc400x_i2c_client, cali))) /*offset will be updated in obj->offset*/
    {
        GSE_ERR("read offset fail, %d\n", err);
        return err;
    }
    data[0] = cali[MXC400X_AXIS_X];
    data[1] = cali[MXC400X_AXIS_Y];
    data[2] = cali[MXC400X_AXIS_Z];
    return 0;
}
static int mxc400x_factory_do_self_test(void)
{
    return 0;
}

static struct accel_factory_fops mxc400x_factory_fops = {
    .enable_sensor = mxc400x_factory_enable_sensor,
    .get_data = mxc400x_factory_get_data,
    .get_raw_data = mxc400x_factory_get_raw_data,
    .enable_calibration = mxc400x_factory_enable_calibration,
    .clear_cali = mxc400x_factory_clear_cali,
    .set_cali = mxc400x_factory_set_cali,
    .get_cali = mxc400x_factory_get_cali,
    .do_self_test = mxc400x_factory_do_self_test,
};

static struct accel_factory_public mxc400x_factory_device = {
    .gain = 1,
    .sensitivity = 1,
    .fops = &mxc400x_factory_fops,
};

/*----------------------------------------------------------------------------*/
static int mxc400x_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
    struct mxc400x_i2c_data *obj = NULL;
    struct acc_control_path ctl = {0};
    struct acc_data_path data = {0};
    int err = 0;

    GSE_DEBUG_FUNC();
    GSE_INFO("driver version = %s\n", DRIVER_VERSION);

    if (!(obj = kzalloc(sizeof(*obj), GFP_KERNEL)))
    {
        err = -ENOMEM;
        goto exit;
    }

    err = get_accel_dts_func(client->dev.of_node, &obj->hw);
    if (err < 0)
    {
        GSE_ERR("get dts info fail\n");
        goto exit_kfree;
    }

    err = hwmsen_get_convert(obj->hw.direction, &obj->cvt);
    if (0 != err)
    {
        GSE_ERR("invalid direction: %d\n", obj->hw.direction);
        goto exit_kfree;
    }

    obj->client = client;
    obj_i2c_data = obj;
    mxc400x_i2c_client = obj->client;
    i2c_set_clientdata(mxc400x_i2c_client, obj);

    atomic_set(&obj->trace, 0);

    if ((err = mxc400x_init_client(mxc400x_i2c_client, 1)))
    {
        goto exit_init_failed;
    }

    /* factory */
    err = accel_factory_device_register(&mxc400x_factory_device);
    if (err)
    {
        GSE_ERR("mxc400x_device register failed.\n");
        goto exit_misc_device_register_failed;
    }

    ctl.open_report_data = mxc400x_open_report_data;
    ctl.enable_nodata = mxc400x_enable_nodata;
    ctl.set_delay = mxc400x_set_delay;
    ctl.batch = mxc400x_batch;
    ctl.flush = mxc400x_flush;
    ctl.is_report_input_direct = false;
    ctl.is_use_common_factory = false;

    err = acc_register_control_path(&ctl);
    if (err)
    {
        GSE_ERR("register acc control path err\n");
        goto exit_kfree;
    }

    data.get_data = mxc400x_get_data;
    data.vender_div = 1000;
    err = acc_register_data_path(&data);
    if (err)
    {
        GSE_ERR("register acc data path err\n");
        goto exit_kfree;
    }
    mxc400x_init_flag = 0;
    GSE_INFO("%s: OK\n", __func__);

    return 0;

exit_misc_device_register_failed:
exit_init_failed:
exit_kfree:
    kfree(obj);
exit:
    GSE_ERR("%s: err = %d\n", __func__, err);
    mxc400x_init_flag = -1;
    obj = NULL;
    obj_i2c_data = NULL;
    mxc400x_i2c_client = NULL;
    return err;
}

static int mxc400x_i2c_remove(struct i2c_client *client)
{
    mxc400x_i2c_client = NULL;
    i2c_unregister_device(client);
    accel_factory_device_deregister(&mxc400x_factory_device);
    kfree(i2c_get_clientdata(client));
    return 0;
}

static int mxc400x_local_init(void)
{
    GSE_DEBUG_FUNC();

    if (i2c_add_driver(&mxc400x_i2c_driver))
    {
        GSE_ERR("add driver error\n");
        return -1;
    }
    if (-1 == mxc400x_init_flag)
    {
        GSE_ERR("mxc400x_local_init failed mxc400x_init_flag=%d\n", mxc400x_init_flag);
        return -1;
    }
    return 0;
}
static int mxc400x_remove(void)
{
    GSE_DEBUG_FUNC();
    i2c_del_driver(&mxc400x_i2c_driver);
    return 0;
}

static int __init mxc400x_driver_init(void)
{

    GSE_DEBUG_FUNC();

    acc_driver_add(&mxc400x_init_info);

    mutex_init(&mxc400x_mutex);

    return 0;
}

static void __exit mxc400x_driver_exit(void)
{
    GSE_DEBUG_FUNC();
    mutex_destroy(&mxc400x_mutex);
}

module_init(mxc400x_driver_init);
module_exit(mxc400x_driver_exit);

MODULE_AUTHOR("Lyon Miao<xlmiao@memsic.com>");
MODULE_DESCRIPTION("MEMSIC MXC400x Accelerometer Sensor Driver");
MODULE_LICENSE("GPL");