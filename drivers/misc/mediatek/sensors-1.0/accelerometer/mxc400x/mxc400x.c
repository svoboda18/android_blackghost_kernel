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

#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/irq.h>
#include <linux/miscdevice.h>
#include <asm/uaccess.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/workqueue.h>
#include <linux/kobject.h>
#include <linux/platform_device.h>
#include <asm/atomic.h>
#include <linux/time.h>
#include <linux/kernel.h>


#include "accel.h"
#include "cust_acc.h"
#include "sensors_io.h"
#include "mxc400x.h"


#define I2C_DRIVERID_MXC400X		120
#define SW_CALIBRATION
#define DRIVER_VERSION				"V60.97.05.01"
#define GSE_DEBUG_ON          		0
#define GSE_DEBUG_FUNC_ON     		0
/* Log define */
#define GSE_INFO(fmt, arg...)      	pr_warn("<<-GSE INFO->> "fmt"\n", ##arg)
#define GSE_ERR(fmt, arg...)          	pr_err("<<-GSE ERROR->> "fmt"\n", ##arg)
#define GSE_DEBUG(fmt, arg...)		do {\
						if (GSE_DEBUG_ON)\
							pr_warn("<<-GSE DEBUG->> [%d]"fmt"\n", __LINE__, ##arg);\
					} while (0)
#define GSE_DEBUG_FUNC()		do {\
						if (GSE_DEBUG_FUNC_ON)\
							pr_debug("<<-GSE FUNC->> Func:%s@Line:%d\n", __func__, __LINE__);\
					} while (0)

#define MXC400X_AXIS_X          	0
#define MXC400X_AXIS_Y          	1
#define MXC400X_AXIS_Z          	2
#define MXC400X_AXES_NUM        	3
#define MXC400X_DATA_LEN        	6
#define C_MAX_FIR_LENGTH 		(32)
#define USE_DELAY
#ifdef USE_DELAY
static int delay_state = 0;
#endif
static const struct i2c_device_id mxc400x_i2c_id[] = { { MXC400X_DEV_NAME, 0 }, { }, };
static int mxc400x_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id);
static int mxc400x_i2c_remove(struct i2c_client *client);
#ifdef CONFIG_PM_SLEEP
static int mxc400x_suspend(struct device *dev);
static int mxc400x_resume(struct device *dev);
#endif
static int  mxc400x_local_init(void);
static int mxc400x_remove(void);
typedef enum {
	ADX_TRC_FILTER   = 0x01,
	ADX_TRC_RAWDATA  = 0x02,
	ADX_TRC_IOCTL	 = 0x04,
	ADX_TRC_CALI	 = 0X08,
	ADX_TRC_INFO	 = 0X10,
} ADX_TRC;

struct scale_factor{
	u8  whole;
	u8  fraction;
};

struct data_resolution {
	struct scale_factor scalefactor;
	int    sensitivity;
};


struct data_filter {
	s16 raw[C_MAX_FIR_LENGTH][MXC400X_AXES_NUM];
	int sum[MXC400X_AXES_NUM];
	int num;
	int idx;
};
static int mxc400x_init_flag = -1;
/*----------------------------------------------------------------------------*/
static struct acc_init_info mxc400x_init_info = {
		.name = "mxc400x",
		.init = mxc400x_local_init,
		.uninit = mxc400x_remove,
};
struct mxc400x_i2c_data {
		 struct i2c_client *client;
		 struct acc_hw hw;
		 struct hwmsen_convert	 cvt;
		 atomic_t layout;
		 /*misc*/
		 struct data_resolution *reso;
		 atomic_t				 trace;
		 atomic_t				 suspend;
		 atomic_t				 selftest;
		 atomic_t				 filter;
		 s16					 cali_sw[MXC400X_AXES_NUM+1];

		 /*data*/
		 s8 					 offset[MXC400X_AXES_NUM+1];  /*+1: for 4-byte alignment*/
		 s16					 data[MXC400X_AXES_NUM+1];
#if defined(CONFIG_MXC400X_LOWPASS)
		 atomic_t				 firlen;
		 atomic_t				 fir_en;
		 struct data_filter 	 fir;
#endif
};

#ifdef CONFIG_OF
static const struct of_device_id accel_of_match[] = {
	{.compatible = "mediatek,gsensor"},
	{},
};
#endif

#ifdef CONFIG_PM_SLEEP
static const struct dev_pm_ops mxc400x_i2c_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(mxc400x_suspend, mxc400x_resume)
};
#endif

static struct i2c_driver mxc400x_i2c_driver = {
	 .driver = {
		// .owner		   = THIS_MODULE,
		.name			 = MXC400X_DEV_NAME,
#ifdef CONFIG_PM_SLEEP
		.pm = &mxc400x_i2c_pm_ops,
#endif
	   #ifdef CONFIG_OF
		.of_match_table = accel_of_match,
	   #endif
	 },
	 .probe 			 = mxc400x_i2c_probe,
	 .remove			 = mxc400x_i2c_remove,
	 .id_table = mxc400x_i2c_id,
};

struct i2c_client *mxc400x_i2c_client = NULL;
static struct mxc400x_i2c_data 	*obj_i2c_data = NULL;
static bool sensor_power = false;
static struct GSENSOR_VECTOR3D gsensor_gain;
static struct mutex mxc400x_mutex;
static bool enable_status = false;

static struct data_resolution mxc400x_data_resolution[] = {
    {{ 0, 9}, 1024},   /*+/-2g  in 12-bit resolution:  0.9 mg/LSB*/
    {{ 1, 9}, 512},   /*+/-4g  in 12-bit resolution:  1.9 mg/LSB*/
    {{ 3, 9},  256},   /*+/-8g  in 12-bit resolution: 3.9 mg/LSB*/
};
static struct data_resolution mxc400x_offset_resolution = {{3, 9}, 256};

static int mxc400x_i2c_read_block(struct i2c_client *client, u8 addr, u8 *data, u8 len)
{
	u8 beg = addr;
	struct i2c_msg msgs[2] = {
		{
			.addr = client->addr,	.flags = 0,
			.len = 1,	.buf = &beg
		},
		{
			.addr = client->addr,	.flags = I2C_M_RD,
			.len = len,	.buf = data,
		}
	};
	int err;

	if (!client)
		return -EINVAL;
	else if (len > C_I2C_FIFO_SIZE) {
		GSE_ERR(" length %d exceeds %d\n", len, C_I2C_FIFO_SIZE);
		return -EINVAL;
	}

	err = i2c_transfer(client->adapter, msgs, sizeof(msgs)/sizeof(msgs[0]));
	if (err != 2) {
		GSE_ERR("i2c_transfer error: (%d %p %d) %d\n",
				addr, data, len, err);
		err = -EIO;
	} else {
		err = 0;
	}
	return err;

}

static int MXC400X_SetDataResolution(struct mxc400x_i2c_data *obj)
{
 	obj->reso = &mxc400x_data_resolution[2];
	return MXC400X_SUCCESS;
}
static int MXC400X_ReadData(struct i2c_client *client, s16 data[MXC400X_AXES_NUM])
{
#ifdef CONFIG_MXC400X_LOWPASS
	mxc400x_i2c_data *priv = i2c_get_clientdata(client);
#endif
	u8 addr = MXC400X_REG_X;
	u8 buf[MXC400X_DATA_LEN] = {0};
	int err = 0;

#ifdef USE_DELAY
	if(delay_state){
		msleep(150);
		delay_state = 0;
	}
#endif

	if(NULL == client)
	{
        	GSE_ERR("client is null\n");
		err = -EINVAL;
	}
	if((err = mxc400x_i2c_read_block(client, addr, buf, MXC400X_DATA_LEN))!=0)
	{
		GSE_ERR("error: %d\n", err);
	}
	else
	{
		data[MXC400X_AXIS_X] = (s16)(buf[0] << 8 | buf[1]) >> 4;
		data[MXC400X_AXIS_Y] = (s16)(buf[2] << 8 | buf[3]) >> 4;
		data[MXC400X_AXIS_Z] = (s16)(buf[4] << 8 | buf[5]) >> 4;
		GSE_DEBUG("reg data x = %d %d y = %d %d z = %d %d\n", buf[0], buf[1], buf[2], buf[3], buf[4], buf[5]);

#ifdef CONFIG_MXC400X_LOWPASS
		if(atomic_read(&priv->filter))
		{
			if(atomic_read(&priv->fir_en) && !atomic_read(&priv->suspend))
			{
				int idx, firlen = atomic_read(&priv->firlen);
				if(priv->fir.num < firlen)
				{
					priv->fir.raw[priv->fir.num][MXC400X_AXIS_X] = data[MXC400X_AXIS_X];
					priv->fir.raw[priv->fir.num][MXC400X_AXIS_Y] = data[MXC400X_AXIS_Y];
					priv->fir.raw[priv->fir.num][MXC400X_AXIS_Z] = data[MXC400X_AXIS_Z];
					priv->fir.sum[MXC400X_AXIS_X] += data[MXC400X_AXIS_X];
					priv->fir.sum[MXC400X_AXIS_Y] += data[MXC400X_AXIS_Y];
					priv->fir.sum[MXC400X_AXIS_Z] += data[MXC400X_AXIS_Z];
					if(atomic_read(&priv->trace) & ADX_TRC_FILTER)
					{
						GSE_DEBUG("add [%2d] [%5d %5d %5d] => [%5d %5d %5d]\n", priv->fir.num,
							priv->fir.raw[priv->fir.num][MXC400X_AXIS_X], priv->fir.raw[priv->fir.num][MXC400X_AXIS_Y], priv->fir.raw[priv->fir.num][MXC400X_AXIS_Z],
							priv->fir.sum[MXC400X_AXIS_X], priv->fir.sum[MXC400X_AXIS_Y], priv->fir.sum[MXC400X_AXIS_Z]);
					}
					priv->fir.num++;
					priv->fir.idx++;
				}
				else
				{
					idx = priv->fir.idx % firlen;
					priv->fir.sum[MXC400X_AXIS_X] -= priv->fir.raw[idx][MXC400X_AXIS_X];
					priv->fir.sum[MXC400X_AXIS_Y] -= priv->fir.raw[idx][MXC400X_AXIS_Y];
					priv->fir.sum[MXC400X_AXIS_Z] -= priv->fir.raw[idx][MXC400X_AXIS_Z];
					priv->fir.raw[idx][MXC400X_AXIS_X] = data[MXC400X_AXIS_X];
					priv->fir.raw[idx][MXC400X_AXIS_Y] = data[MXC400X_AXIS_Y];
					priv->fir.raw[idx][MXC400X_AXIS_Z] = data[MXC400X_AXIS_Z];
					priv->fir.sum[MXC400X_AXIS_X] += data[MXC400X_AXIS_X];
					priv->fir.sum[MXC400X_AXIS_Y] += data[MXC400X_AXIS_Y];
					priv->fir.sum[MXC400X_AXIS_Z] += data[MXC400X_AXIS_Z];
					priv->fir.idx++;
					data[MXC400X_AXIS_X] = priv->fir.sum[MXC400X_AXIS_X]/firlen;
					data[MXC400X_AXIS_Y] = priv->fir.sum[MXC400X_AXIS_Y]/firlen;
					data[MXC400X_AXIS_Z] = priv->fir.sum[MXC400X_AXIS_Z]/firlen;
					if(atomic_read(&priv->trace) & ADX_TRC_FILTER)
					{
						GSE_DEBUG("add [%2d] [%5d %5d %5d] => [%5d %5d %5d] : [%5d %5d %5d]\n", idx,
						priv->fir.raw[idx][MXC400X_AXIS_X], priv->fir.raw[idx][MXC400X_AXIS_Y], priv->fir.raw[idx][MXC400X_AXIS_Z],
						priv->fir.sum[MXC400X_AXIS_X], priv->fir.sum[MXC400X_AXIS_Y], priv->fir.sum[MXC400X_AXIS_Z],
						data[MXC400X_AXIS_X], data[MXC400X_AXIS_Y], data[MXC400X_AXIS_Z]);
					}
				}
			}
		}
#endif
	}
	return err;
}

static int MXC400X_ReadOffset(struct i2c_client *client, s8 ofs[MXC400X_AXES_NUM])
{
	int err;
	err = 0;
#ifdef SW_CALIBRATION
	ofs[0]=ofs[1]=ofs[2]=0x0;
#endif
	return err;
}

static int MXC400X_ResetCalibration(struct i2c_client *client)
{
	struct mxc400x_i2c_data *obj = i2c_get_clientdata(client);
	int err;
	err = 0;

	memset(obj->cali_sw, 0x00, sizeof(obj->cali_sw));
	memset(obj->offset, 0x00, sizeof(obj->offset));
	return err;
}

static int MXC400X_ReadCalibration(struct i2c_client *client, int dat[MXC400X_AXES_NUM])
{
	struct mxc400x_i2c_data *obj = i2c_get_clientdata(client);
	int  err = 0;
	int mul;

#ifdef SW_CALIBRATION
	mul = 0;//only SW Calibration, disable HW Calibration
#else
	if ((err = MXC400X_ReadOffset(client, obj->offset))) {
		GSE_ERR("read offset fail, %d\n", err);
		return err;
	}
	mul = obj->reso->sensitivity/mxc400x_offset_resolution.sensitivity;
#endif

	dat[obj->cvt.map[MXC400X_AXIS_X]] = obj->cvt.sign[MXC400X_AXIS_X]*(obj->offset[MXC400X_AXIS_X]*mul + obj->cali_sw[MXC400X_AXIS_X]);
	dat[obj->cvt.map[MXC400X_AXIS_Y]] = obj->cvt.sign[MXC400X_AXIS_Y]*(obj->offset[MXC400X_AXIS_Y]*mul + obj->cali_sw[MXC400X_AXIS_Y]);
	dat[obj->cvt.map[MXC400X_AXIS_Z]] = obj->cvt.sign[MXC400X_AXIS_Z]*(obj->offset[MXC400X_AXIS_Z]*mul + obj->cali_sw[MXC400X_AXIS_Z]);

	return err;
}
static int MXC400X_ReadCalibrationEx(struct i2c_client *client, int act[MXC400X_AXES_NUM], int raw[MXC400X_AXES_NUM])
{
	/*raw: the raw calibration data; act: the actual calibration data*/
	struct mxc400x_i2c_data *obj = i2c_get_clientdata(client);
	int err;
	int mul;
	err = 0;


#ifdef SW_CALIBRATION
	mul = 0;//only SW Calibration, disable HW Calibration
#else
	if((err = MXC400X_ReadOffset(client, obj->offset)))
	{
		GSE_ERR("read offset fail, %d\n", err);
		return err;
	}
	mul = obj->reso->sensitivity/mxc400x_offset_resolution.sensitivity;
#endif

	raw[MXC400X_AXIS_X] = obj->offset[MXC400X_AXIS_X]*mul + obj->cali_sw[MXC400X_AXIS_X];
	raw[MXC400X_AXIS_Y] = obj->offset[MXC400X_AXIS_Y]*mul + obj->cali_sw[MXC400X_AXIS_Y];
	raw[MXC400X_AXIS_Z] = obj->offset[MXC400X_AXIS_Z]*mul + obj->cali_sw[MXC400X_AXIS_Z];

	act[obj->cvt.map[MXC400X_AXIS_X]] = obj->cvt.sign[MXC400X_AXIS_X]*raw[MXC400X_AXIS_X];
	act[obj->cvt.map[MXC400X_AXIS_Y]] = obj->cvt.sign[MXC400X_AXIS_Y]*raw[MXC400X_AXIS_Y];
	act[obj->cvt.map[MXC400X_AXIS_Z]] = obj->cvt.sign[MXC400X_AXIS_Z]*raw[MXC400X_AXIS_Z];

	return 0;
}
static int MXC400X_WriteCalibration(struct i2c_client *client, int dat[MXC400X_AXES_NUM])
{
	struct mxc400x_i2c_data *obj = i2c_get_clientdata(client);
	int err = 0;
	int cali[MXC400X_AXES_NUM];
#ifdef SW_CALIBRATION
#else
	int lsb;
	lsb = mxc400x_offset_resolution.sensitivity;
	int divisor = obj->reso->sensitivity/lsb;
#endif
	int raw[MXC400X_AXES_NUM];
	if((err = MXC400X_ReadCalibrationEx(client, cali, raw)))	/*offset will be updated in obj->offset*/
	{
		GSE_ERR("read offset fail, %d\n", err);
		return err;
	}

	GSE_DEBUG("OLDOFF: (%+3d %+3d %+3d): (%+3d %+3d %+3d) / (%+3d %+3d %+3d)\n",
			raw[MXC400X_AXIS_X], raw[MXC400X_AXIS_Y], raw[MXC400X_AXIS_Z],
			obj->offset[MXC400X_AXIS_X], obj->offset[MXC400X_AXIS_Y], obj->offset[MXC400X_AXIS_Z],
			obj->cali_sw[MXC400X_AXIS_X], obj->cali_sw[MXC400X_AXIS_Y], obj->cali_sw[MXC400X_AXIS_Z]);

	/*calculate the real offset expected by caller*/
	cali[MXC400X_AXIS_X] += dat[MXC400X_AXIS_X];
	cali[MXC400X_AXIS_Y] += dat[MXC400X_AXIS_Y];
	cali[MXC400X_AXIS_Z] += dat[MXC400X_AXIS_Z];

	GSE_DEBUG("UPDATE: (%+3d %+3d %+3d)\n",
			dat[MXC400X_AXIS_X], dat[MXC400X_AXIS_Y], dat[MXC400X_AXIS_Z]);

#ifdef SW_CALIBRATION
	obj->cali_sw[MXC400X_AXIS_X] = obj->cvt.sign[MXC400X_AXIS_X]*(cali[obj->cvt.map[MXC400X_AXIS_X]]);
	obj->cali_sw[MXC400X_AXIS_Y] = obj->cvt.sign[MXC400X_AXIS_Y]*(cali[obj->cvt.map[MXC400X_AXIS_Y]]);
	obj->cali_sw[MXC400X_AXIS_Z] = obj->cvt.sign[MXC400X_AXIS_Z]*(cali[obj->cvt.map[MXC400X_AXIS_Z]]);
#else
//	int divisor = obj->reso->sensitivity/lsb;//modified
	obj->offset[MXC400X_AXIS_X] = (s8)(obj->cvt.sign[MXC400X_AXIS_X]*(cali[obj->cvt.map[MXC400X_AXIS_X]])/(divisor));
	obj->offset[MXC400X_AXIS_Y] = (s8)(obj->cvt.sign[MXC400X_AXIS_Y]*(cali[obj->cvt.map[MXC400X_AXIS_Y]])/(divisor));
	obj->offset[MXC400X_AXIS_Z] = (s8)(obj->cvt.sign[MXC400X_AXIS_Z]*(cali[obj->cvt.map[MXC400X_AXIS_Z]])/(divisor));

	/*convert software calibration using standard calibration*/
	obj->cali_sw[MXC400X_AXIS_X] = obj->cvt.sign[MXC400X_AXIS_X]*(cali[obj->cvt.map[MXC400X_AXIS_X]])%(divisor);
	obj->cali_sw[MXC400X_AXIS_Y] = obj->cvt.sign[MXC400X_AXIS_Y]*(cali[obj->cvt.map[MXC400X_AXIS_Y]])%(divisor);
	obj->cali_sw[MXC400X_AXIS_Z] = obj->cvt.sign[MXC400X_AXIS_Z]*(cali[obj->cvt.map[MXC400X_AXIS_Z]])%(divisor);

	GSE_DEBUG("NEWOFF: (%+3d %+3d %+3d): (%+3d %+3d %+3d) / (%+3d %+3d %+3d)\n",
			obj->offset[MXC400X_AXIS_X]*divisor + obj->cali_sw[MXC400X_AXIS_X],
			obj->offset[MXC400X_AXIS_Y]*divisor + obj->cali_sw[MXC400X_AXIS_Y],
			obj->offset[MXC400X_AXIS_Z]*divisor + obj->cali_sw[MXC400X_AXIS_Z],
			obj->offset[MXC400X_AXIS_X], obj->offset[MXC400X_AXIS_Y], obj->offset[MXC400X_AXIS_Z],
			obj->cali_sw[MXC400X_AXIS_X], obj->cali_sw[MXC400X_AXIS_Y], obj->cali_sw[MXC400X_AXIS_Z]);

#endif
	msleep(4);
	return err;
}

static int MXC400X_CheckDeviceID(struct i2c_client *client)
{
	u8 databuf[10];
	int res = 0;

	memset(databuf, 0, sizeof(u8)*10);
	databuf[0] = MXC400X_REG_ID;
  msleep(12);
	res = mxc400x_i2c_read_block(client,MXC400X_REG_ID,databuf,0x01);
	if (res)
	{
		GSE_ERR("MXC400X Device ID read faild\n");
		return MXC400X_ERR_I2C;

	}

	databuf[0]= (databuf[0]&0x3f);

	if((databuf[0]!= MXC400X_ID_1) && (databuf[0] != MXC400X_ID_2))
	{
		return MXC400X_ERR_IDENTIFICATION;
	}

	GSE_INFO("MXC400X_CheckDeviceID %d done!\n ", databuf[0]);

	return MXC400X_SUCCESS;
}

static int MXC400X_SetPowerMode(struct i2c_client *client, bool enable)
{
	u8 databuf[2] = {0};
	int res = 0, i=0;

	if(enable == 1)
	{
		databuf[1] = MXC400X_AWAKE;
	}
	else
	{
		databuf[1] = MXC400X_SLEEP;
	}
	msleep(MXC400X_STABLE_DELAY);
	databuf[0] = MXC400X_REG_CTRL;
	while (i++ < 3)
	{
		res = i2c_master_send(client, databuf, 0x2);
		msleep(5);
		if(res > 0)
			break;
	}

	if(res <= 0)
	{
		GSE_ERR("memsic set power mode failed!\n");
		return MXC400X_ERR_I2C;
	}
	sensor_power = enable;
#ifdef USE_DELAY
	delay_state = enable;
#else
	msleep(300);
#endif
	return MXC400X_SUCCESS;
}
static int MXC400X_SetDataFormat(struct i2c_client *client, u8 dataformat)
{
	struct mxc400x_i2c_data *obj = i2c_get_clientdata(client);
	u8 databuf[10];
	int res = 0;

	memset(databuf, 0, sizeof(u8)*10);
	databuf[0] = MXC400X_REG_CTRL;
	databuf[1] = MXC400X_RANGE_8G;

	res = i2c_master_send(client, databuf, 0x2);
	if(res <= 0)
	{
		GSE_ERR("set power mode failed!\n");
		return MXC400X_ERR_I2C;
	}

	return MXC400X_SetDataResolution(obj);
}
static int MXC400X_SetBWRate(struct i2c_client *client, u8 bwrate)
{
	u8 databuf[10];

	memset(databuf, 0, sizeof(u8)*10);

	return MXC400X_SUCCESS;
}

static int mxc400x_init_client(struct i2c_client *client, int reset_cali)
{
	 struct mxc400x_i2c_data *obj = i2c_get_clientdata(client);
	 int res = 0;

	 GSE_DEBUG_FUNC();
	 res = MXC400X_SetPowerMode(client, true);
	 if(res != MXC400X_SUCCESS)
	 {
		return res;
	 }
	 res = MXC400X_CheckDeviceID(client);
	 if(res != MXC400X_SUCCESS)
	 {
	 	 GSE_ERR("MXC400X check device id failed\n");
	 	 return res;
	 }

	res = MXC400X_SetBWRate(client, MXC400X_BW_50HZ);
	if(res != MXC400X_SUCCESS )
	{
		GSE_ERR("MXC400X Set BWRate failed\n");
		return res;
	}

	res = MXC400X_SetDataFormat(client, MXC400X_RANGE_8G);
	if(res != MXC400X_SUCCESS)
	{
		return res;
	}

	gsensor_gain.x = gsensor_gain.y = gsensor_gain.z = obj->reso->sensitivity;

	if(0 != reset_cali)
	{
	 	/*reset calibration only in power on*/
		 res = MXC400X_ResetCalibration(client);
		 if(res != MXC400X_SUCCESS)
		 {
			 return res;
		 }
	}
	GSE_INFO("mxc400x_init_client OK!\n");
#ifdef CONFIG_MXC400X_LOWPASS
	memset(&obj->fir, 0x00, sizeof(obj->fir));
#endif
	msleep(20);

	return MXC400X_SUCCESS;
}

static int MXC400X_ReadChipInfo(struct i2c_client *client, char *buf, int bufsize)
{
	u8 databuf[10];

	memset(databuf, 0, sizeof(u8)*10);

	if((NULL == buf)||(bufsize<=30))
	{
		return -1;
	}

	if(NULL == client)
	{
		*buf = 0;
		return -2;
	}

	sprintf(buf, "MXC400X Chip");
	return 0;
}

static int MXC400X_ReadSensorData(struct i2c_client *client, int *buf, int bufsize)
{
	struct mxc400x_i2c_data *obj = (struct mxc400x_i2c_data*)i2c_get_clientdata(client);
	u8 databuf[20];
	int acc[MXC400X_AXES_NUM] = {0};
	int res = 0;
	GSE_DEBUG_FUNC();

	memset(databuf, 0, sizeof(u8)*10);

	if(NULL == buf)
	{
		GSE_ERR("buf is null !!!\n");
		return -1;
	}
	if(NULL == client)
	{
		*buf = 0;
		GSE_ERR("client is null !!!\n");
		return MXC400X_ERR_STATUS;
	}

	if (atomic_read(&obj->suspend) && !enable_status)
	{
		GSE_DEBUG("sensor in suspend read not data!\n");
		return MXC400X_ERR_GETGSENSORDATA;
	}


	if((res = MXC400X_ReadData(client, obj->data)))
	{
		GSE_ERR("I2C error: ret value=%d", res);
		return MXC400X_ERR_I2C;
	}
	else
	{
		obj->data[MXC400X_AXIS_X] += obj->cali_sw[MXC400X_AXIS_X];
		obj->data[MXC400X_AXIS_Y] += obj->cali_sw[MXC400X_AXIS_Y];
		obj->data[MXC400X_AXIS_Z] += obj->cali_sw[MXC400X_AXIS_Z];

		/*remap coordinate*/
		acc[obj->cvt.map[MXC400X_AXIS_X]] = obj->cvt.sign[MXC400X_AXIS_X]*obj->data[MXC400X_AXIS_X];
		acc[obj->cvt.map[MXC400X_AXIS_Y]] = obj->cvt.sign[MXC400X_AXIS_Y]*obj->data[MXC400X_AXIS_Y];
		acc[obj->cvt.map[MXC400X_AXIS_Z]] = obj->cvt.sign[MXC400X_AXIS_Z]*obj->data[MXC400X_AXIS_Z];

		//Out put the mg
		acc[MXC400X_AXIS_X] = acc[MXC400X_AXIS_X] * GRAVITY_EARTH_1000 / obj->reso->sensitivity;
		acc[MXC400X_AXIS_Y] = acc[MXC400X_AXIS_Y] * GRAVITY_EARTH_1000 / obj->reso->sensitivity;
		acc[MXC400X_AXIS_Z] = acc[MXC400X_AXIS_Z] * GRAVITY_EARTH_1000 / obj->reso->sensitivity;
		buf[0] = acc[MXC400X_AXIS_X];
		buf[1] = acc[MXC400X_AXIS_Y];
		buf[2] = acc[MXC400X_AXIS_Z];

	}

	return res;
}

static int MXC400X_ReadRawData(struct i2c_client *client, char *buf)
{
	struct mxc400x_i2c_data *obj = (struct mxc400x_i2c_data*)i2c_get_clientdata(client);
	int res = 0;

	if (!buf || !client)
	{
        GSE_ERR(" buf or client is null !!\n");
		return EINVAL;
	}

	if((res = MXC400X_ReadData(client, obj->data)))
	{
		GSE_ERR("I2C error: ret value=%d\n", res);
		return EIO;
	}
	else
	{
		buf[0] = (int)obj->data[MXC400X_AXIS_X];
		buf[1] = (int)obj->data[MXC400X_AXIS_Y];
		buf[2] = (int)obj->data[MXC400X_AXIS_Z];
	}

	return 0;
}


static ssize_t show_chipinfo_value(struct device_driver *ddri, char *buf)
{
	struct i2c_client *client = mxc400x_i2c_client;
	char strbuf[MXC400X_BUFSIZE];
	if(NULL == client)
	{
		GSE_ERR("i2c client is null!!\n");
		return 0;
	}

	MXC400X_ReadChipInfo(client, strbuf, MXC400X_BUFSIZE);
	return snprintf(buf, PAGE_SIZE, "%s\n", (char*)strbuf);
}

static ssize_t show_sensordata_value(struct device_driver *ddri, char *buf)
{
	struct i2c_client *client = mxc400x_i2c_client;
	int strbuf[MXC400X_BUFSIZE] = {0};
	if(NULL == client)
	{
		GSE_ERR("i2c client is null!!\n");
		return 0;
	}
	MXC400X_ReadSensorData(client, strbuf, MXC400X_BUFSIZE);
	return snprintf(buf, PAGE_SIZE, "%s\n", (char*)strbuf);
}

static ssize_t show_cali_value(struct device_driver *ddri, char *buf)
{
	struct i2c_client *client = mxc400x_i2c_client;
	struct mxc400x_i2c_data *obj;
	int err, len = 0, mul;
	int tmp[MXC400X_AXES_NUM];

	if(NULL == client)
	{
		GSE_ERR("i2c client is null!!\n");
		return MXC400X_ERR_STATUS;
	}

	obj = i2c_get_clientdata(client);



	if((err = MXC400X_ReadOffset(client, obj->offset)))
	{
		return -EINVAL;
	}
	else if((err = MXC400X_ReadCalibration(client, tmp)))
	{
		return -EINVAL;
	}
	else
	{
		mul = obj->reso->sensitivity/mxc400x_offset_resolution.sensitivity;
		len += snprintf(buf+len, PAGE_SIZE-len, "[HW ][%d] (%+3d, %+3d, %+3d) : (0x%02X, 0x%02X, 0x%02X)\n", mul,
			obj->offset[MXC400X_AXIS_X], obj->offset[MXC400X_AXIS_Y], obj->offset[MXC400X_AXIS_Z],
			obj->offset[MXC400X_AXIS_X], obj->offset[MXC400X_AXIS_Y], obj->offset[MXC400X_AXIS_Z]);
		len += snprintf(buf+len, PAGE_SIZE-len, "[SW ][%d] (%+3d, %+3d, %+3d)\n", 1,
			obj->cali_sw[MXC400X_AXIS_X], obj->cali_sw[MXC400X_AXIS_Y], obj->cali_sw[MXC400X_AXIS_Z]);

		len += snprintf(buf+len, PAGE_SIZE-len, "[ALL]    (%+3d, %+3d, %+3d) : (%+3d, %+3d, %+3d)\n",
			obj->offset[MXC400X_AXIS_X]*mul + obj->cali_sw[MXC400X_AXIS_X],
			obj->offset[MXC400X_AXIS_Y]*mul + obj->cali_sw[MXC400X_AXIS_Y],
			obj->offset[MXC400X_AXIS_Z]*mul + obj->cali_sw[MXC400X_AXIS_Z],
			tmp[MXC400X_AXIS_X], tmp[MXC400X_AXIS_Y], tmp[MXC400X_AXIS_Z]);

		return len;
    }
}
/*----------------------------------------------------------------------------*/
static ssize_t store_cali_value(struct device_driver *ddri, const char *buf, size_t count)
{
	struct i2c_client *client = mxc400x_i2c_client;
	int err, x, y, z;
	int dat[MXC400X_AXES_NUM];

	if(!strncmp(buf, "rst", 3))
	{
		if((err = MXC400X_ResetCalibration(client)))
		{
			GSE_ERR("reset offset err = %d\n", err);
		}
	}
	else if(3 == sscanf(buf, "0x%02X 0x%02X 0x%02X", &x, &y, &z))
	{
		dat[MXC400X_AXIS_X] = x;
		dat[MXC400X_AXIS_Y] = y;
	//	dat[MXC400X_AXIS_Z] = z;
		if((err = MXC400X_WriteCalibration(client, dat)))
		{
			GSE_ERR("write calibration err = %d\n", err);
		}
	}
	else
	{
		GSE_ERR("invalid format\n");
	}

	return 0;
}
static ssize_t show_firlen_value(struct device_driver *ddri, char *buf)
{
#ifdef CONFIG_MXC400X_LOWPASS
	struct i2c_client *client = mxc400x_i2c_client;
	struct mxc400x_i2c_data *obj = i2c_get_clientdata(client);
	if(atomic_read(&obj->firlen))
	{
	 	int idx, len = atomic_read(&obj->firlen);
	 	GSE_DEBUG("len = %2d, idx = %2d\n", obj->fir.num, obj->fir.idx);

		for(idx = 0; idx < len; idx++)
		{
			GSE_INFO("[%5d %5d %5d]\n", obj->fir.raw[idx][MXC400X_AXIS_X], obj->fir.raw[idx][MXC400X_AXIS_Y], obj->fir.raw[idx][MXC400X_AXIS_Z]);
		}

		GSE_INFO("sum = [%5d %5d %5d]\n", obj->fir.sum[MXC400X_AXIS_X], obj->fir.sum[MXC400X_AXIS_Y], obj->fir.sum[MXC400X_AXIS_Z]);
		GSE_INFO("avg = [%5d %5d %5d]\n", obj->fir.sum[MXC400X_AXIS_X]/len, obj->fir.sum[MXC400X_AXIS_Y]/len, obj->fir.sum[MXC400X_AXIS_Z]/len);
	}
	return snprintf(buf, PAGE_SIZE, "%d\n", atomic_read(&obj->firlen));
#else
	return snprintf(buf, PAGE_SIZE, "not support\n");
#endif
}
/*----------------------------------------------------------------------------*/
static ssize_t store_firlen_value(struct device_driver *ddri, const char *buf, size_t count)
{
#ifdef CONFIG_MXC400X_LOWPASS
	struct i2c_client *client = mxc400x_i2c_client;
	struct mxc400x_i2c_data *obj = i2c_get_clientdata(client);
	int firlen;

	if(1 != sscanf(buf, "%d", &firlen))
	{
		GSE_ERR("invallid format\n");
	}
	else if(firlen > C_MAX_FIR_LENGTH)
	{
		GSE_ERR("exceeds maximum filter length\n");
	}
	else
	{
		atomic_set(&obj->firlen, firlen);
		if(NULL == firlen)
		{
			atomic_set(&obj->fir_en, 0);
		}
		else
		{
			memset(&obj->fir, 0x00, sizeof(obj->fir));
			atomic_set(&obj->fir_en, 1);
		}
	}
#endif
	return 0;
}
static ssize_t show_trace_value(struct device_driver *ddri, char *buf)
{
	ssize_t res;
	struct mxc400x_i2c_data *obj = obj_i2c_data;
	if (obj == NULL)
	{
		GSE_ERR("i2c_data obj is null!!\n");
		return 0;
	}

	res = snprintf(buf, PAGE_SIZE, "0x%04X\n", atomic_read(&obj->trace));
	return res;
}
static ssize_t store_trace_value(struct device_driver *ddri, const char *buf, size_t count)
{
	struct mxc400x_i2c_data *obj = obj_i2c_data;
	int trace;
	if (obj == NULL)
	{
		GSE_ERR("i2c_data obj is null!!\n");
		return 0;
	}

	if(1 == sscanf(buf, "0x%x", &trace))
	{
		atomic_set(&obj->trace, trace);
	}
	else
	{
		GSE_ERR("invalid content: '%s', length = %d\n", buf, (int)count);
	}
	return count;
}
static ssize_t show_status_value(struct device_driver *ddri, char *buf)
{
	ssize_t len = 0;
	struct mxc400x_i2c_data *obj = obj_i2c_data;
	if (obj == NULL)
	{
		GSE_ERR("i2c_data obj is null!!\n");
		return 0;
	}

		len += snprintf(buf+len, PAGE_SIZE-len, "CUST: %d %d (%d %d)\n",
	            obj->hw.i2c_num, obj->hw.direction, obj->hw.power_id, obj->hw.power_vol);
		len += snprintf(buf+len, PAGE_SIZE-len, "CUST: NULL\n");

	return len;
}
/*----------------------------------------------------------------------------*/
static ssize_t show_power_status_value(struct device_driver *ddri, char *buf)
{

	if(sensor_power)
		GSE_INFO("G sensor is in work mode, sensor_power = %d\n", sensor_power);
	else
		GSE_INFO("G sensor is in standby mode, sensor_power = %d\n", sensor_power);

	return snprintf(buf, PAGE_SIZE, "%x\n", sensor_power);
}
static ssize_t show_layout_value(struct device_driver *ddri, char *buf)
{
	struct i2c_client *client = mxc400x_i2c_client;
	struct mxc400x_i2c_data *data = i2c_get_clientdata(client);

	return sprintf(buf, "(%d, %d)\n[%+2d %+2d %+2d]\n[%+2d %+2d %+2d]\n",
		data->hw.direction,atomic_read(&data->layout),	data->cvt.sign[0], data->cvt.sign[1],
		data->cvt.sign[2],data->cvt.map[0], data->cvt.map[1], data->cvt.map[2]);
}
/*----------------------------------------------------------------------------*/
static ssize_t store_layout_value(struct device_driver *ddri, const char *buf, size_t count)
{
	struct i2c_client *client = mxc400x_i2c_client;
	struct mxc400x_i2c_data *data = i2c_get_clientdata(client);
	int layout = 0;

	if(1 == sscanf(buf, "%d", &layout))
	{
		atomic_set(&data->layout, layout);
		if(!hwmsen_get_convert(layout, &data->cvt))
		{
			GSE_ERR("HWMSEN_GET_CONVERT function error!\r\n");
		}
		else if(!hwmsen_get_convert(data->hw.direction, &data->cvt))
		{
			GSE_ERR("invalid layout: %d, restore to %d\n", layout, data->hw.direction);
		}
		else
		{
			GSE_ERR("invalid layout: (%d, %d)\n", layout, data->hw.direction);
			hwmsen_get_convert(0, &data->cvt);
		}
	}
	else
	{
		GSE_ERR("invalid format = '%s'\n", buf);
	}

	return count;
}

static DRIVER_ATTR(chipinfo,	S_IWUSR | S_IRUGO, show_chipinfo_value,		NULL);
static DRIVER_ATTR(sensordata,	S_IWUSR | S_IRUGO, show_sensordata_value,	NULL);
static DRIVER_ATTR(cali,	S_IWUSR | S_IRUGO, show_cali_value,		store_cali_value);
static DRIVER_ATTR(firlen,	S_IWUSR | S_IRUGO, show_firlen_value,		store_firlen_value);
static DRIVER_ATTR(trace,	S_IWUSR | S_IRUGO, show_trace_value,		store_trace_value);
static DRIVER_ATTR(layout, 	S_IRUGO | S_IWUSR, show_layout_value,		store_layout_value);
static DRIVER_ATTR(status, 	S_IRUGO, show_status_value,			NULL);
static DRIVER_ATTR(powerstatus, S_IRUGO, show_power_status_value,		NULL);

static struct driver_attribute *mxc400x_attr_list[] = {
	&driver_attr_chipinfo,     /*chip information*/
	&driver_attr_sensordata,   /*dump sensor data*/
	&driver_attr_cali,         /*show calibration data*/
	&driver_attr_firlen,	   /*filter length: 0: disable, others: enable*/
	&driver_attr_trace,		   /*trace log*/
	&driver_attr_layout,
	&driver_attr_status,
	&driver_attr_powerstatus,
};
static int mxc400x_create_attr(struct device_driver *driver)
{
	int idx, err = 0;
	int num = (int)(sizeof(mxc400x_attr_list)/sizeof(mxc400x_attr_list[0]));
	if (driver == NULL)
	{
		return -EINVAL;
	}

	for(idx = 0; idx < num; idx++)
	{
		if((err = driver_create_file(driver, mxc400x_attr_list[idx])))
		{
			GSE_ERR("driver_create_file (%s) = %d\n", mxc400x_attr_list[idx]->attr.name, err);
			break;
		}
	}
	return err;
}
static int mxc400x_delete_attr(struct device_driver *driver)
{
	int idx ,err = 0;
	int num = (int)(sizeof(mxc400x_attr_list)/sizeof(mxc400x_attr_list[0]));

	if(driver == NULL)
	{
		return -EINVAL;
	}


	for(idx = 0; idx < num; idx++)
	{
		driver_remove_file(driver, mxc400x_attr_list[idx]);
	}

	return err;
}

#ifdef CONFIG_PM_SLEEP
static int mxc400x_suspend(struct device *dev)
{
 	 struct i2c_client *client = to_i2c_client(dev);
	 struct mxc400x_i2c_data *obj = i2c_get_clientdata(client);
	 int err = 0;

		 if(obj == NULL)
		 {
			 GSE_ERR("null mxc400x!!\n");
			 return -EINVAL;
		 }
		 mutex_lock(&mxc400x_mutex);
		 atomic_set(&obj->suspend, 1);
		 err = MXC400X_SetPowerMode(obj->client, false);
		 if(err)
		 {
			 GSE_ERR("write power control fail!!\n");
			 mutex_unlock(&mxc400x_mutex);
			 return -EINVAL;
		 }
		 mutex_unlock(&mxc400x_mutex);
	 return err;
}

static int mxc400x_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct mxc400x_i2c_data *obj = i2c_get_clientdata(client);
	int err = 0;


	if(obj == NULL)
	{
		GSE_ERR("null mxc400x!!\n");
		return -EINVAL;
	}
	mutex_lock(&mxc400x_mutex);
	err = MXC400X_SetPowerMode(client, true);
	if(err != MXC400X_SUCCESS)
	{
		GSE_ERR("Set PowerMode fail!!\n");
		mutex_unlock(&mxc400x_mutex);
		return -EINVAL;
	}
	atomic_set(&obj->suspend, 0);
	mutex_unlock(&mxc400x_mutex);
	return err;
}

#endif /*CONFIG_PM_SLEEP*/
// if use  this typ of enable , Gsensor should report inputEvent(x, y, z ,stats, div) to HAL
static int mxc400x_open_report_data(int open)
{
	//should queuq work to report event if  is_report_input_direct=true
	return 0;
}

// if use  this typ of enable , Gsensor only enabled but not report inputEvent to HAL

static int mxc400x_enable_nodata(int en)
{
	int res =0;
	bool power = false;

	if(1==en)
	{
		power = true;
	}
	if(0==en)
	{
		power = false;
	}
	res = MXC400X_SetPowerMode(obj_i2c_data->client, power);
	if(res != MXC400X_SUCCESS)
	{
		GSE_ERR("MXC400X_SetPowerMode fail!\n");
		return -1;
	}
	GSE_DEBUG("MXC400X_enable_nodata OK en = %d sensor_power = %d\n", en, sensor_power);
	enable_status = en;
	return 0;
}

static int gsensor_set_batch(int flag, int64_t samplingPeriodNs, int64_t maxBatchReportLatencyNs)
{
	int value = 0;

	value = (int)samplingPeriodNs/1000/1000;
	/*Fix Me*/
	GSE_INFO("mxc400x set delay = (%d) OK!\n", value);

	return 0;

}
static int gsensor_flush(void)
{
	return acc_flush_report();
}

static int mxc400x_batch(int flag, int64_t samplingPeriodNs, int64_t maxBatchReportLatencyNs)
{
	return 0;
}

static int mxc400x_set_delay(u64 ns)
{
	return 0;
}

static int mxc400x_get_data(int* x ,int* y,int* z, int* status)
{
	int buff[MXC400X_BUFSIZE] = {0};
	MXC400X_ReadSensorData(obj_i2c_data->client, buff, MXC400X_BUFSIZE);
	*x = buff[0];
	*y = buff[1];
	*z = buff[2];

	*status = SENSOR_STATUS_ACCURACY_MEDIUM;

	return 0;
}

static int mxc400x_factory_enable_sensor(bool enabledisable, int64_t sample_periods_ms)
{
	int err;

	err = mxc400x_enable_nodata(enabledisable == true ? 1 : 0);
	if (err) {
		GSE_ERR("%s enable sensor failed!\n", __func__);
		return -1;
	}
	err = mxc400x_batch(0, sample_periods_ms * 1000000, 0);
	if (err) {
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
	char strbuf[MXC400X_BUFSIZE] = { 0 };

	MXC400X_ReadRawData(mxc400x_i2c_client, strbuf);
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
	if (err) {
		GSE_ERR("mxc400x_ResetCalibration failed!\n");
		return -1;
	}
	return 0;
}
static int mxc400x_factory_set_cali(int32_t data[3])
{
	int err = 0;
	int cali[3] = { 0 };

	/* obj */
	obj_i2c_data->cali_sw[MXC400X_AXIS_X] += data[0];
	obj_i2c_data->cali_sw[MXC400X_AXIS_Y] += data[1];
	obj_i2c_data->cali_sw[MXC400X_AXIS_Z] += data[2];

	cali[MXC400X_AXIS_X] = data[0] * gsensor_gain.x / GRAVITY_EARTH_1000;
	cali[MXC400X_AXIS_Y] = data[1] * gsensor_gain.y / GRAVITY_EARTH_1000;
	cali[MXC400X_AXIS_Z] = data[2] * gsensor_gain.z / GRAVITY_EARTH_1000;
	err = MXC400X_WriteCalibration(mxc400x_i2c_client, cali);
	if (err) {
		GSE_ERR("mxc400x_WriteCalibration failed!\n");
		return -1;
	}
	return 0;
}
static int mxc400x_factory_get_cali(int32_t data[3])
{
	data[0] = obj_i2c_data->cali_sw[MXC400X_AXIS_X];
	data[1] = obj_i2c_data->cali_sw[MXC400X_AXIS_Y];
	data[2] = obj_i2c_data->cali_sw[MXC400X_AXIS_Z];
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
	struct i2c_client *new_client = NULL;
	struct mxc400x_i2c_data *obj = NULL;

	int err = 0;

	struct acc_control_path ctl={0};
	struct acc_data_path data={0};
	GSE_DEBUG_FUNC();
	GSE_INFO("driver version = %s\n",DRIVER_VERSION);

	if(!(obj = kzalloc(sizeof(*obj), GFP_KERNEL)))
	{
		err = -ENOMEM;
		goto exit;
	}


	err = get_accel_dts_func(client->dev.of_node, &obj->hw);
	if (err < 0) {
		GSE_ERR("get dts info fail\n");
		goto exit_kfree;
	}
	atomic_set(&obj->layout, obj->hw.direction);
	err = hwmsen_get_convert(obj->hw.direction, &obj->cvt);
	if (0 != err) {
		GSE_ERR("invalid direction: %d\n", obj->hw.direction);
		goto exit_kfree;
	}

	obj_i2c_data = obj;
	obj->client = client;
	new_client = obj->client;
	i2c_set_clientdata(new_client,obj);

	atomic_set(&obj->trace, 0);
	atomic_set(&obj->suspend, 0);


#ifdef CONFIG_MXC400X_LOWPASS
	if(obj->hw->firlen > C_MAX_FIR_LENGTH)
	{
		atomic_set(&obj->firlen, C_MAX_FIR_LENGTH);
	}
	else
	{
		atomic_set(&obj->firlen, obj->hw->firlen);
	}

	if(atomic_read(&obj->firlen) > 0)
	{
		atomic_set(&obj->fir_en, 1);
	}

#endif
	mxc400x_i2c_client = new_client;

	if((err = mxc400x_init_client(new_client, 1)))
	{
		goto exit_init_failed;
	}

	ctl.is_use_common_factory = false;
	/* factory */
	err = accel_factory_device_register(&mxc400x_factory_device);
	if (err) {
		GSE_ERR("mxc400x_device register failed.\n");
		goto exit_misc_device_register_failed;
	}

	if((err = mxc400x_create_attr(&mxc400x_init_info.platform_diver_addr->driver)))
	{
		GSE_ERR("create attribute err = %d\n", err);
		goto exit_create_attr_failed;
	}

	ctl.open_report_data = mxc400x_open_report_data;
	ctl.enable_nodata = mxc400x_enable_nodata;
	ctl.set_delay  = mxc400x_set_delay;
	ctl.batch = gsensor_set_batch;
	ctl.flush = gsensor_flush;
	ctl.is_report_input_direct = false;

	err = acc_register_control_path(&ctl);
	if(err)
	{
		GSE_ERR("register acc control path err\n");
		goto exit_kfree;
	}

	data.get_data = mxc400x_get_data;
	data.vender_div = 1000;
	err = acc_register_data_path(&data);
	if(err)
	{
		GSE_ERR("register acc data path err\n");
		goto exit_kfree;
	}
	mxc400x_init_flag = 0;
	GSE_INFO("%s: OK\n", __func__);
	return 0;

exit_create_attr_failed:
	/* i2c_detach_client(new_client); */
exit_misc_device_register_failed:
exit_init_failed:
exit_kfree:
	kfree(obj);
exit:
	GSE_ERR("%s: err = %d\n", __func__, err);
	mxc400x_init_flag = -1;
	obj = NULL;
	new_client = NULL;
	obj_i2c_data = NULL;
	mxc400x_i2c_client = NULL;
	return err;
}

static int mxc400x_i2c_remove(struct i2c_client *client)
{
	 int err = 0;

	err = mxc400x_delete_attr(&mxc400x_init_info.platform_diver_addr->driver);
	if (err != 0)
		GSE_ERR("mxc400x_delete_attr fail: %d\n", err);

	 mxc400x_i2c_client = NULL;
	 i2c_unregister_device(client);
	 accel_factory_device_deregister(&mxc400x_factory_device);
	 kfree(i2c_get_clientdata(client));
	 return 0;
}


static int mxc400x_local_init(void)
{
	GSE_DEBUG_FUNC();

	if(i2c_add_driver(&mxc400x_i2c_driver))
	{
		 GSE_ERR("add driver error\n");
		 return -1;
	}
	if(-1 == mxc400x_init_flag)
	{
		GSE_ERR("mxc400x_local_init failed mxc400x_init_flag=%d\n",mxc400x_init_flag);
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
