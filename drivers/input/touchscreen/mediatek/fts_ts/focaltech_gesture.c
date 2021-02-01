/*
 *
 * FocalTech TouchScreen driver.
 *
 * Copyright (c) 2010-2017, Focaltech Ltd. All rights reserved.
 * Copyright (c) 2020, svoboda18
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

/*****************************************************************************
* 1.Included header files
*****************************************************************************/
#include "focaltech_core.h"
#include "kpd.h"

extern unsigned int tpd_gesture_status;

#if FTS_GESTURE_EN
/******************************************************************************
* Private constant and macro definitions using #define
*****************************************************************************/
#define GESTURE_LEFT		0x20
#define GESTURE_RIGHT		0x21
#define GESTURE_UP			0x22
#define GESTURE_DOWN		0x23
#define GESTURE_DOUBLECLICK	0x24
#define GESTURE_O		0x30
#define GESTURE_W		0x31
#define GESTURE_M		0x32
#define GESTURE_E		0x33
#define GESTURE_L		0x44
#define GESTURE_S		0x46
#define GESTURE_V		0x54
#define GESTURE_Z		0x41

#define KEY_GESTURE_U 		KEY_U
#define KEY_GESTURE_UP 		KEY_UP
#define KEY_GESTURE_DOWN 	KEY_DOWN
#define KEY_GESTURE_LEFT 	KEY_LEFT 
#define KEY_GESTURE_RIGHT 	KEY_RIGHT
#define KEY_GESTURE_O 		KEY_O
#define KEY_GESTURE_E 		KEY_E
#define KEY_GESTURE_M 		KEY_M 
#define KEY_GESTURE_L 		KEY_L
#define KEY_GESTURE_W 		KEY_W
#define KEY_GESTURE_S 		KEY_S 
#define KEY_GESTURE_V 		KEY_V
#define KEY_GESTURE_Z 		KEY_Z

#define FTS_GESTRUE_POINTS                      255
#define FTS_GESTRUE_POINTS_HEADER               8

/*****************************************************************************
* Private enumerations, structures and unions using typedef
*****************************************************************************/
struct fts_gesture_item {
	int	gesture_id;
	int	action_id;
	char* name;
};

/*
 *
 * header        -   byte0:gesture id
 *                   byte1:pointnum
 *                   byte2~7:reserved
 * coordinate_x  -   buf All gesture point x coordinate
 * coordinate_y  -   buf All gesture point y coordinate
 * mode          -   int 1:enable gesture function(default)
 *               -   int 0:disable
 * name          -   char* gesture name
 *
*/
struct fts_gesture_st {
    u8 header[FTS_GESTRUE_POINTS_HEADER];
    u16 coordinate_x[FTS_GESTRUE_POINTS];
    u16 coordinate_y[FTS_GESTRUE_POINTS];
    u8 mode;
	char* name;
};

/*****************************************************************************
* Static variables
*****************************************************************************/
static struct fts_gesture_st fts_gesture_data;
static struct kobject *fts_gesture_kobj;

/*****************************************************************************
* Functions prototypes
*****************************************************************************/
int fetch_object_sample(unsigned char *buf, short pointnum);
void init_para(int x_pixel, int y_pixel, int time_slot, int cut_x_pixel, int cut_y_pixel);

static ssize_t fts_gesture_show(struct device *dev, struct device_attribute *attr, char *buf);
static ssize_t fts_gesture_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count);
static ssize_t fts_gesture_buf_show(struct device *dev, struct device_attribute *attr, char *buf);
static ssize_t fts_gesture_buf_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count);

/* Sysfs attributes cannot be world-writable. */
static DEVICE_ATTR (fts_gesture_mode, S_IRUGO | S_IWUSR, fts_gesture_show, fts_gesture_store);
static DEVICE_ATTR (fts_gesture_buf, S_IRUGO | S_IWUSR, fts_gesture_buf_show, fts_gesture_buf_store);

static struct fts_gesture_item fts_gesture_array[] = 
{
	{GESTURE_LEFT,	KEY_GESTURE_LEFT,	"LEFT"},
	{GESTURE_RIGHT, KEY_GESTURE_RIGHT,	"RIGHT"},
	{GESTURE_UP, 	KEY_GESTURE_UP, 	"UP"},
	{GESTURE_DOWN, 	KEY_GESTURE_DOWN, 	"DOWN"},
	{GESTURE_DOUBLECLICK, KEY_GESTURE_U,"DOUBLECLICK"},
	{GESTURE_O, 	KEY_GESTURE_O, 		"o"},
	{GESTURE_E, 	KEY_GESTURE_E,		"e"},
	{GESTURE_M, 	KEY_GESTURE_M,		"m"},
	{GESTURE_L,		KEY_GESTURE_L,		"l"},
	{GESTURE_W,		KEY_GESTURE_W,		"w"},
	{GESTURE_S,		KEY_GESTURE_S,		"s"},
	{GESTURE_V,		KEY_GESTURE_V,		"v"},
	{GESTURE_Z,		KEY_GESTURE_Z,		"z"},
	{0}
};

static struct attribute *fts_gesture_mode_attrs[] = {
    &dev_attr_fts_gesture_mode.attr,
    &dev_attr_fts_gesture_buf.attr,
    NULL,
};

static struct attribute_group fts_gesture_group = {
    .attrs = fts_gesture_mode_attrs,
};

/************************************************************************
* Name: fts_gesture_show
*  Brief:
*  Input: device, device attribute, char buf
* Output:
* Return:
***********************************************************************/
static ssize_t fts_gesture_show(struct device *dev, struct device_attribute *attr, char *buf)
{   
    return snprintf(buf, PAGE_SIZE, "Gesture Mode: %s\n", fts_gesture_data.mode ? "on" : "off");
}

/************************************************************************
* Name: fts_gesture_store
*  Brief:
*  Input: device, device attribute, char buf, char count
* Output:
* Return:
***********************************************************************/
static ssize_t fts_gesture_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
    struct input_dev *input_dev = fts_data->input_dev;
    
    mutex_lock(&input_dev->mutex);
    
    if (FTS_SYSFS_ECHO_ON(buf)) {
        FTS_INFO("[GESTURE]enable gesture");
        fts_gesture_data.mode = ENABLE;
		tpd_gesture_status = ENABLE;
    } else if (FTS_SYSFS_ECHO_OFF(buf)) {
        FTS_INFO("[GESTURE]disable gesture");
        fts_gesture_data.mode = DISABLE;
		tpd_gesture_status = DISABLE;
    }
    
    mutex_unlock(&input_dev->mutex);

    return count;
}
/************************************************************************
* Name: fts_gesture_buf_show
*  Brief:
*  Input: device, device attribute, char buf
* Output:
* Return:
***********************************************************************/
static ssize_t fts_gesture_buf_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    int count;
    int i = 0;
    struct input_dev *input_dev = fts_data->input_dev;

    mutex_lock(&input_dev->mutex);
    
    count = snprintf(buf, PAGE_SIZE, "Gesture ID: 0x%x\n", fts_gesture_data.header[0]);
    count += snprintf(buf + count, PAGE_SIZE, "Gesture PointNum: %d\n", fts_gesture_data.header[1]);
	count += snprintf(buf + count, PAGE_SIZE, "Gesture Name: %s\n", fts_gesture_data.name);
    count += snprintf(buf + count, PAGE_SIZE, "Gesture Point Buf:\n");
    for (i = 0; i < fts_gesture_data.header[1]; i++) {
        count += snprintf(buf + count, PAGE_SIZE, "%3d(%4d,%4d) ", i, fts_gesture_data.coordinate_x[i], fts_gesture_data.coordinate_y[i]);
        if ((i + 1) % 4 == 0)
            count += snprintf(buf + count, PAGE_SIZE, "\n");
    }
    count += snprintf(buf + count, PAGE_SIZE, "\n");
    
    mutex_unlock(&input_dev->mutex);

    return count;
}

/************************************************************************
* Name: fts_gesture_buf_store
*  Brief:
*  Input: device, device attribute, char buf, char count
* Output:
* Return:
***********************************************************************/
static ssize_t fts_gesture_buf_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
    /* place holder for future use */
    return -EPERM;
}

/*****************************************************************************
*   Name: fts_create_gesture_sysfs
*  Brief:
*  Input:
* Output:
* Return: 0-success or others-error
*****************************************************************************/
int fts_create_gesture_sysfs(void)
{
    int ret = 0;
	fts_gesture_kobj = kobject_create_and_add(FTS_DRIVER_NAME, kernel_kobj);
	
	if (!fts_gesture_kobj)
		return -ENOMEM;
	
    ret = sysfs_create_group(fts_gesture_kobj, &fts_gesture_group);
    if ( ret != 0) {
        FTS_ERROR( "[GESTURE]fts_gesture_mode_group(sysfs) create failed!");
        sysfs_remove_group(fts_gesture_kobj, &fts_gesture_group);
        return ret;
    }
    return 0;
}

/*****************************************************************************
*   Name: fts_gesture_report
*  Brief:
*  Input:
* Output:
* Return:
*****************************************************************************/
static void fts_gesture_report(struct input_dev *input_dev, int gesture_id)
{
    struct fts_gesture_item* items = fts_gesture_array;

    FTS_FUNC_ENTER();
	
	if (fts_gesture_data.mode != ENABLE) return;

	for(;items->gesture_id; ++items)
	{
		if (items->gesture_id != gesture_id) continue;

		/* report event key */
		kpd_pmic_pwrkey_hal(1);
		kpd_pmic_pwrkey_hal(0);
		
		fts_gesture_data.name = items->name;

		input_report_key(input_dev, items->action_id, 1);
		input_sync(input_dev);
		input_report_key(input_dev, items->action_id, 0);
		input_sync(input_dev);
    }

    FTS_FUNC_EXIT();
}

/************************************************************************
*   Name: fts_gesture_read_buffer
*  Brief: read data from TP register
*  Input:
* Output:
* Return: fail <0
***********************************************************************/
static int fts_gesture_read_buffer(struct i2c_client *client, u8 *buf, int read_bytes)
{
    int remain_bytes;
    int ret;
    int i;

    if (read_bytes <= I2C_BUFFER_LENGTH_MAXINUM) {
        ret = fts_i2c_read(client, buf, 1, buf, read_bytes);
    } else {
        ret = fts_i2c_read(client, buf, 1, buf, I2C_BUFFER_LENGTH_MAXINUM);
        remain_bytes = read_bytes - I2C_BUFFER_LENGTH_MAXINUM;
        for (i = 1; remain_bytes > 0; i++) {
            if (remain_bytes <= I2C_BUFFER_LENGTH_MAXINUM)
                ret = fts_i2c_read(client, buf, 0, buf + I2C_BUFFER_LENGTH_MAXINUM * i, remain_bytes);
            else
                ret = fts_i2c_read(client, buf, 0, buf + I2C_BUFFER_LENGTH_MAXINUM * i, I2C_BUFFER_LENGTH_MAXINUM);
            remain_bytes -= I2C_BUFFER_LENGTH_MAXINUM;
        }
    }

    return ret;
}

/************************************************************************
*   Name: fts_gesture_readdata
*  Brief: read data from TP register
*  Input:
* Output:
* Return: return 0 if succuss, otherwise reture error code
***********************************************************************/
int fts_gesture_readdata(struct fts_ts_data *ts_data)
{   
    struct i2c_client *client = ts_data->client;
    struct input_dev *input_dev = ts_data->input_dev;
	struct fts_gesture_item* items = fts_gesture_array;
    
    u8 buf[FTS_GESTRUE_POINTS * 4] = { 0 };
    u8 pointnum;
    u8 state;
    
    int ret = 0;
    int i = 0;
    int gesture_id = 0;
    int read_bytes = 0;
    int is_valid = 0;

	FTS_FUNC_ENTER();

    if (!ts_data->suspended) {
        return -EINVAL;
    }

    ret = fts_i2c_read_reg(client, FTS_REG_GESTURE_EN, &state);
    if ((ret < 0) || (state != ENABLE)) {
        FTS_DEBUG("gesture not enable, don't process gesture");
        return -EIO;
    }

    /* init variable before read gesture point */
    memset(fts_gesture_data.header, 0, FTS_GESTRUE_POINTS_HEADER);
    memset(fts_gesture_data.coordinate_x, 0, FTS_GESTRUE_POINTS * sizeof(u16));
    memset(fts_gesture_data.coordinate_y, 0, FTS_GESTRUE_POINTS * sizeof(u16));

    buf[0] = FTS_REG_GESTURE_OUTPUT_ADDRESS;
    ret = fts_i2c_read(client, buf, 1, buf, FTS_GESTRUE_POINTS_HEADER);
    if (ret < 0) {
        FTS_ERROR("[GESTURE]Read gesture header data failed!!");
        FTS_FUNC_EXIT();
        return ret;
    }

    /* store gesture data header */
    memcpy(fts_gesture_data.header, buf, FTS_GESTRUE_POINTS_HEADER);
	
    gesture_id = buf[0];
    pointnum = (short)(buf[1]) & 0xff;
    read_bytes = pointnum * 4 + 2;
    
    if(gesture_id != 0xfe) {
        goto report_gesture;
	}

	FTS_DEBUG("[%d] gesture_id: %d, pointnum: %d", __LINE__, gesture_id, pointnum);
    buf[0] = FTS_REG_GESTURE_OUTPUT_ADDRESS;
        
	ret = fts_gesture_read_buffer(client, buf, read_bytes);
    if (ret < 0) {
        FTS_ERROR("[GESTURE]Read gesture touch data failed!!");
        FTS_FUNC_EXIT();
        return ret;
    }
    
    gesture_id = fetch_object_sample(buf, pointnum);
        
	for(;items->gesture_id; ++items)
	{
		if (items->gesture_id != gesture_id) continue;
        is_valid = 1;
    }
    
    FTS_DEBUG("[%d] gesture_id: %d, pointnum: %d, is_valid: %d", __LINE__, gesture_id, pointnum, is_valid);
    
    if (is_valid) {
        goto report_gesture;  
    } else {
        buf[0] = FTS_REG_GESTURE_OUTPUT_ADDRESS;
        ret = fts_gesture_read_buffer(client, buf, read_bytes + 6);
        if (ret < 0) {
            FTS_ERROR("[GESTURE]Read gesture touch data failed!!");
            FTS_FUNC_EXIT();
            return ret;
        }
    }

    gesture_id = fetch_object_sample(buf, pointnum);
    goto report_gesture;

report_gesture:
    FTS_DEBUG("[%d] gesture_id: %d, pointnum: %d", __LINE__, gesture_id, pointnum);
    
    fts_gesture_report(input_dev, gesture_id);
    
	for (i = 0; i < pointnum; i++) {
        fts_gesture_data.coordinate_x[i] = (((s16) buf[0 + (4 * i + 2)]) & 0x0F) << 8
                                           | (((s16) buf[1 + (4 * i + 2)]) & 0xFF);
        fts_gesture_data.coordinate_y[i] = (((s16) buf[2 + (4 * i + 2)]) & 0x0F) << 8
                                           | (((s16) buf[3 + (4 * i + 2)]) & 0xFF);
    }

	FTS_FUNC_EXIT();

    return 0;
}

/*****************************************************************************
*   Name: fts_gesture_recovery
*  Brief: recovery gesture state when reset or power on
*  Input:
* Output:
* Return:
*****************************************************************************/
void fts_gesture_recovery(struct i2c_client *client)
{
	FTS_FUNC_ENTER();

	fts_i2c_write_reg(client, 0xD1, 0xff);
	fts_i2c_write_reg(client, 0xD2, 0xff);
	fts_i2c_write_reg(client, 0xD5, 0xff);
	fts_i2c_write_reg(client, 0xD6, 0xff);
	fts_i2c_write_reg(client, 0xD7, 0xff);
	fts_i2c_write_reg(client, 0xD8, 0xff);
    fts_i2c_write_reg(client, FTS_REG_GESTURE_EN, ENABLE);
	
	FTS_FUNC_EXIT();
}

/*****************************************************************************
*   Name: fts_gesture_suspend
*  Brief:
*  Input:
* Output:
* Return: return 0 if succuss, otherwise return error code
*****************************************************************************/
int fts_gesture_suspend(struct i2c_client *client)
{
    int ret;
    int i;
    u8 state;
	
    FTS_FUNC_ENTER();
    FTS_INFO("gesture suspend...");
    
	for (i = 0; i < 5; i++) {
        fts_i2c_write_reg(client, 0xd1, 0xff);
        fts_i2c_write_reg(client, 0xd2, 0xff);
        fts_i2c_write_reg(client, 0xd5, 0xff);
        fts_i2c_write_reg(client, 0xd6, 0xff);
        fts_i2c_write_reg(client, 0xd7, 0xff);
        fts_i2c_write_reg(client, 0xd8, 0xff);
        fts_i2c_write_reg(client, FTS_REG_GESTURE_EN, ENABLE);
        msleep(1);
        fts_i2c_read_reg(client, FTS_REG_GESTURE_EN, &state);
        if (state == ENABLE)
            break;
    }

    if (i >= 5) {
        FTS_ERROR("[GESTURE]Enter into gesture(suspend) failed!\n");
        fts_gesture_data.mode = DISABLE;
        return -EIO;
    }

    ret = enable_irq_wake(client->irq);
    if (ret) {
        FTS_INFO("enable_irq_wake(irq:%d) failed", client->irq);
    }

    FTS_INFO("[GESTURE]Enter into gesture(suspend) successfully!");
    FTS_FUNC_EXIT();
    return 0;
}

/*****************************************************************************
*  Name: fts_gesture_resume
*  Brief:
*  Input:
* Output:
* Return: return 0 if succuss, otherwise return error code
*****************************************************************************/
int fts_gesture_resume(struct i2c_client *client)
{
    int ret;
    int i;
    u8 state;
	
    FTS_FUNC_ENTER();
    FTS_INFO("gesture resume...");
    
	for (i = 0; i < 5; i++) {
        fts_i2c_write_reg(client, FTS_REG_GESTURE_EN, DISABLE);
        msleep(1);
        fts_i2c_read_reg(client, FTS_REG_GESTURE_EN, &state);
        if (state == DISABLE)
            break;
    }

    if (i >= 5) {
        FTS_ERROR("[GESTURE]Clear gesture(resume) failed!\n");
        fts_gesture_data.mode = ENABLE;
        return -EIO;
    }

    ret = disable_irq_wake(client->irq);
    if (ret) {
        FTS_INFO("disable_irq_wake(irq:%d) failed", client->irq);
    }

    FTS_INFO("[GESTURE]resume from gesture successfully!");
    FTS_FUNC_EXIT();
    return 0;
}

/*****************************************************************************
*   Name: fts_gesture_init
*  Brief:
*  Input:
* Output:
* Return:
*****************************************************************************/
int fts_gesture_init(struct fts_ts_data *ts_data)
{
    struct input_dev *input_dev = ts_data->input_dev;
    struct fts_gesture_item* items = fts_gesture_array;

    FTS_FUNC_ENTER();
    
    init_para(FTS_GESTURE_INIT_PIXLE_X, FTS_GESTURE_INIT_PIXLE_Y, FTS_GESTURE_TIME_FRAME, 0, 0);

	while(items->gesture_id)
	{
		input_set_capability(input_dev, EV_KEY, items->action_id);
		__set_bit(items->action_id, input_dev->keybit);
		items++;
	}

    fts_create_gesture_sysfs();
    fts_gesture_data.mode = tpd_gesture_status;

    FTS_FUNC_EXIT();
    return 0;
}

/************************************************************************
*   Name: fts_gesture_exit
*  Brief: call when driver removed
*  Input:
* Output:
* Return:
***********************************************************************/
int fts_gesture_exit(struct i2c_client *client)
{
    FTS_FUNC_ENTER();
	
    sysfs_remove_group(fts_gesture_kobj, &fts_gesture_group);
	
    FTS_FUNC_EXIT();
    return 0;
}
#endif
