/*
 * Copyright (C) 2010 - 2018 Novatek, Inc.
 *
 * $Revision: 64128 $
 * $Date: 2020-06-10 18:50:08 +0800 (週三, 10 六月 2020) $
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 */
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/gpio.h>
#include <linux/proc_fs.h>
#include <linux/input/mt.h>
#include <linux/of.h>
#include <linux/of_irq.h>
#include <uapi/linux/sched/types.h>
#include <linux/kthread.h>

#include "tpd.h"
#include "nt36xxx.h"
#if NVT_TOUCH_ESD_PROTECT
#include <linux/jiffies.h>
#endif /* #if NVT_TOUCH_ESD_PROTECT */

#if NVT_TOUCH_ESD_PROTECT
static struct delayed_work nvt_esd_check_work;
static struct workqueue_struct *nvt_esd_check_wq;
static unsigned long irq_timer = 0;
uint8_t esd_check = false;
uint8_t esd_retry = 0;
#endif /* #if NVT_TOUCH_ESD_PROTECT */

#ifdef CONFIG_MTK_I2C_EXTENSION
#if I2C_DMA_SUPPORT
#include <linux/dma-mapping.h>

static uint8_t *gpDMABuf_va;
static dma_addr_t gpDMABuf_pa;
#endif
#endif

#if NVT_TOUCH_EXT_PROC
extern int32_t nvt_extra_proc_init(void);
extern void nvt_extra_proc_deinit(void);
#endif

#if NVT_TOUCH_MP
extern int32_t nvt_mp_proc_init(void);
extern void nvt_mp_proc_deinit(void);
#endif

struct nvt_ts_data *ts;

#if BOOT_UPDATE_FIRMWARE
static struct workqueue_struct *nvt_fwu_wq;
extern void Boot_Update_Firmware(struct work_struct *work);
#endif

#if NVT_TOUCH_POLLING
static struct task_struct *nvt_polling_thread;
static int nvt_ts_event_polling(void *arg);
#endif

#if TOUCH_KEY_NUM > 0
const uint16_t touch_key_array[TOUCH_KEY_NUM] = {
	KEY_BACK,
	KEY_HOME,
	KEY_MENU
};
#endif

#if WAKEUP_GESTURE
const uint16_t gesture_key_array[] = {
	KEY_POWER,  //GESTURE_WORD_C
	KEY_POWER,  //GESTURE_WORD_W
	KEY_POWER,  //GESTURE_WORD_V
	KEY_POWER,  //GESTURE_DOUBLE_CLICK
	KEY_POWER,  //GESTURE_WORD_Z
	KEY_POWER,  //GESTURE_WORD_M
	KEY_POWER,  //GESTURE_WORD_O
	KEY_POWER,  //GESTURE_WORD_e
	KEY_POWER,  //GESTURE_WORD_S
	KEY_POWER,  //GESTURE_SLIDE_UP
	KEY_POWER,  //GESTURE_SLIDE_DOWN
	KEY_POWER,  //GESTURE_SLIDE_LEFT
	KEY_POWER,  //GESTURE_SLIDE_RIGHT
};
#endif

static uint8_t bTouchIsAwake;
static int tpd_flag;
static struct task_struct *thread;
static DECLARE_WAIT_QUEUE_HEAD(waiter);

/*******************************************************
Description:
	Novatek touchscreen irq enable/disable function.

return:
	n.a.
*******************************************************/
static void nvt_irq_enable(bool enable)
{
	unsigned long flags;
	struct irq_desc *desc;

	spin_lock_irqsave(&ts->irq_lock, flags);
	if (enable) {
		if (!ts->irq_enabled) {
			ts->irq_enabled = true;
			spin_unlock_irqrestore(&ts->irq_lock, flags);
			enable_irq(ts->client->irq);
		} else {
			spin_unlock_irqrestore(&ts->irq_lock, flags);
			NVT_LOG("irq already enabled!\n");
		}
	} else {
		if (ts->irq_enabled) {
			ts->irq_enabled = false;
			spin_unlock_irqrestore(&ts->irq_lock, flags);
			disable_irq(ts->client->irq);
		} else {
			spin_unlock_irqrestore(&ts->irq_lock, flags);
			NVT_LOG("irq already disabled!\n");
		}
	}

	desc = irq_to_desc(ts->client->irq);
	//NVT_LOG("enable=%d, desc->depth=%d\n", enable, desc->depth);
}

#ifdef CONFIG_MTK_I2C_EXTENSION
#if I2C_DMA_SUPPORT
int32_t i2c_dma_read(struct i2c_client *client, uint16_t addr, uint8_t offset, uint8_t *rxbuf, uint16_t len)
{
	uint8_t buf[2] = {offset,0};
	int32_t ret;
	int32_t retries = 0;

	struct i2c_msg msg[2] = {
		{
			.addr = (addr & I2C_MASK_FLAG),
			.flags = 0,
			.buf = buf,
			.len = 1,
			.timing = client->timing
		},
		{
			.addr = (addr & I2C_MASK_FLAG),
			.ext_flag = (client->ext_flag | I2C_ENEXT_FLAG | I2C_DMA_FLAG),
			.flags = I2C_M_RD,
			.buf = (uint8_t*)gpDMABuf_pa,
			.len = len,
			.timing = client->timing
		},
	};

	if (rxbuf == NULL) {
		NVT_ERR("rxbuf is NULL!\n");
		return -ENOMEM;
	}

	for (retries = 0; retries < 20; ++retries) {
		ret = i2c_transfer(client->adapter, &msg[0], 2);
		if (ret < 0)
			continue;
		memcpy(rxbuf, gpDMABuf_va, len);
		return ret;
	}

	NVT_ERR("Dma I2C Read Error: 0x%04X, %d byte(s), err-code: %d", addr, len, ret);
	return ret;
}

int32_t i2c_dma_write(struct i2c_client *client, uint16_t addr, uint8_t offset, uint8_t *txbuf, uint16_t len)
{
	uint8_t *wr_buf = gpDMABuf_va;
	int32_t ret = -1;
	int32_t retries = 0;

	struct i2c_msg msg = {
		.addr = (addr & I2C_MASK_FLAG),
		.ext_flag = (client->ext_flag | I2C_ENEXT_FLAG | I2C_DMA_FLAG),
		.flags = 0,
		.buf = (uint8_t*)gpDMABuf_pa,
		.len = 1 + len,
		.timing = client->timing
	};

	wr_buf[0] = offset;

	if (txbuf == NULL) {
		NVT_ERR("txbuf is NULL!\n");
		return -ENOMEM;
	}

	memcpy(wr_buf+1, txbuf, len);
	for (retries = 0; retries < 20; ++retries) {
		ret = i2c_transfer(client->adapter, &msg, 1);
		if (ret < 0)
			continue;
		return ret;
	}

	NVT_ERR("Dma I2C Write Error: 0x%04X, %d byte(s), err-code: %d", offset, len, ret);

	return ret;
}

int32_t i2c_read_bytes_dma(struct i2c_client *client, u16 addr, uint8_t offset, uint8_t *rxbuf, uint16_t len)
{
	uint8_t *rd_buf = rxbuf;
	uint16_t left = len;
	uint16_t read_len = 0;
	int32_t ret = -1;

	while (left > 0) {
		if (left > DMA_MAX_TRANSACTION_LENGTH) {
			read_len = DMA_MAX_TRANSACTION_LENGTH;
		} else {
			read_len = left;
		}
		ret = i2c_dma_read(client, addr, offset, rd_buf, read_len);
		if (ret < 0) {
			NVT_ERR("dma i2c read failed!\n");
			return -EIO;
		}

		left -= read_len;
		offset += read_len;
		rd_buf += read_len;
	}

	return ret;
}

int32_t i2c_write_bytes_dma(struct i2c_client *client, u16 addr, uint8_t offset, uint8_t *txbuf, uint16_t len)
{
	uint8_t *wr_buf = txbuf;
	int32_t ret = 0;
	int32_t write_len = 0;
	int32_t left = len;

	while (left > 0) {
		if (left > DMA_MAX_I2C_TRANSFER_SIZE) {
			write_len = DMA_MAX_I2C_TRANSFER_SIZE;
		} else {
			write_len = left;
		}
		ret = i2c_dma_write(client, addr, offset, wr_buf, write_len);

		if (ret < 0) {
			NVT_ERR("dma i2c write failed!\n");
			return -EIO;
		}

		left -= write_len;
		offset += write_len;
		wr_buf += write_len;
	}
	return ret;
}

#else	//I2C_DMA_SUPPORT

int i2c_read_bytes_non_dma(struct i2c_client *client, u16 addr,uint8_t offset, uint8_t *rxbuf, uint16_t len)
{
	uint8_t buf[2] = {0};
	uint16_t left = len;
	uint16_t index = 0;
	int32_t ret = 0;
	int32_t retries = 0;

	struct i2c_msg msg[2] = {
		{
			.addr = ((addr &I2C_MASK_FLAG) | (I2C_ENEXT_FLAG)),
			.flags = 0,
			.buf = buf,
			.len = 1,
			.timing = client->timing
		},
		{
			.addr = ((addr &I2C_MASK_FLAG) | (I2C_ENEXT_FLAG)),
			.flags = I2C_M_RD,
			.timing = client->timing
		},
	};

	if (rxbuf == NULL) {
		NVT_ERR("rxbuf is NULL!\n");
		return -ENOMEM;
	}

	while (left > 0) {
		buf[0] = offset + index;
		msg[1].buf = &rxbuf[index];

		if (left > MAX_TRANSACTION_LENGTH) {
			msg[1].len = MAX_TRANSACTION_LENGTH;
			left -= MAX_TRANSACTION_LENGTH;
			index += MAX_TRANSACTION_LENGTH;
		} else {
			msg[1].len = left;
			left = 0;
		}

		retries = 0;

		while (retries < 20) {
			ret = i2c_transfer(client->adapter, msgs, 2);
			if (ret == 2)   break;
			retries++;
		}

		if (unlikely(retries == 20)) {
			NVT_ERR("error, I2C read 0x%X length=%d failed! ret=%d\n", offset + index, len, ret);
			ret = -EIO;
		}
	}

	return ret;
}

int i2c_write_bytes_non_dma(struct i2c_client *client, u16 addr,uint8_t offset, uint8_t *txbuf, uint16_t len)
{
	uint8_t buf[MAX_TRANSACTION_LENGTH];
	uint16_t left = len;
	uint16_t index = 0;
	int32_t ret = 0;
	int32_t retries = 0;

	struct i2c_msg msg = {
		.addr = ((addr &I2C_MASK_FLAG) | (I2C_ENEXT_FLAG)),
		.flags = 0,
		.buf = buf,
		.timing = client->timing,
	};

	if (txbuf == NULL) {
		NVT_ERR("txbuf is NULL!\n");
		return -ENOMEM;
	}

	while (left > 0) {
		retries = 0;

		buf[0] = (offset + index) & 0xFF;

		if (left > MAX_I2C_TRANSFER_SIZE) {
			memcpy(&buf[1], &txbuf[index], MAX_I2C_TRANSFER_SIZE);
			msg.len = MAX_TRANSACTION_LENGTH;
			left -= MAX_I2C_TRANSFER_SIZE;
			index += MAX_I2C_TRANSFER_SIZE;
		} else {
			memcpy(&buf[1], &txbuf[index], left);
			msg.len = left + 1;
			left = 0;
		}

		while (retries < 20) {
			ret = i2c_transfer(client->adapter, &msgs, 1);
			if (ret == 1)   break;
			retries++;
		}

		if (unlikely(retries == 20)) {
			NVT_ERR("error, I2C write 0x%X length=%d failed! ret=%d\n", offset, len, ret);
			ret = -EIO;
		}
	}

	return ret;
}
#endif	//I2C_DMA_SUPPORT
#endif	//CONFIG_MTK_I2C_EXTENSION

/*******************************************************
Description:
	Novatek touchscreen i2c read function.

return:
	Executive outcomes. 2---succeed. -5---I/O error
*******************************************************/
int32_t CTP_I2C_READ(struct i2c_client *client, uint16_t address, uint8_t *buf, uint16_t len)
{
#ifndef CONFIG_MTK_I2C_EXTENSION
	struct i2c_msg msgs[2];
	int32_t ret = -1;
	int32_t retries = 0;

	mutex_lock(&ts->xbuf_lock);

	msgs[0].flags = !I2C_M_RD;
	msgs[0].addr  = address;
	msgs[0].len   = 1;
	msgs[0].buf   = &buf[0];

	msgs[1].flags = I2C_M_RD;
	msgs[1].addr  = address;
	msgs[1].len   = len - 1;
	msgs[1].buf   = ts->xbuf;

	while (retries < 5) {
		ret = i2c_transfer(client->adapter, msgs, 2);
		if (ret == 2)	break;
		retries++;
	}

	if (unlikely(retries == 5)) {
		NVT_ERR("error, ret=%d\n", ret);
		ret = -EIO;
	}

	memcpy(buf + 1, ts->xbuf, len - 1);

	mutex_unlock(&ts->xbuf_lock);

	return ret;
#else	//CONFIG_MTK_I2C_EXTENSION
	int32_t ret = -1;

	mutex_lock(&ts->xbuf_lock);

	#if I2C_DMA_SUPPORT
	ret = i2c_read_bytes_dma(client, address, buf[0], ts->xbuf, len-1);
	#else
	ret = i2c_read_bytes_non_dma(client, address, buf[0], ts->xbuf, len-1);
	#endif

	memcpy(buf + 1, ts->xbuf, len - 1);

	mutex_unlock(&ts->xbuf_lock);

	return ret;
#endif	//CONFIG_MTK_I2C_EXTENSION
}

/*******************************************************
Description:
	Novatek touchscreen i2c write function.

return:
	Executive outcomes. 1---succeed. -5---I/O error
*******************************************************/
int32_t CTP_I2C_WRITE(struct i2c_client *client, uint16_t address, uint8_t *buf, uint16_t len)
{
#ifndef CONFIG_MTK_I2C_EXTENSION
	struct i2c_msg msg;
	int32_t ret = -1;
	int32_t retries = 0;

	mutex_lock(&ts->xbuf_lock);

	msg.flags = !I2C_M_RD;
	msg.addr  = address;
	msg.len   = len;
	memcpy(ts->xbuf, buf, len);
	msg.buf   = ts->xbuf;

	while (retries < 5) {
		ret = i2c_transfer(client->adapter, &msg, 1);
		if (ret == 1)	break;
		retries++;
	}

	if (unlikely(retries == 5)) {
		NVT_ERR("error, ret=%d\n", ret);
		ret = -EIO;
	}

	mutex_unlock(&ts->xbuf_lock);

	return ret;
#else	//CONFIG_MTK_I2C_EXTENSION
	int32_t ret = -1;

	mutex_lock(&ts->xbuf_lock);

	memcpy(ts->xbuf, buf, len);

	#if I2C_DMA_SUPPORT
    ret = i2c_write_bytes_dma(client, address, ts->xbuf[0], &ts->xbuf[1], len-1);
	#else
    ret = i2c_write_bytes_non_dma(client, address, ts->xbuf[0], &ts->xbuf[1], len-1);
	#endif

	mutex_unlock(&ts->xbuf_lock);

	return ret;
#endif	//CONFIG_MTK_I2C_EXTENSION
}

/*******************************************************
Description:
	Novatek touchscreen set index/page/addr address.

return:
	Executive outcomes. 0---succeed. -5---access fail.
*******************************************************/
int32_t nvt_set_page(uint16_t i2c_addr, uint32_t addr)
{
	uint8_t buf[4] = {0};

	buf[0] = 0xFF;	//set index/page/addr command
	buf[1] = (addr >> 16) & 0xFF;
	buf[2] = (addr >> 8) & 0xFF;

	return CTP_I2C_WRITE(ts->client, i2c_addr, buf, 3);
}

/*******************************************************
Description:
	Novatek touchscreen reset MCU then into idle mode
	function.

return:
	n.a.
*******************************************************/
void nvt_sw_reset_idle(void)
{
	uint8_t buf[4]={0};

	//---write i2c cmds to reset idle---
	buf[0]=0x00;
	buf[1]=0xA5;
	CTP_I2C_WRITE(ts->client, I2C_HW_Address, buf, 2);

	msleep(15);
}

/*******************************************************
Description:
	Novatek touchscreen reset MCU (boot) function.

return:
	n.a.
*******************************************************/
void nvt_bootloader_reset(void)
{
	uint8_t buf[8] = {0};

	NVT_LOG("start\n");

	//---write i2c cmds to reset---
	buf[0] = 0x00;
	buf[1] = 0x69;
	CTP_I2C_WRITE(ts->client, I2C_HW_Address, buf, 2);

	// need 35ms delay after bootloader reset
	msleep(35);

	NVT_LOG("end\n");
}

/*******************************************************
Description:
	Novatek touchscreen clear FW status function.

return:
	Executive outcomes. 0---succeed. -1---fail.
*******************************************************/
int32_t nvt_clear_fw_status(void)
{
	uint8_t buf[8] = {0};
	int32_t i = 0;
	const int32_t retry = 20;

	for (i = 0; i < retry; i++) {
		//---set xdata index to EVENT BUF ADDR---
		nvt_set_page(I2C_FW_Address, ts->mmap->EVENT_BUF_ADDR | EVENT_MAP_HANDSHAKING_or_SUB_CMD_BYTE);

		//---clear fw status---
		buf[0] = EVENT_MAP_HANDSHAKING_or_SUB_CMD_BYTE;
		buf[1] = 0x00;
		CTP_I2C_WRITE(ts->client, I2C_FW_Address, buf, 2);

		//---read fw status---
		buf[0] = EVENT_MAP_HANDSHAKING_or_SUB_CMD_BYTE;
		buf[1] = 0xFF;
		CTP_I2C_READ(ts->client, I2C_FW_Address, buf, 2);

		if (buf[1] == 0x00)
			break;

		usleep_range(10000, 10000);
	}

	if (i >= retry) {
		NVT_ERR("failed, i=%d, buf[1]=0x%02X\n", i, buf[1]);
		return -1;
	} else {
		return 0;
	}
}

/*******************************************************
Description:
	Novatek touchscreen check FW status function.

return:
	Executive outcomes. 0---succeed. -1---failed.
*******************************************************/
int32_t nvt_check_fw_status(void)
{
	uint8_t buf[8] = {0};
	int32_t i = 0;
	const int32_t retry = 50;

	for (i = 0; i < retry; i++) {
		//---set xdata index to EVENT BUF ADDR---
		nvt_set_page(I2C_FW_Address, ts->mmap->EVENT_BUF_ADDR | EVENT_MAP_HANDSHAKING_or_SUB_CMD_BYTE);

		//---read fw status---
		buf[0] = EVENT_MAP_HANDSHAKING_or_SUB_CMD_BYTE;
		buf[1] = 0x00;
		CTP_I2C_READ(ts->client, I2C_FW_Address, buf, 2);

		if ((buf[1] & 0xF0) == 0xA0)
			break;

		usleep_range(10000, 10000);
	}

	if (i >= retry) {
		NVT_ERR("failed, i=%d, buf[1]=0x%02X\n", i, buf[1]);
		return -1;
	} else {
		return 0;
	}
}

/*******************************************************
Description:
	Novatek touchscreen check FW reset state function.

return:
	Executive outcomes. 0---succeed. -1---failed.
*******************************************************/
int32_t nvt_check_fw_reset_state(RST_COMPLETE_STATE check_reset_state)
{
	uint8_t buf[8] = {0};
	int32_t ret = 0;
	int32_t retry = 0;

	while (1) {
		usleep_range(10000, 10000);

		//---read reset state---
		buf[0] = EVENT_MAP_RESET_COMPLETE;
		buf[1] = 0x00;
		CTP_I2C_READ(ts->client, I2C_FW_Address, buf, 6);

		if ((buf[1] >= check_reset_state) && (buf[1] <= RESET_STATE_MAX)) {
			ret = 0;
			break;
		}

		retry++;
		if(unlikely(retry > 100)) {
			NVT_ERR("error, retry=%d, buf[1]=0x%02X, 0x%02X, 0x%02X, 0x%02X, 0x%02X\n",
				retry, buf[1], buf[2], buf[3], buf[4], buf[5]);
			ret = -1;
			break;
		}
	}

	return ret;
}

/*******************************************************
Description:
	Novatek touchscreen get novatek project id information
	function.

return:
	Executive outcomes. 0---success. -1---fail.
*******************************************************/
int32_t nvt_read_pid(void)
{
	uint8_t buf[3] = {0};
	int32_t ret = 0;

	//---set xdata index to EVENT BUF ADDR---
	nvt_set_page(I2C_FW_Address, ts->mmap->EVENT_BUF_ADDR | EVENT_MAP_PROJECTID);

	//---read project id---
	buf[0] = EVENT_MAP_PROJECTID;
	buf[1] = 0x00;
	buf[2] = 0x00;
	CTP_I2C_READ(ts->client, I2C_FW_Address, buf, 3);

	ts->nvt_pid = (buf[2] << 8) + buf[1];

	NVT_LOG("PID=%04X\n", ts->nvt_pid);

	return ret;
}

/*******************************************************
Description:
	Novatek touchscreen get firmware related information
	function.

return:
	Executive outcomes. 0---success. -1---fail.
*******************************************************/
int32_t nvt_get_fw_info(void)
{
	uint8_t buf[64] = {0};
	uint32_t retry_count = 0;
	int32_t ret = 0;

info_retry:
	//---set xdata index to EVENT BUF ADDR---
	nvt_set_page(I2C_FW_Address, ts->mmap->EVENT_BUF_ADDR | EVENT_MAP_FWINFO);

	//---read fw info---
	buf[0] = EVENT_MAP_FWINFO;
	CTP_I2C_READ(ts->client, I2C_FW_Address, buf, 17);
	ts->fw_ver = buf[1];
	ts->x_num = buf[3];
	ts->y_num = buf[4];
	ts->abs_x_max = (uint16_t)((buf[5] << 8) | buf[6]);
	ts->abs_y_max = (uint16_t)((buf[7] << 8) | buf[8]);
	ts->max_button_num = buf[11];

	//---clear x_num, y_num if fw info is broken---
	if ((buf[1] + buf[2]) != 0xFF) {
		NVT_ERR("FW info is broken! fw_ver=0x%02X, ~fw_ver=0x%02X\n", buf[1], buf[2]);
		ts->fw_ver = 0;
		ts->x_num = 18;
		ts->y_num = 32;
		ts->abs_x_max = TOUCH_DEFAULT_MAX_WIDTH;
		ts->abs_y_max = TOUCH_DEFAULT_MAX_HEIGHT;
		ts->max_button_num = TOUCH_KEY_NUM;

		if(retry_count < 3) {
			retry_count++;
			NVT_ERR("retry_count=%d\n", retry_count);
			goto info_retry;
		} else {
			NVT_ERR("Set default fw_ver=%d, x_num=%d, y_num=%d, "
					"abs_x_max=%d, abs_y_max=%d, max_button_num=%d!\n",
					ts->fw_ver, ts->x_num, ts->y_num,
					ts->abs_x_max, ts->abs_y_max, ts->max_button_num);
			ret = -1;
		}
	} else {
		ret = 0;
	}

	//---Get Novatek PID---
	nvt_read_pid();

	return ret;
}

/*******************************************************
  Create Device Node (Proc Entry)
*******************************************************/
#if NVT_TOUCH_PROC
static struct proc_dir_entry *NVT_proc_entry;
#define DEVICE_NAME	"NVTflash"

/*******************************************************
Description:
	Novatek touchscreen /proc/NVTflash read function.

return:
	Executive outcomes. 2---succeed. -5,-14---failed.
*******************************************************/
static ssize_t nvt_flash_read(struct file *file, char __user *buff, size_t count, loff_t *offp)
{
	uint8_t str[68] = {0};
	int32_t ret = -1;
	int32_t retries = 0;
	int8_t i2c_wr = 0;

	if (count > sizeof(str)) {
		NVT_ERR("error count=%zu\n", count);
		return -EFAULT;
	}

	if (copy_from_user(str, buff, count)) {
		NVT_ERR("copy from user error\n");
		return -EFAULT;
	}

	if (str[1] > (sizeof(str) - 2) || str[1] < 1) {
		NVT_ERR("i2c transfer lens invalid\n");
		return -EFAULT;
	}

#if NVT_TOUCH_ESD_PROTECT
	/*
	 * stop esd check work to avoid case that 0x77 report righ after here to enable esd check again
	 * finally lead to trigger esd recovery bootloader reset
	 */
	cancel_delayed_work_sync(&nvt_esd_check_work);
	nvt_esd_check_enable(false);
#endif /* #if NVT_TOUCH_ESD_PROTECT */

	i2c_wr = str[0] >> 7;

	if (i2c_wr == 0) {	//I2C write
		while (retries < 20) {
			ret = CTP_I2C_WRITE(ts->client, (str[0] & 0x7F), &str[2], str[1]);
			if (ret == 1)
				break;
			else
				NVT_ERR("error, retries=%d, ret=%d\n", retries, ret);

			retries++;
		}

		if (unlikely(retries == 20)) {
			NVT_ERR("error, ret = %d\n", ret);
			return -EIO;
		}

		return ret;
	} else if (i2c_wr == 1) {	//I2C read
		while (retries < 20) {
			ret = CTP_I2C_READ(ts->client, (str[0] & 0x7F), &str[2], str[1]);
			if (ret == 2)
				break;
			else
				NVT_ERR("error, retries=%d, ret=%d\n", retries, ret);

			retries++;
		}

		// copy buff to user if i2c transfer
		if (retries < 20) {
			if (copy_to_user(buff, str, count))
				return -EFAULT;
		}

		if (unlikely(retries == 20)) {
			NVT_ERR("error, ret = %d\n", ret);
			return -EIO;
		}

		return ret;
	} else {
		NVT_ERR("Call error, str[0]=%d\n", str[0]);
		return -EFAULT;
	}
}

/*******************************************************
Description:
	Novatek touchscreen /proc/NVTflash open function.

return:
	Executive outcomes. 0---succeed. -12---failed.
*******************************************************/
static int32_t nvt_flash_open(struct inode *inode, struct file *file)
{
	struct nvt_flash_data *dev;

	dev = kmalloc(sizeof(struct nvt_flash_data), GFP_KERNEL);
	if (dev == NULL) {
		NVT_ERR("Failed to allocate memory for nvt flash data\n");
		return -ENOMEM;
	}

	rwlock_init(&dev->lock);
	file->private_data = dev;

	return 0;
}

/*******************************************************
Description:
	Novatek touchscreen /proc/NVTflash close function.

return:
	Executive outcomes. 0---succeed.
*******************************************************/
static int32_t nvt_flash_close(struct inode *inode, struct file *file)
{
	struct nvt_flash_data *dev = file->private_data;

	if (dev)
		kfree(dev);

	return 0;
}

static const struct file_operations nvt_flash_fops = {
	.owner = THIS_MODULE,
	.open = nvt_flash_open,
	.release = nvt_flash_close,
	.read = nvt_flash_read,
};

/*******************************************************
Description:
	Novatek touchscreen /proc/NVTflash initial function.

return:
	Executive outcomes. 0---succeed. -12---failed.
*******************************************************/
static int32_t nvt_flash_proc_init(void)
{
	NVT_proc_entry = proc_create(DEVICE_NAME, 0444, NULL,&nvt_flash_fops);
	if (NVT_proc_entry == NULL) {
		NVT_ERR("Failed!\n");
		return -ENOMEM;
	} else {
		NVT_LOG("Succeeded!\n");
	}

	NVT_LOG("============================================================\n");
	NVT_LOG("Create /proc/%s\n", DEVICE_NAME);
	NVT_LOG("============================================================\n");

	return 0;
}

/*******************************************************
Description:
	Novatek touchscreen /proc/NVTflash deinitial function.

return:
	n.a.
*******************************************************/
static void nvt_flash_proc_deinit(void)
{
	if (NVT_proc_entry != NULL) {
		remove_proc_entry(DEVICE_NAME, NULL);
		NVT_proc_entry = NULL;
		NVT_LOG("Removed /proc/%s\n", DEVICE_NAME);
	}
}
#endif

#if WAKEUP_GESTURE
#define GESTURE_WORD_C          12
#define GESTURE_WORD_W          13
#define GESTURE_WORD_V          14
#define GESTURE_DOUBLE_CLICK    15
#define GESTURE_WORD_Z          16
#define GESTURE_WORD_M          17
#define GESTURE_WORD_O          18
#define GESTURE_WORD_e          19
#define GESTURE_WORD_S          20
#define GESTURE_SLIDE_UP        21
#define GESTURE_SLIDE_DOWN      22
#define GESTURE_SLIDE_LEFT      23
#define GESTURE_SLIDE_RIGHT     24
/* customized gesture id */
#define DATA_PROTOCOL           30

/* function page definition */
#define FUNCPAGE_GESTURE         1

/*******************************************************
Description:
	Novatek touchscreen wake up gesture key report function.

return:
	n.a.
*******************************************************/
void nvt_ts_wakeup_gesture_report(uint8_t gesture_id, uint8_t *data)
{
	uint32_t keycode = 0;
	uint8_t func_type = data[2];
	uint8_t func_id = data[3];

	/* support fw specifal data protocol */
	if ((gesture_id == DATA_PROTOCOL) && (func_type == FUNCPAGE_GESTURE)) {
		gesture_id = func_id;
	} else if (gesture_id > DATA_PROTOCOL) {
		NVT_ERR("gesture_id %d is invalid, func_type=%d, func_id=%d\n", gesture_id, func_type, func_id);
		return;
	}

	NVT_LOG("gesture_id = %d\n", gesture_id);

	switch (gesture_id) {
		case GESTURE_WORD_C:
			NVT_LOG("Gesture : Word-C.\n");
			keycode = gesture_key_array[0];
			break;
		case GESTURE_WORD_W:
			NVT_LOG("Gesture : Word-W.\n");
			keycode = gesture_key_array[1];
			break;
		case GESTURE_WORD_V:
			NVT_LOG("Gesture : Word-V.\n");
			keycode = gesture_key_array[2];
			break;
		case GESTURE_DOUBLE_CLICK:
			NVT_LOG("Gesture : Double Click.\n");
			keycode = gesture_key_array[3];
			break;
		case GESTURE_WORD_Z:
			NVT_LOG("Gesture : Word-Z.\n");
			keycode = gesture_key_array[4];
			break;
		case GESTURE_WORD_M:
			NVT_LOG("Gesture : Word-M.\n");
			keycode = gesture_key_array[5];
			break;
		case GESTURE_WORD_O:
			NVT_LOG("Gesture : Word-O.\n");
			keycode = gesture_key_array[6];
			break;
		case GESTURE_WORD_e:
			NVT_LOG("Gesture : Word-e.\n");
			keycode = gesture_key_array[7];
			break;
		case GESTURE_WORD_S:
			NVT_LOG("Gesture : Word-S.\n");
			keycode = gesture_key_array[8];
			break;
		case GESTURE_SLIDE_UP:
			NVT_LOG("Gesture : Slide UP.\n");
			keycode = gesture_key_array[9];
			break;
		case GESTURE_SLIDE_DOWN:
			NVT_LOG("Gesture : Slide DOWN.\n");
			keycode = gesture_key_array[10];
			break;
		case GESTURE_SLIDE_LEFT:
			NVT_LOG("Gesture : Slide LEFT.\n");
			keycode = gesture_key_array[11];
			break;
		case GESTURE_SLIDE_RIGHT:
			NVT_LOG("Gesture : Slide RIGHT.\n");
			keycode = gesture_key_array[12];
			break;
		default:
			break;
	}

	if (keycode > 0) {
		input_report_key(ts->input_dev, keycode, 1);
		input_sync(ts->input_dev);
		input_report_key(ts->input_dev, keycode, 0);
		input_sync(ts->input_dev);
	}
}
#endif	//WAKEUP_GESTURE

static uint8_t nvt_fw_recovery(uint8_t *point_data)
{
	uint8_t i = 0;
	uint8_t detected = true;

	/* check pattern */
	for (i=1 ; i<7 ; i++) {
		if (point_data[i] != 0x77) {
			detected = false;
			break;
		}
	}

	return detected;
}

#if NVT_TOUCH_ESD_PROTECT
void nvt_esd_check_enable(uint8_t enable)
{
	/* update interrupt timer */
	irq_timer = jiffies;
	/* clear esd_retry counter, if protect function is enabled */
	esd_retry = enable ? 0 : esd_retry;
	/* enable/disable esd check flag */
	esd_check = enable;
}

static void nvt_esd_check_func(struct work_struct *work)
{
	unsigned int timer = jiffies_to_msecs(jiffies - irq_timer);

	//NVT_ERR("esd_check = %d (retry %d)\n", esd_check, esd_retry);	//DEBUG

	if ((timer > NVT_TOUCH_ESD_CHECK_PERIOD) && esd_check) {
		mutex_lock(&ts->lock);
		NVT_ERR("do ESD recovery, timer = %d, retry = %d\n", timer, esd_retry);
		/* do esd recovery, bootloader reset */
		nvt_bootloader_reset();
		mutex_unlock(&ts->lock);
		/* update interrupt timer */
		irq_timer = jiffies;
		/* update esd_retry counter */
		esd_retry++;
	}

	queue_delayed_work(nvt_esd_check_wq, &nvt_esd_check_work,
			msecs_to_jiffies(NVT_TOUCH_ESD_CHECK_PERIOD));
}
#endif /* #if NVT_TOUCH_ESD_PROTECT */

#define POINT_DATA_LEN 65
/*******************************************************
Description:
	Novatek touchscreen work function.

return:
	n.a.
*******************************************************/
static int touch_event_handler(void *unused)
{
	struct sched_param param = { .sched_priority = 4 };

	int32_t ret = -1;
	uint8_t point_data[POINT_DATA_LEN + 1] = {0};
	uint32_t position = 0;
	uint32_t input_x = 0;
	uint32_t input_y = 0;
	uint32_t input_w = 0;
	uint32_t input_p = 0;
	uint8_t input_id = 0;
#if MT_PROTOCOL_B
	uint8_t press_id[TOUCH_MAX_FINGER_NUM] = {0};
#endif /* MT_PROTOCOL_B */

	int32_t i = 0;
	int32_t finger_cnt = 0;

	sched_setscheduler(current, SCHED_RR, &param);
	do
	{
		set_current_state(TASK_INTERRUPTIBLE);

		wait_event_interruptible(waiter, tpd_flag != 0);
		pr_err("zwq touch_event_handler!");
		tpd_flag = 0;
		set_current_state(TASK_RUNNING);

		mutex_lock(&ts->lock);
		memset(point_data, 0, POINT_DATA_LEN + 1);

		ret = CTP_I2C_READ(ts->client, I2C_FW_Address, point_data, POINT_DATA_LEN + 1);
		if (ret < 0) {
			NVT_ERR("CTP_I2C_READ failed.(%d)\n", ret);
			goto XFER_ERROR;
		}

/*
		//--- dump I2C buf ---
		for (i = 0; i < 10; i++) {
		printk("%02X %02X %02X %02X %02X %02X  ",
			point_data[1+i*6], point_data[2+i*6], point_data[3+i*6], point_data[4+i*6], point_data[5+i*6], point_data[6+i*6]);
		}
		printk("\n");
*/

		if (nvt_fw_recovery(point_data)) {
#if NVT_TOUCH_ESD_PROTECT
			nvt_esd_check_enable(true);
#endif /* #if NVT_TOUCH_ESD_PROTECT */
			goto XFER_ERROR;
		}

		if (bTouchIsAwake == 0) {
#if WAKEUP_GESTURE
			input_id = (uint8_t)(point_data[1] >> 3);
			nvt_ts_wakeup_gesture_report(input_id, point_data);
#endif
			nvt_irq_enable(true);
			mutex_unlock(&ts->lock);
			NVT_LOG("return for interrupt after suspend...\n");
			continue;
		}

		finger_cnt = 0;
#if MT_PROTOCOL_B
		memset(press_id, 0, ts->max_touch_num);
#endif /* MT_PROTOCOL_B */

		for (i = 0; i < ts->max_touch_num; i++) {
			position = 1 + 6 * i;
			input_id = (uint8_t)(point_data[position + 0] >> 3);
			if ((input_id == 0) || (input_id > ts->max_touch_num))
				continue;

			if (((point_data[position] & 0x07) == 0x01) || ((point_data[position] & 0x07) == 0x02)) {	//finger down (enter & moving)
#if NVT_TOUCH_ESD_PROTECT
				/* update interrupt timer */
				irq_timer = jiffies;
#endif /* #if NVT_TOUCH_ESD_PROTECT */
				input_x = (uint32_t)(point_data[position + 1] << 4) + (uint32_t) (point_data[position + 3] >> 4);
				input_y = (uint32_t)(point_data[position + 2] << 4) + (uint32_t) (point_data[position + 3] & 0x0F);
				if ((input_x < 0) || (input_y < 0))
					continue;
				if ((input_x > ts->abs_x_max) || (input_y > ts->abs_y_max))
					continue;
				input_w = (uint32_t)(point_data[position + 4]);
				if (input_w == 0)
					input_w = 1;
				if (i < 2) {
					input_p = (uint32_t)(point_data[position + 5]) + (uint32_t)(point_data[i + 63] << 8);
					if (input_p > TOUCH_FORCE_NUM)
						input_p = TOUCH_FORCE_NUM;
				} else {
					input_p = (uint32_t)(point_data[position + 5]);
				}
				if (input_p == 0)
					input_p = 1;

#if MT_PROTOCOL_B
				press_id[input_id - 1] = 1;
				input_mt_slot(ts->input_dev, input_id - 1);
				input_mt_report_slot_state(ts->input_dev, MT_TOOL_FINGER, true);
#else /* MT_PROTOCOL_B */
				input_report_abs(ts->input_dev, ABS_MT_TRACKING_ID, input_id - 1);
				input_report_key(ts->input_dev, BTN_TOUCH, 1);
#endif /* MT_PROTOCOL_B */

				input_report_abs(ts->input_dev, ABS_MT_POSITION_X, input_x);
				input_report_abs(ts->input_dev, ABS_MT_POSITION_Y, input_y);
				input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR, input_w);
				input_report_abs(ts->input_dev, ABS_MT_PRESSURE, input_p);

#if MT_PROTOCOL_B
#else /* MT_PROTOCOL_B */
				input_mt_sync(ts->input_dev);
#endif /* MT_PROTOCOL_B */

				finger_cnt++;
			}
		}

#if MT_PROTOCOL_B
		for (i = 0; i < ts->max_touch_num; i++) {
			if (press_id[i] != 1) {
				input_mt_slot(ts->input_dev, i);
				input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR, 0);
				input_report_abs(ts->input_dev, ABS_MT_PRESSURE, 0);
				input_mt_report_slot_state(ts->input_dev, MT_TOOL_FINGER, false);
			}
		}

		input_report_key(ts->input_dev, BTN_TOUCH, (finger_cnt > 0));
#else /* MT_PROTOCOL_B */
		if (finger_cnt == 0) {
			input_report_key(ts->input_dev, BTN_TOUCH, 0);
			input_mt_sync(ts->input_dev);
		}
#endif /* MT_PROTOCOL_B */

#if TOUCH_KEY_NUM > 0
		if (point_data[61] == 0xF8) {
#if NVT_TOUCH_ESD_PROTECT
			/* update interrupt timer */
			irq_timer = jiffies;
#endif /* #if NVT_TOUCH_ESD_PROTECT */
			for (i = 0; i < ts->max_button_num; i++) {
				input_report_key(ts->input_dev, touch_key_array[i], ((point_data[62] >> i) & 0x01));
			}
		} else {
			for (i = 0; i < ts->max_button_num; i++) {
				input_report_key(ts->input_dev, touch_key_array[i], 0);
			}
		}
#endif

		input_sync(ts->input_dev);

XFER_ERROR:
		nvt_irq_enable(true);

		mutex_unlock(&ts->lock);

	} while (!kthread_should_stop());

	return 0;
}

/*******************************************************
Description:
	External interrupt service routine.

return:
	irq execute status.
*******************************************************/
static irqreturn_t nvt_ts_irq_handler(int32_t irq, void *dev_id)
{
	unsigned long flags;

	spin_lock_irqsave(&ts->irq_lock, flags);
	if (!ts->irq_enabled) {
		spin_unlock_irqrestore(&ts->irq_lock, flags);
		return IRQ_HANDLED;
	}

	tpd_flag = 1;
	ts->irq_enabled = false;
	spin_unlock_irqrestore(&ts->irq_lock, flags);
	disable_irq_nosync(ts->client->irq);

#if WAKEUP_GESTURE
	if (bTouchIsAwake == 0) {
		pm_wakeup_event(&ts->input_dev->dev, 5000);
	}
#endif

	wake_up_interruptible(&waiter);

	return IRQ_HANDLED;
}

/*******************************************************
 * nvt_ts_event_polling used for bring up(irq can't use)
 *******************************************************/
#if NVT_TOUCH_POLLING
static int nvt_ts_event_polling(void *arg)
{
	struct sched_param param = { .sched_priority = 4 };

	sched_setscheduler(current, SCHED_RR, &param);
	do {
		usleep_range(7000, 8000);
		tpd_flag = 1;
		NVT_LOG("polling thread!");
		wake_up_interruptible(&waiter);
	} while (!kthread_should_stop());
	return 0;
}
#endif
/*******************************************************
Description:
	Register interrupt handler

return:
	irq execute status.
*******************************************************/
static int nvt_irq_registration(void)
{
	struct device_node *node = NULL;
	int ret = 0;
	u32 ints[2] = { 0, 0 };

	NVT_LOG("Device Tree Tpd_irq_registration!\n");

	node = of_find_matching_node(node, touch_of_match);
	if (node) {
		of_property_read_u32_array(node, "debounce", ints, ARRAY_SIZE(ints));
		//gpio_set_debounce(ints[0], ints[1]);

		ts->client->irq = irq_of_parse_and_map(node, 0);
		NVT_LOG("int_trigger_type=%d\n", ts->int_trigger_type);
		ret = request_irq(ts->client->irq, nvt_ts_irq_handler, ts->int_trigger_type, NVT_I2C_NAME, ts);
		if (ret > 0) {
			ret = -1;
			NVT_ERR("tpd request_irq IRQ LINE NOT AVAILABLE!.\n");
		}
	} else {
		NVT_ERR("tpd request_irq can not find touch eint device node!.\n");
		ret = -1;
	}
	NVT_LOG("irq:%d, debounce:%d-%d:\n", ts->client->irq, ints[0], ints[1]);

	return ret;
}

/*******************************************************
Description:
	Novatek touchscreen check and stop crc reboot loop.

return:
	n.a.
*******************************************************/
void nvt_stop_crc_reboot(void)
{
	uint8_t buf[8] = {0};
	int32_t retry = 0;

	//read dummy buffer to check CRC fail reboot is happening or not

	//---change I2C index to prevent geting 0xFF, but not 0xFC---
	nvt_set_page(I2C_BLDR_Address, 0x1F64E);

	//---read to check if buf is 0xFC which means IC is in CRC reboot ---
	buf[0] = 0x4E;
	CTP_I2C_READ(ts->client, I2C_BLDR_Address, buf, 4);

	if ((buf[1] == 0xFC) ||
		((buf[1] == 0xFF) && (buf[2] == 0xFF) && (buf[3] == 0xFF))) {

		//IC is in CRC fail reboot loop, needs to be stopped!
		for (retry = 5; retry > 0; retry--) {

			//---write i2c cmds to reset idle : 1st---
			buf[0]=0x00;
			buf[1]=0xA5;
			CTP_I2C_WRITE(ts->client, I2C_HW_Address, buf, 2);

			//---write i2c cmds to reset idle : 2rd---
			buf[0]=0x00;
			buf[1]=0xA5;
			CTP_I2C_WRITE(ts->client, I2C_HW_Address, buf, 2);
			msleep(1);

			//---clear CRC_ERR_FLAG---
			nvt_set_page(I2C_BLDR_Address, 0x3F135);

			buf[0] = 0x35;
			buf[1] = 0xA5;
			CTP_I2C_WRITE(ts->client, I2C_BLDR_Address, buf, 2);

			//---check CRC_ERR_FLAG---
			nvt_set_page(I2C_BLDR_Address, 0x3F135);

			buf[0] = 0x35;
			buf[1] = 0x00;
			CTP_I2C_READ(ts->client, I2C_BLDR_Address, buf, 2);

			if (buf[1] == 0xA5)
				break;
		}
		if (retry == 0)
			NVT_ERR("CRC auto reboot is not able to be stopped! buf[1]=0x%02X\n", buf[1]);
	}

	return;
}

/*******************************************************
Description:
	Novatek touchscreen check chip version trim function.

return:
	Executive outcomes. 0---NVT IC. -1---not NVT IC.
*******************************************************/
static int8_t nvt_ts_check_chip_ver_trim(uint32_t chip_ver_trim_addr)
{
	uint8_t buf[8] = {0};
	int32_t retry = 0;
	int32_t list = 0;
	int32_t i = 0;
	int32_t found_nvt_chip = 0;
	int32_t ret = -1;

	nvt_bootloader_reset(); // NOT in retry loop

	//---Check for 5 times---
	for (retry = 5; retry > 0; retry--) {
		nvt_sw_reset_idle();

		buf[0] = 0x00;
		buf[1] = 0x35;
		CTP_I2C_WRITE(ts->client, I2C_HW_Address, buf, 2);
		msleep(10);

		nvt_set_page(I2C_BLDR_Address, chip_ver_trim_addr);

		buf[0] = chip_ver_trim_addr & 0xFF;
		buf[1] = 0x00;
		buf[2] = 0x00;
		buf[3] = 0x00;
		buf[4] = 0x00;
		buf[5] = 0x00;
		buf[6] = 0x00;
		CTP_I2C_READ(ts->client, I2C_BLDR_Address, buf, 7);
		NVT_LOG("buf[1]=0x%02X, buf[2]=0x%02X, buf[3]=0x%02X, buf[4]=0x%02X, buf[5]=0x%02X, buf[6]=0x%02X\n",
			buf[1], buf[2], buf[3], buf[4], buf[5], buf[6]);

		//---Stop CRC check to prevent IC auto reboot---
		if ((buf[1] == 0xFC) ||
			((buf[1] == 0xFF) && (buf[2] == 0xFF) && (buf[3] == 0xFF))) {
			nvt_stop_crc_reboot();
			continue;
		}

		// compare read chip id on supported list
		for (list = 0; list < (sizeof(trim_id_table) / sizeof(struct nvt_ts_trim_id_table)); list++) {
			found_nvt_chip = 0;

			// compare each byte
			for (i = 0; i < NVT_ID_BYTE_MAX; i++) {
				if (trim_id_table[list].mask[i]) {
					if (buf[i + 1] != trim_id_table[list].id[i])
						break;
				}
			}

			if (i == NVT_ID_BYTE_MAX) {
				found_nvt_chip = 1;
			}

			if (found_nvt_chip) {
				NVT_LOG("This is NVT touch IC\n");
				ts->mmap = trim_id_table[list].mmap;
				ts->carrier_system = trim_id_table[list].hwinfo->carrier_system;
				ret = 0;
				goto out;
			} else {
				ts->mmap = NULL;
				ret = -1;
			}
		}

		msleep(10);
	}

out:
	return ret;
}

/*******************************************************
Description:
	Novatek touchscreen driver probe function.

return:
	Executive outcomes. 0---succeed. negative---failed
*******************************************************/
static int32_t nvt_ts_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	int32_t ret = 0;
#if ((TOUCH_KEY_NUM > 0) || WAKEUP_GESTURE)
	int32_t retry = 0;
#endif

	NVT_LOG("start\n");

	ts = (struct nvt_ts_data *)kzalloc(sizeof(struct nvt_ts_data), GFP_KERNEL);
	if (ts == NULL) {
		NVT_ERR("failed to allocated memory for nvt ts data\n");
		return -ENOMEM;
	}

	ts->xbuf = (uint8_t *)kzalloc(NVT_XBUF_LEN, GFP_KERNEL);
	if (ts->xbuf == NULL) {
		NVT_ERR("kzalloc for xbuf failed!\n");
		ret = -ENOMEM;
		goto err_malloc_xbuf;
	}

	ts->client = client;
	i2c_set_clientdata(client, ts);

#if NVT_TOUCH_SUPPORT_HW_RST
	NVT_GPIO_OUTPUT(GTP_RST_PORT, 1);
#endif
	//---request INT-pin---
	NVT_GPIO_AS_INT(GTP_INT_PORT);

	//---check i2c func.---
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		NVT_ERR("i2c_check_functionality failed. (no I2C_FUNC_I2C)\n");
		ret = -ENODEV;
		goto err_check_functionality_failed;
	}

	mutex_init(&ts->lock);
	mutex_init(&ts->xbuf_lock);
	spin_lock_init(&ts->irq_lock);

	// need 10ms delay after POR(power on reset)
	msleep(10);

	//---check input device---
	if (tpd->dev == NULL) {
		NVT_LOG("input device tpd->dev is NULL, allocate for ts->input_dev\n");
		//---allocate input device---
		ts->input_dev = input_allocate_device();
		if (ts->input_dev == NULL) {
			NVT_ERR("allocate input device failed\n");
			ret = -ENOMEM;
			goto err_input_dev_alloc_failed;
		}

		//---register input device---
		ts->input_dev->name = NVT_TS_NAME;
		ret = input_register_device(ts->input_dev);
		if (ret) {
			NVT_ERR("register input device (%s) failed. ret=%d\n", ts->input_dev->name, ret);
			goto err_input_register_device_failed;
		}
	} else {
		ts->input_dev = tpd->dev;
	}

#ifdef CONFIG_MTK_I2C_EXTENSION
#if I2C_DMA_SUPPORT
	ts->input_dev->dev.coherent_dma_mask = DMA_BIT_MASK(32);

	gpDMABuf_va = (uint8_t *)dma_alloc_coherent(&ts->input_dev->dev,
			DMA_MAX_TRANSACTION_LENGTH, &gpDMABuf_pa, GFP_KERNEL);
	if(!gpDMABuf_va){
		NVT_ERR("Allocate DMA I2C Buffer failed!\n");
		goto err_dma_alloc_coherent_failed;
	}
	memset(gpDMABuf_va, 0, DMA_MAX_TRANSACTION_LENGTH);
#endif
#endif

	//---check chip version trim---
	ret = nvt_ts_check_chip_ver_trim(CHIP_VER_TRIM_ADDR);
	if (ret) {
		NVT_LOG("try to check from old chip ver trim address\n");
		ret = nvt_ts_check_chip_ver_trim(CHIP_VER_TRIM_OLD_ADDR);
		if (ret) {
			NVT_ERR("chip is not identified\n");
			ret = -EINVAL;
			goto err_chipvertrim_failed;
		}
	}

	nvt_bootloader_reset();
	nvt_check_fw_reset_state(RESET_STATE_INIT);
	nvt_get_fw_info();

	thread = kthread_run(touch_event_handler, 0, TPD_DEVICE);
	if (IS_ERR(thread))
	{
		ret = PTR_ERR(thread);
		NVT_ERR("failed to create kernel thread: %d\n", ret);
		goto err_create_kthread_failed;
	}

	ts->max_touch_num = TOUCH_MAX_FINGER_NUM;

#if TOUCH_KEY_NUM > 0
	ts->max_button_num = TOUCH_KEY_NUM;
#endif

	ts->int_trigger_type = INT_TRIGGER_TYPE;


	//---set input device info.---
	ts->input_dev->evbit[0] = BIT_MASK(EV_SYN) | BIT_MASK(EV_KEY) | BIT_MASK(EV_ABS);
	ts->input_dev->keybit[BIT_WORD(BTN_TOUCH)] = BIT_MASK(BTN_TOUCH);
	ts->input_dev->propbit[0] = BIT(INPUT_PROP_DIRECT);

#if MT_PROTOCOL_B
	input_mt_init_slots(ts->input_dev, ts->max_touch_num, 0);
#endif

	input_set_abs_params(ts->input_dev, ABS_MT_PRESSURE, 0, TOUCH_FORCE_NUM, 0, 0);    //pressure = TOUCH_FORCE_NUM

#if TOUCH_MAX_FINGER_NUM > 1
	input_set_abs_params(ts->input_dev, ABS_MT_TOUCH_MAJOR, 0, 255, 0, 0);    //area = 255

	input_set_abs_params(ts->input_dev, ABS_MT_POSITION_X, 0, ts->abs_x_max, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_POSITION_Y, 0, ts->abs_y_max, 0, 0);
#if MT_PROTOCOL_B
	// no need to set ABS_MT_TRACKING_ID, input_mt_init_slots() already set it
#else
	input_set_abs_params(ts->input_dev, ABS_MT_TRACKING_ID, 0, ts->max_touch_num, 0, 0);
#endif //MT_PROTOCOL_B
#endif //TOUCH_MAX_FINGER_NUM > 1

#if TOUCH_KEY_NUM > 0
	for (retry = 0; retry < ts->max_button_num; retry++) {
		input_set_capability(ts->input_dev, EV_KEY, touch_key_array[retry]);
	}
#endif

#if WAKEUP_GESTURE
	for (retry = 0; retry < (sizeof(gesture_key_array) / sizeof(gesture_key_array[0])); retry++) {
		input_set_capability(ts->input_dev, EV_KEY, gesture_key_array[retry]);
	}
#endif

	sprintf(ts->phys, "input/ts");
	ts->input_dev->phys = ts->phys;
	ts->input_dev->id.bustype = BUS_I2C;

	ts->irq_enabled = true;
	ret = nvt_irq_registration();
	if (ret != 0) {
		NVT_ERR("request irq failed. ret=%d\n", ret);
		goto err_int_request_failed;
	} else {
		nvt_irq_enable(false);
		NVT_LOG("request irq %d succeed\n", client->irq);
	}

#if WAKEUP_GESTURE
	device_init_wakeup(&ts->input_dev->dev, 1);
#endif

#if BOOT_UPDATE_FIRMWARE
	nvt_fwu_wq = alloc_workqueue("nvt_fwu_wq", WQ_UNBOUND | WQ_MEM_RECLAIM, 1);
	if (!nvt_fwu_wq) {
		NVT_ERR("nvt_fwu_wq create workqueue failed\n");
		ret = -ENOMEM;
		goto err_create_nvt_fwu_wq_failed;
	}
	INIT_WORK(&ts->nvt_fwu_work, Boot_Update_Firmware);
	// please make sure boot update start after display reset(RESX) sequence
	queue_work(nvt_fwu_wq, &ts->nvt_fwu_work);
#endif

	NVT_LOG("NVT_TOUCH_ESD_PROTECT is %d\n", NVT_TOUCH_ESD_PROTECT);
#if NVT_TOUCH_ESD_PROTECT
	INIT_DELAYED_WORK(&nvt_esd_check_work, nvt_esd_check_func);
	nvt_esd_check_wq = alloc_workqueue("nvt_esd_check_wq", WQ_MEM_RECLAIM, 1);
	if (!nvt_esd_check_wq) {
		NVT_ERR("nvt_esd_check_wq create workqueue failed\n");
		ret = -ENOMEM;
		goto err_create_nvt_esd_check_wq_failed;
	}
	queue_delayed_work(nvt_esd_check_wq, &nvt_esd_check_work,
			msecs_to_jiffies(NVT_TOUCH_ESD_CHECK_PERIOD));
#endif /* #if NVT_TOUCH_ESD_PROTECT */

	//---set device node---
#if NVT_TOUCH_PROC
	ret = nvt_flash_proc_init();
	if (ret != 0) {
		NVT_ERR("nvt flash proc init failed. ret=%d\n", ret);
		goto err_flash_proc_init_failed;
	}
#endif

#if NVT_TOUCH_EXT_PROC
	ret = nvt_extra_proc_init();
	if (ret != 0) {
		NVT_ERR("nvt extra proc init failed. ret=%d\n", ret);
		goto err_extra_proc_init_failed;
	}
#endif

#if NVT_TOUCH_MP
	ret = nvt_mp_proc_init();
	if (ret != 0) {
		NVT_ERR("nvt mp proc init failed. ret=%d\n", ret);
		goto err_mp_proc_init_failed;
	}
#endif

#if NVT_TOUCH_POLLING
	nvt_polling_thread = kthread_run(nvt_ts_event_polling,
					0, "nvt_polling_thread");
#endif

	bTouchIsAwake = 1;
	tpd_load_status = 1;
	NVT_LOG("end\n");

	nvt_irq_enable(true);

	return 0;

#if NVT_TOUCH_MP
	nvt_mp_proc_deinit();
err_mp_proc_init_failed:
#endif
#if NVT_TOUCH_EXT_PROC
	nvt_extra_proc_deinit();
err_extra_proc_init_failed:
#endif
#if NVT_TOUCH_PROC
	nvt_flash_proc_deinit();
err_flash_proc_init_failed:
#endif
#if NVT_TOUCH_ESD_PROTECT
	if (nvt_esd_check_wq) {
		cancel_delayed_work_sync(&nvt_esd_check_work);
		destroy_workqueue(nvt_esd_check_wq);
		nvt_esd_check_wq = NULL;
	}
err_create_nvt_esd_check_wq_failed:
#endif
#if BOOT_UPDATE_FIRMWARE
	if (nvt_fwu_wq) {
		cancel_work_sync(&ts->nvt_fwu_work);
		destroy_workqueue(nvt_fwu_wq);
		nvt_fwu_wq = NULL;
	}
err_create_nvt_fwu_wq_failed:
#endif
#if WAKEUP_GESTURE
	device_init_wakeup(&ts->input_dev->dev, 0);
#endif
	free_irq(client->irq, ts);
err_int_request_failed:
err_create_kthread_failed:
err_chipvertrim_failed:
#ifdef CONFIG_MTK_I2C_EXTENSION
#if I2C_DMA_SUPPORT
	if (gpDMABuf_va) {
		dma_free_coherent(NULL, DMA_MAX_TRANSACTION_LENGTH, gpDMABuf_va, gpDMABuf_pa);
		gpDMABuf_va = NULL;
	}
err_dma_alloc_coherent_failed:
#endif
#endif
	if (tpd->dev == NULL) {
		input_unregister_device(ts->input_dev);
		ts->input_dev = NULL;
	}
err_input_register_device_failed:
	if (tpd->dev == NULL) {
		if (ts->input_dev) {
			input_free_device(ts->input_dev);
			ts->input_dev = NULL;
		}
	}
err_input_dev_alloc_failed:
	mutex_destroy(&ts->xbuf_lock);
	mutex_destroy(&ts->lock);
err_check_functionality_failed:
	i2c_set_clientdata(client, NULL);
	if (ts->xbuf) {
		kfree(ts->xbuf);
		ts->xbuf = NULL;
	}
err_malloc_xbuf:
	if (ts) {
		kfree(ts);
		ts = NULL;
	}

	return ret;
}

/*******************************************************
Description:
	Novatek touchscreen driver release function.

return:
	Executive outcomes. 0---succeed.
*******************************************************/
static int32_t nvt_ts_remove(struct i2c_client *client)
{
	NVT_LOG("Removing driver...\n");

#if NVT_TOUCH_MP
	nvt_mp_proc_deinit();
#endif
#if NVT_TOUCH_EXT_PROC
	nvt_extra_proc_deinit();
#endif
#if NVT_TOUCH_PROC
	nvt_flash_proc_deinit();
#endif

#if NVT_TOUCH_ESD_PROTECT
	if (nvt_esd_check_wq) {
		cancel_delayed_work_sync(&nvt_esd_check_work);
		nvt_esd_check_enable(false);
		destroy_workqueue(nvt_esd_check_wq);
		nvt_esd_check_wq = NULL;
	}
#endif

#if BOOT_UPDATE_FIRMWARE
	if (nvt_fwu_wq) {
		cancel_work_sync(&ts->nvt_fwu_work);
		destroy_workqueue(nvt_fwu_wq);
		nvt_fwu_wq = NULL;
	}
#endif

#if WAKEUP_GESTURE
	device_init_wakeup(&ts->input_dev->dev, 0);
#endif

	nvt_irq_enable(false);
	free_irq(client->irq, ts);

#ifdef CONFIG_MTK_I2C_EXTENSION
#if I2C_DMA_SUPPORT
	if (gpDMABuf_va) {
		dma_free_coherent(NULL, DMA_MAX_TRANSACTION_LENGTH, gpDMABuf_va, gpDMABuf_pa);
		gpDMABuf_va = NULL;
	}
#endif
#endif

	if (tpd->dev == NULL) {
		if (ts->input_dev) {
			input_unregister_device(ts->input_dev);
			ts->input_dev = NULL;
		}
	}

	mutex_destroy(&ts->xbuf_lock);
	mutex_destroy(&ts->lock);

	i2c_set_clientdata(client, NULL);

	if (ts->xbuf) {
		kfree(ts->xbuf);
		ts->xbuf = NULL;
	}

	if (ts) {
		kfree(ts);
		ts = NULL;
	}

	return 0;
}

static void nvt_ts_shutdown(struct i2c_client *client)
{
	NVT_LOG("Shutdown driver...\n");

	nvt_irq_enable(false);

#if NVT_TOUCH_MP
	nvt_mp_proc_deinit();
#endif
#if NVT_TOUCH_EXT_PROC
	nvt_extra_proc_deinit();
#endif
#if NVT_TOUCH_PROC
	nvt_flash_proc_deinit();
#endif

#if NVT_TOUCH_ESD_PROTECT
	if (nvt_esd_check_wq) {
		cancel_delayed_work_sync(&nvt_esd_check_work);
		nvt_esd_check_enable(false);
		destroy_workqueue(nvt_esd_check_wq);
		nvt_esd_check_wq = NULL;
	}
#endif /* #if NVT_TOUCH_ESD_PROTECT */

#if BOOT_UPDATE_FIRMWARE
	if (nvt_fwu_wq) {
		cancel_work_sync(&ts->nvt_fwu_work);
		destroy_workqueue(nvt_fwu_wq);
		nvt_fwu_wq = NULL;
	}
#endif

#if WAKEUP_GESTURE
	device_init_wakeup(&ts->input_dev->dev, 0);
#endif
}

static int nvt_i2c_detect(struct i2c_client *client, struct i2c_board_info *info)
{
	strcpy(info->type, NVT_I2C_NAME);
	return 0;
}

static const struct i2c_device_id nvt_ts_id[] = {
	{ NVT_I2C_NAME, 0 },
	{ }
};

#ifdef CONFIG_OF
static const struct of_device_id nvt_match_table[] = {
	{.compatible = "novatek,cap_touch"},
	{},
};
#endif

static struct i2c_driver nvt_i2c_driver = {
	.probe = nvt_ts_probe,
	.remove = nvt_ts_remove,
	.shutdown = nvt_ts_shutdown,
	.detect = nvt_i2c_detect,
	.driver.name = NVT_I2C_NAME,
	.driver = {
		.name = NVT_I2C_NAME,
		.owner = THIS_MODULE,
#ifdef CONFIG_OF
		.of_match_table = nvt_match_table,
#endif
	},
	.id_table = nvt_ts_id,
};

static int nvt_local_init(void)
{
	int ret = 0;

	NVT_LOG("start\n");

	ret = i2c_add_driver(&nvt_i2c_driver);
	if (ret) {
		NVT_ERR("unable to add i2c driver.\n");
		return -1;
	}

	if (tpd_load_status == 0) {
		NVT_ERR("add error touch panel driver.\n");
		i2c_del_driver(&nvt_i2c_driver);
		return -1;
	}

	if (tpd_dts_data.use_tpd_button) {
		/*initialize tpd button data*/
		tpd_button_setting(tpd_dts_data.tpd_key_num, tpd_dts_data.tpd_key_local,
				tpd_dts_data.tpd_key_dim_local);
	}

	tpd_type_cap = 1;

	NVT_LOG("end\n");

	return 0;
}

/*******************************************************
Description:
	Novatek touchscreen driver suspend function.

return:
	Executive outcomes. 0---succeed.
*******************************************************/
static void nvt_ts_suspend(struct device *dev)
{
	uint8_t buf[4] = {0};
#if MT_PROTOCOL_B
	uint32_t i = 0;
#endif

	if (!bTouchIsAwake) {
		NVT_LOG("Touch is already suspend\n");
		return;
	}

#if !WAKEUP_GESTURE
	nvt_irq_enable(false);
#endif

#if NVT_TOUCH_ESD_PROTECT
	cancel_delayed_work_sync(&nvt_esd_check_work);
	nvt_esd_check_enable(false);
#endif /* #if NVT_TOUCH_ESD_PROTECT */

	mutex_lock(&ts->lock);

	NVT_LOG("start\n");

	bTouchIsAwake = 0;

#if WAKEUP_GESTURE
	//---write command to enter "wakeup gesture mode"---
	buf[0] = EVENT_MAP_HOST_CMD;
	buf[1] = 0x13;
	CTP_I2C_WRITE(ts->client, I2C_FW_Address, buf, 2);

	enable_irq_wake(ts->client->irq);

	NVT_LOG("Enabled touch wakeup gesture\n");

#else // WAKEUP_GESTURE
	//---write command to enter "deep sleep mode"---
	buf[0] = EVENT_MAP_HOST_CMD;
	buf[1] = 0x11;
	CTP_I2C_WRITE(ts->client, I2C_FW_Address, buf, 2);
#endif // WAKEUP_GESTURE

	mutex_unlock(&ts->lock);

	/* release all touches */
#if MT_PROTOCOL_B
	for (i = 0; i < ts->max_touch_num; i++) {
		input_mt_slot(ts->input_dev, i);
		input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR, 0);
		input_report_abs(ts->input_dev, ABS_MT_PRESSURE, 0);
		input_mt_report_slot_state(ts->input_dev, MT_TOOL_FINGER, 0);
	}
#endif
	input_report_key(ts->input_dev, BTN_TOUCH, 0);
#if !MT_PROTOCOL_B
	input_mt_sync(ts->input_dev);
#endif
	input_sync(ts->input_dev);

	msleep(50);

	NVT_LOG("end\n");

	return;
}

/*******************************************************
Description:
	Novatek touchscreen driver resume function.

return:
	Executive outcomes. 0---succeed.
*******************************************************/
static void nvt_ts_resume(struct device *dev)
{
	if (bTouchIsAwake) {
		NVT_LOG("Touch is already resume\n");
		return;
	}

	mutex_lock(&ts->lock);

	NVT_LOG("start\n");

	// please make sure display reset(RESX) sequence and mipi dsi cmds sent before this
#if NVT_TOUCH_SUPPORT_HW_RST
	NVT_GPIO_OUTPUT(GTP_RST_PORT, 1);
#endif
	// need to uncomment the following code for NT36672, NT36772 IC due to no boot-load when RESX/TP_RESX
	//nvt_bootloader_reset();
	if (nvt_check_fw_reset_state(RESET_STATE_REK)) {
		NVT_ERR("FW is not ready! Try to bootloader reset...\n");
		nvt_bootloader_reset();
		nvt_check_fw_reset_state(RESET_STATE_REK);
	}

#if !WAKEUP_GESTURE
	nvt_irq_enable(true);
#endif

#if NVT_TOUCH_ESD_PROTECT
	nvt_esd_check_enable(false);
	queue_delayed_work(nvt_esd_check_wq, &nvt_esd_check_work,
			msecs_to_jiffies(NVT_TOUCH_ESD_CHECK_PERIOD));
#endif /* #if NVT_TOUCH_ESD_PROTECT */

	bTouchIsAwake = 1;

	mutex_unlock(&ts->lock);

	NVT_LOG("end\n");

	return;
}

static struct device_attribute *novatek_attrs[] = {
};

static struct tpd_driver_t nvt_device_driver =
{
	.tpd_device_name = NVT_I2C_NAME,
	.tpd_local_init = nvt_local_init,
	.suspend = nvt_ts_suspend,
	.resume = nvt_ts_resume,
	.attrs = {
		.attr = novatek_attrs,
		.num  = ARRAY_SIZE(novatek_attrs),
	},
};

/*******************************************************
Description:
	Driver Install function.

return:
	Executive Outcomes. 0---succeed. not 0---failed.
********************************************************/
static int32_t __init nvt_driver_init(void)
{
	int32_t ret = 0;

	NVT_LOG("start\n");
	tpd_get_dts_info();

	ret = tpd_driver_add(&nvt_device_driver);
	if (ret < 0){
		NVT_ERR("failed to add i2c driver");
		goto err_driver;
	}

	NVT_LOG("end\n");

err_driver:
	return ret;
}

/*******************************************************
Description:
	Driver uninstall function.

return:
	n.a.
********************************************************/
static void __exit nvt_driver_exit(void)
{
	tpd_driver_remove(&nvt_device_driver);

#if BOOT_UPDATE_FIRMWARE
	if (nvt_fwu_wq)
		destroy_workqueue(nvt_fwu_wq);
#endif

#if NVT_TOUCH_ESD_PROTECT
	if (nvt_esd_check_wq)
		destroy_workqueue(nvt_esd_check_wq);
#endif /* #if NVT_TOUCH_ESD_PROTECT */
}

module_init(nvt_driver_init);
module_exit(nvt_driver_exit);

MODULE_DESCRIPTION("Novatek Touchscreen Driver");
MODULE_LICENSE("GPL");
