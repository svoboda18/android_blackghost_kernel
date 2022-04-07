#include "tlsc6x_main.h"
#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#endif


#include "tpd.h"

#ifdef TPD_PROXIMITY
#include "hwmsensor.h"
#include "sensors_io.h"
#include "hwmsen_helper.h"
#include <alsps.h>
#endif

#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/of_irq.h>
#include <linux/pm_wakeup.h>
#ifdef TPD_PROXIMITY
#define APS_ERR(fmt,arg...)             printk("<<proximity>> "fmt,##arg)
#define TPD_PROXIMITY_DEBUG(fmt,arg...) printk("<<proximity>> "fmt,##arg)
#define TPD_PROXIMITY_DMESG(fmt,arg...) printk("<<proximity>> "fmt,##arg)

static u8 tpd_proximity_flag            = 0;
static u8 tpd_proximity_suspend         = 0;
static u8 tpd_proximity_detect      = 1;//0-->close ; 1--> far away
static u8 tpd_proximity_detect_prev= 0xff;//0-->close ; 1--> far away
#endif

#define	MULTI_PROTOCOL_TYPE_B	1
#if MULTI_PROTOCOL_TYPE_B
#include <linux/input/mt.h>
#endif
#define TPD_SUPPORT_POINTS  2
#define TLSC_TPD_NAME   "tlsc6x_touch"
//#define TPD_IIC_MASTER_PORT 1   //case by case

#define TP_HAVE_BUTTON

#define I2C_RETRY_NUMBER                    3

extern struct tpd_device *tpd;
static struct i2c_client *tlsc6x_i2c_client = NULL;
//struct wakeup_source tlsc6x_wakelock;

u8 *tpd_i2c_dma_va = NULL;
dma_addr_t tpd_i2c_dma_pa = 0;

#ifdef TLSC_APK_DEBUG
static unsigned char send_data_flag = 0;
static unsigned char get_data_flag = 0;
static int send_count = 0;
static char buf_in[1026] = {0};
static char buf_out[1026] = {0};
static unsigned char get_data_buf[10] = {0};

#endif

static DECLARE_WAIT_QUEUE_HEAD(waiter);

//extern void app_get_ctp_name(char *name);

static int tpd_probe(struct i2c_client *client, const struct i2c_device_id *id);
static int tpd_i2c_detect(struct i2c_client *client, struct i2c_board_info *info);
static int tpd_remove(struct i2c_client *client);
static void tpd_resume(struct device *h);
static void tpd_suspend(struct device *h);
static void tlsc6x_clear_report_data(void);


void get_data_start(unsigned char *local_buf);
void get_data_stop(void);


static unsigned int tpd_rst_gpio_number = 0;
static unsigned int tpd_int_gpio_number = 1;

int g_is_telink_comp = 0;
#define TS_NAME		"tlsc6x_ts"

struct tlsc6x_data *g_tp_drvdata = NULL;
static struct i2c_client *this_client;
#if 1
/******************blf zhangce_20170705 start**********************/
//create fwversion/project_id/ito_test fs
u32 _tlsc6x_create_sysfs(struct i2c_client *client);
// static u8 vendor_id;
//static u8 fw_version;
//static u8 project_id;
/******************blf zhangce_20170705 end**********************/
#endif

struct touch_info {
	int y[TPD_SUPPORT_POINTS];
	int x[TPD_SUPPORT_POINTS];
	int p[TPD_SUPPORT_POINTS];
	int id[TPD_SUPPORT_POINTS];
	int count;
};

/*dma declare, allocate and release*/
#define __MSG_DMA_MODE__

#ifdef __MSG_DMA_MODE__

static int msg_dma_alloct(void)
{
    int ret = 1;
    if(NULL == tpd_i2c_dma_va){
        tpd->dev->dev.coherent_dma_mask = DMA_BIT_MASK(32);
        tpd_i2c_dma_va = (u8 *)dma_alloc_coherent(&tpd->dev->dev, 1024, &tpd_i2c_dma_pa, GFP_KERNEL);
    }
    if(NULL == tpd_i2c_dma_va){
        ret = 0;
    }
    
    return ret;
}
static void msg_dma_release(void)
{
    if(tpd_i2c_dma_va){
        dma_free_coherent(NULL, 1024, tpd_i2c_dma_va, tpd_i2c_dma_pa);
        tpd_i2c_dma_va = NULL;
        tpd_i2c_dma_pa = 0;
    }
}
#endif

static DEFINE_MUTEX(i2c_access);
DEFINE_MUTEX(i2c_rw_access);

static const struct i2c_device_id tlsc6x_tpd_id[] = {{TLSC_TPD_NAME, 0}, {} };
static const struct of_device_id tlsc6x_dt_match[] = {
	{.compatible = "mediatek,cap_touch"},
	{},
};
MODULE_DEVICE_TABLE(of, tlsc6x_dt_match);

void tlsc6x_tpd_reset_force(void)
{
	//struct tlsc6x_platform_data *pdata = g_tp_drvdata->platform_data;

	TLSC_FUNC_ENTER();
	tpd_gpio_output(tpd_rst_gpio_number, 0);
	msleep(20);
	tpd_gpio_output(tpd_rst_gpio_number, 1);
	msleep(30);
}

static void tlsc6x_tpd_reset(void)
{
	TLSC_FUNC_ENTER();
	if (g_tp_drvdata->needKeepRamCode) {
		return;
	}

	tlsc6x_tpd_reset_force();
}

static int of_get_tlsc6x_platform_data(struct device *dev)
{
    if(dev->of_node){
        const struct of_device_id *match;

        match = of_match_device(of_match_ptr(tlsc6x_dt_match), dev);
        if(!match){
            TPD_DMESG("Error: No device match found\n");
            return -ENODEV;
        }
    }

    return 0;
}
#ifdef TLSC_GESTRUE
static int _is_open_gesture_mode = 0; 
static int tlsc6x_read_Gestruedata(void);
//static unsigned char gesture_enable = 0;
//static unsigned char gesture_state = 0;
#endif

#ifdef TLSC_GESTRUE
#define GESTURE_LEFT		0x20
#define GESTURE_RIGHT		0x21
#define GESTURE_UP		    0x22
#define GESTURE_DOWN		0x23
#define GESTURE_DOUBLECLICK	0x24
#define GESTURE_O		    0x30
#define GESTURE_W		    0x31
#define GESTURE_M		    0x32
#define GESTURE_E		    0x33
#define GESTURE_C		    0x34
#define GESTURE_S           0x46
#define GESTURE_V           0x54
#define GESTURE_Z           0x65
#define GESTURE_L           0x44

#endif

#ifdef TPD_PROXIMITY
static int tpd_get_ps_value(void)
{
    if(0 == tpd_proximity_detect){
        return 0;       
    }else if(1 == tpd_proximity_detect){
        return 10;
    }else{
        return tpd_proximity_detect;
    }
}

static int tpd_enable_ps(int enable)
{
    u8 state;
    tlsc_info("%s, tpd_enable_ps is %d.\n", __func__, enable);
    if(enable){
        state = 0x01;
        tpd_proximity_flag = 1;
		tpd_proximity_detect_prev = 0xff;
    }else{
        state = 0x00;
        tpd_proximity_flag = 0;
    }

    tlsc6x_write_reg(tlsc6x_i2c_client, 0xB0, state);

    return 0;
}
#if 0
static int tpd_ps_operate(void* self, uint32_t command, void* buff_in, int size_in,
    void* buff_out, int size_out, int* actualout)
{
	int err = 0;
	int value;

	struct hwm_sensor_data *sensor_data;
	TPD_DEBUG("[proxi_command = 0x%02X\n", command);
	switch (command)
	{
		case SENSOR_DELAY:
			if((buff_in == NULL) || (size_in < sizeof(int)))
			{
				APS_ERR("Set delay parameter error!\n");
				err = -EINVAL;
			}
			// Do nothing
			break;

		case SENSOR_ENABLE:
			if((buff_in == NULL) || (size_in < sizeof(int)))
			{
				APS_ERR("Enable sensor parameter error!\n");
				err = -EINVAL;
			}
			else
			{
				value = *(int *)buff_in;
				if(value)
				{
					if((tpd_enable_ps(1) != 0))
					{
						APS_ERR("enable ps fail: %d\n", err); 
						return -1;
					}
				}
				else
				{
					if((tpd_enable_ps(0) != 0))
					{
						APS_ERR("disable ps fail: %d\n", err);
						return -1;
					}
				}
			}
			break;

		case SENSOR_GET_DATA:
			if((buff_out == NULL) || (size_out< sizeof(struct hwm_sensor_data)))
			{
				APS_ERR("get sensor data parameter error!\n");
				err = -EINVAL;
			}
			else
			{

				sensor_data = (struct hwm_sensor_data *)buff_out;

				sensor_data->values[0] = tpd_get_ps_value();
				sensor_data->value_divide = 1;
				sensor_data->status = SENSOR_STATUS_ACCURACY_MEDIUM;
			}
			break;
		default:
			APS_ERR("proxmy sensor operate function no this parameter %d!\n", command);
			err = -1;
			break;
	}

	return err;
}
#endif
#endif

int tlsc6x_i2c_read_sub(struct i2c_client *client, char *writebuf, int writelen, char *readbuf, int readlen)
{
	int ret = 0;

	if (client == NULL) {
		tlsc_err("[IIC][%s]i2c_client==NULL!\n", __func__);
		return -EINVAL;
	}

	if (readlen > 0) {
		if (writelen > 0) {
			struct i2c_msg msgs[] = {
				{
				 .addr = client->addr,
				 .flags = 0,
				 .timing = 400,
				 .len = writelen,
				 .buf = writebuf,
				 },
				{
				 .addr = client->addr,
				 .flags = I2C_M_RD,
				 .timing = 400,
				 .len = readlen,
				 .buf = readbuf,
				 },
			};
			ret = i2c_transfer(client->adapter, msgs, 1);
			if (ret < 0) {
				tlsc_err("[IIC]: i2c_transfer(2) error, addr= 0x%x!!\n", writebuf[0]);
				tlsc_err("[IIC]: i2c_transfer(2) error, ret=%d, rlen=%d, wlen=%d!!\n", ret, readlen,
				       writelen);
			}else {
				ret = i2c_transfer(client->adapter, &msgs[1], 1);
				if (ret < 0) {
					tlsc_err("[IIC]: i2c_transfer(2) error, addr= 0x%x!!\n", writebuf[0]);
					tlsc_err("[IIC]: i2c_transfer(2) error, ret=%d, rlen=%d, wlen=%d!!\n", ret, readlen,
					       writelen);
				}
			}
		} else {
			struct i2c_msg msgs[] = {
				{
				 .addr = client->addr,
				 .flags = I2C_M_RD,
				 .timing = 400,
				 .len = readlen,
				 .buf = readbuf,
				 },
			};
			ret = i2c_transfer(client->adapter, msgs, 1);
			if (ret < 0) {
				tlsc_err("[IIC]: i2c_transfer(read) error, ret=%d, rlen=%d, wlen=%d!!", ret, readlen,
				       writelen);
			}
		}
	}

	return ret;
}

/* fail : <0 */
int tlsc6x_i2c_read(struct i2c_client *client, char *writebuf, int writelen, char *readbuf, int readlen)
{
	int ret = 0;
	int offset;
	u8 regBuf[2];
	int subLen;
	int retry;
	

	/* lock in this function so we can do direct mode iic transfer in debug fun */
	mutex_lock(&i2c_rw_access);
	offset = 0;
	while(readlen > 0){
	        if(2 == writelen) {
	        regBuf[0] = (u8)(writebuf[0]);
            regBuf[1] = (u8)(writebuf[1]+offset);
            }
            else if(1 == writelen)  {
                    regBuf[0] = (u8)(writebuf[0]+offset);
            }

	        if(readlen > MAX_TRX_LEN){
	            readlen -= MAX_TRX_LEN;
	            subLen = MAX_TRX_LEN;
	        }else{
	            subLen = readlen;
	            readlen = 0;
	        }

		retry = 0;
		while (tlsc6x_i2c_read_sub(client, regBuf, writelen, &readbuf[offset],subLen) < 0) {
			if (retry++ == 3) {
				ret = -1;
				break;
			}
		}
		offset += MAX_TRX_LEN;
		if (ret < 0) {
			break;
		}
	}

	//ret = tlsc6x_i2c_read_sub(client, writebuf, writelen, readbuf, readlen);

	mutex_unlock(&i2c_rw_access);

	return ret;
}

/* fail : <0 */
int tlsc6x_i2c_write_sub(struct i2c_client *client, char *writebuf, int writelen)
{
	int ret = 0;

	if (client == NULL) {
		tlsc_err("[IIC][%s]i2c_client==NULL!\n", __func__);
		return -EINVAL;
	}

	if (writelen > 0) {
		struct i2c_msg msgs[] = {
			{
			 .addr = client->addr,
			 .flags = 0,
			 .timing = 400,
			 .len = writelen,
			 .buf = writebuf,
			 },
		};
		ret = i2c_transfer(client->adapter, msgs, 1);
		if (ret < 0) {
			tlsc_err("[IIC]: i2c_transfer(write) error, ret=%d!!\n", ret);
		}
	}

	return ret;

}

/* fail : <0 */
int tlsc6x_i2c_write(struct i2c_client *client, char *writebuf, int writelen)
{
	int ret = 0;

	mutex_lock(&i2c_rw_access);
	ret = tlsc6x_i2c_write_sub(client, writebuf, writelen);
	mutex_unlock(&i2c_rw_access);

	return ret;

}

// fail : <0
int tlsc6x_write_reg(struct i2c_client *client, u8 regaddr, u8 regvalue)
{
	unsigned char buf[2] = {0};

	buf[0] = regaddr;
	buf[1] = regvalue;

	return tlsc6x_i2c_write(client, buf, sizeof(buf));
}
// fail : <0
int tlsc6x_read_reg(struct i2c_client *client, u8 regaddr, u8 *regvalue)
{
	return tlsc6x_i2c_read(client, &regaddr, 1, regvalue, 1);
}

//#if (defined TPD_AUTO_UPGRADE_PATH) || (defined TLSC_APK_DEBUG)
extern int tlsx6x_update_running_cfg(u16* ptcfg);
extern int tlsx6x_update_burn_cfg(u16* ptcfg);
extern int tlsc6x_load_ext_binlib(u8* pcode, u16 len);
extern int tlsc6x_update_f_combboot(u8* pdata, u16 len);

int auto_upd_busy = 0;
//0:sucess
//1: no file OR open fail
//2: wrong file size OR read error
//-1:op-fial
int tlsc6x_proc_cfg_update(u8 *dir, int behave)
{
	int ret = 1;
	u8 *pbt_buf = NULL;
	u32 fileSize;
	mm_segment_t old_fs;
	static struct file *file = NULL;

	TLSC_FUNC_ENTER();
	tlsc_info("tlsc6x proc-file:%s\n", dir);

	file = filp_open(dir, O_RDONLY, 0);
	if (IS_ERR(file)) {
		tlsc_err("tlsc6x proc-file:open error!\n");
	} else {
		ret = 2;
		old_fs = get_fs();
		set_fs(KERNEL_DS);
		fileSize = file->f_op->llseek(file, 0, SEEK_END);
		tlsc_info("tlsc6x proc-file, size:%d\n", fileSize);
		pbt_buf = kmalloc(fileSize, GFP_KERNEL);

		file->f_op->llseek(file, 0, SEEK_SET);
		if (fileSize == file->f_op->read(file, (char *)pbt_buf, fileSize, &file->f_pos)) {
			tlsc_info("tlsc6x proc-file, read ok1!\n");
			ret = 3;
		}

		if (ret == 3) {
			auto_upd_busy = 1;
			disable_irq(this_client->irq);
			msleep(1000);
			//wake_lock_timeout(&tlsc6x_wakelock, msecs_to_jiffies(2000));
			//__pm_wakeup_event(&tlsc6x_wakelock, msecs_to_jiffies(2000));
			if (behave == 0) {
				if (fileSize == 204) {
					ret = tlsx6x_update_running_cfg((u16 *) pbt_buf);
				} else if (fileSize > 0x400) {
					tlsc6x_load_ext_binlib((u8 *) pbt_buf, (u16) fileSize);
				}
			} else if (behave == 1) {
				if (fileSize == 204) {
					ret = tlsx6x_update_burn_cfg((u16 *) pbt_buf);
				} else if (fileSize > 0x400) {
					ret = tlsc6x_update_f_combboot((u8 *) pbt_buf, (u16) fileSize);
				}
				tlsc6x_tpd_reset();
			}
			enable_irq(this_client->irq);
			auto_upd_busy = 0;
		}

		filp_close(file, NULL);
		set_fs(old_fs);

		kfree(pbt_buf);
	}

	return ret;
}

//#endif

//#ifdef TPD_AUTO_UPGRADE_PATH
static int __maybe_unused tlsc6x_update_path_proc(void *unused)
{
    do{
        msleep(5000);
        if(1 != tlsc6x_proc_cfg_update("sdcard/tlsc6x_auto_cfg_upd.bin",1)){
            break;
        }
    }while(!kthread_should_stop());

    return 0;
}
//#endif

#ifdef TLSC_APK_DEBUG
unsigned char proc_out_len;
unsigned char proc_out_buf[256];

unsigned char debug_type;
unsigned char iic_reg[2];
unsigned char sync_flag_addr[3];
unsigned char sync_buf_addr[2];
unsigned char reg_len;

static struct proc_dir_entry *tlsc6x_proc_entry = NULL;
extern int tlsc6x_read_bytes_u16addr_sub(struct i2c_client *client, u16 addr, u8 *rxbuf, u16 len);
extern int tlsc6x_write_bytes_u16addr_sub(struct i2c_client *client, u16 addr, u8 *txbuf, u16 len);
static int debug_read(char *writebuf, int writelen, char *readbuf, int readlen)
{
	int ret = 0;
	u16 reg;

	mutex_lock(&i2c_rw_access);
	tlsc6x_set_dd_mode_sub();
	reg = writebuf[0];
	reg = (reg<<8)+writebuf[1];
	ret = tlsc6x_read_bytes_u16addr_sub(this_client, reg, readbuf, readlen);
	//ret = tlsc6x_i2c_read_sub(this_client, writebuf, writelen, readbuf, readlen);

	tlsc6x_set_nor_mode_sub();
	mutex_unlock(&i2c_rw_access);
	if (ret >= 0) {
		ret = readlen;
	}
	return ret;
}

static int debug_write(char *writebuf, int writelen)
{
	int ret = 0;
	u16 reg;

	mutex_lock(&i2c_rw_access);
	tlsc6x_set_dd_mode_sub();
	reg = writebuf[0];
	reg = (reg<<8)+writebuf[1];
	ret = tlsc6x_write_bytes_u16addr_sub(this_client, reg, &writebuf[2], writelen-2);
	//ret = tlsc6x_i2c_write_sub(this_client, writebuf, writelen);

	tlsc6x_set_nor_mode_sub();
	mutex_unlock(&i2c_rw_access);
	if (ret >= 0) {
		ret = writelen;
	}
	return ret;
}

static int debug_read_sync(char *writebuf, int writelen, char *readbuf, int readlen)
{
	int ret = 0;
	int retryTime;
	u16 reg;

	mutex_lock(&i2c_rw_access);
	tlsc6x_set_dd_mode_sub();
	sync_flag_addr[2] = 1;
	ret = tlsc6x_i2c_write_sub(this_client, sync_flag_addr, 3);

	retryTime = 100;
	do {
		reg = sync_flag_addr[0];
		reg = (reg<<8)+sync_flag_addr[1];
		ret = tlsc6x_read_bytes_u16addr_sub(this_client, reg, &sync_flag_addr[2], 1);
		//ret = tlsc6x_i2c_read_sub(this_client, sync_flag_addr, 2, &sync_flag_addr[2], 1);
		if (ret < 0) {
			mutex_unlock(&i2c_rw_access);
			return ret;
		}
		retryTime--;
	} while (retryTime>0&&sync_flag_addr[2] == 1);
	if(retryTime==0&&sync_flag_addr[2] == 1) {
		mutex_unlock(&i2c_rw_access);
		return -EFAULT;
	}
	if (ret >= 0) {
		/* read data */
		reg=sync_buf_addr[0];
		reg = (reg<<8)+sync_buf_addr[1];
		ret = tlsc6x_read_bytes_u16addr_sub(this_client, reg, readbuf, readlen);
		//ret = tlsc6x_i2c_read_sub(this_client, sync_buf_addr, 2, readbuf, readlen);
	}

	tlsc6x_set_nor_mode_sub();
	mutex_unlock(&i2c_rw_access);
	if (ret >= 0) {
		ret = readlen;
	}
	return ret;
}

extern int tlsc6x_load_ext_binlib(u8 *pdata, u16 len);
static int tlsc6x_rawdata_test_3535allch(u8 * buf,int len)
{
	int ret;
	int retryTime;
	u8 writebuf[4];
	buf[len] = '\0';
	ret=0;
	disable_irq(this_client->irq);
	g_tp_drvdata->esdHelperFreeze=1;
	tlsc6x_tpd_reset();
	if (tlsc6x_load_ext_binlib((u8 *) &buf[2], len-2)){	
	ret = -EIO;
	}
	msleep(30);

	mutex_lock(&i2c_rw_access);
	//write addr
	writebuf[0]= 0x9F;
	writebuf[1]= 0x20;
	writebuf[2]= 48;
	writebuf[3]= 0xFF;
	ret = tlsc6x_i2c_write_sub(this_client, writebuf, 4);
	writebuf[0]= 0x9F;
	writebuf[1]= 0x24;
	writebuf[2]= 1;
	
	ret = tlsc6x_i2c_write_sub(this_client, writebuf, 3);
	retryTime = 100;
	do {
		ret = tlsc6x_i2c_read_sub(this_client, writebuf, 2, &writebuf[2], 1);
		if (ret < 0) {
			break;
		}
		retryTime--;
		msleep(30);
	} while (retryTime>0&&writebuf[2] == 1);

	if (ret>=0) {
		writebuf[0]= 0x9F;
		writebuf[1]= 0x26;
		ret = tlsc6x_i2c_read_sub(this_client, writebuf, 2, proc_out_buf, 96);
		if (ret>=0){
			proc_out_len=96;
		}
	}

	mutex_unlock(&i2c_rw_access);

	tlsc6x_tpd_reset();
	
	g_tp_drvdata->esdHelperFreeze=0;
        enable_irq(this_client->irq);
	return ret;
}
static ssize_t tlsc6x_proc_write(struct file *filp, const char __user *buff, size_t len, loff_t *data)
{
	int ret;
	int buflen = len;
	unsigned char *local_buf;
	if (buflen > 4100) {
		return -EFAULT;
	}
	local_buf = kmalloc(buflen+1, GFP_KERNEL);
	if(local_buf == NULL) {
		tlsc_err("%s,Can not malloc the buf!\n", __func__);
		return -EFAULT;
	}

	if (copy_from_user(local_buf, buff, buflen)) {
		tlsc_err("%s:copy from user error\n", __func__);
		return -EFAULT;
	}
	ret = 0;
	debug_type = local_buf[0];
	/* format:cmd+para+data0+data1+data2... */
	switch (local_buf[0]) {
	case 0:		/* cfg version */
		proc_out_len = 4;
		proc_out_buf[0] = g_tlsc6x_cfg_ver;
		proc_out_buf[1] = g_tlsc6x_cfg_ver >> 8;
		proc_out_buf[2] = g_tlsc6x_cfg_ver >> 16;
		proc_out_buf[3] = g_tlsc6x_cfg_ver >> 24;
		break;
	case 1:
		local_buf[buflen] = '\0';
		if (tlsc6x_proc_cfg_update(&local_buf[2], 0)) {
			len = -EIO;
		}
		break;
	case 2:
		local_buf[buflen] = '\0';
		if (tlsc6x_proc_cfg_update(&local_buf[2], 1)) {
			len = -EIO;
		}
		break;
	case 3:
		ret = debug_write(&local_buf[1], len - 1);
		break;
	case 4:		/* read */
		reg_len = local_buf[1];
		iic_reg[0] = local_buf[2];
		iic_reg[1] = local_buf[3];
		break;
	case 5:		/* read with sync */
		ret = debug_write(&local_buf[1], 4);	/* write size */
		if (ret >= 0) {
			ret = debug_write(&local_buf[5], 4);	/* write addr */
		}
		sync_flag_addr[0] = local_buf[9];
		sync_flag_addr[1] = local_buf[10];
		sync_buf_addr[0] = local_buf[11];
		sync_buf_addr[1] = local_buf[12];
		break;
	case 8: // Force reset ic
		tlsc6x_tpd_reset_force();
		break;
        case 9: // Force reset ic
		ret=tlsc6x_rawdata_test_3535allch(local_buf,buflen);
		break;
	case 14:	/* e, esd control */
		g_tp_drvdata->esdHelperFreeze = (int)local_buf[1];
		break;
        case 15:	
		memset(get_data_buf, 0x00, (sizeof(char) * 8));
             memcpy(get_data_buf, local_buf, (sizeof(char) * 8));
             get_data_start(get_data_buf);

             send_data_flag = 1;
             send_count = 0;
             get_data_flag = 0;
		break;
        case 16:	
             get_data_stop();
		break;
		
	default:
		break;
	}
	if (ret < 0) {
		len = ret;
	}
	kfree(local_buf);
	return len;
}

static ssize_t tlsc6x_proc_read(struct file *filp, char __user *page, size_t len, loff_t *pos)
{
	int ret = 0;

	if (*pos!=0) {
		return 0;
	}

	switch (debug_type) {
	case 0:		/* version information */
		proc_out_len = 4;
		proc_out_buf[0] = g_tlsc6x_cfg_ver;
		proc_out_buf[1] = g_tlsc6x_cfg_ver >> 8;
		proc_out_buf[2] = g_tlsc6x_cfg_ver >> 16;
		proc_out_buf[3] = g_tlsc6x_cfg_ver >> 24;
		if (copy_to_user(page, proc_out_buf, proc_out_len)) {
			ret = -EFAULT;
		} else {
			ret = proc_out_len;
		}
		break;
	case 1:
		break;
	case 2:
		break;
	case 3:
		break;
	case 4:
		len = debug_read(iic_reg, reg_len, proc_out_buf, len);
		if (len > 0) {
			if (copy_to_user(page, proc_out_buf, len)) {
				ret = -EFAULT;
			} else {
				ret = len;
			}
		} else {
			ret = len;
		}
		break;
	case 5:
		len = debug_read_sync(iic_reg, reg_len, proc_out_buf, len);
		if (len > 0) {
			if (copy_to_user(page, proc_out_buf, len)) {
				ret = -EFAULT;
			} else {
				ret = len;
			}
		} else {
			ret = len;
		}
		break;
        case 9:
		if (proc_out_buf>0){
			if (copy_to_user(page, proc_out_buf, len)) {
				ret = -EFAULT;
			} else {
				ret = proc_out_len;
			}
		}
		break;
        case 15:
		if (1 == get_data_flag){
                        get_data_flag = 0;
			if (copy_to_user(page, buf_out, len)) {
				ret = -EFAULT;
			} else {
				ret = len;
			}
		}
                else{
                    ret = -EFAULT;
                }
		break;
	default:
		break;
	}

	if(ret>0) {
		*pos +=ret;
	}

	return ret;
}

static struct file_operations tlsc6x_proc_ops = {
	.owner = THIS_MODULE,
	.read = tlsc6x_proc_read,
	.write = tlsc6x_proc_write,
};

void tlsc6x_release_apk_debug_channel(void)
{
	if (tlsc6x_proc_entry) {
		remove_proc_entry("tlsc6x-debug", NULL);
	}
}

int tlsc6x_create_apk_debug_channel(struct i2c_client *client)
{
	tlsc6x_proc_entry = proc_create("tlsc6x-debug", 0666, NULL, &tlsc6x_proc_ops);

	if (tlsc6x_proc_entry == NULL) {
		dev_err(&client->dev, "Couldn't create proc entry!\n");
		return -ENOMEM;
	}
	dev_info(&client->dev, "Create proc entry success!\n");

	return 0;
}
#endif

extern unsigned int g_mccode;
static ssize_t show_tlsc_version(struct device *dev,
	struct device_attribute *attr,
	char *buf)
{
	u8 reg[2];
	u8 readBuf[4];
	char *ptr = buf;
	u8 vender_id;
	mutex_lock(&i2c_rw_access);
	tlsc6x_set_dd_mode_sub();

	reg[0] = 0x80;
	reg[1] = 0x04;
	tlsc6x_i2c_read_sub(this_client, reg, 2, readBuf, 2);
	ptr += sprintf(ptr,"The boot version is %04X.\n",(readBuf[0]+(readBuf[1]<<8)));

	if (g_mccode == 0) {
		reg[0] = 0xD6;
		reg[1] = 0xE0;
	} else {
		reg[0] = 0x9E;
		reg[1] = 0x00;
	}
	tlsc6x_i2c_read_sub(this_client, reg, 2, readBuf, 4);
	ptr += sprintf(ptr,"The config version is %d.\n", readBuf[3]>>2);

	vender_id = readBuf[1]>>1;
	ptr += sprintf(ptr,"The vender id is %d, the vender name is ", vender_id);

	switch (vender_id) {
	case 1:
		ptr += sprintf(ptr,"xufang");
		break;
	case 2:
		ptr += sprintf(ptr,"xuri");
		break;
	case 3:
		ptr += sprintf(ptr,"yuye");
		break;
	case 4:
		ptr += sprintf(ptr,"tianyi");
		break;
	case 5:
		ptr += sprintf(ptr,"minglang");
		break;
	case 6:
		ptr += sprintf(ptr,"duoxinda");
		break;
	case 7:
		ptr += sprintf(ptr,"zhenhua");
		break;
	case 8:
		ptr += sprintf(ptr,"jitegao");
		break;
	case 9:
		ptr += sprintf(ptr,"guangjishengtai");
		break;
	case 10:
		ptr += sprintf(ptr,"shengguang");
		break;
	case 11:
		ptr += sprintf(ptr,"xintiantong");
		break;
	case 12:
		ptr += sprintf(ptr,"xinyou");
		break;
	case 13:
		ptr += sprintf(ptr,"yanqi");
		break;
	case 14:
		ptr += sprintf(ptr,"zhongcheng");
		break;
	case 15:
		ptr += sprintf(ptr,"xinmaoxin");
		break;
	case 16:
		ptr += sprintf(ptr,"zhenzhixin");
		break;
	case 17:
		ptr += sprintf(ptr,"helitai");
		break;
	case 18:
		ptr += sprintf(ptr,"huaxin");
		break;
	case 19:
		ptr += sprintf(ptr,"lihaojie");
		break;
	case 20:
		ptr += sprintf(ptr,"jiandong");
		break;
	case 21:
		ptr += sprintf(ptr,"xinpengda");
		break;
	case 22:
		ptr += sprintf(ptr,"jiake");
		break;
	case 23:
		ptr += sprintf(ptr,"yijian");
		break;
	case 24:
		ptr += sprintf(ptr,"yixing");
		break;
	case 25:
		ptr += sprintf(ptr,"zhongguangdian");
		break;
	case 26:
		ptr += sprintf(ptr,"hongzhan");
		break;
	case 27:
		ptr += sprintf(ptr,"huaxingda");
		break;
	case 28:
		ptr += sprintf(ptr,"dongjianhuanyu");
		break;
	case 29:
		ptr += sprintf(ptr,"dawosi");
		break;
	case 30:
		ptr += sprintf(ptr,"dacheng");
		break;
	case 31:
		ptr += sprintf(ptr,"mingwangda");
		break;
	case 32:
		ptr += sprintf(ptr,"huangze");
		break;
	case 33:
		ptr += sprintf(ptr,"jinxinxiang");
		break;
	case 34:
		ptr += sprintf(ptr,"gaoge");
		break;
	case 35:
		ptr += sprintf(ptr,"zhihui");
		break;
	case 36:
		ptr += sprintf(ptr,"miaochu");
		break;
	case 37:
		ptr += sprintf(ptr,"qicai");
		break;
	case 38:
		ptr += sprintf(ptr,"zhenghai");
		break;
	case 39:
		ptr += sprintf(ptr,"hongfazhan");
		break;
	case 40:
		ptr += sprintf(ptr,"lianchuang");
		break;
	case 41:
		ptr += sprintf(ptr,"saihua");
		break;
	case 42:
		ptr += sprintf(ptr,"keleli");
		break;
	
	case 43:
		ptr += sprintf(ptr,"weiyi");
		break;
	case 44:
		ptr += sprintf(ptr,"futuo");
		break;
	default:
		ptr += sprintf(ptr,"unknown");
		break;
	}
	ptr += sprintf(ptr,".\n");

	tlsc6x_set_nor_mode_sub();
	mutex_unlock(&i2c_rw_access);


	return (ptr-buf);
}

//extern int g_ctp_fwvr_info;
//extern int g_ctp_vendor_info;
static ssize_t read_version_info(void)
{
	u8 reg[2];
	u8 readBuf[4];
	//char *ptr = buf;
	u8 vender_id;
	mutex_lock(&i2c_rw_access);
	tlsc6x_set_dd_mode_sub();
	reg[0] = 0x80;
	reg[1] = 0x04;
	tlsc6x_i2c_read_sub(this_client, reg, 2, readBuf, 2);
	//ptr += sprintf(ptr,"The boot version is %04X.\n",(readBuf[0]+(readBuf[1]<<8)));
	if (g_mccode == 0) {
		reg[0] = 0xD6;
		reg[1] = 0xE0;
	} else {
		reg[0] = 0x9E;
		reg[1] = 0x00;
	}
	tlsc6x_i2c_read_sub(this_client, reg, 2, readBuf, 4);
	//g_ctp_fwvr_info = readBuf[3]>>2;
	//ptr += sprintf(ptr,"The config version is %d.\n", readBuf[3]>>2);
	vender_id = readBuf[1]>>1;
	//g_ctp_vendor_info = vender_id;
	//ptr += sprintf(ptr,"The vender id is %d, the vender name is ", vender_id);
	tlsc6x_set_nor_mode_sub();
	mutex_unlock(&i2c_rw_access);
	return 1;
}


static ssize_t store_tlsc_version(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	
	return -EPERM;
}

static ssize_t show_tlsc_info(struct device *dev,
	struct device_attribute *attr,
	char *buf)
{
	char *ptr = buf;
	ptr += sprintf(ptr,"Max finger number is %0d.\n",TPD_SUPPORT_POINTS);
	ptr += sprintf(ptr,"Int irq is %d.\n",this_client->irq);
	ptr += sprintf(ptr,"I2c address is 0x%02X(0x%02X).\n",this_client->addr,(this_client->addr)<<1);

	return (ptr-buf);
}

static ssize_t store_tlsc_info(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	
	return -EPERM;
}

static ssize_t show_tlsc_ps(struct device *dev,
	struct device_attribute *attr,
	char *buf)
{
	char *ptr = buf;
#ifdef TPD_PROXIMITY

	ptr += sprintf(ptr,"%d\n",tpd_proximity_flag);
#else
	ptr += sprintf(ptr,"No proximity function.\n");
#endif

	return (ptr-buf);
}

static ssize_t store_tlsc_ps(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
#ifdef TPD_PROXIMITY
	if (buf[0] == '0') {
		tpd_enable_ps(0);
		//ps off
		
	} else if (buf[0] == '1') {
		//ps on
		tpd_enable_ps(1);
	}
#endif
	return count;
}

static ssize_t show_tlsc_gesture(struct device *dev,
	struct device_attribute *attr,
	char *buf)
{
	char *ptr = buf;
#ifdef TLSC_GESTRUE

	ptr += sprintf(ptr,"%d\n",_is_open_gesture_mode);
#else
	ptr += sprintf(ptr,"No gesture function.\n");
#endif

	return (ptr-buf);
}

static ssize_t store_tlsc_gesture(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
#ifdef TLSC_GESTRUE
	if (buf[0] == '0') {
		//gesture off
		_is_open_gesture_mode = 0;
		
	} else if (buf[0] == '1') {
		//gesture on
		_is_open_gesture_mode = 1;
	}
#endif
	return count;
}

//static int debugResult=0;
u8 readFlashbuf[204];

u8 cfgStatic[] = {
0x06,0x3A,0x0E,0x00,0x02,0x22,0x32,0x53,0x37,0x03,0x02,0x1F,0x75,0xA1,0x00,0x00,
0x00,0x00,0x55,0x55,0x00,0x00,0x00,0x00,0x00,0x00,0x0F,0x06,0x10,0x07,0x11,0x08,
0x00,0x00,0x12,0x09,0x1B,0x24,0x1A,0x23,0x19,0x22,0x18,0x21,0x17,0x20,0x00,0x00,
0x16,0x1F,0x15,0x1E,0x14,0x1D,0x13,0x1C,0x0A,0x01,0x0B,0x02,0x00,0x00,0x0C,0x03,
0x0D,0x04,0x0E,0x05,0x00,0x00,0x48,0x92,0x6E,0x02,0x45,0x00,0xE0,0x01,0xC0,0x03,
0x50,0x00,0xF0,0x00,0x90,0x01,0x00,0x02,0x84,0x03,0x10,0x10,0x00,0x00,0x00,0x00,
0x64,0x40,0x08,0x84,0x06,0x1E,0x1E,0x1E,0x1E,0x1E,0x5C,0x06,0x97,0x08,0x15,0x15,
0x23,0x23,0x67,0xAD,0x1A,0x26,0x01,0x31,0x88,0x3E,0x85,0xAF,0x9A,0x64,0x91,0x01,
0xC8,0x68,0x00,0x00,0x25,0x11,0x28,0x3A,0x4B,0x32,0x1A,0x32,0xA0,0x28,0xFF,0x6E,
0x5A,0x0D,0x82,0xA2,0xB5,0x06,0x69,0x09,0x36,0x06,0x6C,0x08,0x36,0x06,0x6C,0x08,
0xB5,0x06,0x69,0x09,0x02,0x02,0x39,0x51,0x39,0x51,0x00,0x00,0x00,0x00,0xCD,0x03,
0x0E,0x40,0x08,0x08,0x28,0x24,0x32,0x03,0x51,0x28,0x14,0x14,0x14,0x14,0x14,0x14,
0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0xBF,0xF9
};

static ssize_t show_tlsc_debug_flash(struct device *dev,
	struct device_attribute *attr,
	char *buf)
{
	char *ptr = buf;
	int i;

	for(i=0;i<204;i++) {
		ptr += sprintf(ptr,"%d,",readFlashbuf[i]);	
	}

	ptr += sprintf(ptr,"\n");

	return (ptr-buf);
}

extern int tlsc6x_download_ramcode(u8 *pcode, u16 len);
extern int tlsc6x_write_burn_space(u8 *psrc, u16 adr, u16 len);
extern int tlsc6x_read_burn_space(u8 *pdes, u16 adr, u16 len);
extern int tlsc6x_set_nor_mode(void);
extern unsigned char fw_fcode_burn[2024];
int writeFlash(u8* buf ,u16 addr,int len)
{
	//int ret=0;
	auto_upd_busy = 1;
	disable_irq(this_client->irq);
	if (tlsc6x_download_ramcode(fw_fcode_burn, sizeof(fw_fcode_burn))) {
		tlsc_err("Tlsc6x:write flash error:ram-code error!\n");
		return -EPERM;
	}

	if (tlsc6x_write_burn_space((unsigned char *)buf, addr, len)) {
		tlsc_err("Tlsc6x:write flash  fail!\n");
		return -EPERM;
	}
	tlsc6x_set_nor_mode();

	tlsc6x_tpd_reset();

	auto_upd_busy=0;
	enable_irq(this_client->irq);
	return 0;
}

int readFlash(u16 addr,int len)
{
	//int ret=0;
	auto_upd_busy = 1;
	disable_irq(this_client->irq);
	if (tlsc6x_download_ramcode(fw_fcode_burn, sizeof(fw_fcode_burn))) {
		tlsc_err("Tlsc6x:write flash error:ram-code error!\n");
		return -EPERM;
	}

	if (tlsc6x_read_burn_space((unsigned char *)readFlashbuf, addr, len)) {
		tlsc_err("Tlsc6x:write flash  fail!\n");
		return -EPERM;
	}
	tlsc6x_set_nor_mode();

	tlsc6x_tpd_reset();

	auto_upd_busy=0;
	enable_irq(this_client->irq);
	return 0;
}

static ssize_t store_tlsc_debug_flash(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	u8 wBuf[16]={0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
	if (buf[0] == '0') {
		//gesture off
		
		
	} else if (buf[0] == '1') {
		//gesture on
		writeFlash(cfgStatic ,0xF000,204);
	} else if (buf[0] == '2') {
		//gesture on
		writeFlash(wBuf ,0,16);
	} else if (buf[0] == '3') {
		//gesture on
		writeFlash(wBuf ,0x8000,16);
	} else if (buf[0] == '4') {
		//gesture on
		readFlash(0xF000,204);
	} else if (buf[0] == '5') {
		//gesture on
		readFlash(0,204);
	} else if (buf[0] == '6') {
		//gesture on
		readFlash(0x8000,204);
	}

	return count;
}

static DEVICE_ATTR(tlsc_version, 0664, show_tlsc_version, store_tlsc_version);
static DEVICE_ATTR(tlsc_tp_info, 0664, show_tlsc_info, store_tlsc_info);
static DEVICE_ATTR(tlsc_ps_ctl, 0664, show_tlsc_ps, store_tlsc_ps);
static DEVICE_ATTR(tlsc_gs_ctl, 0664, show_tlsc_gesture, store_tlsc_gesture);
static DEVICE_ATTR(tlsc_flash_ctl, 0664, show_tlsc_debug_flash, store_tlsc_debug_flash);
static struct attribute *tlsc_attrs[] = {
	&dev_attr_tlsc_version.attr,
	&dev_attr_tlsc_tp_info.attr,
	&dev_attr_tlsc_ps_ctl.attr,
	&dev_attr_tlsc_gs_ctl.attr,
	&dev_attr_tlsc_flash_ctl.attr,
	NULL, // Can not delete this line!!! The phone will reset.
};

static struct attribute_group tlsc_attr_group = {
	.attrs = tlsc_attrs,
};


#ifdef TLSC_ESD_HELPER_EN
static int tpd_esd_flag = 0;
static struct hrtimer tpd_esd_kthread_timer;
static DECLARE_WAIT_QUEUE_HEAD(tpd_esd_waiter);

static int tlsc6x_power_on(void)
{
    int err = 0;
    
#ifdef CONFIG_TPD_POWER_SOURCE_VIA_EXT_LDO
    tpd_ldo_power_enable(1);
#else
    int retry=0;
    u8 test_val;
    while(retry++ < 30){
        err = regulator_enable(tpd->reg);
        if(regulator_is_enabled(tpd->reg)){
            break;
        }
    }
    tlsc6x_read_reg(tlsc6x_i2c_client, retry,&test_val);
#endif

    return err;
}
static int tlsc6x_power_off(void)
{
    int err = 0;

#ifdef CONFIG_TPD_POWER_SOURCE_VIA_EXT_LDO
    tpd_ldo_power_enable(0);
#else
    int retry=0;
    u8 test_val;
    
    while(retry++ < 30){
        err = regulator_disable(tpd->reg);
        if(0==regulator_is_enabled(tpd->reg)){
            break;
        }
    }
    tlsc6x_read_reg(tlsc6x_i2c_client, retry,&test_val);
#endif
    
    return err;
}

static int esd_check_work(void)
{
    int ret = -1;
    u8 test_val;

    ret = tlsc6x_read_reg(tlsc6x_i2c_client, 0x0,&test_val);

    if(ret < 0){    //maybe confused by some noise,so retry is make sense.
        msleep(60);
        tlsc6x_read_reg(tlsc6x_i2c_client, 0x0,&test_val);
        ret = tlsc6x_read_reg(tlsc6x_i2c_client, 0x0,&test_val);
        if(ret < 0){
            // re-power-on
            tlsc6x_power_off();
            msleep(20);
            tlsc6x_power_on();
            tlsc6x_tpd_reset();
        }
    }

    return ret;
}
static int esd_checker_handler(void *unused)
{
	ktime_t ktime;

	do {
		wait_event_interruptible(tpd_esd_waiter, tpd_esd_flag != 0);
		tpd_esd_flag = 0;

		ktime = ktime_set(4, 0);
		hrtimer_start(&tpd_esd_kthread_timer, ktime, HRTIMER_MODE_REL);

		if (g_tp_drvdata->esdHelperFreeze) {
			continue;
		}
#if (defined TPD_AUTO_UPGRADE_PATH) || (defined TLSC_APK_DEBUG)
		if (auto_upd_busy) {
			continue;
		}
#endif
		esd_check_work();

	} while (!kthread_should_stop());

	return 0;
}
	
enum hrtimer_restart tpd_esd_kthread_hrtimer_func(struct hrtimer *timer)
{
    tpd_esd_flag = 1;
    wake_up_interruptible(&tpd_esd_waiter);

    return HRTIMER_NORESTART;
}
#endif

#ifdef TPD_PROXIMITY

static int ps_open_report_data(int open)
{
    /* should queuq work to report event if  is_report_input_direct=true */
    return 0;
}

/* if use  this typ of enable , Gsensor only enabled but not report inputEvent to HAL*/
static int ps_enable_nodata(int en)
{
    tpd_enable_ps(en);

    return 0;
}

static int ps_set_delay(u64 ns)
{
    return 0;
}

static int ps_get_data(int *value, int *status)
{

    *value = tpd_get_ps_value();
    *status = SENSOR_STATUS_ACCURACY_MEDIUM;

    return 0;
}

static int ps_batch(int flag, int64_t samplingPeriodNs, int64_t maxBatchReportLatencyNs)
{
    int value = 0;

    value = (int)samplingPeriodNs / 1000 / 1000;
    /*FIX  ME */

    return 0;
}

static int ps_flush(void)
{
	return ps_flush_report();
}
#endif

static void tlsc6x_clear_report_data(void)
{
	int i;

	for (i = 0; i < TPD_SUPPORT_POINTS; i++) {
#if MULTI_PROTOCOL_TYPE_B
		input_mt_slot(tpd->dev, i);
		input_mt_report_slot_state(tpd->dev, MT_TOOL_FINGER, false);
#endif
	}

	input_report_key(tpd->dev, BTN_TOUCH, 0);
#if !MULTI_PROTOCOL_TYPE_B
	input_mt_sync(tpd->dev);
#endif
	input_sync(tpd->dev);
}

#ifdef TLSC_APK_DEBUG

void get_data_start(unsigned char *local_buf)
{
    u8 writebuf[4];
    
    g_tp_drvdata->esdHelperFreeze = 1;
    
    tlsc6x_set_dd_mode();
    {
        writebuf[0] = 0x9f; 
        writebuf[1] = 0x22; 
        writebuf[2] = local_buf[2]; 
        writebuf[3] = local_buf[3]; 
        tlsc6x_i2c_write(this_client, writebuf, 4);

        writebuf[0] = 0x9f; 
        writebuf[1] = 0x20; 
        writebuf[2] = 61; 
        tlsc6x_i2c_write(this_client, writebuf, 3);

        writebuf[0] = 0x9f; 
        writebuf[1] = 0x24; 
        writebuf[2] = 0x01; 
        tlsc6x_i2c_write(this_client, writebuf, 3);
    }
}

void get_data_stop(void)
{
    u8 writebuf[4];
    
    writebuf[0] = 0x9f; 
    writebuf[1] = 0x22; 
    writebuf[2] = 0xff; 
    writebuf[3] = 0xff; 
    tlsc6x_i2c_write(this_client, writebuf, 4);		
    msleep(20);
    g_tp_drvdata->esdHelperFreeze = 0;
    send_data_flag = 0;
    tlsc6x_set_nor_mode();
}

int tssc_get_debug_info(struct i2c_client *i2c_client, char *p_data)
{
	char writebuf[10] = {0};
	short size = 61;
	static unsigned int cnt;
	unsigned char loop, k;
	unsigned char cmd[2];
	unsigned short check, rel_size;
	char buft[128];
	unsigned short *p16_buf = (unsigned short *)buft;
	cmd[0] = 1;

	loop = 0;
	rel_size = size * 2;
	
	while (loop++ < 2) {
		writebuf[0] = 0x9f; //0xc2; //pt_dbg_mtk
		writebuf[1] = 0x26; //0x80; //0x80c280
		tlsc6x_i2c_read(i2c_client, writebuf,  2, buft, rel_size + 2);  //124 2->checksum
		for (k = 0, check = 0; k < size; k++) {
			check += p16_buf[k];
		}
		if (check == p16_buf[size]) {
			p16_buf[size] = 0x5555;
			break;
		} else {
			p16_buf[size] = 0xaaaa;
		}
	}
	//*((unsigned int)(&buft[124])) = cnt;
	buft[124] = (cnt) & 0xff;
	buft[125] = (cnt >> 8) & 0xff;
	buft[126] = (cnt >> 16) & 0xff;
	buft[127] = (cnt >> 24) & 0xff;
	cnt++;
        
        memcpy(&buf_in[send_count * 128], buft, (sizeof(char) * 128));
        if(send_count++ >= 7){
            //tlsc6x_irq_disable();
            memcpy(buf_out, buf_in, (sizeof(char) * 1024));
            memset(buf_in, 0xff,(sizeof(char) * 1024));
            //tlsc6x_irq_enable();
            send_count = 0;
            get_data_flag = 1;
        }

	{
        static unsigned char msk_o;
        unsigned char msk = 0;
        unsigned short *p_point = &p16_buf[61 - 5];
        unsigned char tcnt = buft[61*2 - 2] & 0xf;
        unsigned short x0 = p_point[0];
        unsigned char id0 = (x0 & 0x8000) >> 15;
        unsigned short y0;
        unsigned char id1;
        unsigned short x1;
        unsigned short y1;
        unsigned char mch;
        unsigned char act;

		x0 = x0 & 0x3fff;
		y0 = p_point[1];
		if(x0>0 && y0>0) {
			msk = 1 << id0;
		}
		x1 = p_point[2];
		id1 = (x1 & 0x8000) >> 15;
		x1 = x1 & 0x3fff;
		y1 = p_point[3];
		if(x1>0 && y1>0) {
			msk |= 1 << id1;
		}
		mch = msk ^ msk_o;
		if ((3 == mch) && (1 == tcnt)) {
			tcnt = 0;
			msk = 0;
			mch = msk_o;
			x0 = x1 = 0;
		}
		msk_o = msk;
		memset(p_data, 0xff, 18);
		
		p_data[0] = 0;
		p_data[1] = 0;
      
                if(tpd_proximity_flag) {
                    if(p_point[0]&0x4000) {
                        p_data[1] = 0xC0;
                    }
                    else {
                        p_data[1] = 0xE0;
                    }
                }
        
		p_data[2] = tcnt;
		act = 0;
		if (x0 > 0 && y0 > 0) {
			act = (0 == (mch & (0x01 << id0))) ? 0x80 : 0;
		} else {
			id0 = !id1;
			act = 0x40;
		}
		p_data[3] = (act | (x0 >> 8));
		p_data[4] = (x0 & 0xff);
		p_data[5] = (id0 << 4) | (y0 >> 8);
		p_data[6] = (y0 & 0xff);
		p_data[7] = 0x0d;
		p_data[8] = 0x10;
		
		if (x1 > 0 && y1 > 0) {
			act = (0 == (mch & (0x01 << id1))) ? 0x80 : 0;
		} else {
			id1 = !id0;
			act = 0x40;
		}
		p_data[9] = (act | (x1 >> 8));
		p_data[10] = (x1 & 0xff);
		p_data[11] = (id1 << 4) | (y1 >> 8);
		p_data[12] = (y1 & 0xff);
		p_data[13] = 0x0d;
		p_data[14] = 0x10;
	}
	return 0;
}
#endif

#ifdef TP_HAVE_BUTTON
static int tlsc6x_input_report_key(u16 x, u16 y,int index){
	
	tlsc_info("tlsc6x_input_report_key X is %d, Y is %d\n",x,y);
	
	if (index == 1)
		tpd_button(x, y, 1);
	else
		tpd_button(x, y, 0);
	
	return 0;
}
#endif
static int tlsc6x_update_data(void)
{
	u8 buf[20] = { 0 };
	int ret = -1;
	int i;
	u16 x, y;
	u8 tlsc_pressure, tlsc_size;
	u8 touchNum;
#ifdef TPD_PROXIMITY
	u8 state; 
    
#endif

    #ifdef TLSC_APK_DEBUG
        if(send_data_flag) {
	    tssc_get_debug_info(this_client,buf);
        }
        else {
            ret = tlsc6x_i2c_read(this_client, buf, 1, buf, 16);
            if (ret < 0) {
            	tlsc_err("%s read_data i2c_rxdata failed: %d\n", __func__, ret);
            	return ret;
            }
        }
    #else
	ret = tlsc6x_i2c_read(this_client, buf, 1, buf, 18);
	if (ret < 0) {
		tlsc_err("%s read_data i2c_rxdata failed: %d\n", __func__, ret);
		return ret;
	}
    #endif
	touchNum = buf[2] & 0x07;

#ifdef TLSC_GESTRUE        
	i = 0;        
	tlsc6x_read_reg(this_client, 0xd0, (u8 *)&i);        
	if(i ==1){            
		tlsc6x_read_Gestruedata();           
		//continue;
		return 0;
	}
#endif


#ifdef TPD_PROXIMITY
        if(tpd_proximity_flag == 1 && (touchNum == 0)){
            tlsc6x_read_reg(tlsc6x_i2c_client, 0xb0, &state);
            if(0x01 != state){
                tpd_enable_ps(1);
            }
            //tlsc6x_read_reg(tlsc6x_i2c_client, 0x01,&proximity_status);

            if(buf[1] == 0xC0){
                tpd_proximity_detect = 0;
            }else if(buf[1] == 0xE0){
                tpd_proximity_detect = 1;
            }

            if(tpd_proximity_detect != tpd_proximity_detect_prev){
                tpd_proximity_detect_prev = tpd_proximity_detect;
                tlsc_info("proximity dis is %d.\n",tpd_proximity_detect);
                if(ps_report_interrupt_data(tpd_get_ps_value())){
                    TPD_PROXIMITY_DMESG(" call hwmsen_get_interrupt_data failed\n");
                }
            }
        }  
#endif


	for (i = 0; i < TPD_SUPPORT_POINTS; i++) {
		if ((buf[6 * i + 3] & 0xc0) == 0xc0) {
			continue;
		}
		x = (s16) (buf[6 * i + 3] & 0x0F) << 8 | (s16) buf[6 * i + 4];
		y = (s16) (buf[6 * i + 5] & 0x0F) << 8 | (s16) buf[6 * i + 6];
#ifdef TP_HAVE_BUTTON
		tlsc_info("tpd_dts_data.tpd_resolution[0]=%d; tpd_dts_data.tpd_resolution[1]=%d.\n",tpd_dts_data.tpd_resolution[0],tpd_dts_data.tpd_resolution[1]);
		if (y > tpd_dts_data.tpd_resolution[1]){
			if (touchNum == 0){
				   if(tlsc6x_input_report_key(x,y,0)){
					   return 0;
					}
				}else{
					if(tlsc6x_input_report_key(x,y,1)){
						return 0;
					 }
				}
		}
#endif
		tlsc_pressure = buf[6 * i + 7];
		if (tlsc_pressure > 127) {
			tlsc_pressure = 127;
		}
		tlsc_size = (buf[6 * i + 8] >> 4) & 0x0F;
		if ((buf[6 * i + 3] & 0x40) == 0x0) {
			tlsc_info("pointer report X is %d, Y is %d, id is %d.\n",x,y,buf[6 * i + 5] >> 4);
#if MULTI_PROTOCOL_TYPE_B
			input_mt_slot(tpd->dev, buf[6 * i + 5] >> 4);
			input_mt_report_slot_state(tpd->dev, MT_TOOL_FINGER, true);
#else
			input_report_abs(tpd->dev, ABS_MT_TRACKING_ID, buf[6 * i + 5] >> 4);
#endif
			input_report_abs(tpd->dev, ABS_MT_POSITION_X, x);
			input_report_abs(tpd->dev, ABS_MT_POSITION_Y, y);
			//input_report_abs(tpd->dev, ABS_MT_PRESSURE, 15);
		//	input_report_abs(tpd->dev, ABS_MT_TOUCH_MAJOR, ft_size);
			input_report_key(tpd->dev, BTN_TOUCH, 1);
#if !MULTI_PROTOCOL_TYPE_B
			input_mt_sync(tpd->dev);
#endif
		} else {
#if MULTI_PROTOCOL_TYPE_B
			input_mt_slot(tpd->dev, buf[6 * i + 5] >> 4);
			input_mt_report_slot_state(tpd->dev, MT_TOOL_FINGER, false);
#endif
		}
	}
	if (touchNum == 0) {
		tlsc6x_clear_report_data();
	}
	input_sync(tpd->dev);

	return 0;

}


static irqreturn_t touch_event_thread_handler(int irq, void *devid)
{

	tlsc6x_update_data();
	
	return IRQ_HANDLED;
}

static int tpd_i2c_detect(struct i2c_client *client, struct i2c_board_info *info)
{
    strcpy(info->type, TPD_DEVICE);
    return 0;
}

static int tpd_irq_registration(void)
{
    int ret = 0;
    // u32 ints[2] = {0,0};
    struct device_node *node = NULL;

    node = of_find_matching_node(node, touch_of_match);
    if (NULL == node) {
        printk("Can not find touch eint device node!");
        return -ENODATA;
    }
	 
    /*touch_irq = gpio_to_irq(tpd_int_gpio_number);*/
    // of_property_read_u32_array(node,"debounce", ints, ARRAY_SIZE(ints));
    //  gpio_set_debounce(ints[0], ints[1]);

    this_client->irq = irq_of_parse_and_map(node, 0);
    ret = request_threaded_irq(this_client->irq,
				   NULL, touch_event_thread_handler,
				   IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
				   "tlsc6x_tpd_irq", NULL);
	if (ret < 0) {
		tlsc_err("Request irq thread error!");
		return  ret;
	}
    
    return ret;
}

#ifdef TLSC_GESTRUE
static int check_gesture(int gesture_id)
{	
    int keycode = 0;
    struct tlsc6x_data *data = i2c_get_clientdata(this_client);

	pr_notice("[tlsc]%s: ges id:%d\n", __func__, gesture_id);
    switch(gesture_id){
        case GESTURE_LEFT:
            keycode = KEY_LEFT;
            break;
        case GESTURE_RIGHT:
            keycode = KEY_RIGHT;
            break;
        case GESTURE_UP:
            keycode = KEY_UP;
            break;
        case GESTURE_DOWN:
            keycode = KEY_DOWN;
            break;
        case GESTURE_DOUBLECLICK:
            keycode = KEY_U;    //KEY_POWER;//
            break;
        case GESTURE_O:
            keycode = KEY_O;
            break;
        case GESTURE_W:
            keycode = KEY_W;
            break;
        case GESTURE_M:
            keycode = KEY_M;
            break;
        case GESTURE_E:
            keycode = KEY_E;
            break;
        case GESTURE_C:
            keycode = KEY_C;
            break;
        case GESTURE_S:
            keycode = KEY_S;
            break;
         case GESTURE_V:
            keycode = KEY_V;
            break;
        case GESTURE_Z:
            keycode = KEY_UP;
            break;
        case GESTURE_L:
            keycode = KEY_L;
            break;
        default:
            break;
    }
    if(keycode){
        input_report_key(data->input_dev, keycode, 1);
        input_sync(data->input_dev);
        input_report_key(data->input_dev, keycode, 0);
        input_sync(data->input_dev);
    }
    return keycode;
}

static int tlsc6x_read_Gestruedata(void)
{
    int ret = -1;
    int gestrue_id = 0;
    u8 buf[4] = {0xd3, 0xd3};
    
    ret = tlsc6x_i2c_read(this_client, buf, 1, buf, 2);
    if(ret < 0){
        pr_err("%s read_data i2c_rxdata failed: %d\n", __func__, ret);
        return ret;
    }

	pr_notice("[tlsc]%s: ges(buf[0]:%d,buf[1]:%d) Enter\n", __func__, buf[0], buf[1]);

    if(buf[1] != 0){
        gestrue_id = 0x24;
    }else{
        gestrue_id = buf[0];
    }
    check_gesture(gestrue_id);;
    return 0;
}
#endif

static int tpd_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
    int retval = 0;
    int reset_count = 0;

    int idx;
    TLSC_FUNC_ENTER();
	
    printk("Tlsc6x:%s ++!client->addr=0x%x\n", __func__, client->addr);
	client->addr = 0x2e;
	printk("Tlsc6x:%s ++!client->addr=0x%x\n", __func__, client->addr);	
	
    tlsc6x_i2c_client = client;
 

    g_tp_drvdata = kzalloc(sizeof(*g_tp_drvdata), GFP_KERNEL);	/* auto clear */
	if (!g_tp_drvdata) {
		retval = -ENOMEM;
		goto exit_alloc_data_failed;
	}

	this_client = client;
	g_tp_drvdata->client = client;

	i2c_set_clientdata(client, g_tp_drvdata);

    of_get_tlsc6x_platform_data(&client->dev);

#ifdef CONFIG_TPD_POWER_SOURCE_VIA_EXT_LDO
    tpd_ldo_power_enable(1);
#else
    tpd->reg = regulator_get(tpd->tpd_dev, "vtouch");
    retval = regulator_set_voltage(tpd->reg, 2800000, 2800000);
    if(retval != 0){
        printk("Tlsc6x error::Failed to set regulator voltage: %d\n", retval);
    }
    
    retval = regulator_enable(tpd->reg);    // set 2.8v at tpd_local_init
    if(0==regulator_is_enabled(tpd->reg)){
        printk("Tlsc6x error::Failed to enable regulator :%d\n", retval);
        regulator_put(tpd->reg);
        return -1;
    }
#endif
	

    /* set INT mode */
    tpd_gpio_as_int(tpd_int_gpio_number);

    tpd_irq_registration();
    
#ifdef __MSG_DMA_MODE__
    if(0 == msg_dma_alloct()){
        printk("Tlsc6x error::tlsc6x iic dma alloc fail!\n"); 
    }
#endif
    
    if(tpd_dts_data.use_tpd_button){
        for (idx = 0; idx < tpd_dts_data.tpd_key_num; idx++){
            input_set_capability(tpd->dev, EV_KEY, tpd_dts_data.tpd_key_local[idx]);
        }
    }

#if MULTI_PROTOCOL_TYPE_B

	input_mt_init_slots(tpd->dev, TPD_SUPPORT_POINTS, INPUT_MT_DIRECT);
#endif

    reset_count = 0;
    g_is_telink_comp = 0;
    while(++reset_count < 3 ){
        tlsc6x_tpd_reset();
        
        g_is_telink_comp = tlsc6x_tp_dect(client);
        if(g_is_telink_comp){
            break;
        }
    }

    g_tp_drvdata->needKeepRamCode = g_needKeepRamCode;

    if(0 == g_is_telink_comp){
#ifdef CONFIG_TPD_POWER_SOURCE_VIA_EXT_LDO
        tpd_ldo_power_enable(0);
#else
        retval = regulator_disable(tpd->reg); //disable regulator
        if(retval){
            printk("Tlsc6x error::regulator_disable() failed in tpd_probe!\n");
        }
        regulator_put(tpd->reg);
#endif

#ifdef TLSC_GESTRUE
        input_set_capability(input_dev, EV_KEY, KEY_POWER);
	input_set_capability(input_dev, EV_KEY, KEY_LEFT);
	input_set_capability(input_dev, EV_KEY, KEY_RIGHT);
	input_set_capability(input_dev, EV_KEY, KEY_UP);
	input_set_capability(input_dev, EV_KEY, KEY_DOWN);
	input_set_capability(input_dev, EV_KEY, KEY_D);
	input_set_capability(input_dev, EV_KEY, KEY_O);
	input_set_capability(input_dev, EV_KEY, KEY_W);
	input_set_capability(input_dev, EV_KEY, KEY_M);
	input_set_capability(input_dev, EV_KEY, KEY_E);
	input_set_capability(input_dev, EV_KEY, KEY_C);
	input_set_capability(input_dev, EV_KEY, KEY_S);
	input_set_capability(input_dev, EV_KEY, KEY_V);
	input_set_capability(input_dev, EV_KEY, KEY_Z);

	__set_bit(KEY_LEFT,  input_dev->keybit);
	__set_bit(KEY_RIGHT,  input_dev->keybit);
	__set_bit(KEY_UP,  input_dev->keybit);
	__set_bit(KEY_DOWN,  input_dev->keybit);
	__set_bit(KEY_D,  input_dev->keybit);
	__set_bit(KEY_O,  input_dev->keybit);
	__set_bit(KEY_W,  input_dev->keybit);
	__set_bit(KEY_M,  input_dev->keybit);
	__set_bit(KEY_E,  input_dev->keybit);
	__set_bit(KEY_C,  input_dev->keybit);
	__set_bit(KEY_S,  input_dev->keybit);
	__set_bit(KEY_V,  input_dev->keybit);
	__set_bit(KEY_Z,  input_dev->keybit);
#endif
        msg_dma_release();
        free_irq(this_client->irq,NULL);
        gpio_free(tpd_rst_gpio_number);
        gpio_free(tpd_int_gpio_number);
   	 printk("Tlsc6x:%s, This is not Tlsc6x, leave now!\n", __func__);
        return -1;
    }

    tlsc6x_tpd_reset();

    tpd_load_status = 1;

    //wakeup_source_init(&tlsc6x_wakelock, "tlsc6x_wakelock");

#ifdef TPD_AUTO_UPGRADE_PATH
    kthread_run(tlsc6x_update_path_proc, (void *)NULL, "tlsc6x_path_update");
#endif

 
#if 1
//create fwversion/project_id/ito_test fs
//	_tlsc6x_create_sysfs(client);
#endif

#ifdef TLSC_APK_DEBUG
        tlsc6x_create_apk_debug_channel(tlsc6x_i2c_client);
#endif
	retval=sysfs_create_group(&client->dev.kobj, &tlsc_attr_group);
	if (retval < 0) {
		tlsc_err("Can not create sysfs group!");
	}

#ifdef TLSC_ESD_HELPER_EN
    {    // esd issue: i2c monitor thread
        ktime_t ktime = ktime_set(30, 0);
        hrtimer_init(&tpd_esd_kthread_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
        tpd_esd_kthread_timer.function = tpd_esd_kthread_hrtimer_func;
        hrtimer_start(&tpd_esd_kthread_timer, ktime, HRTIMER_MODE_REL);	
        kthread_run(esd_checker_handler, 0, TPD_DEVICE);
    }
#endif
    read_version_info();
    printk("Tlsc6x:%s --!\n", __func__);
exit_alloc_data_failed:
    return 0;
}

static int tpd_remove(struct i2c_client *client)
{
    TPD_DEBUG("TPD removed\n");

#ifdef __MSG_DMA_MODE__
    msg_dma_release();
#endif
    gpio_free(tpd_rst_gpio_number);
    gpio_free(tpd_int_gpio_number);

    return 0;
}



static void tpd_resume(struct device *h)
{

#ifdef TPD_PROXIMITY
    if(tpd_proximity_suspend == 0){
        goto leave_resume;
    }else{
        tpd_proximity_suspend = 0;
    }
#endif
printk("wangguiwu ^^^^^ \r\n");

    tlsc6x_tpd_reset();

#ifdef TLSC_GESTRUE
    if(_is_open_gesture_mode == 1){
        tlsc6x_write_reg(tlsc6x_i2c_client,0xD0,0x00);
    }
#endif

	tlsc6x_clear_report_data();

    #ifdef TLSC_APK_DEBUG
       if(1 == send_data_flag) {
            get_data_start(get_data_buf);
       }
   #endif

    enable_irq(this_client->irq);
#ifdef TPD_PROXIMITY
leave_resume:
#endif
#ifdef TLSC_ESD_HELPER_EN
    hrtimer_start(&tpd_esd_kthread_timer,  ktime_set(1, 0), HRTIMER_MODE_REL);
#endif
    return;
}


static void tpd_suspend(struct device *h)
{

#ifdef TLSC_ESD_HELPER_EN
    hrtimer_cancel(&tpd_esd_kthread_timer);
#endif

#ifdef TPD_PROXIMITY
    if(tpd_proximity_flag == 1){
        tpd_proximity_suspend = 0;
        return;
    }else{
        tpd_proximity_suspend = 1;
    }
#endif
	
#ifdef TLSC_GESTRUE
    if(_is_open_gesture_mode == 1){
        tlsc6x_write_reg(tlsc6x_i2c_client, 0xd0, 0x01);
        return;
    }
#endif

    disable_irq(this_client->irq);
    tlsc6x_write_reg(tlsc6x_i2c_client, 0xA5, 0x3);  /* TP enter sleep mode */

    tlsc6x_clear_report_data();

}

static struct i2c_driver tpd_i2c_driver = {
    .driver = {
        .of_match_table = of_match_ptr(tlsc6x_dt_match),
        .name = TLSC_TPD_NAME,
    },
    .driver.name = TLSC_TPD_NAME,
    .probe = tpd_probe,
    .remove = tpd_remove,
    .id_table = tlsc6x_tpd_id,
    .detect = tpd_i2c_detect,
};

#ifdef TPD_PROXIMITY
static int tlsc6x_proximity_local_init(void)
{
    int nErr;

    struct ps_control_path ps_ctl = { 0 };
    struct ps_data_path ps_data = { 0 };

    ps_ctl.open_report_data = ps_open_report_data;
    ps_ctl.enable_nodata = ps_enable_nodata;
    ps_ctl.set_delay = ps_set_delay;
    ps_ctl.is_report_input_direct = true;
    ps_ctl.batch = ps_batch;
    ps_ctl.flush = ps_flush;
    ps_ctl.is_support_batch = false;


    nErr = ps_register_control_path(&ps_ctl);
    if(nErr){
        printk("Tlsc6x error::ps_register_control_path() failed = %d\n", nErr);
    }

    ps_data.get_data = ps_get_data;
    ps_data.vender_div = 100;
    nErr = ps_register_data_path(&ps_data);
    if (nErr) {
        printk("Tlsc6x error::ps_register_data_path() failed = %d\n", nErr);
    }
	
    return 0;
}

/*----------------------------------------------------------------------------*/

static int tlsc6x_proximity_local_uninit(void)
{
    return 0;
}

static struct alsps_init_info tlsc6x_proximity_info = {
    .name = "tlsc6x-proximity",
    .init = tlsc6x_proximity_local_init,
    .uninit = tlsc6x_proximity_local_uninit,
};


#endif

static int tpd_local_init(void)
{ 
    printk("Tlsc6x:%s ++!\n", __func__);

    if(i2c_add_driver(&tpd_i2c_driver) != 0){
        printk("Tlsc6x error::unable to add i2c driver.\n");
        return -1;
    }

    if(tpd_load_status == 0){
        printk("Tlsc6x error::add touch panel driver error.\n");
        i2c_del_driver(&tpd_i2c_driver);
        return -1;
    }
#ifdef TPD_PROXIMITY
	tlsc6x_proximity_local_init();
#endif

    if(tpd_dts_data.use_tpd_button){
        tpd_button_setting(tpd_dts_data.tpd_key_num, tpd_dts_data.tpd_key_local,tpd_dts_data.tpd_key_dim_local);
    }

    tpd_type_cap = 1;

    printk("Tlsc6x:%s --!\n", __func__);
    
    return 0;
}



static struct tpd_driver_t tpd_device_driver = {
    .tpd_device_name = TLSC_TPD_NAME,
    .tpd_local_init = tpd_local_init,
    .suspend = tpd_suspend,
    .resume = tpd_resume,
#ifdef TP_HAVE_BUTTON
    .tpd_have_button = 1,
#else
    .tpd_have_button = 0,
#endif
};

 //static struct i2c_board_info __initdata tlsc6x_i2c_tpd={ I2C_BOARD_INFO(TLSC_TPD_NAME, (0x5c>>1))};

/* called when loaded into kernel */
static int __init tpd_driver_init(void)
{
    printk("Tlsc6x:%s ++!\n", __func__);

    tpd_get_dts_info();

    // i2c_register_board_info(TPD_IIC_MASTER_PORT, &tlsc6x_i2c_tpd, 1);

    if(tpd_driver_add(&tpd_device_driver) < 0){
        printk("add Tlsc6x tpd driver failed\n");
    }

    #ifdef TPD_PROXIMITY
    alsps_driver_add(&tlsc6x_proximity_info);
    #endif

    printk("Tlsc6x:%s --!\n", __func__);

    return 0;
}

/* should never be called */
static void __exit tpd_driver_exit(void)
{
    TPD_DMESG("MediaTek Tlsc6x touch driver exit\n");
    tpd_driver_remove(&tpd_device_driver);
}

module_init(tpd_driver_init);
module_exit(tpd_driver_exit);
