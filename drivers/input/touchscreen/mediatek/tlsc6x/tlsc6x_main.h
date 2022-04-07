#ifndef __tlsc6x_main_h__  
#define __tlsc6x_main_h__


#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/sched.h>
#include <linux/kthread.h>
//#include <linux/rtpm_prio.h>
#include <linux/wait.h>
#include <linux/time.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/gpio.h>
#include <linux/hrtimer.h>
#include <linux/string.h>
#include <linux/vmalloc.h>
#include <linux/input.h>
#include <linux/slab.h>
#include <linux/bitops.h>
#include <linux/semaphore.h>
#include <linux/mutex.h>
#include <linux/syscalls.h>
#include <linux/byteorder/generic.h>
#include <linux/interrupt.h>
#include <asm/unistd.h>
//#include <mach/mt_pm_ldo.h>
//#include <mach/mt_typedefs.h>
//#include <mach/mt_boot.h>
//#include <mach/irqs.h>
#include <linux/jiffies.h>
#include <linux/version.h>
#include <linux/types.h>
#include <linux/sched.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <asm/uaccess.h>
#include <linux/mount.h>
#include <linux/unistd.h>
#include <linux/proc_fs.h>
#include <linux/netdevice.h>
#include <../fs/proc/internal.h>

/*
    proximity function via touch.
*/
#define TPD_PROXIMITY
/*
    can build new FW-VERSION into driver and OPEN this macro, this driver whill upgrate it into tp-chip
*/

/*
    if OPEN this macro, one kernel thread whill be create, this thread poll tp's state and re-power & reset tp if needed.
*/
#define TLSC_ESD_HELPER_EN	/* esd helper, close:undef */
//#define TLSC_GESTRUE
/*
    if OPEN this macro, one named PROC-ENTRY whill be created, apk can access this PROC-ENTRY.
*/
//正式版本可以关闭此宏
#define TLSC_APK_DEBUG      // apk debugger, close:undef
#define TLSC_BUILDIN_BOOT 

#define tlsc_info(x...) pr_notice("[tlsc] " x)
#define tlsc_err(x...) pr_err("[tlsc][error] " x)
#define TLSC_FUNC_ENTER() pr_notice("[tlsc]%s: Enter\n", __func__)
struct tlsc6x_platform_data {
	u32 irq_gpio_number;
	u32 reset_gpio_number;
	//const char *vdd_name;
	u32 tpd_tlsc_firmware_update;
	u32 virtualkeys[12];
	u32 x_res_max;
	u32 y_res_max;
};
struct tlsc6x_data {
	struct input_dev *input_dev;
	struct input_dev *ps_input_dev;
	struct i2c_client *client;
#if defined(CONFIG_ADF)
	struct notifier_block fb_notif;
#elif defined(CONFIG_HAS_EARLYSUSPEND)
	struct early_suspend early_suspend;
#endif
	struct work_struct resume_work;
	struct workqueue_struct *tp_resume_workqueue;
	int irq_gpio_number;
	int reset_gpio_number;
	int isVddAlone;
	int needKeepRamCode;
	int esdHelperFreeze;
	//struct regulator *reg_vdd;
	struct tlsc6x_platform_data *platform_data;
};
/*
 * struct tlsc6x_updfile_data - upgrade file description
 * @sig:		file tag
 *	@n_cfg:	contain tp-cfg number
 * @n_match:	supported vendor number
 * @size_cfg:	tp-cfg size if exist
 * @size_boot:	boot size if exist
 */
struct tlsc6x_updfile_header {
	u32 sig;
	u32 resv;
	u32 n_cfg;
	u32 n_match;
	u32 len_cfg;
	u32 len_boot;
};

#define MAX_TRX_LEN (8)	/* max IIC data length */

extern struct tlsc6x_data *g_tp_drvdata;
extern struct mutex i2c_rw_access;

extern unsigned int g_tlsc6x_cfg_ver;
extern unsigned int g_tlsc6x_boot_ver;
extern unsigned short g_tlsc6x_chip_code;
extern unsigned int g_needKeepRamCode;

extern int tlsc6x_tp_dect(struct i2c_client *client);
extern int tlsc6x_auto_upgrade_buidin(void);
extern int tlsc6x_load_gesture_binlib(void);
extern void tlsc6x_data_crash_deal(void);

extern int tlsx6x_update_burn_cfg(u16 *ptcfg);
extern int tlsx6x_update_running_cfg(u16 *ptcfg);
extern int tlsc6x_set_dd_mode_sub(void);
extern int tlsc6x_set_nor_mode_sub(void);
extern int tlsc6x_set_dd_mode(void);
extern int tlsc6x_set_nor_mode(void);

extern int tlsc6x_write_reg(struct i2c_client *client, u8 regaddr, u8 regvalue);
extern int tlsc6x_read_reg(struct i2c_client *client, u8 regaddr, u8 *regvalue);
extern int tlsc6x_i2c_read(struct i2c_client *client, char *writebuf,int writelen, char *readbuf, int readlen);
extern int tlsc6x_i2c_write(struct i2c_client *client, char *writebuf, int writelen);
extern int tlsc6x_i2c_read_sub(struct i2c_client *client, char *writebuf, int writelen, char *readbuf, int readlen);
extern int tlsc6x_i2c_write_sub(struct i2c_client *client, char *writebuf, int writelen);

#if (defined TPD_AUTO_UPGRADE_PATH) || (defined TLSC_APK_DEBUG)
extern int tlsc6x_proc_cfg_update(u8 *dir, int behave);
#endif
extern void tlsc6x_tpd_reset_force(void);
extern int tlsc6x_fif_write(char *fname, u8 *pdata, u16 len);
#endif
