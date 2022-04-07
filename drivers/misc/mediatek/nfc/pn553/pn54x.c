/*
 * Copyright (C) 2010 Trusted Logic S.A.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 *
 */
/******************************************************************************
 *
 *  The original Work has been changed by NXP Semiconductors.
 *
 *  Copyright (C) 2013-2014 NXP Semiconductors
 *   *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 ******************************************************************************/

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/list.h>
#include <linux/i2c.h>
#include <linux/irq.h>
#include <linux/jiffies.h>
#include <linux/uaccess.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/platform_device.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/miscdevice.h>
#include <linux/spinlock.h>
#include <asm/siginfo.h>
#include <linux/rcupdate.h>
#include <linux/sched.h>
#include <linux/signal.h>
#include <linux/sched/signal.h>

#ifdef CONFIG_PM_WAKELOCKS
#include <linux/pm_wakeup.h>
#else
#include <linux/wakelock.h>
#endif

#include "pn54x.h"

#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>

#define NEXUS5x    1
#define HWINFO     1
#define DRAGON_NFC 1
#define SIG_NFC 44
#define MAX_BUFFER_SIZE 512


#include <linux/version.h>

#if (KERNEL_VERSION(4, 4, 0) > LINUX_VERSION_CODE)
// Legacy implementation, also used on recent kernels for legacy platforms
// such as (6580 and 6735)
# define KRNMTKLEGACY_I2C 1
# define KRNMTKLEGACY_CLK 1
# define KRNMTKLEGACY_GPIO 1
#endif
// Kernel 4.9 on some platforms is using legacy drivers (kernel-4.9-lc)
// I2C: CONFIG_MACH_MT6735 / 6735M / 6753 / 6580 / 6755 use legacy driver
// CLOCK: 4.9 has right includes, no need for special handling.
// GPIO : same as I2C -- we use the same condition at the moment.
//#if (defined(CONFIG_MACH_MT6735) || defined(CONFIG_MACH_MT6735M) ||
//    defined(CONFIG_MACH_MT6753) || defined(CONFIG_MACH_MT6580) ||
//	defined(CONFIG_MACH_MT6755))
// test on I2C special define instead of listing the platforms
#ifdef CONFIG_MTK_I2C_EXTENSION
# define KRNMTKLEGACY_I2C 1
# define KRNMTKLEGACY_GPIO 1
#endif

#ifdef KRNMTKLEGACY_I2C
#include <linux/dma-mapping.h>
#define NFC_CLIENT_TIMING 400		 /* I2C speed */
static char *I2CDMAWriteBuf; /*= NULL;*/ /* unnecessary initialise */
static unsigned int I2CDMAWriteBuf_pa;   /* = NULL; */
static char *I2CDMAReadBuf; /*= NULL;*/  /* unnecessary initialise */
static unsigned int I2CDMAReadBuf_pa;    /* = NULL; */
#endif	

struct pn544_dev    {
    wait_queue_head_t   read_wq;
    struct mutex        read_mutex;
    struct i2c_client   *client;
    struct miscdevice   pn544_device;
    unsigned int        ven_gpio;
    unsigned int        firm_gpio;
    unsigned         irq_gpio;
    unsigned int        ese_pwr_gpio; /* gpio used by SPI to provide power to p61 via NFCC */
    struct mutex        p61_state_mutex; /* used to make p61_current_state flag secure */
    p61_access_state_t  p61_current_state; /* stores the current P61 state */
    bool                nfc_ven_enabled; /* stores the VEN pin state powered by Nfc */
    bool                spi_ven_enabled; /* stores the VEN pin state powered by Spi */
    bool                irq_enabled;
    spinlock_t          irq_enabled_lock;
    long                nfc_service_pid; /*used to signal the nfc the nfc service */
};

#ifdef CONFIG_PM_WAKELOCKS
struct wakeup_source *nfc_wake_lock;
#else
struct wake_lock nfc_wake_lock;
#endif
#if HWINFO
struct hw_type_info hw_info;
#endif

static bool  sIsWakeLocked = false;
static struct pn544_dev *pn544_dev;
static struct semaphore ese_access_sema;
static struct semaphore svdd_sync_onoff_sema;
static void release_ese_lock(p61_access_state_t  p61_current_state);
int get_ese_lock(p61_access_state_t  p61_current_state, int timeout);
#if HWINFO
static void check_hw_info(void);
#endif
static void pn544_disable_irq(struct pn544_dev *pn544_dev)
{
    unsigned long flags;

    spin_lock_irqsave(&pn544_dev->irq_enabled_lock, flags);
    if (pn544_dev->irq_enabled) {
        disable_irq_nosync(pn544_dev->client->irq);
        disable_irq_wake(pn544_dev->client->irq);
        pn544_dev->irq_enabled = false;
    }
    spin_unlock_irqrestore(&pn544_dev->irq_enabled_lock, flags);
}

static irqreturn_t pn544_dev_irq_handler(int irq, void *dev_id)
{
    struct pn544_dev *pn544_dev = dev_id;
	printk("zhf irqreturn_t pn544_dev_irq_handler\n");
    pn544_disable_irq(pn544_dev);
    if (sIsWakeLocked == false)
    {	
#ifdef CONFIG_PM_WAKELOCKS
        __pm_stay_awake(nfc_wake_lock);
#else
		
        wake_lock(&nfc_wake_lock);
#endif			
        sIsWakeLocked = true;
    } else {
            pr_debug("%s already wake locked!\n", __func__);
    }
    /* Wake up waiting readers */
    wake_up(&pn544_dev->read_wq);

    return IRQ_HANDLED;
}

static ssize_t pn544_dev_read(struct file *filp, char __user *buf,
        size_t count, loff_t *offset)
{
    struct pn544_dev *pn544_dev = filp->private_data;
#ifndef KRNMTKLEGACY_I2C
    char tmp[MAX_BUFFER_SIZE];
#endif
    int ret;

    if (count > MAX_BUFFER_SIZE)
        count = MAX_BUFFER_SIZE;

    pr_debug("%s : reading   %zu bytes.\n", __func__, count);

    mutex_lock(&pn544_dev->read_mutex);

    if (!gpio_get_value(pn544_dev->irq_gpio)) {
        if (filp->f_flags & O_NONBLOCK) {
            ret = -EAGAIN;
            goto fail;
        }

        while (1) {
            pn544_dev->irq_enabled = true;
            enable_irq(pn544_dev->client->irq);
            enable_irq_wake(pn544_dev->client->irq);
            ret = wait_event_interruptible(
                    pn544_dev->read_wq,
                    !pn544_dev->irq_enabled);

            pn544_disable_irq(pn544_dev);

            if (ret)
                goto fail;

            if (gpio_get_value(pn544_dev->irq_gpio))
                break;

            pr_warning("%s: spurious interrupt detected\n", __func__);
        }
    }

    /* Read data */
/* Read data */
#ifdef KRNMTKLEGACY_I2C
	pn544_dev->client->addr =
		(pn544_dev->client->addr & I2C_MASK_FLAG);
	pn544_dev->client->ext_flag |= I2C_DMA_FLAG;
	pn544_dev->client->timing = NFC_CLIENT_TIMING;

	/* Read data */
	ret = i2c_master_recv(pn544_dev->client,
			(unsigned char *)(uintptr_t)I2CDMAReadBuf_pa,
			count);
#else
    ret = i2c_master_recv(pn544_dev->client, tmp, count);
#endif
    if (sIsWakeLocked == true) {
#ifdef CONFIG_PM_WAKELOCKS
        __pm_relax(nfc_wake_lock);
#else
				
        wake_unlock(&nfc_wake_lock);
#endif		
        sIsWakeLocked = false;
    }
    mutex_unlock(&pn544_dev->read_mutex);

    /* pn544 seems to be slow in handling I2C read requests
     * so add 1ms delay after recv operation */
#if !NEXUS5x
    udelay(1000);
#endif

    if (ret < 0) {
        pr_err("%s: i2c_master_recv returned %d\n", __func__, ret);
        return ret;
    }
    if (ret > count) {
        pr_err("%s: received too many bytes from i2c (%d)\n",
                __func__, ret);
        return -EIO;
    }
#ifdef KRNMTKLEGACY_I2C
	if (copy_to_user(buf, I2CDMAReadBuf, ret)) {
#else
    if (copy_to_user(buf, tmp, ret)) {
#endif
        pr_warning("%s : failed to copy to user space\n", __func__);
        return -EFAULT;
    }
    return ret;

    fail:
    mutex_unlock(&pn544_dev->read_mutex);
    return ret;
}

static ssize_t pn544_dev_write(struct file *filp, const char __user *buf,
        size_t count, loff_t *offset)
{
    struct pn544_dev  *pn544_dev;
#ifndef KRNMTKLEGACY_I2C
	char tmp[MAX_BUFFER_SIZE];
#endif
    int ret;

    pn544_dev = filp->private_data;

    if (count > MAX_BUFFER_SIZE)
        count = MAX_BUFFER_SIZE;

#ifdef KRNMTKLEGACY_I2C
	if (copy_from_user(I2CDMAWriteBuf, buf, count)) {
#else
    if (copy_from_user(tmp, buf, count)) {
#endif
        pr_err("%s : failed to copy from user space\n", __func__);
        return -EFAULT;
    }

    pr_debug("%s : writing %zu bytes.\n", __func__, count);
	
/* Write data */
#ifdef KRNMTKLEGACY_I2C
	pn544_dev->client->addr =
		(pn544_dev->client->addr & I2C_MASK_FLAG);

	pn544_dev->client->ext_flag |= I2C_DMA_FLAG;
	pn544_dev->client->timing = NFC_CLIENT_TIMING;

	ret = i2c_master_send(pn544_dev->client,
				(unsigned char *)(uintptr_t)I2CDMAWriteBuf_pa,
				count);
#else
    ret = i2c_master_send(pn544_dev->client, tmp, count);
#endif
    if (ret != count) {
        pr_err("%s : i2c_master_send returned %d\n", __func__, ret);
        ret = -EIO;
    }

    /* pn544 seems to be slow in handling I2C write requests
     * so add 1ms delay after I2C send oparation */
    udelay(1000);

    return ret;
}

static void p61_update_access_state(struct pn544_dev *pn544_dev, p61_access_state_t current_state, bool set)
{
    pr_info("%s: Enter current_state = %x\n", __func__, pn544_dev->p61_current_state);
    if (current_state)
    {
        if(set){
            if(pn544_dev->p61_current_state == P61_STATE_IDLE)
                pn544_dev->p61_current_state = P61_STATE_INVALID;
            pn544_dev->p61_current_state |= current_state;
        }
        else{
            pn544_dev->p61_current_state ^= current_state;
            if(!pn544_dev->p61_current_state)
                pn544_dev->p61_current_state = P61_STATE_IDLE;
        }
    }
    pr_info("%s: Exit current_state = %x\n", __func__, pn544_dev->p61_current_state);
}

static void p61_get_access_state(struct pn544_dev *pn544_dev, p61_access_state_t *current_state)
{

    if (current_state == NULL) {
        //*current_state = P61_STATE_INVALID;
        pr_err("%s : invalid state of p61_access_state_t current state  \n", __func__);
    } else {
        *current_state = pn544_dev->p61_current_state;
    }
}
static void p61_access_lock(struct pn544_dev *pn544_dev)
{
    pr_info("%s: Enter\n", __func__);
    mutex_lock(&pn544_dev->p61_state_mutex);
    pr_info("%s: Exit\n", __func__);
}
static void p61_access_unlock(struct pn544_dev *pn544_dev)
{
    pr_info("%s: Enter\n", __func__);
    mutex_unlock(&pn544_dev->p61_state_mutex);
    pr_info("%s: Exit\n", __func__);
}

static int signal_handler(p61_access_state_t state, long nfc_pid)
{
    struct siginfo sinfo;
    pid_t pid;
    struct task_struct *task;
    int sigret = 0;
    int ret = 0;
    pr_info("%s: Enter\n", __func__);

    memset(&sinfo, 0, sizeof(struct siginfo));
    sinfo.si_signo = SIG_NFC;
    sinfo.si_code = SI_QUEUE;
    sinfo.si_int = state;
    pid = nfc_pid;

    task = pid_task(find_vpid(pid), PIDTYPE_PID);
    if(task)
    {
        pr_info("%s.\n", task->comm);
        sigret = force_sig_info(SIG_NFC, &sinfo, task);
        if(sigret < 0){
            pr_info("send_sig_info failed..... sigret %d.\n", sigret);
            ret = -1;
        }
    }
    else
    {
        pr_info("finding task from PID failed\r\n");
        ret = -1;
    }
    pr_info("%s: Exit ret = %d\n", __func__, ret);
    return ret;
}
static void svdd_sync_onoff(long nfc_service_pid, p61_access_state_t origin)
{
    int timeout = 100; //100 ms timeout
    unsigned long tempJ = msecs_to_jiffies(timeout);
    pr_info("%s: Enter nfc_service_pid: %ld\n", __func__, nfc_service_pid);
    if(nfc_service_pid)
    {
        if (0 == signal_handler(origin, nfc_service_pid))
        {
            sema_init(&svdd_sync_onoff_sema, 0);
            pr_info("Waiting for svdd protection response");
            if(down_timeout(&svdd_sync_onoff_sema, tempJ) != 0)
            {
                pr_info("svdd wait protection: Timeout");
            }
            pr_info("svdd wait protection : released");
        }
    }
    pr_info("%s: Exit\n", __func__);
}
static int release_svdd_wait(void)
{
    pr_info("%s: Enter \n", __func__);
    up(&svdd_sync_onoff_sema);
    pr_info("%s: Exit\n", __func__);
    return 0;
}
static int pn544_dev_open(struct inode *inode, struct file *filp)
{
    struct pn544_dev *pn544_dev = container_of(filp->private_data,
            struct pn544_dev,
            pn544_device);

    filp->private_data = pn544_dev;

    pr_debug("%s : %d,%d\n", __func__, imajor(inode), iminor(inode));

    return 0;
}

long  pn544_dev_ioctl(struct file *filp, unsigned int cmd,
        unsigned long arg)
{
    pr_info("%s :enter cmd = %u, arg = %ld\n", __func__, cmd, arg);

    if (cmd == P544_GET_ESE_ACCESS)
    {
        return get_ese_lock(P61_STATE_WIRED, arg);
    }
    else if(cmd == P544_REL_SVDD_WAIT)
    {
        return release_svdd_wait();
    }
    p61_access_lock(pn544_dev);
    switch (cmd) {
    case PN544_SET_PWR:
    {
        p61_access_state_t current_state = P61_STATE_INVALID;
        p61_get_access_state(pn544_dev, &current_state);
        if (arg == 2) {
            if (current_state & (P61_STATE_SPI|P61_STATE_SPI_PRIO))
            {
                /* NFCC fw/download should not be allowed if p61 is used
                 * by SPI
                 */
                pr_info("%s NFCC should not be allowed to reset/FW download \n", __func__);
                p61_access_unlock(pn544_dev);
                return -EBUSY; /* Device or resource busy */
            }
            pn544_dev->nfc_ven_enabled = true;
            if (pn544_dev->spi_ven_enabled == false)
            {
                /* power on with firmware download (requires hw reset)
                 */
                pr_info("%s power on with firmware\n", __func__);
                gpio_set_value(pn544_dev->ven_gpio, 1);
                msleep(10);
                if (pn544_dev->firm_gpio) {
                    p61_update_access_state(pn544_dev, P61_STATE_DWNLD, true);
                    gpio_set_value(pn544_dev->firm_gpio, 1);
                }
                msleep(10);
                gpio_set_value(pn544_dev->ven_gpio, 0);
                msleep(10);
                gpio_set_value(pn544_dev->ven_gpio, 1);
                msleep(10);
            }
        } else if (arg == 1) {
            /* power on */
            pr_info("%s power on\n", __func__);
            if (pn544_dev->firm_gpio) {
                if ((current_state & (P61_STATE_WIRED|P61_STATE_SPI|P61_STATE_SPI_PRIO))== 0){
                    p61_update_access_state(pn544_dev, P61_STATE_IDLE, true);
                }
                gpio_set_value(pn544_dev->firm_gpio, 0);
            }

            pn544_dev->nfc_ven_enabled = true;
            if (pn544_dev->spi_ven_enabled == false) {
                gpio_set_value(pn544_dev->ven_gpio, 1);
            }
        } else if (arg == 0) {
            /* power off */
            pr_info("%s power off\n", __func__);
            if (pn544_dev->firm_gpio) {
                if ((current_state & (P61_STATE_WIRED|P61_STATE_SPI|P61_STATE_SPI_PRIO))== 0){
                    p61_update_access_state(pn544_dev, P61_STATE_IDLE, true);
                }
                gpio_set_value(pn544_dev->firm_gpio, 0);
            }

            pn544_dev->nfc_ven_enabled = false;
            /* Don't change Ven state if spi made it high */
            if (pn544_dev->spi_ven_enabled == false) {
                gpio_set_value(pn544_dev->ven_gpio, 0);
            }
            if (sIsWakeLocked == true) {
#ifdef CONFIG_PM_WAKELOCKS
        __pm_relax(nfc_wake_lock);
#else
				
        wake_unlock(&nfc_wake_lock);
#endif	
                sIsWakeLocked = false;
            }
        } else {
            pr_err("%s bad arg %lu\n", __func__, arg);
            /* changed the p61 state to idle*/
            p61_access_unlock(pn544_dev);
            return -EINVAL;
        }
    }
    break;
    case P61_SET_SPI_PWR:
    {
        p61_access_state_t current_state = P61_STATE_INVALID;
        p61_get_access_state(pn544_dev, &current_state);
        if (arg == 1) {
            pr_info("%s : PN61_SET_SPI_PWR - power on ese\n", __func__);
            if ((current_state & (P61_STATE_SPI|P61_STATE_SPI_PRIO)) == 0)
            {
                p61_update_access_state(pn544_dev, P61_STATE_SPI, true);
                /*To handle triple mode protection signal
                NFC service when SPI session started*/
                if ((current_state & P61_STATE_JCOP_DWNLD) == 0)
                {
                    if(pn544_dev->nfc_service_pid){
                        pr_info("nfc service pid %s   ---- %ld", __func__, pn544_dev->nfc_service_pid);
                        signal_handler(P61_STATE_SPI, pn544_dev->nfc_service_pid);
                    }
                    else{
                        pr_info(" invalid nfc service pid....signalling failed%s   ---- %ld", __func__, pn544_dev->nfc_service_pid);
                    }
                }
                pn544_dev->spi_ven_enabled = true;
                if (pn544_dev->nfc_ven_enabled == false)
                {
                    /* provide power to NFCC if, NFC service not provided */
                    gpio_set_value(pn544_dev->ven_gpio, 1);
                    msleep(10);
                }
                /* pull the gpio to high once NFCC is power on*/
                //gpio_set_value(pn544_dev->ese_pwr_gpio, 1);
                msleep(10);
            } else {
                pr_info("%s : PN61_SET_SPI_PWR -  power on ese failed \n", __func__);
                p61_access_unlock(pn544_dev);
                return -EBUSY; /* Device or resource busy */
            }
        } else if (arg == 0) {
            pr_info("%s : PN61_SET_SPI_PWR - power off ese\n", __func__);
            if(current_state & P61_STATE_SPI_PRIO){
                p61_update_access_state(pn544_dev, P61_STATE_SPI_PRIO, false);
                if (!(current_state & P61_STATE_WIRED))
                {
                    if((current_state & P61_STATE_JCOP_DWNLD) == 0)
                    {
                        svdd_sync_onoff(pn544_dev->nfc_service_pid, P61_STATE_SPI_SVDD_SYNC_START |
                         P61_STATE_SPI_PRIO_END);
                    }
                    else
                    {
                        svdd_sync_onoff(pn544_dev->nfc_service_pid, P61_STATE_SPI_SVDD_SYNC_START);
                    }
                    //gpio_set_value(pn544_dev->ese_pwr_gpio, 0);
                    svdd_sync_onoff(pn544_dev->nfc_service_pid, P61_STATE_SPI_SVDD_SYNC_END);
                }
                else if ((current_state & P61_STATE_JCOP_DWNLD) == 0)
                {
                    if(pn544_dev->nfc_service_pid){
                        pr_info("nfc service pid %s   ---- %ld", __func__, pn544_dev->nfc_service_pid);
                        signal_handler(P61_STATE_SPI_PRIO_END, pn544_dev->nfc_service_pid);
                    }
                    else
                    {
                        pr_info(" invalid nfc service pid....signalling failed%s   ---- %ld", __func__, pn544_dev->nfc_service_pid);
                    }
                }
                if(current_state & P61_STATE_JCOP_DWNLD)
                    p61_update_access_state(pn544_dev, P61_STATE_JCOP_DWNLD, false);
                pn544_dev->spi_ven_enabled = false;
                 if (pn544_dev->nfc_ven_enabled == false) {
                     gpio_set_value(pn544_dev->ven_gpio, 0);
                     msleep(10);
                 }
              }else if(current_state & P61_STATE_SPI){
                  p61_update_access_state(pn544_dev, P61_STATE_SPI, false);
                  if (!(current_state & P61_STATE_WIRED))
                  {
                      if((current_state & P61_STATE_JCOP_DWNLD) == 0)
                      {
                        svdd_sync_onoff(pn544_dev->nfc_service_pid, P61_STATE_SPI_SVDD_SYNC_START | P61_STATE_SPI_END);
                      }
                      else
                      {
                        svdd_sync_onoff(pn544_dev->nfc_service_pid, P61_STATE_SPI_SVDD_SYNC_START);
                      }
                      //gpio_set_value(pn544_dev->ese_pwr_gpio, 0);
                      svdd_sync_onoff(pn544_dev->nfc_service_pid, P61_STATE_SPI_SVDD_SYNC_END);
                  }
                  /*If JCOP3.2 or 3.3 for handling triple mode
                  protection signal NFC service */
                  else
                  {
                      if ((current_state & P61_STATE_JCOP_DWNLD) == 0)
                      {
                          if(pn544_dev->nfc_service_pid){
                              pr_info("nfc service pid %s   ---- %ld", __func__, pn544_dev->nfc_service_pid);
                              signal_handler(P61_STATE_SPI_END, pn544_dev->nfc_service_pid);
                           }
                           else{
                               pr_info(" invalid nfc service pid....signalling failed%s   ---- %ld", __func__, pn544_dev->nfc_service_pid);
                           }
                      }
                  }
                  if(current_state & P61_STATE_JCOP_DWNLD)
                      p61_update_access_state(pn544_dev, P61_STATE_JCOP_DWNLD, false);
                  pn544_dev->spi_ven_enabled = false;
                  if (pn544_dev->nfc_ven_enabled == false) {
                      gpio_set_value(pn544_dev->ven_gpio, 0);
                      msleep(10);
                  }
            } else {
                pr_err("%s : PN61_SET_SPI_PWR - failed, current_state = %x \n",
                        __func__, pn544_dev->p61_current_state);
                p61_access_unlock(pn544_dev);
                return -EPERM; /* Operation not permitted */
            }
        }else if (arg == 2) {
            pr_info("%s : PN61_SET_SPI_PWR - reset\n", __func__);
            if (current_state & (P61_STATE_IDLE|P61_STATE_SPI|P61_STATE_SPI_PRIO)) {
                if (pn544_dev->spi_ven_enabled == false)
                {
                    pn544_dev->spi_ven_enabled = true;
                    if (pn544_dev->nfc_ven_enabled == false) {
                        /* provide power to NFCC if, NFC service not provided */
                        gpio_set_value(pn544_dev->ven_gpio, 1);
                        msleep(10);
                    }
                }
                svdd_sync_onoff(pn544_dev->nfc_service_pid, P61_STATE_SPI_SVDD_SYNC_START);
                //gpio_set_value(pn544_dev->ese_pwr_gpio, 0);
                msleep(10);
                svdd_sync_onoff(pn544_dev->nfc_service_pid, P61_STATE_SPI_SVDD_SYNC_END);
                //gpio_set_value(pn544_dev->ese_pwr_gpio, 1);
                msleep(10);
            } else {
                pr_info("%s : PN61_SET_SPI_PWR - reset  failed \n", __func__);
                p61_access_unlock(pn544_dev);
                return -EBUSY; /* Device or resource busy */
            }
        }else if (arg == 3) {
            pr_info("%s : PN61_SET_SPI_PWR - Prio Session Start power on ese\n", __func__);
            if ((current_state & (P61_STATE_SPI|P61_STATE_SPI_PRIO)) == 0) {
                p61_update_access_state(pn544_dev, P61_STATE_SPI_PRIO, true);
                //if (current_state & P61_STATE_WIRED)
                {
                    if(pn544_dev->nfc_service_pid){
                        pr_info("nfc service pid %s   ---- %ld", __func__, pn544_dev->nfc_service_pid);
                        signal_handler(P61_STATE_SPI_PRIO, pn544_dev->nfc_service_pid);
                    }
                    else{
                        pr_info(" invalid nfc service pid....signalling failed%s   ---- %ld", __func__, pn544_dev->nfc_service_pid);
                    }
                }
                pn544_dev->spi_ven_enabled = true;
                if (pn544_dev->nfc_ven_enabled == false) {
                    /* provide power to NFCC if, NFC service not provided */
                    gpio_set_value(pn544_dev->ven_gpio, 1);
                    msleep(10);
                }
                /* pull the gpio to high once NFCC is power on*/
                //gpio_set_value(pn544_dev->ese_pwr_gpio, 1);
                msleep(10);
            }else {
                pr_info("%s : Prio Session Start power on ese failed \n", __func__);
                p61_access_unlock(pn544_dev);
                return -EBUSY; /* Device or resource busy */
            }
        }else if (arg == 4) {
            if (current_state & P61_STATE_SPI_PRIO)
            {
                pr_info("%s : PN61_SET_SPI_PWR - Prio Session Ending...\n", __func__);
                p61_update_access_state(pn544_dev, P61_STATE_SPI_PRIO, false);
                /*after SPI prio timeout, the state is changing from SPI prio to SPI */
                p61_update_access_state(pn544_dev, P61_STATE_SPI, true);
                //if (current_state & P61_STATE_WIRED)
                {
                    if(pn544_dev->nfc_service_pid){
                        pr_info("nfc service pid %s   ---- %ld", __func__, pn544_dev->nfc_service_pid);
                        signal_handler(P61_STATE_SPI_PRIO_END, pn544_dev->nfc_service_pid);
                    }
                    else{
                        pr_info(" invalid nfc service pid....signalling failed%s   ---- %ld", __func__, pn544_dev->nfc_service_pid);
                    }
               }
            }
            else
            {
                pr_info("%s : PN61_SET_SPI_PWR -  Prio Session End failed \n", __func__);
                p61_access_unlock(pn544_dev);
                return -EBADRQC; /* Device or resource busy */
            }
        } else if(arg == 5){
            release_ese_lock(P61_STATE_SPI);
        }
        else {
            pr_info("%s bad ese pwr arg %lu\n", __func__, arg);
            p61_access_unlock(pn544_dev);
            return -EBADRQC; /* Invalid request code */
        }
    }
    break;

    case P61_SET_PWR_STATUS:
    {
        pr_info("%s: P61_SET_PWR_STATUS = %lx",__func__, arg);
        p61_update_access_state(pn544_dev, arg, true);
    }
    break;
    case P61_GET_PWR_STATUS:
    {
        p61_access_state_t current_state = P61_STATE_INVALID;
        p61_get_access_state(pn544_dev, &current_state);
        pr_info("%s: P61_GET_PWR_STATUS  = %x",__func__, current_state);
        put_user(current_state, (int __user *)arg);
    }
    break;
    case P61_SET_WIRED_ACCESS:
    {
        p61_access_state_t current_state = P61_STATE_INVALID;
        p61_get_access_state(pn544_dev, &current_state);
        if (arg == 1)
        {
            if (current_state)
            {
                pr_info("%s : P61_SET_WIRED_ACCESS - enabling\n", __func__);
                p61_update_access_state(pn544_dev, P61_STATE_WIRED, true);
                if (current_state & P61_STATE_SPI_PRIO)
                {
                    if(pn544_dev->nfc_service_pid){
                        pr_info("nfc service pid %s   ---- %ld", __func__, pn544_dev->nfc_service_pid);
                        signal_handler(P61_STATE_SPI_PRIO, pn544_dev->nfc_service_pid);
                    }
                    else{
                        pr_info(" invalid nfc service pid....signalling failed%s   ---- %ld", __func__, pn544_dev->nfc_service_pid);
                    }
                }
                if((current_state & (P61_STATE_SPI|P61_STATE_SPI_PRIO)) == 0)
                {
                    //gpio_set_value(pn544_dev->ese_pwr_gpio, 1);
                    msleep(10);
                }
            } else {
                pr_info("%s : P61_SET_WIRED_ACCESS -  enabling failed \n", __func__);
                p61_access_unlock(pn544_dev);
                return -EBUSY; /* Device or resource busy */
            }
        } else if (arg == 0) {
            pr_info("%s : P61_SET_WIRED_ACCESS - disabling \n", __func__);
            if (current_state & P61_STATE_WIRED){
                p61_update_access_state(pn544_dev, P61_STATE_WIRED, false);
                if((current_state & (P61_STATE_SPI|P61_STATE_SPI_PRIO)) == 0)
                {
                    svdd_sync_onoff(pn544_dev->nfc_service_pid, P61_STATE_DWP_SVDD_SYNC_START);
                    //gpio_set_value(pn544_dev->ese_pwr_gpio, 0);
                    svdd_sync_onoff(pn544_dev->nfc_service_pid, P61_STATE_DWP_SVDD_SYNC_END);
                }
            } else {
                pr_err("%s : P61_SET_WIRED_ACCESS - failed, current_state = %x \n",
                        __func__, pn544_dev->p61_current_state);
                p61_access_unlock(pn544_dev);
                return -EPERM; /* Operation not permitted */
            }
        }
        else if(arg == 2)
        {
             pr_info("%s : P61_ESE_GPIO_LOW  \n", __func__);
            svdd_sync_onoff(pn544_dev->nfc_service_pid, P61_STATE_DWP_SVDD_SYNC_START);
             //gpio_set_value(pn544_dev->ese_pwr_gpio, 0);
            svdd_sync_onoff(pn544_dev->nfc_service_pid, P61_STATE_DWP_SVDD_SYNC_END);
        }
        else if(arg == 3)
        {
            pr_info("%s : P61_ESE_GPIO_HIGH  \n", __func__);
            //gpio_set_value(pn544_dev->ese_pwr_gpio, 1);
            msleep(10);
        }
        else if(arg == 4)
        {
            release_ese_lock(P61_STATE_WIRED);
        }
        else {
             pr_info("%s P61_SET_WIRED_ACCESS - bad arg %lu\n", __func__, arg);
             p61_access_unlock(pn544_dev);
             return -EBADRQC; /* Invalid request code */
        }
    }
    break;
    case P544_SET_NFC_SERVICE_PID:
    {
        pr_info("%s : The NFC Service PID is %ld\n", __func__, arg);
        pn544_dev->nfc_service_pid = arg;

    }
    break;
    default:
        pr_err("%s bad ioctl %u\n", __func__, cmd);
        p61_access_unlock(pn544_dev);
        return -EINVAL;
    }
    p61_access_unlock(pn544_dev);
    pr_info("%s :exit cmd = %u, arg = %ld\n", __func__, cmd, arg);
    return 0;
}
EXPORT_SYMBOL(pn544_dev_ioctl);

int get_ese_lock(p61_access_state_t  p61_current_state, int timeout)
{
    unsigned long tempJ = msecs_to_jiffies(timeout);
    if(down_timeout(&ese_access_sema, tempJ) != 0)
    {
        printk("get_ese_lock: timeout p61_current_state = %d\n", p61_current_state);
        return -EBUSY;
    }
    return 0;
}
EXPORT_SYMBOL(get_ese_lock);

static void release_ese_lock(p61_access_state_t  p61_current_state)
{
    up(&ese_access_sema);
}


static const struct file_operations pn544_dev_fops = {
        .owner  = THIS_MODULE,
        .llseek = no_llseek,
        .read   = pn544_dev_read,
        .write  = pn544_dev_write,
        .open   = pn544_dev_open,
        .unlocked_ioctl  = pn544_dev_ioctl,
};
#if DRAGON_NFC
static int pn544_parse_dt(struct device *dev,
    struct pn544_i2c_platform_data *data)
{
    struct device_node *np;// = dev->of_node;
    int errorno = 0;
    np = of_find_compatible_node(NULL, NULL, "mediatek,nfc-gpio-v2");
#if !NEXUS5x
        data->irq_gpio = of_get_named_gpio(np, "nxp,pn544-irq", 0);
        if ((!gpio_is_valid(data->irq_gpio)))
                return -EINVAL;

        data->ven_gpio = of_get_named_gpio(np, "nxp,pn544-ven", 0);
        if ((!gpio_is_valid(data->ven_gpio)))
                return -EINVAL;

        data->firm_gpio = of_get_named_gpio(np, "nxp,pn544-fw-dwnld", 0);
        if ((!gpio_is_valid(data->firm_gpio)))
                return -EINVAL;

   /*     data->ese_pwr_gpio = of_get_named_gpio(np, "nxp,pn544-ese-pwr", 0);
        if ((!gpio_is_valid(data->ese_pwr_gpio)))
                return -EINVAL;*/
#else
	if (np) {
		printk("%s :zhf np get done\n",__func__);
		data->ven_gpio = of_get_named_gpio_flags(np,"nxp,gpio_ven", 0, NULL);
		data->firm_gpio = of_get_named_gpio_flags(np,"nxp,gpio-rst", 0, NULL);
		data->irq_gpio = of_get_named_gpio_flags(np,"nxp,gpio_irq", 0, NULL);						
	} else {
		printk("%s : get gpio num err.\n", __func__);
		return -1;
	}
#endif
    pr_info("%s: %d, %d, %d, %d\n", __func__,
        data->irq_gpio, data->ven_gpio, data->firm_gpio, errorno);

    return errorno;
}
#endif

static int pn544_probe(struct i2c_client *client,
        const struct i2c_device_id *id)
{
    int ret;
    struct pn544_i2c_platform_data *platform_data;
    //struct pn544_dev *pn544_dev;
	
	struct device_node *st_node;
	struct device_node *node = client->dev.of_node;

#ifdef KRNMTKLEGACY_I2C
#ifdef CONFIG_64BIT
	I2CDMAWriteBuf = (char *)dma_alloc_coherent(
		&client->dev, MAX_BUFFER_SIZE, (dma_addr_t *)&I2CDMAWriteBuf_pa,
		GFP_KERNEL);
#else
	I2CDMAWriteBuf = (char *)dma_alloc_coherent(
		NULL, MAX_BUFFER_SIZE, (dma_addr_t *)&I2CDMAWriteBuf_pa,
		GFP_KERNEL);
#endif

	if (I2CDMAWriteBuf == NULL)
		pr_err("%s : failed to allocate dma buffer\n", __func__);
#ifdef CONFIG_64BIT
	I2CDMAReadBuf = (char *)dma_alloc_coherent(
		&client->dev, MAX_BUFFER_SIZE, (dma_addr_t *)&I2CDMAReadBuf_pa,
		GFP_KERNEL);
#else
	I2CDMAReadBuf = (char *)dma_alloc_coherent(
		NULL, MAX_BUFFER_SIZE, (dma_addr_t *)&I2CDMAReadBuf_pa,
		GFP_KERNEL);
#endif

	if (I2CDMAReadBuf == NULL)
		pr_err("%s : failed to allocate dma buffer\n", __func__);
	pr_debug("%s :I2CDMAWriteBuf_pa %d, I2CDMAReadBuf_pa,%d\n", __func__,
		I2CDMAWriteBuf_pa, I2CDMAReadBuf_pa);
#endif /* KRNMTKLEGACY_I2C */


#if !DRAGON_NFC
    platform_data = client->dev.platform_data;
#else

    if (node) {
        platform_data = devm_kzalloc(&client->dev,
            sizeof(struct pn544_i2c_platform_data), GFP_KERNEL);
        if (!platform_data) {
            dev_err(&client->dev,
                "nfc-nci probe: Failed to allocate memory\n");
            return -ENOMEM;
        }
        ret = pn544_parse_dt(&client->dev, platform_data);
        if (ret)
        {
            pr_info("%s pn544_parse_dt failed", __func__);
        }
		//printk("zhf irq_gpio = %d\n",platform_data->irq_gpio);
        //client->irq = gpio_to_irq(platform_data->irq_gpio);
		st_node = of_find_compatible_node(NULL, NULL, "mediatek,nfc-gpio-v2");
        if (st_node) {
			client->irq = irq_of_parse_and_map(st_node, 0);
			pr_info("%s : MT IRQ GPIO = %d\n", __func__, client->irq);
        } else {
                pr_err("%s : can not find NFC eint compatible node\n",__func__);
        }
        if (client->irq < 0)
        {
            pr_info("%s gpio to irq failed,client->irq = %d\n", __func__,client->irq);
        }	
    } else {
        platform_data = client->dev.platform_data;
    }
#endif
    if (platform_data == NULL) {
        pr_err("%s : nfc probe fail\n", __func__);
        return  -ENODEV;
    }

    if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
        pr_err("%s : need I2C_FUNC_I2C\n", __func__);
        return  -ENODEV;
    }
//#if !DRAGON_NFC
#if DRAGON_NFC
    ret = gpio_request(platform_data->irq_gpio, "nfc_int");
	printk("zhf nfc_int ret = %d\n",ret);
    if (ret)
        return  -ENODEV;
    ret = gpio_request(platform_data->ven_gpio, "nfc_ven");
	printk("zhf nfc_ven ret = %d\n",ret);
    if (ret)
        goto err_ven;
  /*  ret = gpio_request(platform_data->ese_pwr_gpio, "nfc_ese_pwr");
    if (ret)
        goto err_ese_pwr;*/
    if (platform_data->firm_gpio) {
        ret = gpio_request(platform_data->firm_gpio, "nfc_firm");
		printk("zhf nfc_firm ret = %d\n",ret);
        if (ret)
            goto err_firm;
    }
#endif
    pn544_dev = kzalloc(sizeof(*pn544_dev), GFP_KERNEL);
    if (pn544_dev == NULL) {
        dev_err(&client->dev,
                "failed to allocate memory for module data\n");
        ret = -ENOMEM;
        goto err_exit;
    }

    pn544_dev->irq_gpio = platform_data->irq_gpio;
    pn544_dev->ven_gpio  = platform_data->ven_gpio;
    pn544_dev->firm_gpio  = platform_data->firm_gpio;
    //pn544_dev->ese_pwr_gpio  = platform_data->ese_pwr_gpio;
    pn544_dev->p61_current_state = P61_STATE_IDLE;
    pn544_dev->nfc_ven_enabled = false;
    pn544_dev->spi_ven_enabled = false;
    pn544_dev->client   = client;

   ret = gpio_direction_input(pn544_dev->irq_gpio);
    if (ret < 0) {
        pr_err("%s :not able to set irq_gpio as input\n", __func__);
        goto err_ven;
    }
    ret = gpio_direction_output(pn544_dev->ven_gpio, 0);
    if (ret < 0) {
        pr_err("%s : not able to set ven_gpio as output\n", __func__);
        goto err_firm;
    }
 /*   ret = gpio_direction_output(pn544_dev->ese_pwr_gpio, 0);
    if (ret < 0) {
        pr_err("%s : not able to set ese_pwr gpio as output\n", __func__);
        goto err_ese_pwr;
    }*/
    if (platform_data->firm_gpio) {
        ret = gpio_direction_output(pn544_dev->firm_gpio, 0);
        if (ret < 0) {
            pr_err("%s : not able to set firm_gpio as output\n",
                    __func__);
            goto err_exit;
        }
    }

    /* init mutex and queues */
    init_waitqueue_head(&pn544_dev->read_wq);
    mutex_init(&pn544_dev->read_mutex);
    sema_init(&ese_access_sema, 1);
    mutex_init(&pn544_dev->p61_state_mutex);
    spin_lock_init(&pn544_dev->irq_enabled_lock);

    pn544_dev->pn544_device.minor = MISC_DYNAMIC_MINOR;
    pn544_dev->pn544_device.name = "pn553";	//"pn54x";
    pn544_dev->pn544_device.fops = &pn544_dev_fops;

    ret = misc_register(&pn544_dev->pn544_device);
    if (ret) {
        pr_err("%s : misc_register failed\n", __FILE__);
        goto err_misc_register;
    }
	
#ifdef CONFIG_PM_WAKELOCKS
    nfc_wake_lock = wakeup_source_register(NULL, "NFCWAKE");
#else	
    wake_lock_init(&nfc_wake_lock, WAKE_LOCK_SUSPEND, "NFCWAKE");
#endif
    /* request irq.  the irq is set whenever the chip has data available
     * for reading.  it is cleared when all data has been read.
     */
    printk("%s : requesting IRQ %d\n", __func__, client->irq);
    pn544_dev->irq_enabled = true;
    ret = request_irq(client->irq, pn544_dev_irq_handler,
            IRQF_TRIGGER_HIGH, client->name, pn544_dev);
    if (ret) {
        dev_err(&client->dev, "request_irq failed\n");
        goto err_request_irq_failed;
    }
    printk("%s : Enabling IRQ Wake\n", __func__);
    enable_irq_wake(pn544_dev->client->irq);
    pn544_disable_irq(pn544_dev);
    i2c_set_clientdata(client, pn544_dev);
#if HWINFO
	/*
	 * This function is used only if
	 * hardware info is required during probe*/
	check_hw_info();
#endif
	printk("%s : zhf probe done done\n", __func__);
    return 0;

    err_request_irq_failed:
    misc_deregister(&pn544_dev->pn544_device);
    err_misc_register:
    mutex_destroy(&pn544_dev->read_mutex);
    mutex_destroy(&pn544_dev->p61_state_mutex);
    kfree(pn544_dev);
    err_exit:
    if (pn544_dev->firm_gpio)
        gpio_free(platform_data->firm_gpio);
    err_firm:
 // gpio_free(platform_data->ese_pwr_gpio);
 // err_ese_pwr:
    gpio_free(platform_data->ven_gpio);
    err_ven:
    gpio_free(platform_data->irq_gpio);
    return ret;
}

static int pn544_remove(struct i2c_client *client)
{
    struct pn544_dev *pn544_dev;

#ifdef KRNMTKLEGACY_I2C
	if (I2CDMAWriteBuf) {
#ifdef CONFIG_64BIT
		dma_free_coherent(&client->dev, MAX_BUFFER_SIZE, I2CDMAWriteBuf,
				I2CDMAWriteBuf_pa);
#else
		dma_free_coherent(NULL, MAX_BUFFER_SIZE, I2CDMAWriteBuf,
				I2CDMAWriteBuf_pa);
#endif
		I2CDMAWriteBuf = NULL;
		I2CDMAWriteBuf_pa = 0;
	}

	if (I2CDMAReadBuf) {
#ifdef CONFIG_64BIT
		dma_free_coherent(&client->dev, MAX_BUFFER_SIZE, I2CDMAReadBuf,
				I2CDMAReadBuf_pa);
#else
		dma_free_coherent(NULL, MAX_BUFFER_SIZE, I2CDMAReadBuf,
				I2CDMAReadBuf_pa);
#endif
		I2CDMAReadBuf = NULL;
		I2CDMAReadBuf_pa = 0;
	}
#endif /* KRNMTKLEGACY_I2C */

    pn544_dev = i2c_get_clientdata(client);
    free_irq(client->irq, pn544_dev);
    misc_deregister(&pn544_dev->pn544_device);
    mutex_destroy(&pn544_dev->read_mutex);
    mutex_destroy(&pn544_dev->p61_state_mutex);
    gpio_free(pn544_dev->irq_gpio);
    gpio_free(pn544_dev->ven_gpio);
    //gpio_free(pn544_dev->ese_pwr_gpio);
    pn544_dev->p61_current_state = P61_STATE_INVALID;
    pn544_dev->nfc_ven_enabled = false;
    pn544_dev->spi_ven_enabled = false;

    if (pn544_dev->firm_gpio)
        gpio_free(pn544_dev->firm_gpio);
    kfree(pn544_dev);

    return 0;
}

static const struct i2c_device_id pn544_id[] = {
#if NEXUS5x
        { "pn548", 0 },
#else
        { "pn544", 0 },
#endif
        { }
};
#if DRAGON_NFC
static struct of_device_id pn544_i2c_dt_match[] = {
    {
#if NEXUS5x
        .compatible = "mediatek,nfc",
#else
        .compatible = "mediatek,nfc",
#endif
    },
    {}
};
#endif
static struct i2c_driver pn544_driver = {
        .id_table   = pn544_id,
        .probe      = pn544_probe,
        .remove     = pn544_remove,
        .driver     = {
                .owner = THIS_MODULE,
#if NEXUS5x
                .name  = "pn548",
#else
                .name  = "pn544",
#endif
#if DRAGON_NFC
                .of_match_table = pn544_i2c_dt_match,
#endif
        },
};

#if HWINFO
/******************************************************************************
 * Function         check_hw_info
 *
 * Description      This function is called during pn544_probe to retrieve
 *                  HW info.
 *                  Useful get HW information in case of previous FW download is
 *                  interrupted and core reset is not allowed.
 *                  This function checks if core reset  is allowed, if not
 *                  sets DWNLD_REQ(firm_gpio) , ven reset and sends firmware
 *                  get version command.
 *                  In response HW information will be received.
 *
 * Returns          None
 *
 ******************************************************************************/
static void check_hw_info()
{
	char read_data[20];
	int ret, get_version_len = 8, retry_count = 0;
	static uint8_t cmd_reset_nci[] = { 0x20, 0x00, 0x01, 0x00 };
	char get_version_cmd[] =
	    { 0x00, 0x04, 0xF1, 0x00, 0x00, 0x00, 0x6E, 0xEF };

	pr_info("%s :Enter\n", __func__);

	/*
	 * Ven Reset  before sending core Reset
	 * This is to check core reset is allowed or not.
	 * If not allowed then previous FW download is interrupted in between
	 * */
	pr_info("%s :Ven Reset \n", __func__);
	gpio_set_value(pn544_dev->ven_gpio, 1);
	msleep(10);
	gpio_set_value(pn544_dev->ven_gpio, 0);
	msleep(10);
	gpio_set_value(pn544_dev->ven_gpio, 1);
	msleep(10);
	ret = i2c_master_send(pn544_dev->client, cmd_reset_nci, 4);

	if (ret == 4) {
		pr_info("%s : core reset write success\n", __func__);
	} else {

		/*
		 * Core reset  failed.
		 * set the DWNLD_REQ , do ven reset
		 * send firmware download info command
		 * */
		pr_err("%s : write failed\n", __func__);
		pr_info("%s power on with firmware\n", __func__);
		gpio_set_value(pn544_dev->ven_gpio, 1);
		msleep(10);
		if (pn544_dev->firm_gpio) {
			p61_update_access_state(pn544_dev, P61_STATE_DWNLD,
						true);
			gpio_set_value(pn544_dev->firm_gpio, 1);
		}
		msleep(10);
		gpio_set_value(pn544_dev->ven_gpio, 0);
		msleep(10);
		gpio_set_value(pn544_dev->ven_gpio, 1);
		msleep(10);
		ret =
		    i2c_master_send(pn544_dev->client, get_version_cmd,
				    get_version_len);
		if (ret != get_version_len) {
			ret = -EIO;
			pr_err("%s : write_failed \n", __func__);
		} else {
			pr_info("%s :data sent\n", __func__);
		}

		ret = 0;

		while (retry_count < 10) {

			/*
			 * Wait for read interrupt
			 * If spurious interrupt is received retry again
			 * */
			pn544_dev->irq_enabled = true;
			enable_irq(pn544_dev->client->irq);
			enable_irq_wake(pn544_dev->client->irq);
			ret = wait_event_interruptible(pn544_dev->read_wq,
						       !pn544_dev->irq_enabled);

			pn544_disable_irq(pn544_dev);

			if (gpio_get_value(pn544_dev->irq_gpio))
				break;

			pr_warning("%s: spurious interrupt detected\n",
				   __func__);
			retry_count++;
		}

		if (ret) {
			return;
		}

		/*
		 * Read response data and copy into hw_type_info
		 * */
		ret = i2c_master_recv(pn544_dev->client, read_data, 14);

		if (ret) {
			memcpy(hw_info.data, read_data, ret);
			hw_info.len = ret;
			pr_info("%s :data received len  : %d\n", __func__,
				hw_info.len);
		} else {
			pr_err("%s :Read Failed\n", __func__);
		}
	}
}
#endif
/*
 * module load/unload record keeping
 */

static int __init pn544_dev_init(void)
{
    pr_info("Loading pn544 driver\n");
    return i2c_add_driver(&pn544_driver);
}
module_init(pn544_dev_init);

static void __exit pn544_dev_exit(void)
{
    pr_info("Unloading pn544 driver\n");
    i2c_del_driver(&pn544_driver);
}
module_exit(pn544_dev_exit);

MODULE_AUTHOR("Sylvain Fonteneau");
MODULE_DESCRIPTION("NFC PN544 driver");
MODULE_LICENSE("GPL");
