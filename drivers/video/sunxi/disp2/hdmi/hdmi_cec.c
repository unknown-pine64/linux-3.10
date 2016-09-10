/*
 * Copyright (C) 2012 Freescale Semiconductor, Inc. All Rights Reserved.
 * Copyright (C) 2016 Joachim Damm
 */

/*
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */

/*!
 * @file hdmi_cec.c
 *
 * @brief HDMI CEC system initialization and file operation implementation
 *
 * @ingroup HDMI
 */

#include <linux/kernel.h>
#include <linux/mm.h>
#include <linux/fs.h>           /* for struct file_operations */
#include <linux/stat.h>
#include <linux/platform_device.h>
#include <linux/poll.h>
#include <linux/wait.h>
#include <linux/list.h>
#include <linux/delay.h>
#include <linux/fsl_devices.h>
#include <linux/uaccess.h>
#include <linux/io.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/vmalloc.h>
#include <linux/workqueue.h>
#include <asm/sizes.h>
#include <linux/module.h>
#include <linux/bitrev.h>
#include <linux/kthread.h>
#include <linux/console.h>
#include <linux/types.h>

#include "dw_hdmi.h"
#include "hdmi_cec.h"
#include "hdmi_edid.h"
#include "drv_hdmi_i.h"
#include "hdmi_bsp.h"
#include "hdmi_core.h"

#define MAX_MESSAGE_LEN         17

#define MESSAGE_TYPE_RECEIVE_SUCCESS            1
#define MESSAGE_TYPE_NOACK              2
#define MESSAGE_TYPE_DISCONNECTED               3
#define MESSAGE_TYPE_CONNECTED          4
#define MESSAGE_TYPE_SEND_SUCCESS               5

#define CEC_TX_INPROGRESS -1
#define CEC_TX_AVAIL 0

#define NACK_RETRY_COUNT 2
#define FAIL_RETRY_COUNT 2

struct hdmi_cec_priv {
    int  receive_error;
    int sent_error;
    u8 Logical_address;
    bool cec_state;
    int latest_cec_stat;
    u8 last_msg[MAX_MESSAGE_LEN];
    u8 msg_len;
    int tx_answer;
    u8 link_status;
    u32 cec_irq;
    spinlock_t irq_lock;
    struct delayed_work hdmi_cec_work;
    struct mutex lock;
};

struct hdmi_cec_event {
    int event_type;
    int msg_len;
    u8 msg[MAX_MESSAGE_LEN];
    struct list_head list;
};

static LIST_HEAD(head);

struct task_struct *rxack_task;
static int hdmi_cec_major;
static struct class *hdmi_cec_class;
static struct hdmi_cec_priv hdmi_cec_data;
static u8 open_count;
static u8 cec_l_addr_l = 0, cec_l_addr_h = 0;
static wait_queue_head_t hdmi_cec_queue;
static wait_queue_head_t tx_cec_queue;

static int tx_reg[16] = {HDMI_CEC_TX_DATA0, HDMI_CEC_TX_DATA1, HDMI_CEC_TX_DATA2,\
            HDMI_CEC_TX_DATA3, HDMI_CEC_TX_DATA4, HDMI_CEC_TX_DATA5,\
            HDMI_CEC_TX_DATA6, HDMI_CEC_TX_DATA7, HDMI_CEC_TX_DATA8,\
            HDMI_CEC_TX_DATA9, HDMI_CEC_TX_DATA10, HDMI_CEC_TX_DATA11,\
            HDMI_CEC_TX_DATA12, HDMI_CEC_TX_DATA13, HDMI_CEC_TX_DATA14,\
            HDMI_CEC_TX_DATA15};

static int rx_reg[16] = {HDMI_CEC_RX_DATA0, HDMI_CEC_RX_DATA1, HDMI_CEC_RX_DATA2,\
            HDMI_CEC_RX_DATA3, HDMI_CEC_RX_DATA4, HDMI_CEC_RX_DATA5,\
            HDMI_CEC_RX_DATA6, HDMI_CEC_RX_DATA7, HDMI_CEC_RX_DATA8,\
            HDMI_CEC_RX_DATA9, HDMI_CEC_RX_DATA10, HDMI_CEC_RX_DATA11,\
            HDMI_CEC_RX_DATA12, HDMI_CEC_RX_DATA13, HDMI_CEC_RX_DATA14,\
            HDMI_CEC_RX_DATA15};

static void hdmi_writel(u32 value, unsigned int reg)
{
    __raw_writel(value, (unsigned long long)hdmi_base_addr + reg);
}

static u32 hdmi_readl(unsigned int reg)
{
    u32 value;

    value = __raw_readl((unsigned long long)hdmi_base_addr + reg);
    return value;
}

static u8 hdmi_readb(unsigned int reg)
{
    u8 value;

    /* unlock read access */
    hdmi_writel(0x54524545, 0x10010);
    value = __raw_readb((unsigned long long)hdmi_base_addr + reg);
    return value;
}

static void hdmi_writeb(u8 value, unsigned int reg)
{
    __raw_writeb(value, (unsigned long long)hdmi_base_addr + reg);
}

static int sunxi_hdmi_notify(struct notifier_block *nb,
                             unsigned long code, void *unused)
{
    u8 val = 0;
    struct hdmi_cec_event *event = NULL;
    pr_info("[CEC]sunxi_hdmi_notify: %d\n", code);

    if (open_count) {
        switch (code) {
            case 0x00: // Unplug
                pr_info("[CEC]HDMI link disconnected\n");
                event = vmalloc(sizeof(struct hdmi_cec_event));
                if (NULL == event) {
                    pr_err("%s: Not enough memory!\n", __func__);
                    break;
                }
                memset(event, 0, sizeof(struct hdmi_cec_event));
                event->event_type = MESSAGE_TYPE_DISCONNECTED;
                mutex_lock(&hdmi_cec_data.lock);
                list_add_tail(&event->list, &head);
                mutex_unlock(&hdmi_cec_data.lock);
                wake_up(&hdmi_cec_queue);
                break;
            case 0x04: // Plug
                pr_info("[CEC]HDMI link connected\n");
                event = vmalloc(sizeof(struct hdmi_cec_event));
                if (NULL == event) {
                    pr_err("%s: Not enough memory\n", __func__);
                    break;
                }
                memset(event, 0, sizeof(struct hdmi_cec_event));
                event->event_type = MESSAGE_TYPE_CONNECTED;
                mutex_lock(&hdmi_cec_data.lock);
                list_add_tail(&event->list, &head);
                mutex_unlock(&hdmi_cec_data.lock);
                wake_up(&hdmi_cec_queue);
                break;
            case 0x05: // reinit done
                pr_info("[CEC]HDMI reinitialized\n");
                val = hdmi_readb(HDMI_MC_CLKDIS);
                val &= ~HDMI_MC_CLKDIS_CECCLK_DISABLE;
                hdmi_writeb(val, HDMI_MC_CLKDIS);
                hdmi_writeb(0x02, HDMI_CEC_CTRL);
                /* enable CEC receive */
                hdmi_writel(0x84, 0x1003c);
                /* Force read unlock */
                hdmi_writeb(0x0, HDMI_CEC_LOCK);
                val = HDMI_IH_CEC_STAT0_ERROR_INIT | HDMI_IH_CEC_STAT0_NACK | HDMI_IH_CEC_STAT0_EOM | HDMI_IH_CEC_STAT0_DONE;
                hdmi_writeb(val, HDMI_CEC_POLARITY);
                val = HDMI_IH_CEC_STAT0_WAKEUP | HDMI_IH_CEC_STAT0_ERROR_FOLL | HDMI_IH_CEC_STAT0_ARB_LOST;
                hdmi_writeb(val, HDMI_CEC_MASK);
                hdmi_writeb(val, HDMI_IH_MUTE_CEC_STAT0);
                hdmi_writeb(cec_l_addr_l, HDMI_CEC_ADDR_L);
                hdmi_writeb(cec_l_addr_h, HDMI_CEC_ADDR_H);
                break;
        }
    }
    return NOTIFY_DONE;
}

static struct notifier_block sunxi_hdmi_nb = {
        .notifier_call = sunxi_hdmi_notify,
};

static int hdmi_cec_read_reg_bit(void)    /* 1bit */
{
    return (hdmi_readl(0x1003c) & 0x2) ? 1 : 0;
}

static int hdmi_cec_is_receiving(void)    /* 1bit */
{
    return hdmi_readl(0x1003c) & 0x4;
}

static unsigned int hdmi_cec_phy()
{
  return hdmi_readl(0x1003c);
}

static void set_hdmi_cec_phy(unsigned int value)
{
  hdmi_writel(value, 0x1003c);
}

#define cur_in_usecs ktime_to_us(ktime_get())

static int last_signal = -1;
static unsigned last_signal_tick = 0;

static int wait_for_signal_interval(int signal, int interval, int timeout)
{
  unsigned start = cur_in_usecs;

  for (;;) {
    int current_signal = hdmi_cec_read_reg_bit();
    if (last_signal != current_signal) {
      last_signal = current_signal;
      last_signal_tick = cur_in_usecs;
    }

    if (current_signal == signal) {
      return cur_in_usecs - start;
    }

    if (cur_in_usecs - start > timeout) {
      return -1;
    }

    usleep_range(interval, interval);

    if (kthread_should_stop()) {
        return -1;
    }
  }

  return -1;
}

static int wait_for_signal(int signal, int timeout)
{
  return wait_for_signal_interval(signal, 100, timeout);
}

static int wait_in_state(int signal, int min_time, int max_time)
{
  int signal_time = wait_for_signal(!signal, max_time);
  if (signal_time < 0) {
    return -1;
  }

  if (signal_time >= min_time) {
    return 0;
  } else {
    return -1;
  }
}

static int wait_for_start_bit()
{
    if (wait_for_signal_interval(0, 300, 5000) < 0) {
      return -1;
    }

    if (wait_in_state(0, 2800, 3800) < 0) {
      return -1;
    }

    if (wait_in_state(1, 600, 1300) < 0) {
      return -1;
    }

    return 0;
}

static int wait_for_bit()
{
    if (wait_in_state(0, 400, 1700) < 0) {
      return -1;
    }

    if(wait_in_state(1, 700, 2400) < 0) {
      return -1;
    }

    return 0;
}

static int wait_for_9_bits()
{
  int i;

  for (i = 0; i < 9; ++i) {
    if (wait_for_bit() < 0) {
      return -1;
    }
  }

  return 0;
}

static int rxack_thread(void *data)
{
  while (!kthread_should_stop()) {
    usleep_range(100, 100);

    if (wait_for_start_bit() < 0) {
      continue;
    }

    for (;;) {
      if (wait_for_9_bits() < 0) {
        break;
      }

      if (!hdmi_cec_is_receiving()) {
        break;
      }

      set_hdmi_cec_phy(0x4);
      usleep_range(1800, 1800);
      set_hdmi_cec_phy(0x84);

      if (wait_for_signal(0, 2000) < 0) {
        break;
      }
    }
  }

  return 0;
}

static irqreturn_t hdmi_cec_isr(int irq, void *data)
{
    struct hdmi_cec_priv *hdmi_cec = data;
    u16 cec_stat;
    unsigned long flags;
    irqreturn_t ret = IRQ_HANDLED;

    spin_lock_irqsave(&hdmi_cec->irq_lock, flags);

    hdmi_writeb(0x7f, HDMI_IH_MUTE_CEC_STAT0);
    cec_stat = hdmi_readb(HDMI_IH_CEC_STAT0);
    hdmi_writeb(cec_stat, HDMI_IH_CEC_STAT0);

    if ((cec_stat & (HDMI_IH_CEC_STAT0_ERROR_INIT | \
        HDMI_IH_CEC_STAT0_NACK | HDMI_IH_CEC_STAT0_EOM | \
        HDMI_IH_CEC_STAT0_DONE)) == 0) {
        //hdmi_writeb(0x0, HDMI_CEC_LOCK);
        //hdmi_writel(0x84, 0x1003c);
        spin_unlock_irqrestore(&hdmi_cec->irq_lock, flags);
        return ret;
    }

    /* disable CEC sending */
    if (!(cec_stat & HDMI_IH_CEC_STAT0_EOM)) {
        hdmi_writel(0x84, 0x1003c);
    }

    pr_debug("HDMI CEC interrupt received: %d\n", cec_stat);

    hdmi_cec->latest_cec_stat = cec_stat;

    schedule_delayed_work(&hdmi_cec->hdmi_cec_work, msecs_to_jiffies(20));

    spin_unlock_irqrestore(&hdmi_cec->irq_lock, flags);

    return ret;
}

void hdmi_cec_handle(u16 cec_stat)
{
    u8 val = 0, i = 0;
    struct hdmi_cec_event *event = NULL;
    /*The current transmission is successful (for initiator only).*/

    if (cec_stat & HDMI_IH_CEC_STAT0_DONE) {
        printk(KERN_INFO "[HDMICEC] wrote %d\n", hdmi_cec_data.msg_len);
        if(hdmi_cec_data.tx_answer == CEC_TX_INPROGRESS) {
            hdmi_cec_data.tx_answer = cec_stat;
            wake_up(&tx_cec_queue);
        }
    }
    /*EOM is detected so that the received data is ready in the receiver data buffer*/
    else if (cec_stat & HDMI_IH_CEC_STAT0_EOM) {
        //hdmi_writeb(0x02, HDMI_IH_CEC_STAT0);
        event = vmalloc(sizeof(struct hdmi_cec_event));
        if (NULL == event) {
            pr_err("%s: Not enough memory!\n", __func__);
            return;
        }
        memset(event, 0, sizeof(struct hdmi_cec_event));
        event->msg_len = hdmi_readb(HDMI_CEC_RX_CNT);
        if (!event->msg_len) {
            pr_err("%s: Invalid CEC message length!\n", __func__);
            return;
        }
        printk(KERN_INFO "[HDMICEC] received %d\n", event->msg_len);
        event->event_type = MESSAGE_TYPE_RECEIVE_SUCCESS;
        for (i = 0; i < event->msg_len; i++)
            event->msg[i] = hdmi_readb(rx_reg[i]);
        hdmi_writeb(0x0, HDMI_CEC_LOCK);
        mutex_lock(&hdmi_cec_data.lock);
        if(open_count) {
          list_add_tail(&event->list, &head);
        } else {
          vfree(event);
        }
        mutex_unlock(&hdmi_cec_data.lock);
        wake_up(&hdmi_cec_queue);
    }
    /*An error is detected on cec line (for initiator only). */
    else if (cec_stat & HDMI_IH_CEC_STAT0_ERROR_INIT) {
        printk(KERN_INFO "[HDMICEC] write FAIL\n");
        if(hdmi_cec_data.sent_error++ < FAIL_RETRY_COUNT) {
          mutex_lock(&hdmi_cec_data.lock);
          hdmi_writel(0x0, 0x1003c);
          val = hdmi_readb(HDMI_CEC_CTRL);
          val |= 0x01;
          hdmi_writeb(val, HDMI_CEC_CTRL);
          mutex_unlock(&hdmi_cec_data.lock);
        } else if(hdmi_cec_data.tx_answer == CEC_TX_INPROGRESS) {
            hdmi_cec_data.tx_answer = cec_stat;
            wake_up(&tx_cec_queue);
        }
    }
    /*A frame is not acknowledged in a directly addressed message. Or a frame is negatively acknowledged in
    a broadcast message (for initiator only).*/
    else if (cec_stat & HDMI_IH_CEC_STAT0_NACK) {
        printk(KERN_INFO "[HDMICEC] write NACK\n");
        if(hdmi_cec_data.sent_error++ < NACK_RETRY_COUNT) {
          mutex_lock(&hdmi_cec_data.lock);
          hdmi_writel(0x0, 0x1003c);
          val = hdmi_readb(HDMI_CEC_CTRL);
          val |= 0x01;
          hdmi_writeb(val, HDMI_CEC_CTRL);
          mutex_unlock(&hdmi_cec_data.lock);
        } else if(hdmi_cec_data.tx_answer == CEC_TX_INPROGRESS) {
            hdmi_cec_data.tx_answer = cec_stat;
            wake_up(&tx_cec_queue);
        }
    }
    /*An error is notified by a follower. Abnormal logic data bit error (for follower).*/
    else if (cec_stat & HDMI_IH_CEC_STAT0_ERROR_FOLL) {
        printk(KERN_INFO "[HDMICEC] receive ERROR\n");
        hdmi_cec_data.receive_error++;
    }
    return;
}

static void hdmi_cec_worker(struct work_struct *work)
{
    u8 val;
    hdmi_cec_handle(hdmi_cec_data.latest_cec_stat);
    val = HDMI_IH_CEC_STAT0_WAKEUP | HDMI_IH_CEC_STAT0_ERROR_FOLL | HDMI_IH_CEC_STAT0_ARB_LOST;
    hdmi_writeb(val, HDMI_IH_MUTE_CEC_STAT0);
}

/*!
 * @brief open function for cec file operation
 *
 * @return  0 on success or negative error code on error
 */
static int hdmi_cec_open(struct inode *inode, struct file *filp)
{
    mutex_lock(&hdmi_cec_data.lock);
    if (open_count) {
        mutex_unlock(&hdmi_cec_data.lock);
        return -EBUSY;
    }
    open_count = 1;
    filp->private_data = (void *)(&hdmi_cec_data);
    hdmi_cec_data.Logical_address = 15;
    hdmi_cec_data.cec_state = false;
    mutex_unlock(&hdmi_cec_data.lock);
    return 0;
}

static ssize_t hdmi_cec_read(struct file *file, char __user *buf, size_t count,
                loff_t *ppos)
{
    struct hdmi_cec_event *event = NULL;

    if (!open_count) {
        printk(KERN_INFO "hdmi_cec_read ENODEV\n");
        return -ENODEV;
    }
    mutex_lock(&hdmi_cec_data.lock);
    if (false == hdmi_cec_data.cec_state) {
        mutex_unlock(&hdmi_cec_data.lock);
        printk(KERN_INFO "hdmi_cec_read EACCES\n");
        return -EACCES;
    }

    if (list_empty(&head)) {
        if (file->f_flags & O_NONBLOCK) {
            mutex_unlock(&hdmi_cec_data.lock);
            printk(KERN_INFO "hdmi_cec_read EAGAIN\n");
            return -EAGAIN;
        } else {
            do {
                mutex_unlock(&hdmi_cec_data.lock);
                if (wait_event_interruptible(hdmi_cec_queue, (!list_empty(&head)))) {
                    printk(KERN_INFO "hdmi_cec_read ERESTARTSYS\n");
                    return -ERESTARTSYS;
                }
                mutex_lock(&hdmi_cec_data.lock);
            } while (list_empty(&head));
        }
    }

    event = list_first_entry(&head, struct hdmi_cec_event, list);
    list_del(&event->list);
    mutex_unlock(&hdmi_cec_data.lock);
    if (copy_to_user(buf, event,
             sizeof(struct hdmi_cec_event) - sizeof(struct list_head))) {
        vfree(event);
                    printk(KERN_INFO "hdmi_cec_read EFAULT\n");
        return -EFAULT;
    }
    vfree(event);
    return (sizeof(struct hdmi_cec_event) - sizeof(struct list_head));
}

static ssize_t hdmi_cec_write(struct file *file, const char __user *buf,
                 size_t count, loff_t *ppos)
{
    int ret = 0 , i = 0;
    u8 msg[MAX_MESSAGE_LEN];
    u8 msg_len = 0, val = 0;

    if (!open_count) {
        printk(KERN_INFO "hdmi_cec_read write ENODEV\n");
        return -ENODEV;
    }
    mutex_lock(&hdmi_cec_data.lock);
    if (false == hdmi_cec_data.cec_state) {
        mutex_unlock(&hdmi_cec_data.lock);
        pr_err("[CEC]hdmi_cec_write EACCES: %d\n", hdmi_cec_data.cec_state);
        return -EACCES;
    }
    /* Ensure that there is only one writer who is the only listener of tx_cec_queue */
    if (hdmi_cec_data.tx_answer != CEC_TX_AVAIL) {
        mutex_unlock(&hdmi_cec_data.lock);
        pr_err("[CEC]hdmi_cec_write EBUSY: %d\n", hdmi_cec_data.tx_answer);
        return -EBUSY;
    }
    mutex_unlock(&hdmi_cec_data.lock);
    if (count > MAX_MESSAGE_LEN) {
        pr_err("[CEC]hdmi_cec_write EINVAL.\n");
        return -EINVAL;
    }
    memset(&msg, 0, MAX_MESSAGE_LEN);
    ret = copy_from_user(&msg, buf, count);
    if (ret) {
        pr_err("[CEC]hdmi_cec_write EACCES (copy_from_user).\n");
        return -EACCES;
    }
    if ((cur_in_usecs - last_signal_tick) < 5000 && last_signal > 0) {
        pr_err("[CEC]No free cec line detected.\n");
        ret = -EBUSY;
        goto tx_out;
    }

    mutex_lock(&hdmi_cec_data.lock);
    hdmi_cec_data.tx_answer = CEC_TX_INPROGRESS;
    hdmi_cec_data.sent_error = 0;
    msg_len = count;
    hdmi_writeb(msg_len, HDMI_CEC_TX_CNT);
    for (i = 0; i < msg_len; i++) {
        hdmi_writeb(msg[i], tx_reg[i]);
    }

    /* Enable Sending */
    hdmi_writel(0x00, 0x1003c);

    val = hdmi_readb(HDMI_CEC_CTRL);
    val |= 0x01;
    hdmi_writeb(val, HDMI_CEC_CTRL);
    memcpy(hdmi_cec_data.last_msg, msg, msg_len);
    hdmi_cec_data.msg_len = msg_len;

    mutex_unlock(&hdmi_cec_data.lock);

    ret = wait_event_interruptible_timeout(tx_cec_queue, hdmi_cec_data.tx_answer != CEC_TX_INPROGRESS, HZ);

    if (ret < 0) {
        ret = -ERESTARTSYS;
    } else if (hdmi_cec_data.tx_answer & HDMI_IH_CEC_STAT0_DONE) {
        /* msg correctly sent */
        ret = msg_len;
    } else {
        ret = -EIO;
    }

tx_out:
    hdmi_cec_data.tx_answer = CEC_TX_AVAIL;
    pr_err("[CEC]hdmi_cec_write length: %d, ret: %d.\n", msg_len, ret);
    return ret;
}

static void hdmi_stop_device(void)
{
    u8 val;

    hdmi_writeb(0x10, HDMI_CEC_CTRL);
    val = HDMI_IH_CEC_STAT0_WAKEUP | HDMI_IH_CEC_STAT0_ERROR_FOLL | HDMI_IH_CEC_STAT0_ERROR_INIT | HDMI_IH_CEC_STAT0_ARB_LOST | \
            HDMI_IH_CEC_STAT0_NACK | HDMI_IH_CEC_STAT0_EOM | HDMI_IH_CEC_STAT0_DONE;
    hdmi_writeb(val, HDMI_CEC_MASK);
    hdmi_writeb(val, HDMI_IH_MUTE_CEC_STAT0);
    hdmi_writeb(0x0, HDMI_CEC_POLARITY);
    val = hdmi_readb(HDMI_MC_CLKDIS);
    val |= HDMI_MC_CLKDIS_CECCLK_DISABLE;
    hdmi_writeb(val, HDMI_MC_CLKDIS);
    mutex_lock(&hdmi_cec_data.lock);
    hdmi_cec_data.cec_state = false;
    mutex_unlock(&hdmi_cec_data.lock);
    if (rxack_task != NULL){
        kthread_stop(rxack_task);
        rxack_task = NULL;
    }
}

/*!
 * @brief IO ctrl function for cec file operation
 * @param cmd IO ctrl command
 * @return  0 on success or negative error code on error
 */
static long hdmi_cec_ioctl(struct file *filp, u_int cmd,
             u_long arg)
{
    int ret = 0, status = 0;
    u8 val = 0;
    if (!open_count)
        return -ENODEV;
    switch (cmd) {
    case HDMICEC_IOC_SETLOGICALADDRESS:
        mutex_lock(&hdmi_cec_data.lock);
        if (false == hdmi_cec_data.cec_state) {
            mutex_unlock(&hdmi_cec_data.lock);
            pr_err("[CEC]Trying to set logical address while not started.\n");
            return -EACCES;
        }
        hdmi_cec_data.Logical_address = (u8)arg;
        if (hdmi_cec_data.Logical_address <= 7) {
            cec_l_addr_l = 1 << hdmi_cec_data.Logical_address;
            cec_l_addr_h = 0;
        } else if (hdmi_cec_data.Logical_address > 7 && hdmi_cec_data.Logical_address <= 15) {
            cec_l_addr_l = 0;
            cec_l_addr_h = 1 << (hdmi_cec_data.Logical_address - 8);
        } else{
            mutex_unlock(&hdmi_cec_data.lock);
            ret = -EINVAL;
            break;
        }
        hdmi_writeb(cec_l_addr_l, HDMI_CEC_ADDR_L);
        hdmi_writeb(cec_l_addr_h, HDMI_CEC_ADDR_H);
        mutex_unlock(&hdmi_cec_data.lock);
        break;
    case HDMICEC_IOC_STARTDEVICE:
        val = hdmi_readb(HDMI_MC_CLKDIS);
        val &= ~HDMI_MC_CLKDIS_CECCLK_DISABLE;
        hdmi_writeb(val, HDMI_MC_CLKDIS);

        hdmi_writeb(0x02, HDMI_CEC_CTRL);
        /* enable CEC receive */
        hdmi_writel(0x84, 0x1003c);
        /* Force read unlock */
        hdmi_writeb(0x0, HDMI_CEC_LOCK);
        val = HDMI_IH_CEC_STAT0_ERROR_INIT | HDMI_IH_CEC_STAT0_NACK | HDMI_IH_CEC_STAT0_EOM | HDMI_IH_CEC_STAT0_DONE;
        hdmi_writeb(val, HDMI_CEC_POLARITY);
        val = HDMI_IH_CEC_STAT0_WAKEUP | HDMI_IH_CEC_STAT0_ERROR_FOLL | HDMI_IH_CEC_STAT0_ARB_LOST;
        hdmi_writeb(val, HDMI_CEC_MASK);
        hdmi_writeb(val, HDMI_IH_MUTE_CEC_STAT0);
        hdmi_cec_data.link_status = hdmi_readb(HDMI_PHY_STAT0) & 0x02;
        mutex_lock(&hdmi_cec_data.lock);
        hdmi_cec_data.cec_state = true;
        mutex_unlock(&hdmi_cec_data.lock);
        if (rxack_task == NULL)
           rxack_task = kthread_run(rxack_thread,NULL,"cec_rxack");
        break;
    case HDMICEC_IOC_STOPDEVICE:
        hdmi_stop_device();
        break;
    case HDMICEC_IOC_GETPHYADDRESS:
        status = copy_to_user((void __user *)arg,
                     &cec_phy_addr,
                     sizeof(u32));
        if (status)
            ret = -EFAULT;
        break;
    default:
        ret = -EINVAL;
        break;
    }
    return ret;
}

/*!
 * @brief Release function for cec file operation
 * @return  0 on success or negative error code on error
 */
static int hdmi_cec_release(struct inode *inode, struct file *filp)
{
    mutex_lock(&hdmi_cec_data.lock);
    if (open_count) {
        open_count = 0;
        hdmi_cec_data.cec_state = false;
        hdmi_cec_data.Logical_address = 15;
    }
    mutex_unlock(&hdmi_cec_data.lock);

    return 0;
}

static unsigned int hdmi_cec_poll(struct file *file, poll_table *wait)
{
    unsigned int mask = 0;

    poll_wait(file, &hdmi_cec_queue, wait);

    /* Always writable */
    mask =  (POLLOUT | POLLWRNORM);
    mutex_lock(&hdmi_cec_data.lock);
    if (!list_empty(&head))
            mask |= (POLLIN | POLLRDNORM);
    mutex_unlock(&hdmi_cec_data.lock);
    return mask;
}


const struct file_operations hdmi_cec_fops = {
    .owner = THIS_MODULE,
    .read = hdmi_cec_read,
    .write = hdmi_cec_write,
    .open = hdmi_cec_open,
    .unlocked_ioctl = hdmi_cec_ioctl,
    .release = hdmi_cec_release,
    .poll = hdmi_cec_poll,
};

static int __init hdmi_cec_init(void)
{
    int err = 0;
    /* FIXME : get IRQ from resource */
    u32 irqhdmi = 120;
    struct device *temp_class;

    if(!hdmi_base_addr) {
        pr_err("hdmi_cec: unable to find hdmi_base_addr\n");
        err = -EBUSY;
        goto out;
    }

    printk(KERN_INFO "HDMI CEC base address: %p\n", hdmi_base_addr);

    hdmi_writeb(0xFF, HDMI_IH_MUTE);
    hdmi_writeb(0xFF, HDMI_PHY_MASK0);
    hdmi_writeb(0xFF, HDMI_IH_MUTE_PHY_STAT0);
    hdmi_writeb(0xFF, HDMI_IH_MUTE_I2CM_STAT0);
    hdmi_writeb(0x00, HDMI_IH_MUTE);
    hdmi_writeb(0xFF, HDMI_CEC_ADDR_L);
    hdmi_writeb(0xFF, HDMI_CEC_ADDR_H);

    printk(KERN_INFO "HDMI CEC registering chrdev\n");

    hdmi_cec_major = register_chrdev(hdmi_cec_major, "sunxi_hdmi_cec", &hdmi_cec_fops);
    if (hdmi_cec_major < 0) {
        pr_err("hdmi_cec: unable to get a major for HDMI CEC\n");
        err = -EBUSY;
        goto out;
    }

    printk(KERN_INFO "HDMI CEC request IRQ\n");

    spin_lock_init(&hdmi_cec_data.irq_lock);
    hdmi_cec_data.cec_irq = irqhdmi;

    err = request_irq(hdmi_cec_data.cec_irq, hdmi_cec_isr, IRQF_SHARED,
              "sunxi_hdmi_cec", &hdmi_cec_data);
    if (err < 0) {
        pr_err("hdmi_cec:Unable to request irq: %d\n", err);
        goto err_out_chrdev;
    }

    printk(KERN_INFO "HDMI CEC create sunxi_hdmi_cec\n");

    hdmi_cec_class = class_create(THIS_MODULE, "sunxi_hdmi_cec");
    if (IS_ERR(hdmi_cec_class)) {
        err = PTR_ERR(hdmi_cec_class);
        goto err_out_chrdev;
    }

    printk(KERN_INFO "HDMI CEC device_create\n");

    temp_class = device_create(hdmi_cec_class, NULL, MKDEV(hdmi_cec_major, 0),
                   NULL, "sunxi_hdmi_cec");
    if (IS_ERR(temp_class)) {
        err = PTR_ERR(temp_class);
        goto err_out_class;
    }

    init_waitqueue_head(&hdmi_cec_queue);
    init_waitqueue_head(&tx_cec_queue);

    INIT_LIST_HEAD(&head);

    mutex_init(&hdmi_cec_data.lock);
    hdmi_cec_data.Logical_address = 15;
    hdmi_cec_data.tx_answer = CEC_TX_AVAIL;
    INIT_DELAYED_WORK(&hdmi_cec_data.hdmi_cec_work, hdmi_cec_worker);
    register_sunxi_hdmi_notifier(&sunxi_hdmi_nb);
    printk(KERN_INFO "HDMI CEC initialized\n");

    goto out;

err_out_class:
    device_destroy(hdmi_cec_class, MKDEV(hdmi_cec_major, 0));
    class_destroy(hdmi_cec_class);
err_out_chrdev:
    unregister_chrdev(hdmi_cec_major, "sunxi_hdmi_cec");
out:
    return err;
}

static void __exit hdmi_cec_exit(void)
{
    unregister_sunxi_hdmi_notifier(&sunxi_hdmi_nb);

    if (rxack_task != NULL) {
        kthread_stop(rxack_task);
        rxack_task = NULL;
    }
    if (hdmi_cec_data.cec_irq > 0) {
        free_irq(hdmi_cec_data.cec_irq, &hdmi_cec_data);
    }

    if (hdmi_cec_major > 0) {
        device_destroy(hdmi_cec_class, MKDEV(hdmi_cec_major, 0));
        class_destroy(hdmi_cec_class);
        unregister_chrdev(hdmi_cec_major, "sunxi_hdmi_cec");
        hdmi_cec_major = 0;
    }

    return;
}

MODULE_AUTHOR("Joachim Damm");
MODULE_DESCRIPTION("Linux HDMI CEC driver for Allwiner H3");
MODULE_LICENSE("GPL");

late_initcall(hdmi_cec_init);
module_exit(hdmi_cec_exit);
