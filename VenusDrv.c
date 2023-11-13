/* NITGEN USB Fingkey Hamster (hfdu04) driver  - 1.0.4
 * Copyright(C) 2012~2015, NITGEN&COMPANY Co., Ltd.
 * History:
 * 24/10/2007 : first release
 * 01/12/2010 : Modified for NorthStar OEM
 * 22/03/2012 : Modified i/o control macro
 * 07/08/2012 : Modified time out
 * 22/01/2015 : supports for Ubuntu-14.04(Kernel v3.13.0) by jphwang
 * 23/03/2015 : supports for Ubuntu-14.04(Kernel v3.16.0) by jphwang
 *              deprecated function 'interruptible_sleep_on' replaced
 *              'wait_event_interruptible'.
 * 14/02/2018 : supports for Ubuntu-17.10(Kernel v4.13.0) by jphwang
 * 06/08/2018 : supports for Ubuntu-18.04(Kernel v4.15.0) by jphwang
 * 22/10/2023 : support for linux kernel higher or equal than v.5.19 by Lucas Andreatta
 *
 */

/* driver  include files ***********************************************************/
#include <linux/module.h> /* dynamic loading of modules */
#include <linux/kernel.h> /* dynamic loading of modules */
#include <linux/init.h>   /*_init _exit */
#include <linux/slab.h>   /* kmalloc () .. */
#include <linux/errno.h>  /* err numer */
#include <linux/version.h>
#include <linux/list.h> /* link list impl */
#include <linux/wait.h> /* interruptible_sleep_on */
#include <linux/workqueue.h>
#include <linux/delay.h>
#include <linux/completion.h>

#if LINUX_VERSION_CODE > KERNEL_VERSION(2, 6, 35)
#include <linux/semaphore.h>
#elif LINUX_VERSION_CODE > KERNEL_VERSION(2, 6, 16)
#include <linux/mutex.h>
#endif

#include <linux/sched.h> /* v1.0.4-5.1 */
#include <linux/vmalloc.h>
#include <asm/uaccess.h> /* user access from kernel space copy_to_user */
#include <asm/atomic.h>  /* atomic_inc() and .. */

#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 10)
#include <linux/smp_lock.h> /* spin_lock_..  */
#else
#include <linux/spinlock.h>
#endif

#include <linux/usb.h> /* USB  APIs */
#include <linux/sched.h>
#include <linux/err.h>
#include <linux/slab.h>
#include <linux/fs.h> /* file system  functions  */

/* v1.0.4-5 */
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 12, 0) /* for signal_pending() api */
#include <linux/uaccess.h>                         /* for copy_to_user/copy_from_user() api */
#endif
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 11, 0) /* for signal_pending() api */
#include <linux/sched/signal.h>                    /* for signal_pending() api */
#endif

#include "VenusDrv.h"

/* driver  define ******************************************************************/
#ifndef __VERSION_INCLUDED
#define __VERSION_INCLUDED

#define MODULE_MAJOR_VERSION 0x01
#define MODULE_MINOR_VERSION 0x00
#define MODULE_RELEASE_VERSION 0x03
#define MODULE_BUILD_VERSION 0x0A

#define MODULE_RELEASE_STRING "1.0.4-5.1"

#endif

#define CONFIG_USB_DEBUG

#ifdef CONFIG_USB_DEBUG
#define DEBUG
int debug = 1;
#else
int debug = 0;
#endif

#define MSG_TIME_OUT 10 * 1000 // 10 sec

/* Use our own dbg macro */
#undef dbg
#define dbg(format, arg...)                                      \
    do                                                           \
    {                                                            \
        if (debug)                                               \
            printk(KERN_DEBUG __FILE__ ": " format "\n", ##arg); \
    } while (0)

/* Driver  Information */
#define DRIVER_VERSION "v1.0.4-5.1"
#define DRIVER_AUTHOR "NITGEN Hardware Development Team"
#define DRIVER_DESC "NITGEN USB FDU01/04/06 Fingkey Hamster Driver"

/* Define these values to match your device */
#define USB_HFDU04_VENDOR_ID 0x0A86
#define USB_HFDU04_PRODUCT_ID 0x0100

/* We can have up to this number of device plugged in at once */
#define HFDU04_MAX_DEVICES 8

#define _ISOPIPESIZE 4200
#define ANCHOR_LOAD_INTERNAL 0xA0 /* Vender specific request code for Anchor upload/Download */

#define VENDOR_REQUEST_OUT 0x40

#define CPUCS_REG 0x7F92 /* EZ-USB Control and Status Register.  Bit 0 controls 8051 reset */

#define E2PROM_SIZE 15 * 1024
#define HFDU06_MAX_PACKET_SIZE 512 * 782

#define MAX_READ_DATA (240) * 512

#if LINUX_VERSION_CODE > KERNEL_VERSION(2, 6, 16)
static struct workqueue_struct *comm_queue;
#endif

// O1 Ver //////////////////
#define MAX_INTEL_HEX_RECORD_LENGTH 16

/* Get a minor range for your devices from the usb maintainer
 *  still we need  free minor numbers
 */
#define USB_HFDU04_MINOR_BASE 230

/* prevent races between  open() disconnect() read() */
#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 36)
static DECLARE_MUTEX(disconnect_sem);
#else
static DEFINE_SEMAPHORE(disconnect_sem);
#endif

/* driver  structure ***************************************************************/
/* table of devices that work with this driver */
static struct usb_device_id hfdu04_device_ids[] = {
    {USB_DEVICE(USB_HFDU04_VENDOR_ID, USB_HFDU04_PRODUCT_ID)},
    {},
    /* Terminating entry */
};

typedef enum
{
    _stopped = 0,
    _started
} driver_state_t;

MODULE_DEVICE_TABLE(usb, hfdu04_device_ids);

struct hfdu04_data
{
    struct usb_device *udev;
#if LINUX_VERSION_CODE > KERNEL_VERSION(2, 6, 16)
    struct usb_interface *interface;
#endif

    driver_state_t state; /* State of the device */
    unsigned char minor;  /* Scanner minor - used in disconnect() */
    __u8 present;         /* Not zero if device is present */
    __u8 opencount;       /* Not zero if device is opened */

    unsigned char *bulk_in_buffer;  /* the buffer to receive data */
    unsigned char *bulk_out_buffer; /* the buffer to send data */

    size_t bulk_out_size; /* the maximum bulk packet size */
    size_t bulk_in_size;  /* the maximum bulk packet size */
    size_t orig_bi_size;  /* same as above, but reported by the device */

    __u8 bulk_in_endpointAddr;         /* Address of bulkin endpoint */
    __u8 bulk_in_endpointAddr_image;   /* Address of bulkin endpoint for image*/
    __u8 bulk_in_endpointAddr_fw;      /* Address of bulkin endpoint for firmware*/
    size_t bulk_out_buffer_used;       /* the quantity of buffered data to send */
    __u8 bulk_out_endpointAddr;        /* Address of bulkout endpoint */
    __u8 bulk_out_endpointAddr_normal; /* Address of bulkout endpoint */
    __u8 bulk_out_endpointAddr_fw;     /* Address of bulkout endpoint */

    __u8 iso_in_endpointAddr;        /* Address of iso in endpoint */
    atomic_t pending_io;             /* Pending i/o transaction  */
    unsigned int readptr;            /* Read pointer */
    wait_queue_head_t wait;          /* urb wait  */
    wait_queue_head_t wait1;         /* urb wait  */
    spinlock_t lock;                 /* lock  list  */
    struct semaphore sem;            /* Lock to prevent concurrent reads or writes  */
    struct list_head free_buff_list; /* Isoc free buffer list  */
    struct list_head rec_buff_list;  /* Isoc rec buffer list  */
    unsigned int got_mem;
    unsigned int overruns;

    unsigned int bcdDevice;
    unsigned int maxbulktransfersize;

    bool IsOpened;
    bool IsConnected;

    unsigned int idVendor;
    unsigned int idProduct;
};

/* driver  specific functions ******************************************************/
static int hfdu04_freequeue(struct list_head *i);
static int hfdu04_freebuffers(struct hfdu04_data *);
static void hfdu04_disconnect(struct usb_interface *);
static int hfdu04_open(struct inode *, struct file *);

static int hfdu04_allocbuffers(struct hfdu04_data *);
static int hfdu04_release(struct inode *, struct file *);
static int hfdu04_stopgrabbingdata(struct hfdu04_data *);
static int hfdu04_startgrabbingdata(struct hfdu04_data *);

static void hfdu04_isocomplete(struct urb *, struct pt_regs *);
static ssize_t hfdu04_read(struct file *, char *, size_t, loff_t *);
static ssize_t hfdu04_write(struct file *, const char *, size_t, loff_t *);
static int hfdu04_cancelqueue(struct hfdu04_data *, struct list_head *);

static int hfdu04_probe(struct usb_interface *, const struct usb_device_id *);

#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 36)
static int hfdu04_ioctl(struct inode *, struct file *, unsigned int, unsigned long);
#else
#if 1 /* jphwang++(2015.01.21) */
static long hfdu04_ioctl(struct file *, unsigned int, unsigned long);
#else
static int hfdu04_ioctl(struct file *, unsigned int, unsigned long);
#endif
#endif

static int hfdu04_addbuffertail(struct hfdu04_data *, struct list_head *, struct list_head *);

static int hfdu04_init_device(struct usb_device *udev);
int hfdu04_user_command_in(struct usb_device *udev, unsigned char *value);
int hfdu04_user_command_out(struct usb_device *udev, unsigned char *value);

/* driver  structure ***************************************************************/
/* intel hex format */
typedef struct _INTEL_HEX_RECORD
{
    __u32 length;
    __u32 address;
    __u32 type;
    __u8 data[MAX_INTEL_HEX_RECORD_LENGTH];
} intel_hex_record, *pintel_hex_record;

/* Iso transfer  */
typedef struct
{
    struct hfdu04_data *dev;    /* Pointer hfdu04_data structure */
    struct urb *purb;           /* Pointer to urb */
    struct list_head buff_list; /* Isoc buffer list */
} buff_t, *pbuff_t;

// #if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,17)
/* Array of pointers to our devices that are currently connected  */
// static struct hfdu04_data *hfdu04_table[HFDU04_MAX_DEVICES] =
//{ NULL, /* ... */  };
// #endif

static struct
    file_operations hfdu04_fops = {
        .owner = THIS_MODULE,
        .read = hfdu04_read,
        .write = hfdu04_write,
        .open = hfdu04_open,
        .release = hfdu04_release,
#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 36)
        .ioctl = hfdu04_ioctl,
#else
        .unlocked_ioctl = hfdu04_ioctl,
#endif
};

/* USB class driver information in order to get a minor number
 * from the usb core the driver core and  file operations .
 */
static struct usb_class_driver hfdu04_class = {
    .name = "nitgen%d",
    .fops = &hfdu04_fops,
#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 16)
    .mode = S_IFCHR | S_IRUSR | S_IWUSR | S_IRGRP | S_IWGRP | S_IROTH,
#endif
    .minor_base = USB_HFDU04_MINOR_BASE,
};

/* USB specific object needed to register this driver with the usb sub-system */
static struct usb_driver hfdu04_driver = {
    .name = "hfdu04",
#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 16)
    .owner = THIS_MODULE,
#endif
    .id_table = hfdu04_device_ids,
    .probe = hfdu04_probe,
    .disconnect = hfdu04_disconnect,
};

#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 17)

#else
/* prevent races between open() and disconnect() */
static DEFINE_MUTEX(disconnect_mutex);
#endif

/* driver  implemented function ****************************************************/

/* Iso Transfers */
/* adds buffer to list  */
static int hfdu04_addbuffertail(struct hfdu04_data *dev,
                                struct list_head *dst,
                                struct list_head *src)
{
    unsigned long flags;
    struct list_head *tmp;
    int ret = 0;

    spin_lock_irqsave(&dev->lock, flags);

    if (list_empty(src))
    {
        /* no elements in source buffer */
        ret = -1;
        goto err;
    }
    tmp = src->next;
    list_move_tail(tmp, dst);

err:
    spin_unlock_irqrestore(&dev->lock, flags);
    return ret;
}

/* dump urb information if needed  */
#ifdef DEBUG
static void dump_urb(struct urb *urb)
{
    dbg("urb                   :%p", urb);
    dbg("dev                   :%p", urb->dev);
    dbg("pipe                  :%08X", urb->pipe);
    dbg("status                :%d", urb->status);
    dbg("transfer_flags        :%08X", urb->transfer_flags);
    dbg("transfer_buffer       :%p", urb->transfer_buffer);
    dbg("transfer_buffer_length:%d", urb->transfer_buffer_length);
    dbg("actual_length         :%d", urb->actual_length);
    dbg("setup_packet          :%p", urb->setup_packet);
    dbg("start_frame           :%d", urb->start_frame);
    dbg("number_of_packets     :%d", urb->number_of_packets);
    dbg("interval              :%d", urb->interval);
    dbg("error_count           :%d", urb->error_count);
    dbg("context               :%p", urb->context);
    dbg("complete              :%p", urb->complete);
}
#endif
/*unlink the queue of urbs */
static int hfdu04_cancelqueue(struct hfdu04_data *dev, struct list_head *q)
{
    if (dev->bcdDevice < 0x2000)
    {
        unsigned long flags;
        struct list_head *p;
        pbuff_t b;

        // dbg("hfdu04_cancelqueue");

        spin_lock_irqsave(&dev->lock, flags);

        for (p = q->next; p != q; p = p->next)
        {
            b = list_entry(p, buff_t, buff_list);

#ifdef DEBUG
            dump_urb(b->purb);
#endif
            usb_unlink_urb(b->purb);
        }
        spin_unlock_irqrestore(&dev->lock, flags);
    }
    return 0;
}

/* free usb list  */
static int hfdu04_freequeue(struct list_head *q)
{
    // if ( bcdDevice < 0x2000)
    {
        struct list_head *tmp;
        struct list_head *p;
        pbuff_t b;

        // dbg("hfdu04_freequeue");
        for (p = q->next; p != q;)
        {
            b = list_entry(p, buff_t, buff_list);

#ifdef DEBUG
            dump_urb(b->purb);
#endif
            if (b->purb->transfer_buffer)
                kfree(b->purb->transfer_buffer);

            usb_free_urb(b->purb);
            tmp = p->next;
            list_del(p);
            kfree(b);
            p = tmp;
        }
    }

    return 0;
}

/* free buffers which are allocated  */
static int hfdu04_freebuffers(struct hfdu04_data *dev)
{
    if (dev->bcdDevice < 0x2000)
    {
        unsigned long flags;
        // dbg("hfdu04_freebuffers");

        spin_lock_irqsave(&dev->lock, flags);

        hfdu04_freequeue(&dev->free_buff_list);
        hfdu04_freequeue(&dev->rec_buff_list);

        spin_unlock_irqrestore(&dev->lock, flags);

        dev->got_mem = 0;
    }
    return 0;
}

/* urb filling call back function for iso read  */
// #define EILSEQ          84      /* Illegal byte sequence */

static void hfdu04_isocomplete(struct urb *purb, struct pt_regs *regs)
{
    pbuff_t b = purb->context;
    struct hfdu04_data *dev = b->dev;
    int i;
    int len;
    int dst = 0;
    unsigned int pipe;
    int pipesize;
    void *buf = purb->transfer_buffer;

    // dbg("hfdu04_isocomplete");
    // printk("#1. hfdu04_isocomplete\n");

    /* process if URB was not killed */
    if (purb->status != -ENOENT)
    {
        pipe = usb_rcvisocpipe(purb->dev, dev->iso_in_endpointAddr);
#if LINUX_VERSION_CODE > KERNEL_VERSION(5, 19, 0)
        pipesize = usb_maxpacket(purb->dev, pipe);
#else
        pipesize = usb_maxpacket(purb->dev, pipe, usb_pipeout(pipe));
#endif
        for (i = 0; i < purb->number_of_packets; i++)
        {
            if (!purb->iso_frame_desc[i].status)
            {
                len = purb->iso_frame_desc[i].actual_length;
                if (len <= pipesize)
                {
                    memcpy(buf + dst, buf + purb->iso_frame_desc[i].offset, len);
                    dst += len;
                }
                else
                {
#if LINUX_VERSION_CODE > KERNEL_VERSION(3, 3, 0)
                    printk("%s: invalid len %d", __FUNCTION__, len);
#else
                    err("%s: invalid len %d", __FUNCTION__, len);
#endif
                }
            }
            else
            {
                // warn("%s: corrupted packet status: %d", __FUNCTION__, purb->iso_frame_desc[i].status);
            }
        }

        if (dst != purb->actual_length)
        {
#if LINUX_VERSION_CODE > KERNEL_VERSION(3, 3, 0)
            printk("dst != purb->actual_length: %d! = %d", dst, purb->actual_length);
#else
            err("dst != purb->actual_length: %d! = %d", dst, purb->actual_length);
#endif
            return;
        }
    }

    if (atomic_dec_and_test(&dev->pending_io) && (dev->state != _stopped))
    {
        dev->overruns++;
        // err("overrun (%d)", dev->overruns);
    }

    // printk("#2. hfdu04_isocomplete\n");
    wake_up(&dev->wait);
    // wake_up(&dev->wait1);
}

/* allocate memory for buffers  Only 01 Ver */
static int hfdu04_allocbuffers(struct hfdu04_data *dev)
{
    if (dev->bcdDevice < 0x2000)
    {
        unsigned int buffers = 0;
        pbuff_t b = NULL;
        unsigned int pipe;
        int pipesize, packets;
        unsigned int transfer_buffer_length, totalbuffersize;
        int i;

        pipe = usb_rcvisocpipe(dev->udev, dev->iso_in_endpointAddr);

#if LINUX_VERSION_CODE > KERNEL_VERSION(5, 19, 0)
        pipesize = usb_maxpacket(dev->udev, pipe);
#else
        pipesize = usb_maxpacket(dev->udev, pipe, usb_pipeout(pipe));
#endif

        packets = _ISOPIPESIZE / pipesize;

        transfer_buffer_length = packets * pipesize;

        totalbuffersize = 840 * 280;

        while (buffers < totalbuffersize)
        {
            b = (pbuff_t)kmalloc(sizeof(buff_t), GFP_KERNEL);
            if (!b)
            {
#if LINUX_VERSION_CODE > KERNEL_VERSION(3, 3, 0)
                printk("kmalloc( sizeof( buff_t ) ) == NULL");
#else
                err("kmalloc( sizeof( buff_t ) ) == NULL");
#endif
                goto err;
            }

            memset(b, 0, sizeof(buff_t));
            b->dev = dev;
            b->purb = usb_alloc_urb(packets, GFP_KERNEL);
            if (!b->purb)
            {
#if LINUX_VERSION_CODE > KERNEL_VERSION(3, 3, 0)
                printk("usb_alloc_urb == NULL");
#else
                err("usb_alloc_urb == NULL");
#endif
                kfree(b);
                goto err;
            }

            b->purb->transfer_buffer = kmalloc(transfer_buffer_length, GFP_KERNEL);
            if (!b->purb->transfer_buffer)
            {
                // usb_free_urb(b->purb->transfer_buffer);
                kfree(b->purb);
                kfree(b);
#if LINUX_VERSION_CODE > KERNEL_VERSION(3, 3, 0)
                printk("transfre_buffer_length kmalloc( %d ) == NULL", transfer_buffer_length);
#else
                err("transfre_buffer_length kmalloc( %d ) == NULL", transfer_buffer_length);
#endif
                goto err;
            }

            b->purb->transfer_buffer_length = transfer_buffer_length;
            b->purb->number_of_packets = packets;
            b->purb->complete = (usb_complete_t)hfdu04_isocomplete;
            b->purb->context = b;
            b->purb->interval = 1;
            b->purb->dev = dev->udev;
            b->purb->pipe = pipe;
            b->purb->transfer_flags = URB_ISO_ASAP;

            for (i = 0; i < packets; i++)
            {
                b->purb->iso_frame_desc[i].offset = i * pipesize;
                b->purb->iso_frame_desc[i].length = pipesize;
            }

            buffers += transfer_buffer_length;
            list_add(&b->buff_list, &dev->free_buff_list);
        }
        dev->got_mem = buffers;
        return 0;

    err:
        hfdu04_freebuffers(dev);
        return -ENOMEM;
    }
    else
    {
        return 1;
    }
}

/* stops catching data  */
static int hfdu04_stopgrabbingdata(struct hfdu04_data *dev)
{
    dev->state = _stopped;
    hfdu04_cancelqueue(dev, &dev->rec_buff_list);
    dev->pending_io.counter = 0;

    return 0;
}

/* starts catching data */
static int hfdu04_startgrabbingdata(struct hfdu04_data *dev)
{
    if (!dev->got_mem && dev->state != _started)
    {
        if (hfdu04_allocbuffers(dev) < 0)
            return -ENOMEM;

        hfdu04_stopgrabbingdata(dev);
        dev->state = _started;
        dev->readptr = 0;
    }

    if (!list_empty(&dev->free_buff_list))
    {
        pbuff_t end;
        int ret;

        while (!hfdu04_addbuffertail(dev, &dev->rec_buff_list, &dev->free_buff_list))
        {
            end = list_entry(dev->rec_buff_list.prev, buff_t, buff_list);
            ret = usb_submit_urb(end->purb, GFP_KERNEL);
            // msleep(1);
            if (ret != 0)
            {
#if LINUX_VERSION_CODE > KERNEL_VERSION(3, 3, 0)
                printk("usb_submit_urb returned: %d", ret);
#else
                err("usb_submit_urb returned: %d", ret);
#endif
                if (hfdu04_addbuffertail(dev, &dev->free_buff_list, &dev->rec_buff_list))
#if LINUX_VERSION_CODE > KERNEL_VERSION(3, 3, 0)
                    printk("startgrabbing: hfdu04_addbuffer tail failed");
#else
                    err("startgrabbing: hfdu04_addbuffer tail failed");
#endif
                break;
            }
            else
            {
                atomic_inc(&dev->pending_io);
            }
        }
        // udelay(50000);
        // msleep(5);
    }

    return 0;
}

/* read call we implemented ISOC read here */
static ssize_t hfdu04_read(struct file *file, char *buf, size_t count, loff_t *ppos)
{
    struct hfdu04_data *dev = (struct hfdu04_data *)file->private_data;
    // unsigned ret = 0;
    int ret = 0;
    int rem;
    int cnt;
    struct urb *purb = NULL;
    int user_result;
    int actual_length;

    // int while_count = 0;
    int remain_count = 0;
    char *u_buf = buf;
    int bulk_ret;
    int bulk_size;
    int bulk_read_size;
    // int incount = 0;
    // int loopcount = 0;

    // printk("%s : %d\n", __FUNCTION__, __LINE__);
    //  Only 01 Ver bcdDevice < 0x2000

    if (dev->IsOpened == false)
    {
        // printk(" %s : %d - disconnected or not opened \n", __FUNCTION__, __LINE__);
        return -EIO;
    }

    if (*ppos)
        return -ESPIPE;

    if (!dev->udev)
        return -EIO;

        // printk("%s : %d\n", __FUNCTION__, __LINE__);
#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 17)
    /* prevent disconnections */
    down(&disconnect_sem);
#else
    /* prevent disconnects */
    mutex_lock(&disconnect_mutex);
#endif
    down(&dev->sem);

    // printk("%s : %d\n", __FUNCTION__, __LINE__);
    if (dev->bcdDevice >= 0x4000)
    {
#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 23)
        // ret = -EFAULT;
        // goto err;

        remain_count = count;
        bulk_read_size = 32 * PAGE_SIZE;
#else
        remain_count = count;
        bulk_read_size = count; //(dev->maxbulktransfersize)/8;
#endif
        if (dev->bulk_in_endpointAddr == dev->bulk_in_endpointAddr_fw)
            bulk_read_size = dev->maxbulktransfersize;

        while (remain_count > 0)
        {
            bulk_size = min_t(uint, remain_count, bulk_read_size /*dev->maxbulktransfersize*/);
            // ��ũ����� �̿��Ͽ� �����͸� �д´�.
            bulk_ret = usb_bulk_msg(dev->udev,
                                    usb_rcvbulkpipe(dev->udev, dev->bulk_in_endpointAddr),
                                    dev->bulk_in_buffer,
                                    bulk_size, // test: 512, 256, 128, 64
                                    &actual_length,
                                    MSG_TIME_OUT);

            remain_count = remain_count - actual_length;

            // ���� �����͸� copy_to_user �Լ��� �̿��Ͽ� ���ø����̼����� �ѱ��.
            user_result = copy_to_user(u_buf, dev->bulk_in_buffer, actual_length);
            u_buf += actual_length;

            if (bulk_ret != 0)
            {
                printk("bulk error!!! : %d\n", bulk_ret);
                break;
            }
        }
        ret = u_buf - buf;
        // printk("%s : %d ret: %d\n", __FUNCTION__, __LINE__, ret);
    }
    else if (dev->bcdDevice >= 0x2000)
    {
        remain_count = count;
        // printk("%s : %d - remain_count: %d \n", __FUNCTION__, __LINE__, remain_count);
        while (remain_count > 0)
        {
            bulk_size = min_t(uint, remain_count, dev->maxbulktransfersize);
            // ��ũ����� �̿��Ͽ� �����͸� �д´�.
            bulk_ret = usb_bulk_msg(dev->udev,
                                    usb_rcvbulkpipe(dev->udev, dev->bulk_in_endpointAddr),
                                    dev->bulk_in_buffer,
                                    bulk_size, // test: 512, 256, 128, 64
                                    &actual_length,
                                    MSG_TIME_OUT);

            remain_count = remain_count - actual_length;
            // printk("%s : %d remain_count - %ld, actual_length - %d\n", __FUNCTION__, __LINE__, remain_count, actual_length);

            // ���� �����͸� copy_to_user �Լ��� �̿��Ͽ� ���ø����̼����� �ѱ��.
            user_result = copy_to_user(u_buf, dev->bulk_in_buffer, actual_length);
            u_buf += actual_length;

            if (bulk_ret != 0)
            {
                printk("bulk error!!! : %d\n", bulk_ret);
                break;
            }
        }
        ret = u_buf - buf;
        // printk("%s : %d ret: %d\n", __FUNCTION__, __LINE__, ret);
    }
    else if (dev->bcdDevice < 0x2000)
    {
        unsigned long flags;
        pbuff_t b;

        while (count > 0)
        {
            // 01 �������� ����ϴ� ISO ����� �̿��� ������ ���� ���.
            // ISO ����� ��� hfdu04_startgrabbingdata �Լ��� �ִ� hfdu04_allocbuffers�� �̿��Ͽ�, URB �ʱ�ȭ �ϰ�,
            // �ʱ�ȭ�� URB�� usb_submit_urb �Լ��� �̿��Ͽ� URB�� USB �ھ������ �Ѱ��ְ� �ȴ�.
            // usb_submit_urb �� ���������� ������ �Ǹ�, URB�� �ʱ�ȭ �Ҷ� ����� callback �Լ� ( hfdu04_isocomplete )�� ȣ���ϰ� �ȴ�.
            // �� hfdu04_isocomplete �Լ����� �����͸� copy_to_user �Լ��� �̿��Ͽ� ���ø����̼����� �ѱ��.

            hfdu04_startgrabbingdata(dev);

            spin_lock_irqsave(&dev->lock, flags);
            if (list_empty(&dev->rec_buff_list))
            {
                spin_unlock_irqrestore(&dev->lock, flags);
#if LINUX_VERSION_CODE > KERNEL_VERSION(3, 3, 0)
                printk("%s error: rec_buf_list is empty %d ", __FUNCTION__, __LINE__);
#else
                err("%s error: rec_buf_list is empty %d ", __FUNCTION__, __LINE__);
#endif
                goto err;
            }

            b = list_entry(dev->rec_buff_list.next, buff_t, buff_list);
            purb = b->purb;

            spin_unlock_irqrestore(&dev->lock, flags);

            if (purb->status == -EINPROGRESS)
            {
                if (file->f_flags & O_NONBLOCK) // return nonblocking
                {
                    if (!ret)
                    {
                        ret = -EAGAIN;
                    }
                    goto err;
                }

#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 13, 0)
                /* jphwang++(2015.03.23) 'interruptible_sleep_on()' is deprecated. replace to 'wait_event_interruptible()' */
                // waits for the data
                wait_event_interruptible(dev->wait, 0 /* force condition is false */);
#else /* org */
                // waits for the data
                interruptible_sleep_on(&dev->wait);
#endif
                if (signal_pending(current))
                {
                    if (!ret)
                    {
                        ret = -ERESTARTSYS;
                    }
                    goto err;
                }

                spin_lock_irqsave(&dev->lock, flags);

                if (list_empty(&dev->rec_buff_list))
                {
                    spin_unlock_irqrestore(&dev->lock, flags);
#if LINUX_VERSION_CODE > KERNEL_VERSION(3, 3, 0)
                    printk("%s error: still no buffer available.", __FUNCTION__);
#else
                    err("%s error: still no buffer available.", __FUNCTION__);
#endif
                    goto err;
                }
                spin_unlock_irqrestore(&dev->lock, flags);
                dev->readptr = 0;
            }

            rem = purb->actual_length - dev->readptr; // set remaining bytes to copy

            if (count >= rem)
            {
                cnt = rem;
            }
            else
            {
                cnt = count;
            }

            // dbg("copy_to_user:%p %p %d", buf, purb->transfer_buffer + dev->readptr, cnt);
            // printk("copy_to_user:  buf:%p (purb->transfer_buffer + dev->readptr):%p cnt:%d\n", buf, purb->transfer_buffer + dev->readptr, cnt);
            if (copy_to_user(buf, (purb->transfer_buffer + dev->readptr), cnt))
            {
                if (!ret)
                    ret = -EFAULT;
                goto err;
            }

            dev->readptr += cnt;
            count -= cnt;
            buf += cnt;
            ret += cnt;

            if (dev->readptr == purb->actual_length)
            {
                // finished, take next buffer
                if (hfdu04_addbuffertail(dev, &dev->free_buff_list, &dev->rec_buff_list))
                {
#if LINUX_VERSION_CODE > KERNEL_VERSION(3, 3, 0)
                    printk("read: hfdu04_addbuffertail failed");
#else
                    err("read: hfdu04_addbuffertail failed");
#endif
                }
                dev->readptr = 0;
            }
        }
    }

err:
#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 17)
    up(&disconnect_sem);
#else
    mutex_unlock(&disconnect_mutex);
#endif
    up(&dev->sem);

    return ret;
}

/* write call we implemented ISOC write here */
static ssize_t hfdu04_write(struct file *file, const char *buf, size_t count, loff_t *ppos)
{
    struct hfdu04_data *dev = (struct hfdu04_data *)file->private_data;
    int ret = 0;
    int user_result;
    int actual_length;

    int remain_count = 0;
    const char *u_buf = buf;
    int bulk_ret;
    int bulk_size;
    int bulk_write_size;
    size_t bulk_write_total;

    bulk_write_total = 0;

    // printk("%s : %d\n", __FUNCTION__, __LINE__);

    if (dev->IsOpened == false)
    {
        // printk(" %s : %d - disconnected or not opened \n", __FUNCTION__, __LINE__);
        return -EIO;
    }

    if (*ppos)
        return -ESPIPE;

    if (!dev->udev)
        return -EIO;

        // printk("%s : %d\n", __FUNCTION__, __LINE__);
#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 17)
    /* prevent disconnections */
    down(&disconnect_sem);
#else
    /* prevent disconnects */
    mutex_lock(&disconnect_mutex);
#endif
    down(&dev->sem);

    // printk("%s : %d\n", __FUNCTION__, __LINE__);
    if (dev->bcdDevice >= 0x4000)
    {
#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 23)
        remain_count = count;
        bulk_write_size = 32 * PAGE_SIZE;
#else
        remain_count = count;
        bulk_write_size = count;
#endif

        // printk("%s : %d - r %d, b %d\n", __FUNCTION__, __LINE__, remain_count, bulk_write_size);
        while (remain_count > 0)
        {
            bulk_size = min_t(uint, remain_count, dev->bulk_out_size);

            // printk("%s : %d - bulk_out_endpoint %d, bulk_size %d\n", __FUNCTION__, __LINE__, dev->bulk_out_endpointAddr, bulk_size);
            //  �� �����͸� copy_from_user �Լ��� �̿��Ͽ� kernel buffer�� �ű��.
            user_result = copy_from_user(dev->bulk_out_buffer, buf + bulk_write_total, bulk_size);

            // ��ũ����� �̿��Ͽ� �����͸� ����.
            bulk_ret = usb_bulk_msg(dev->udev,
                                    usb_sndbulkpipe(dev->udev, dev->bulk_out_endpointAddr),
                                    dev->bulk_out_buffer,
                                    bulk_size,
                                    &actual_length,
                                    MSG_TIME_OUT);

            remain_count = remain_count - actual_length;
            bulk_write_total += actual_length;

            // u_buf += actual_length;

            if (bulk_ret != 0)
            {
                printk("bulk error!!! : %d\n", bulk_ret);
                break;
            }
        }
        // ret = u_buf - buf;
        ret = bulk_write_total;
        // printk("%s : %d ret: %d\n", __FUNCTION__, __LINE__, ret);

        dev->bulk_out_endpointAddr = dev->bulk_out_endpointAddr_normal;
    }
    else if (dev->bcdDevice >= 0x2000)
    {
#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 23)
        remain_count = count;
        bulk_write_size = 32 * PAGE_SIZE;
#else
        remain_count = count;
        bulk_write_size = count;
#endif
        // printk("%s : %d - remain_count: %d \n", __FUNCTION__, __LINE__, remain_count);
        while (remain_count > 0)
        {
            bulk_size = min_t(uint, remain_count, dev->maxbulktransfersize);

            // �� �����͸� copy_from_user �Լ��� �̿��Ͽ� kernel buffer�� �ű��.
            user_result = copy_from_user(dev->bulk_out_buffer, u_buf, bulk_size);

            // ��ũ����� �̿��Ͽ� �����͸� ����.
            bulk_ret = usb_bulk_msg(dev->udev,
                                    usb_sndbulkpipe(dev->udev, dev->bulk_out_endpointAddr),
                                    dev->bulk_out_buffer,
                                    bulk_size,
                                    &actual_length,
                                    MSG_TIME_OUT);

            remain_count = remain_count - actual_length;
            // printk("%s : %d remain_count - %ld, actual_length - %d\n", __FUNCTION__, __LINE__, remain_count, actual_length);

            u_buf += actual_length;

            if (bulk_ret != 0)
            {
                printk("bulk error!!! : %d\n", bulk_ret);
                break;
            }
        }
        ret = u_buf - buf;
        // printk("%s : %d ret: %d\n", __FUNCTION__, __LINE__, ret);

        dev->bulk_out_endpointAddr = dev->bulk_out_endpointAddr_normal;
    }
    else if (dev->bcdDevice < 0x2000)
    {
        ret = 0; // HFDU01 �� bulk write �� �������� �ʴ´�.
    }

#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 17)
    up(&disconnect_sem);
#else
    mutex_unlock(&disconnect_mutex);
#endif
    up(&dev->sem);

    // printk("%s : %d ret: %d\n", __FUNCTION__, __LINE__, ret);

    return ret;
}

/* This is the first operation performed on the device file,
 * this routine is provided for, driver to do initialization
 * in preparation for later opertions and increments the usage
 * count for the device so that the module won't be unloaded before
 *  the file is closed.
 */
static int hfdu04_open(struct inode *inode, struct file *file)
{
    struct hfdu04_data *dev = NULL;

    // unsigned subminor;
    //  FALinux
    struct usb_interface *interface;
    // FALinux

    int retval = 0;

    // printk("open device : %d\n", __LINE__);

#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 17)
    /* prevent disconnections */
    down(&disconnect_sem);
#else
    /* prevent disconnects */
    mutex_lock(&disconnect_mutex);
#endif

    {
        /* get the interface from minor number and driver information */
        interface = usb_find_interface(&hfdu04_driver, iminor(inode));
        if (!interface)
        {
#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 17)
            up(&disconnect_sem);
#else
            mutex_unlock(&disconnect_mutex);
#endif
            return -ENODEV;
        }

        /* get the device information block from the interface */
        dev = usb_get_intfdata(interface);
        if (!dev)
        {
#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 17)
            up(&disconnect_sem);
#else
            mutex_unlock(&disconnect_mutex);
#endif
            return -ENODEV;
        }
    }

    /* Lock this device */
    down(&dev->sem);

    /* Save our object in file's private structure  used by read
     * and write method */
    // file->f_pos = 0;
    file->private_data = dev;

    /* Unlcok this device */
    up(&dev->sem);

    // exit_open_err:
#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 17)
    up(&disconnect_sem);
#else
    mutex_unlock(&disconnect_mutex);
#endif

    printk("opened device node : #%d\n", interface->minor);
    dev->IsOpened = true;

    return retval;
}

/* ioctl function calls */
/* bulk read write  ioctl */
static int ezusb_bulk_read_write(struct hfdu04_data *dev, pbulk_transfer_t pb, int is_write)
{
    if (dev->bcdDevice < 0x2000)
    {
        int ret;
        unsigned int pipe;
        int actual_length;

        if (is_write)
            pipe = usb_sndbulkpipe(dev->udev, dev->bulk_out_endpointAddr);
        else
        {
            pipe = usb_rcvbulkpipe(dev->udev, dev->bulk_in_endpointAddr);
            memset(pb->data, 0x00, sizeof(pb->data));
        }

        // ret = usb_bulk_msg(dev->udev, pipe, pb->data, pb->size, &actual_length, 10 * HZ);
        ret = usb_bulk_msg(dev->udev, pipe, pb->data, pb->size, &actual_length, MSG_TIME_OUT);

        if (ret < 0)
        {
#if LINUX_VERSION_CODE > KERNEL_VERSION(3, 3, 0)
            printk(" hfdu04: usb_bulk_msg failed(%d) ", ret);
#else
            err(" hfdu04: usb_bulk_msg failed(%d) ", ret);
#endif
            /*  needs  to set interface again  to altsetting 1 */
            if (usb_set_interface(dev->udev, 0, 1) < 0)
            {
#if LINUX_VERSION_CODE > KERNEL_VERSION(3, 3, 0)
                printk("%s set_interface failed %d ", __FUNCTION__, __LINE__);
#else
                err("%s set_interface failed %d ", __FUNCTION__, __LINE__);
#endif
                return -EINVAL;
            }
        }

        if (ret == -EPIPE)
        {
            // warn("CLEAR_FEATURE request to remove STALL condition.");
            // printk("CLEAR_FEATURE request to remove STALL condition.\n");
            if (usb_clear_halt(dev->udev, usb_pipeendpoint(pipe)))
            {
#if LINUX_VERSION_CODE > KERNEL_VERSION(3, 3, 0)
                printk(" clear_halt request failed");
#else
                err(" clear_halt request failed");
#endif
            }
        }

        // dbg("%s no of bytes asked(%d) actual no bytes written/read(%d)  ", __FUNCTION__, pb->size, actual_length);

        pb->size = actual_length;
        return ret;
    }

    return 0;
}

/* reset pipe we reset only ISO pipe  */
static int ezusb_reset_pipe(struct hfdu04_data *dev)
{
    if (dev->bcdDevice < 0x2000)
    {
        int clear_ret;

        clear_ret = usb_clear_halt(dev->udev, usb_pipeendpoint(usb_rcvisocpipe(dev->udev, dev->iso_in_endpointAddr)));
        if (clear_ret)
        {
#if LINUX_VERSION_CODE > KERNEL_VERSION(3, 3, 0)
            printk("Unable to clear endpoint iso in ");
#else
            err("Unable to clear endpoint iso in ");
#endif
            goto reset_err;
        }
    reset_err:
        return clear_ret;
    }

    return 0;
}

#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 36)
static int hfdu04_ioctl(struct inode *inode, struct file *file, unsigned int cmd, unsigned long arg)
#else
#if 1 /* jphwang++(2015.01.21) */
static long hfdu04_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
#else
static int hfdu04_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
#endif
#endif
{
    int retval = 0;
    pbulk_transfer_t pbulk_buffer;
    int bulk_flag = 0; /*flag=1 for write 0 for read */
    struct hfdu04_data *dev = (struct hfdu04_data *)file->private_data;

    if (dev->IsOpened == false)
    {
        // printk(" %s : %d - disconnected or not opened \n", __FUNCTION__, __LINE__);
        return -EIO;
    }

#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 17)
    down(&disconnect_sem);
#endif

    down(&dev->sem);

    if (!dev->udev)
    {
#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 17)
        up(&disconnect_sem);
#endif

        up(&dev->sem);
#if LINUX_VERSION_CODE > KERNEL_VERSION(3, 3, 0)
        printk("%s no device present ", __FUNCTION__);
#else
        err("%s no device present ", __FUNCTION__);
#endif
        return -EIO;
    }

#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 10)
    /* Device I/O Control implementation */
    switch (cmd)
    {
    case EZUSB_BULK_WRITE:
        if (dev->bcdDevice < 0x2000)
        {
            bulk_flag = 1; /* bulk write mode is set */
        }
    case EZUSB_BULK_READ:
        if (dev->bcdDevice < 0x2000)
        {
            pbulk_buffer = (pbulk_transfer_t)kmalloc(sizeof(bulk_transfer_t), GFP_KERNEL);

            if (!pbulk_buffer)
            {
                retval = -ENOMEM;
                break;
            }
            if (copy_from_user(pbulk_buffer, (void *)arg, sizeof(bulk_transfer_t)))
            {
                retval = -EFAULT;
                kfree(pbulk_buffer);
                break;
            }
            retval = ezusb_bulk_read_write(dev, pbulk_buffer, bulk_flag);
            if (retval == 0)
            {
                if (copy_to_user((void *)arg, pbulk_buffer, sizeof(bulk_transfer_t)))
                {
                    retval = -EFAULT;
                }
            }
            kfree(pbulk_buffer);
        }
        break;

    case EZUSB_RESET_PIPE:
        if (dev->bcdDevice < 0x2000)
        {
            retval = ezusb_reset_pipe(dev);
        }
        break;

    case EZUSB_GET_DEVICE_DESCRIPTOR:
        if ((retval = copy_to_user((void *)arg, &(dev->udev->descriptor), sizeof(struct usb_device_descriptor))))
        {
            retval = -EFAULT;
        }
        break;

    case GET_MODULE_VERSION:
        if (1)
        {
            unsigned char *pValue;
            pValue = kmalloc(64, GFP_KERNEL);
            memset(pValue, 0x00, sizeof(unsigned char) * 64);

            pValue[0] = MODULE_MAJOR_VERSION;
            pValue[1] = MODULE_MINOR_VERSION;
            pValue[2] = MODULE_RELEASE_VERSION;
            pValue[3] = MODULE_BUILD_VERSION;

            pValue[4] = ((dev->bcdDevice >> 8) & 0xFF);
            pValue[5] = ((dev->bcdDevice) & 0xFF);

            pValue[6] = ((dev->idVendor >> 8) & 0xFF);
            pValue[7] = ((dev->idVendor) & 0xFF);

            pValue[8] = ((dev->idProduct >> 8) & 0xFF);
            pValue[9] = ((dev->idProduct) & 0xFF);

            if ((retval = copy_to_user((void *)arg, pValue, sizeof(unsigned char) * 64)))
            {
                retval = -EFAULT;
            }
            kfree(pValue);
        }
        break;

    case FDU04_USER_COMMAND:
    {
        int user_command_len = 0;
        unsigned char *user_command = NULL;
        user_command = kmalloc(_USER_COMMAND_LEN, GFP_KERNEL);
        memset(user_command, 0, _USER_COMMAND_LEN);

        if ((retval = copy_from_user(user_command, (void *)arg, _USER_COMMAND_LEN)))
        {
            retval = -EFAULT;
        }
        else
        {
            if (user_command[1] == 0x54) // GET_E2PROM_DATA
                dev->bulk_in_endpointAddr = dev->bulk_in_endpointAddr_fw;
            if (user_command[1] == 0x58) // SET_E2PROM_DATA
                dev->bulk_out_endpointAddr = dev->bulk_out_endpointAddr_fw;

            if (user_command[1] == 0xF1 || user_command[1] == 0xF3 || user_command[1] == 0xF4 || user_command[1] == 0xF8) // GET_IMAGE_DATA
                dev->bulk_in_endpointAddr = dev->bulk_in_endpointAddr_image;

            if (user_command[0] == USB_DIR_IN) /* to host */
            {
                user_command_len = ((user_command[7] << 8) | user_command[8]);
                hfdu04_user_command_in(dev->udev, user_command);

                if ((retval = copy_to_user((void *)arg, &user_command[9], sizeof(unsigned char) * user_command_len))))
                    {
                        retval = -EFAULT;
                    }
            }
            else /* to device */
            {
                hfdu04_user_command_out(dev->udev, user_command);
            }
        }

        kfree(user_command);
    }
    break;

    default:
#if LINUX_VERSION_CODE > KERNEL_VERSION(3, 3, 0)
        printk("not impl ioctl hfdu06: 0x%08x", cmd);
#else
        err("not impl ioctl hfdu06: 0x%08x", cmd);
#endif
        retval = -ENOIOCTLCMD;
    }
#else
    /* Device I/O Control implementation */
    switch (_IOC_NR(cmd))
    {
    case _IOC_NR(EZUSB_BULK_WRITE):
        if (dev->bcdDevice < 0x2000)
        {
            bulk_flag = 1; /* bulk write mode is set */
        }

    case _IOC_NR(EZUSB_BULK_READ):
        if (dev->bcdDevice < 0x2000)
        {
            pbulk_buffer = (pbulk_transfer_t)kmalloc(sizeof(bulk_transfer_t), GFP_KERNEL);

            if (!pbulk_buffer)
            {
                retval = -ENOMEM;
                break;
            }
#if 1 /* jphwang++(2104.01.21) */
            memset(pbulk_buffer, 0x00, sizeof(bulk_transfer_t));
#else
            memset(pbulk_buffer, 0x00, sizeof(pbulk_transfer_t));
#endif

            // if ( bulk_flag == 1)
            {
                if (copy_from_user(pbulk_buffer, (void *)arg, sizeof(bulk_transfer_t)))
                {
                    retval = -EFAULT;
                    kfree(pbulk_buffer);
                    break;
                }
            }

            retval = ezusb_bulk_read_write(dev, pbulk_buffer, bulk_flag);

            if (retval == 0)
            {
                if (copy_to_user((void *)arg, pbulk_buffer, sizeof(bulk_transfer_t)))
                {
                    retval = -EFAULT;
                }
            }
            kfree(pbulk_buffer);
        }
        break;

    case _IOC_NR(EZUSB_RESET_PIPE):
        if (dev->bcdDevice < 0x2000)
        {
            retval = ezusb_reset_pipe(dev);
        }
        break;

    case _IOC_NR(EZUSB_GET_DEVICE_DESCRIPTOR):
        if ((retval = copy_to_user((void *)arg, &(dev->udev->descriptor), sizeof(struct usb_device_descriptor))))
        {
            retval = -EFAULT;
        }
        break;

    case _IOC_NR(GET_MODULE_VERSION):
        if (1)
        {
            unsigned char *pValue;
            pValue = kmalloc(64, GFP_KERNEL);
            memset(pValue, 0x00, sizeof(unsigned char) * 64);

            pValue[0] = MODULE_MAJOR_VERSION;
            pValue[1] = MODULE_MINOR_VERSION;
            pValue[2] = MODULE_RELEASE_VERSION;
            pValue[3] = MODULE_BUILD_VERSION;

            pValue[4] = ((dev->bcdDevice >> 8) & 0xFF);
            pValue[5] = ((dev->bcdDevice) & 0xFF);

            pValue[6] = ((dev->idVendor >> 8) & 0xFF);
            pValue[7] = ((dev->idVendor) & 0xFF);

            pValue[8] = ((dev->idProduct >> 8) & 0xFF);
            pValue[9] = ((dev->idProduct) & 0xFF);

            if ((retval = copy_to_user((void *)arg, pValue, sizeof(unsigned char) * 64)))
            {
                retval = -EFAULT;
            }
            kfree(pValue);
        }
        break;

    case _IOC_NR(FDU04_USER_COMMAND):
    {
        int user_command_len = 0;
        unsigned char *user_command = NULL;
        user_command = kmalloc(_USER_COMMAND_LEN, GFP_KERNEL);
        memset(user_command, 0, _USER_COMMAND_LEN);

        if ((retval = copy_from_user(user_command, (void *)arg, _USER_COMMAND_LEN)))
        {
            retval = -EFAULT;
        }
        else
        {
            if (user_command[1] == 0x54) // GET_E2PROM_DATA
                dev->bulk_in_endpointAddr = dev->bulk_in_endpointAddr_fw;
            if (user_command[1] == 0x58) // SET_E2PROM_DATA
                dev->bulk_out_endpointAddr = dev->bulk_out_endpointAddr_fw;

            if (user_command[1] == 0xF1 || user_command[1] == 0xF3 || user_command[1] == 0xF4 || user_command[1] == 0xF8) // GET_IMAGE_DATA
                dev->bulk_in_endpointAddr = dev->bulk_in_endpointAddr_image;

            if (user_command[0] == USB_DIR_IN) /* to host */
            {
                user_command_len = ((user_command[7] << 8) | user_command[8]);
                hfdu04_user_command_in(dev->udev, user_command);

                if ((retval = copy_to_user((void *)arg, &user_command[9], sizeof(unsigned char) * user_command_len)))
                {
                    retval = -EFAULT;
                }
            }
            else /* to device */
            {
                hfdu04_user_command_out(dev->udev, user_command);
            }
        }

        kfree(user_command);
    }
    break;

    default:
#if LINUX_VERSION_CODE > KERNEL_VERSION(3, 3, 0)
        printk("not impl ioctl hfdu04: 0x%08x", cmd);
#else
        err("not impl ioctl hfdu04: 0x%08x", cmd);
#endif
        retval = -ENOIOCTLCMD;
    }
#endif
#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 17)
    up(&disconnect_sem);
#endif

    up(&dev->sem);
    return retval;
}

// Device Init
static int hfdu04_init_device(struct usb_device *udev)
{
    int result_cmd;
    unsigned char *buf = NULL;
    __u16 nIndex = 0;

    buf = kmalloc(_USER_COMMAND_LEN, GFP_KERNEL);
    memset(buf, 0, _USER_COMMAND_LEN);

    if (udev->descriptor.bcdDevice >= 0x4000)
    {
        if (udev->descriptor.bcdDevice >= 0x4200)
            nIndex = 0x1200;
        else if (udev->descriptor.bcdDevice >= 0x4004)
            nIndex = 0x1100;
        else
            nIndex = 0x0000;

        buf[0] = USB_DIR_IN;
        buf[1] = 0xFF;
        buf[2] = 0xC0;
        buf[5] = ((nIndex >> 8) & 0xFF);
        buf[6] = ((nIndex) & 0xFF);
        buf[8] = 8;

        result_cmd = hfdu04_user_command_in(udev, buf);
    }
    else
    {
        buf[0] = USB_DIR_IN;
        buf[1] = 0xFF;
        buf[2] = 0xC0;
        buf[3] = 0x00; // value
        buf[4] = 0x50; // value
        buf[5] = 0x00;
        buf[6] = 0x00;
        buf[8] = 8;

        result_cmd = hfdu04_user_command_in(udev, buf);

        if (buf[0] == 0 && buf[1] == 0)
        {
            buf[0] = USB_DIR_IN;
            buf[1] = 0xFF;
            buf[2] = 0xC0;
            buf[3] = 0x00; // value
            buf[4] = 0x53; // value
            buf[5] = 0x00;
            buf[6] = 0x00;
            buf[8] = 8;
            result_cmd = hfdu04_user_command_in(udev, buf);
        }
        else if (buf[1] == 0)
        {
            buf[0] = USB_DIR_IN;
            buf[1] = 0xFF;
            buf[2] = 0xC0;
            buf[3] = 0x00; // value
            buf[4] = 0x52; // value
            buf[5] = 0x00;
            buf[6] = 0x00;
            buf[8] = 8;
            result_cmd = hfdu04_user_command_in(udev, buf);
        }
    }

    kfree(buf);
    return result_cmd;
}

int hfdu04_user_command_in(struct usb_device *udev, unsigned char *value)
{
    int result_cmd;

    memset(&value[9], 0, ((value[7] << 8) | value[8]));
    result_cmd = usb_control_msg(udev,                                         // dev
                                 usb_rcvctrlpipe(udev, 0),                     // pipe
                                 value[1],                                     // request
                                 value[2],                                     // request type
                                 ((value[3] << 8) | value[4]),                 // value
                                 ((value[5] << 8) | value[6]),                 // index
                                 ((value[7] << 8) | value[8]) ? &value[9] : 0, // data
                                 ((value[7] << 8) | value[8]),                 // size
                                 MSG_TIME_OUT                                  // timeout
    );

    return result_cmd;
}

int hfdu04_user_command_out(struct usb_device *udev, unsigned char *value)
{
    int result_cmd;

    result_cmd = usb_control_msg(udev,                                         // dev
                                 usb_sndctrlpipe(udev, 0),                     // pipe
                                 value[1],                                     // request
                                 value[2],                                     // request type
                                 ((value[3] << 8) | value[4]),                 // value
                                 ((value[5] << 8) | value[6]),                 // index
                                 ((value[7] << 8) | value[8]) ? &value[9] : 0, // data
                                 ((value[7] << 8) | value[8]),                 // size
                                 MSG_TIME_OUT                                  // timeout
    );

    return result_cmd;
}

/* This routine is invoked very first before any other routines
 * by the host controller, whenever a USB device is connected,
 * host controller invokes this routine. Here we check device
 * idVendor  and idproduct and we register file system.
 */
static int hfdu04_probe(struct usb_interface *interface,
                        const struct usb_device_id *id)
{
    struct usb_device *udev = interface_to_usbdev(interface);
    struct hfdu04_data *dev = NULL;
    int retval = 0;
    struct usb_host_interface *iface_desc;
    struct usb_endpoint_descriptor *endpoint;
    int i;
    unsigned char *buf = NULL;

    // int incount;

    __u16 buffer_size;

    /* See if the device offered us matches what we can accept  */
    if ((udev->descriptor.idVendor != USB_HFDU04_VENDOR_ID) && (udev->descriptor.idProduct != USB_HFDU04_PRODUCT_ID))
    {
#if LINUX_VERSION_CODE > KERNEL_VERSION(3, 3, 0)
        printk("Invalid idVendor(%x) idProduct(%x) ", udev->descriptor.idVendor, udev->descriptor.idProduct);
#else
        err("Invalid idVendor(%x) idProduct(%x) ", udev->descriptor.idVendor, udev->descriptor.idProduct);
#endif
        return -ENODEV;
    }

    /* Allocate memory for our device state and initialize it */
    dev = kmalloc(sizeof(*dev), GFP_KERNEL);
    if (dev == NULL)
    {
        goto error;
    }
    memset(dev, 0x00, sizeof(*dev));

#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 36)
    init_MUTEX(&dev->sem);
#else
    sema_init(&dev->sem, 1);
#endif

    dev->idVendor = udev->descriptor.idVendor;
    dev->idProduct = udev->descriptor.idProduct;
    dev->bcdDevice = udev->descriptor.bcdDevice;
    if (dev->bcdDevice < 0x2000)
    {
        init_waitqueue_head(&dev->wait);
        init_waitqueue_head(&dev->wait1);
        spin_lock_init(&dev->lock);
        INIT_LIST_HEAD(&dev->free_buff_list);
        INIT_LIST_HEAD(&dev->rec_buff_list);
    }

    dev->udev = udev;

#if 1
    if (dev->bcdDevice < 0x2000)
    {
        dev->present = 1;
        dev->opencount = 0;

        /* Set altsetting to 0 for downloadng firmware */
        if ((retval = usb_set_interface(dev->udev, 0, 0)))
        {
            // if ((retval = usb_set_interface(dev->udev, interface, 0))) {
#if LINUX_VERSION_CODE > KERNEL_VERSION(3, 3, 0)
            printk("%s set_interface failed with alt setting 0 : ", __FUNCTION__);
#else
            err("%s set_interface failed with alt setting 0 : ", __FUNCTION__);
#endif
            goto error;
        }

        /* Set altsetting to 1 because this interface is having more bandwidth than 0th interface.*/
        if ((retval = usb_set_interface(dev->udev, 0, 1)))
        {
            // if ((retval = usb_set_interface(dev->udev, interface, 1))) {
#if LINUX_VERSION_CODE > KERNEL_VERSION(3, 3, 0)
            printk("%s set_interface failed with alt setting 1 : ", __FUNCTION__);
#else
            err("%s set_interface failed with alt setting 1 : ", __FUNCTION__);
#endif
            goto error;
        }
        // delay
        // udelay(10000);
        mdelay(10);
    }
    else
    {
#if LINUX_VERSION_CODE > KERNEL_VERSION(2, 6, 16)
        dev->interface = interface;
#endif
    }

#endif

    // dev->interface = interface;

    /**
     * Set up the endpoint information.
     * Use only the first bulk-in and bulk-out endpoints.
     */

    if (dev->bcdDevice < 0x2000)
    {
        iface_desc = &interface->altsetting[1];
    }
    else
    {
        iface_desc = interface->cur_altsetting;
    }

    // printk("NumEndpoints: %d\n", iface_desc->desc.bNumEndpoints);

    // delay
    mdelay(10);
    for (i = 0; i < iface_desc->desc.bNumEndpoints; ++i)
    {
        endpoint = &iface_desc->endpoint[i].desc;

        if (dev->bcdDevice < 0x2000)
        {
#if 0
         if (
            (endpoint->bEndpointAddress & USB_DIR_IN) && 
            (endpoint->bmAttributes & USB_ENDPOINT_XFERTYPE_MASK) == USB_ENDPOINT_XFER_ISOC && 
            (!dev->iso_in_endpointAddr)
            )
         {
            dev->iso_in_endpointAddr = endpoint->bEndpointAddress;
         }
#endif

            if (
                (!dev->iso_in_endpointAddr) && /*usb_endpoint_is_iso_in (endpoint))*/
                ((endpoint->bEndpointAddress & USB_ENDPOINT_DIR_MASK) == USB_DIR_IN) &&
                ((endpoint->bmAttributes & USB_ENDPOINT_XFERTYPE_MASK) == USB_ENDPOINT_XFER_ISOC))
            {
                dev->iso_in_endpointAddr = endpoint->bEndpointAddress;
            }

            if (
                (endpoint->bEndpointAddress & USB_DIR_IN) &&
                (endpoint->bmAttributes & USB_ENDPOINT_XFERTYPE_MASK) == USB_ENDPOINT_XFER_BULK &&
                (!dev->bulk_in_endpointAddr))
            {
                dev->bulk_in_endpointAddr = endpoint->bEndpointAddress;
            }

            if (
                !(endpoint->bEndpointAddress & USB_DIR_IN) &&
                (endpoint->bmAttributes & USB_ENDPOINT_XFERTYPE_MASK) == USB_ENDPOINT_XFER_BULK &&
                (!dev->bulk_out_endpointAddr))
            {
                dev->bulk_out_endpointAddr = endpoint->bEndpointAddress;
            }
        }
        else
        {
            if (!dev->bulk_out_endpointAddr && /*usb_endpoint_is_bulk_out(endpoint))*/
                ((endpoint->bEndpointAddress & USB_ENDPOINT_DIR_MASK) == USB_DIR_OUT) &&
                ((endpoint->bmAttributes & USB_ENDPOINT_XFERTYPE_MASK) == USB_ENDPOINT_XFER_BULK))
            {
                /* We found a bulk out endpoint */
                buffer_size = le16_to_cpu(endpoint->wMaxPacketSize);
                dev->bulk_out_size = buffer_size;
                // printk("find bulk-out endpoint = %X\n", endpoint->bEndpointAddress);
                dev->bulk_out_endpointAddr = endpoint->bEndpointAddress;
                dev->bulk_out_endpointAddr_normal = endpoint->bEndpointAddress;

                if (dev->bcdDevice < 0x4000)
                {
                    dev->bulk_out_endpointAddr_fw = endpoint->bEndpointAddress;

                    dev->bulk_out_buffer = kmalloc(15 * 1024, GFP_KERNEL);
                    if (dev->bulk_out_buffer == NULL)
                    {
                        printk("could not allocate bulk_out_buffer\n");
                        goto error;
                    }
                }
            }

            if (dev->bulk_out_endpointAddr && /*usb_endpoint_is_bulk_out(endpoint))*/
                ((endpoint->bEndpointAddress & USB_ENDPOINT_DIR_MASK) == USB_DIR_OUT) &&
                ((endpoint->bmAttributes & USB_ENDPOINT_XFERTYPE_MASK) == USB_ENDPOINT_XFER_BULK))
            {
                if (dev->bcdDevice >= 0x4000)
                {
                    /* We found a bulk out endpoint */
                    buffer_size = le16_to_cpu(endpoint->wMaxPacketSize);
                    dev->bulk_out_size = buffer_size;
                    // printk("find bulk-out endpoint = %X\n", endpoint->bEndpointAddress);
                    dev->bulk_out_endpointAddr_fw = endpoint->bEndpointAddress;

                    dev->bulk_out_buffer = kmalloc(15 * 1024, GFP_KERNEL);
                    if (dev->bulk_out_buffer == NULL)
                    {
                        printk("could not allocate bulk_out_buffer\n");
                        goto error;
                    }
                }
            }

            if (!dev->bulk_in_endpointAddr && /*usb_endpoint_is_bulk_in(endpoint))*/
                ((endpoint->bEndpointAddress & USB_ENDPOINT_DIR_MASK) == USB_DIR_IN) &&
                ((endpoint->bmAttributes & USB_ENDPOINT_XFERTYPE_MASK) == USB_ENDPOINT_XFER_BULK))
            {
                // printk("find bulk-in endpoint = %X\n", endpoint->bEndpointAddress);
                /* We found a bulk in endpoint */
                buffer_size = le16_to_cpu(endpoint->wMaxPacketSize);
                dev->bulk_in_size = buffer_size;
                if (buffer_size == 64) // for full speed
                {
                    if (dev->bcdDevice >= 0x4000)
                    {
                        dev->maxbulktransfersize = 512 /*64*/; // HFDU06_MAX_PACKET_SIZE;
                    }
                    else // if (dev->bcdDevice >= 0x2000)
                    {
                        // dev->maxbulktransfersize = 384; // 384 = 64*6 (1ms �� 6���� transaction�� �õ���)
                        dev->maxbulktransfersize = 512;
                    }
                }
                else
                {
                    if (dev->bcdDevice >= 0x4000)
                    {
                        dev->maxbulktransfersize = 512; // HFDU06_MAX_PACKET_SIZE;
                    }
                    else // if (dev->bcdDevice >= 0x2000)
                    {
                        dev->maxbulktransfersize = dev->bulk_in_size;
                    }
                }

                dev->bulk_in_endpointAddr = endpoint->bEndpointAddress;
                dev->bulk_in_endpointAddr_image = endpoint->bEndpointAddress;
                if (dev->bcdDevice < 0x4000)
                    dev->bulk_in_endpointAddr_fw = endpoint->bEndpointAddress;

                if (dev->bcdDevice >= 0x4000)
                {
#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 23)
                    // dev->bulk_in_buffer = kmalloc(dev->maxbulktransfersize, GFP_KERNEL);
                    dev->bulk_in_buffer = kmalloc(32 * PAGE_SIZE, GFP_KERNEL);
                    // dev->bulk_in_buffer = kmalloc(HFDU06_MAX_PACKET_SIZE, GFP_KERNEL);

                    // for (incount = 0; incount < MAX_FDU06_IN_BUF; incount++)
                    //{
                    //    dev->inbuf[incount].length = 0;
                    //    dev->inbuf[incount].pInBuf = (unsigned char*)kmalloc(32*PAGE_SIZE, GFP_KERNEL);
                    // }
#else
                    dev->bulk_in_buffer = kmalloc(HFDU06_MAX_PACKET_SIZE, GFP_KERNEL);
#endif
                }
                else // if (dev->bcdDevice >= 0x2000)
                {
                    dev->bulk_in_buffer = kmalloc(dev->maxbulktransfersize /*buffer_size*/, GFP_KERNEL);
                }

                if (dev->bulk_in_buffer == NULL)
                {
                    printk("could not allocate bulk_in_buffer\n");
                    goto error;
                }
            }

            if (dev->bulk_in_endpointAddr && /*usb_endpoint_is_bulk_in(endpoint))*/
                ((endpoint->bEndpointAddress & USB_ENDPOINT_DIR_MASK) == USB_DIR_IN) &&
                ((endpoint->bmAttributes & USB_ENDPOINT_XFERTYPE_MASK) == USB_ENDPOINT_XFER_BULK))
            {
                if (dev->bcdDevice >= 0x4000)
                {
                    /* We found a bulk in endpoint */
                    // printk("find bulk-in endpoint = %X\n", endpoint->bEndpointAddress);
                    dev->bulk_in_endpointAddr_fw = endpoint->bEndpointAddress;
                }
            }
        }
    }

    if (!(dev->bulk_in_endpointAddr && dev->bulk_out_endpointAddr))
    {
        printk("could not find both bulk-in and bulk-out endpoints\n");
        goto error;
    }

    // printk(" isoc in endpoint address =%x  \n", dev->iso_in_endpointAddr);
    // printk(" bulk in endpoint address =%x \n", dev->bulk_in_endpointAddr);
    // printk(" bulk out endpoint address =%x \n", dev->bulk_out_endpointAddr);

    /* we can register the device now, as it is ready */
    usb_set_intfdata(interface, dev);

    retval = usb_register_dev(interface, &hfdu04_class);

#if 1 /* jphwang++(2015.01.21) for test */

#ifdef CONFIG_USB_DYNAMIC_MINORS
    printk("DynamicMinors-->retval(%d),minor_base(%d) ignored\n", retval, hfdu04_class.minor_base);
#else
    printk("-->retval(%d),minor_base(%d)\n", retval, hfdu04_class.minor_base);
#endif
#endif

    if (retval)
    {
        /* something prevented us from registering this driver */
#if LINUX_VERSION_CODE > KERNEL_VERSION(3, 3, 0)
        printk("unable to get a minor for this device.");
#else
        err("unable to get a minor for this device.");
#endif
        usb_set_intfdata(interface, NULL);
        // usb_deregister_dev(interface, &hfdu04_class);
        goto error;
    }

    /* save minor number */
    dev->minor = interface->minor;
    // info("%s connected to USB  minor: %d\n", DRIVER_DESC, dev->minor);
    printk("%s connected to USB minor: %d\n", DRIVER_DESC, dev->minor);
    // printk("transfer_buffer_length: %d\n", dev->bulk_in_size);

    // for MIRU, 2010.05.03
    // venus_probe_ok(dev->minor);

    hfdu04_init_device(udev);

    /* connection test */
    buf = kmalloc(_USER_COMMAND_LEN, GFP_KERNEL);
    memset(buf, 0, _USER_COMMAND_LEN);
    buf[0] = USB_DIR_OUT;
    buf[1] = 0x90;
    buf[2] = 0x40;
    buf[8] = 0x40;

    if (dev->bcdDevice >= 0x4000)
    {
        for (i = 0; i < 3; i++)
        {
            buf[9] = 1;
            hfdu04_user_command_out(udev, buf);
            msleep(50);

            buf[9] = 0;
            hfdu04_user_command_out(udev, buf);
            msleep(50);
        }
    }
    else if (dev->bcdDevice >= 0x2000)
    {
        for (i = 0; i < 3; i++)
        {
            buf[3] = 0x00; // value
            buf[4] = 0x51; // value - ON
            buf[8] = 0x00;
            hfdu04_user_command_out(udev, buf);
            msleep(50);

            buf[3] = 0x00; // value
            buf[4] = 0x50; // value - OFF
            buf[8] = 0x00;
            hfdu04_user_command_out(udev, buf);
            msleep(50);
        }
    }

    kfree(buf);

    dev->IsConnected = true;
    dev->IsOpened = false;

    return 0;

error:
    if (dev->bulk_in_buffer)
        kfree(dev->bulk_in_buffer);
    if (dev->bulk_out_buffer)
        kfree(dev->bulk_out_buffer);
    // for(incount = 0; incount < MAX_FDU06_IN_BUF; incount++)
    //    if (dev->inbuf[incount].pInBuf) kfree(dev->inbuf[incount].pInBuf);
    if (dev)
        kfree(dev);
    dev->IsConnected = false;
    dev->IsOpened = false;

    return retval;
}

/* This is invoked when the file structure is being released.
 * This routine deallocates that open allocated in file->private_  data.
 * Shutdown the device and decrements the usage count.
 */
static int hfdu04_release(struct inode *inode, struct file *file)
{
    int retval = 0;
    struct hfdu04_data *dev = (struct hfdu04_data *)file->private_data;

    printk("released device node : #%d\n", dev->minor);

    dev->IsOpened = false;
    if (dev->minor < USB_HFDU04_MINOR_BASE || dev->IsConnected == false)
    {
        return retval;
    }

    /* Lock our  device */
    down(&dev->sem);

    if (dev->minor < USB_HFDU04_MINOR_BASE)
    {
        retval = -ENODEV;
        goto exit_on_error;
    }

    if (!dev->present)
    {
        retval = -ENODEV;
        goto exit_on_error;
    }

    if (dev->bcdDevice < 0x2000)
    {
        if (dev->opencount <= 0)
        {
            // err("%s - device not opened", __FUNCTION__);
            retval = -ENODEV;
            goto exit_on_error;
        }

        hfdu04_stopgrabbingdata(dev);
        hfdu04_freebuffers(dev);
    }

    /* Decrement usage count for this driver */

    if (dev->bcdDevice < 0x2000)
    {
        --dev->opencount;
    }
    file->private_data = NULL;

exit_on_error:
    up(&dev->sem);
    return retval;
}

/* This routine will be called by the USB core, when the device is
 * removed from the system. It clears the interface data and
 * clears the device structure.
 */
static void hfdu04_disconnect(struct usb_interface *interface)
{
    // int incount;
    int minor;
    struct hfdu04_data *dev = usb_get_intfdata(interface);

    if (dev == NULL)
    {
        return;
    }

    /* USB driver saves NULL to a USB driver specific structure */
    usb_set_intfdata(interface, NULL);

    /* Lock the device */
    down(&dev->sem);

    /* Get minor number for info to user */
    minor = dev->minor;

    /* Remove the minor for the particular device */
    usb_deregister_dev(interface, &hfdu04_class);
    set_current_state(TASK_UNINTERRUPTIBLE);
    /* prevent device read  ioctl */
    dev->present = 0;

    if (dev->bcdDevice < 0x2000)
    {
        if (!dev->opencount)
        {
            up(&dev->sem);
        }
        else
        {
            dev->udev = NULL;
            up(&dev->sem);
        }
    }

    /* Initialize  overrun */
    dev->overruns = 0;

    if (dev->bulk_in_buffer)
        kfree(dev->bulk_in_buffer);
    if (dev->bulk_out_buffer)
        kfree(dev->bulk_out_buffer);
    // for(incount = 0; incount < MAX_FDU06_IN_BUF; incount++)
    //    if (dev->inbuf[incount].pInBuf) kfree(dev->inbuf[incount].pInBuf);
    kfree(dev);

    dev->IsConnected = false;
    dev->IsOpened = false;

    /* Let user know device disconnected */
    // info("%s #%d is now disconnected ", DRIVER_DESC, minor);
    printk("%s #%d is now disconnected \n", DRIVER_DESC, minor);

    // for MIRU, 2010.05.03
    // venus_disconnect_ok(minor);
}

static void __exit hfdu04_exit(void)
{
#if LINUX_VERSION_CODE > KERNEL_VERSION(2, 6, 16)
    destroy_workqueue(comm_queue);
#endif

    /* deregisters  driver with the USB subsystem  */
    usb_deregister(&hfdu04_driver);

    // for MIRU, 2010.05.03
    // venus_exit_ok();
}

static int __init hfdu04_init(void)
{
    int result;

    // info(DRIVER_DESC " " DRIVER_VERSION);
    printk(DRIVER_DESC " " DRIVER_VERSION "\n");

#if LINUX_VERSION_CODE > KERNEL_VERSION(2, 6, 16)
    comm_queue = create_singlethread_workqueue("hfdu04");
    if (comm_queue == NULL)
    {
        printk("could not create work queue\n");
        return -ENOMEM;
    }
#endif

    /* register this driver with the USB subsystem */
    result = usb_register(&hfdu04_driver);
    if (result)
    {
#if LINUX_VERSION_CODE > KERNEL_VERSION(3, 3, 0)
        printk("usb_register failed. Error number %d", result);
#else
        err("usb_register failed. Error number %d", result);
#endif

#if LINUX_VERSION_CODE > KERNEL_VERSION(2, 6, 16)
        destroy_workqueue(comm_queue);
#endif
        return result;
    }

    // for MIRU, 2010.05.03
    // venus_init_ok();

    return result;
}

module_init(hfdu04_init);
module_exit(hfdu04_exit);

#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 7)
/* Module paramaters */
MODULE_PARM(debug, "i");
MODULE_PARM_DESC(debug, "Debug enabled or not"); /* Module parameters */

#elif LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 10)
module_param(debug, int, 0);
MODULE_PARM_DESC(debug, "Debug enabled or not");

#elif LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 17)
/* Module paramaters */
MODULE_PARM(debug, "i");
MODULE_PARM_DESC(debug, "Debug enabled or not");
MODULE_AUTHOR(DRIVER_AUTHOR);
MODULE_DESCRIPTION(DRIVER_DESC);
MODULE_LICENSE("NITGEN Co., Ltd.");

#else
module_param(debug, int, 0444);
MODULE_PARM_DESC(debug, "Debug enabled or not");
MODULE_AUTHOR(DRIVER_AUTHOR);
MODULE_DESCRIPTION(DRIVER_DESC);
MODULE_LICENSE("GPL");
#endif
