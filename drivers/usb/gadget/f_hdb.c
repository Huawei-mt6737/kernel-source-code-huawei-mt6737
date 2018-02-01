/*
 * Gadget Driver for Android ADB
 *
 * Copyright (C) 2008 Google, Inc.
 * Author: Mike Lockwood <lockwood@android.com>
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
#ifdef pr_fmt
#undef pr_fmt
#endif
#define pr_fmt(fmt) "["KBUILD_MODNAME"]" fmt

#include <linux/module.h>
#include <linux/init.h>
#include <linux/poll.h>
#include <linux/delay.h>
#include <linux/wait.h>
#include <linux/err.h>
#include <linux/interrupt.h>
#include <linux/sched.h>
#include <linux/types.h>
#include <linux/device.h>
#include <linux/miscdevice.h>
#include <linux/ratelimit.h>

#define HDB_BULK_BUFFER_SIZE           32768

/* number of tx requests to allocate */
#define TX_REQ_MAX 4

static const char hdb_shortname[] = "android_hdb";

struct hdb_dev {
	struct usb_function function;
	struct usb_composite_dev *cdev;
	spinlock_t lock;

	struct usb_ep *hdb_ep_in;
	struct usb_ep *hdb_ep_out;

	int online;
	int error;

	atomic_t read_excl;
	atomic_t write_excl;
	atomic_t open_excl;

	struct list_head tx_idle;

	wait_queue_head_t read_wq;
	wait_queue_head_t write_wq;
	struct usb_request *rx_req;
	int rx_done;
};

/* __ADB_DEBUG__ start */
struct usb_ep *hdb_ep_in = NULL;
struct usb_ep *hdb_ep_out = NULL;
int bitdebug_enabled;
unsigned hdb_bitdebug_writeCnt = 1;
unsigned hdb_bitdebug_readCnt = 0;

struct hdb_amessage {
	unsigned command;	/* command identifier constant      */
	unsigned arg0;		/* first argument                   */
	unsigned arg1;		/* second argument                  */
	unsigned data_length;	/* length of payload (0 is allowed) */
	unsigned data_check;	/* checksum of data payload         */
	unsigned magic;		/* command ^ 0xffffffff             */
};

struct hdb_debuginfo {
	unsigned headtoken;
	unsigned command;	/* command identifier constant      */
	unsigned msg_check;
	unsigned data_check;
	unsigned count;
	unsigned dummy;
	unsigned tailtoken;
};


typedef struct hdb_amessage hdb_amessage;
typedef struct hdb_debuginfo hdb_debuginfo;

#define A_SYNC 0x434e5953
#define A_CNXN 0x4e584e43
#define A_OPEN 0x4e45504f
#define A_OKAY 0x59414b4f
#define A_CLSE 0x45534c43
#define A_WRTE 0x45545257
#define A_AUTH 0x48545541
#define A_DBUG 0x41424a42

#define DBGHEADTOKEN 0x13579bdf
#define DBGTAILTOKEN 0xdca86420

/* __ADB_DEBUG__ end */

static struct usb_interface_descriptor hdb_interface_desc = {
	.bLength = USB_DT_INTERFACE_SIZE,
	.bDescriptorType = USB_DT_INTERFACE,
	.bInterfaceNumber = 0,
	.bNumEndpoints = 2,
	.bInterfaceClass = 0xFF,
	.bInterfaceSubClass = 0x48,
	.bInterfaceProtocol = 1,
};

static struct usb_endpoint_descriptor hdb_superspeed_in_desc = {
	.bLength = USB_DT_ENDPOINT_SIZE,
	.bDescriptorType = USB_DT_ENDPOINT,
	.bEndpointAddress = USB_DIR_IN,
	.bmAttributes = USB_ENDPOINT_XFER_BULK,
	.wMaxPacketSize = __constant_cpu_to_le16(1024),
};

static struct usb_ss_ep_comp_descriptor hdb_superspeed_in_comp_desc = {
	.bLength = sizeof(hdb_superspeed_in_comp_desc),
	.bDescriptorType = USB_DT_SS_ENDPOINT_COMP,

	/* the following 2 values can be tweaked if necessary */
	/* .bMaxBurst =         0, */
	/* .bmAttributes =      0, */
};

static struct usb_endpoint_descriptor hdb_superspeed_out_desc = {
	.bLength = USB_DT_ENDPOINT_SIZE,
	.bDescriptorType = USB_DT_ENDPOINT,
	.bEndpointAddress = USB_DIR_OUT,
	.bmAttributes = USB_ENDPOINT_XFER_BULK,
	.wMaxPacketSize = __constant_cpu_to_le16(1024),
};

static struct usb_ss_ep_comp_descriptor hdb_superspeed_out_comp_desc = {
	.bLength = sizeof(hdb_superspeed_out_comp_desc),
	.bDescriptorType = USB_DT_SS_ENDPOINT_COMP,

	/* the following 2 values can be tweaked if necessary */
	/* .bMaxBurst =         0, */
	/* .bmAttributes =      0, */
};

static struct usb_endpoint_descriptor hdb_highspeed_in_desc = {
	.bLength = USB_DT_ENDPOINT_SIZE,
	.bDescriptorType = USB_DT_ENDPOINT,
	.bEndpointAddress = USB_DIR_IN,
	.bmAttributes = USB_ENDPOINT_XFER_BULK,
	.wMaxPacketSize = __constant_cpu_to_le16(512),
};

static struct usb_endpoint_descriptor hdb_highspeed_out_desc = {
	.bLength = USB_DT_ENDPOINT_SIZE,
	.bDescriptorType = USB_DT_ENDPOINT,
	.bEndpointAddress = USB_DIR_OUT,
	.bmAttributes = USB_ENDPOINT_XFER_BULK,
	.wMaxPacketSize = __constant_cpu_to_le16(512),
};

static struct usb_endpoint_descriptor hdb_fullspeed_in_desc = {
	.bLength = USB_DT_ENDPOINT_SIZE,
	.bDescriptorType = USB_DT_ENDPOINT,
	.bEndpointAddress = USB_DIR_IN,
	.bmAttributes = USB_ENDPOINT_XFER_BULK,
};

static struct usb_endpoint_descriptor hdb_fullspeed_out_desc = {
	.bLength = USB_DT_ENDPOINT_SIZE,
	.bDescriptorType = USB_DT_ENDPOINT,
	.bEndpointAddress = USB_DIR_OUT,
	.bmAttributes = USB_ENDPOINT_XFER_BULK,
};

static struct usb_descriptor_header *fs_hdb_descs[] = {
	(struct usb_descriptor_header *)&hdb_interface_desc,
	(struct usb_descriptor_header *)&hdb_fullspeed_in_desc,
	(struct usb_descriptor_header *)&hdb_fullspeed_out_desc,
	NULL,
};

static struct usb_descriptor_header *hs_hdb_descs[] = {
	(struct usb_descriptor_header *)&hdb_interface_desc,
	(struct usb_descriptor_header *)&hdb_highspeed_in_desc,
	(struct usb_descriptor_header *)&hdb_highspeed_out_desc,
	NULL,
};

static struct usb_descriptor_header *ss_hdb_descs[] = {
	(struct usb_descriptor_header *)&hdb_interface_desc,
	(struct usb_descriptor_header *)&hdb_superspeed_in_desc,
	(struct usb_descriptor_header *)&hdb_superspeed_in_comp_desc,
	(struct usb_descriptor_header *)&hdb_superspeed_out_desc,
	(struct usb_descriptor_header *)&hdb_superspeed_out_comp_desc,
	NULL,
};

static void hdb_debug_read_copy_from_user(char __user *buf, struct usb_request *req)
{
	if (sizeof(hdb_debuginfo) == req->length) {
		unsigned long ret;
		ret = copy_from_user(req->buf, buf, req->length);
		if (ret != 0) {
			printk("copy_from_user fail\n");
		}
	}
}

static void hdb_debug_read_copy_to_user(char __user *buf, struct usb_request *req)
{
	hdb_debuginfo *dbg = (hdb_debuginfo *) req->buf;

	if (dbg != NULL && dbg->command == A_DBUG && dbg->headtoken == DBGHEADTOKEN
	    && dbg->tailtoken == DBGTAILTOKEN) {
		unsigned long ret;
		ret = copy_to_user(buf, req->buf, req->length);
		if (ret != 0) {
			printk("copy_to_user fail\n");
		}
		printk(KERN_INFO "adb_read A_DBUG (0x%x) (0x%x) (0x%x)\n", dbg->command,
		       dbg->msg_check, dbg->data_check);
	}
}

static void hdb_ready_callback(void);
static void hdb_closed_callback(void);

/* temporary variable used between adb_open() and adb_gadget_bind() */
static struct hdb_dev *_hdb_dev;

static inline struct hdb_dev *func_to_hdb(struct usb_function *f)
{
	return container_of(f, struct hdb_dev, function);
}


static struct usb_request *hdb_request_new(struct usb_ep *ep, int buffer_size)
{
	struct usb_request *req = usb_ep_alloc_request(ep, GFP_KERNEL);
	if (!req)
		return NULL;

	/* now allocate buffers for the requests */
	req->buf = kmalloc(buffer_size, GFP_KERNEL);
	if (!req->buf) {
		printk("%s %s %d: kmalloc failed\n", __FILE__, __func__, __LINE__);
		usb_ep_free_request(ep, req);
		return NULL;
	}

	return req;
}

static void hdb_request_free(struct usb_request *req, struct usb_ep *ep)
{
	if (req) {
		kfree(req->buf);
		usb_ep_free_request(ep, req);
	}
}

static inline int hdb_lock(atomic_t *excl)
{
	/*printk(KERN_DEBUG "%s %s %d: excl: %d\n", __FILE__, __func__, __LINE__, atomic_read(excl)); */
	if (atomic_inc_return(excl) == 1) {
		return 0;
	} else {
		atomic_dec(excl);
		return -1;
	}
}

static inline void hdb_unlock(atomic_t *excl)
{
	/*printk(KERN_DEBUG "%s %s %d: excl: %d\n", __FILE__, __func__, __LINE__, atomic_read(excl)); */
	atomic_dec(excl);
}

/* add a request to the tail of a list */
void hdb_req_put(struct hdb_dev *dev, struct list_head *head, struct usb_request *req)
{
	unsigned long flags;

	spin_lock_irqsave(&dev->lock, flags);
	list_add_tail(&req->list, head);
	spin_unlock_irqrestore(&dev->lock, flags);
}

/* remove a request from the head of a list */
struct usb_request *hdb_req_get(struct hdb_dev *dev, struct list_head *head)
{
	unsigned long flags;
	struct usb_request *req;

	spin_lock_irqsave(&dev->lock, flags);
	if (list_empty(head)) {
		req = 0;
	} else {
		req = list_first_entry(head, struct usb_request, list);
		list_del(&req->list);
	}
	spin_unlock_irqrestore(&dev->lock, flags);
	return req;
}

static void hdb_complete_in(struct usb_ep *ep, struct usb_request *req)
{
	struct hdb_dev *dev = _hdb_dev;

	if (req->status != 0)
		dev->error = 1;

	hdb_req_put(dev, &dev->tx_idle, req);

	wake_up(&dev->write_wq);
}

static void hdb_complete_out(struct usb_ep *ep, struct usb_request *req)
{
	struct hdb_dev *dev = _hdb_dev;

	dev->rx_done = 1;
	if (req->status != 0 && req->status != -ECONNRESET)
		dev->error = 1;

	wake_up(&dev->read_wq);
}

static int hdb_create_bulk_endpoints(struct hdb_dev *dev,
				     struct usb_endpoint_descriptor *in_desc,
				     struct usb_endpoint_descriptor *out_desc)
{
	struct usb_composite_dev *cdev = dev->cdev;
	struct usb_request *req;
	struct usb_ep *ep;
	int i;

	DBG(cdev, "%s %s %d: create_bulk_endpoints dev: %p\n", __FILE__, __func__, __LINE__, dev);

	ep = usb_ep_autoconfig(cdev->gadget, in_desc);
	/* __ADB_DEBUG__ start */
	hdb_ep_in = ep;
	/* __ADB_DEBUG__ end */
	if (!ep) {
		DBG(cdev, "%s %s %d: usb_ep_autoconfig for hdb_ep_in failed\n", __FILE__, __func__,
		    __LINE__);
		return -ENODEV;
	}
	DBG(cdev, "%s %s %d: usb_ep_autoconfig for hdb_ep_in got %s\n", __FILE__, __func__, __LINE__,
	    ep->name);
	ep->driver_data = dev;	/* claim the endpoint */
	dev->hdb_ep_in = ep;

	ep = usb_ep_autoconfig(cdev->gadget, out_desc);
	/* __ADB_DEBUG__ start */
	hdb_ep_out = ep;
	/* __ADB_DEBUG__ end */
	if (!ep) {
		DBG(cdev, "%s %s %d: usb_ep_autoconfig for hdb_ep_out failed\n", __FILE__, __func__,
		    __LINE__);
		return -ENODEV;
	}
	DBG(cdev, "%s %s %d: usb_ep_autoconfig for adb hdb_ep_out got %s\n", __FILE__, __func__,
	    __LINE__, ep->name);
	ep->driver_data = dev;	/* claim the endpoint */
	dev->hdb_ep_out = ep;

	/* now allocate requests for our endpoints */
	req = hdb_request_new(dev->hdb_ep_out, HDB_BULK_BUFFER_SIZE);
	if (!req)
		goto fail;
	req->complete = hdb_complete_out;
	dev->rx_req = req;

	for (i = 0; i < TX_REQ_MAX; i++) {
		req = hdb_request_new(dev->hdb_ep_in, HDB_BULK_BUFFER_SIZE);
		if (!req)
			goto fail;
		req->complete = hdb_complete_in;
		hdb_req_put(dev, &dev->tx_idle, req);
	}

	return 0;

 fail:
	printk(KERN_ERR "%s %s %d: Hdb_bind() could not allocate requests\n", __FILE__, __func__,
	       __LINE__);
	return -1;
}

static ssize_t hdb_read(struct file *fp, char __user *buf, size_t count, loff_t *pos)
{
	struct hdb_dev *dev = fp->private_data;
	struct usb_request *req;
	int r = count, xfer;
	int ret;
	static DEFINE_RATELIMIT_STATE(ratelimit, 1 * HZ, 10);

	pr_debug("%s %s %d: (%d)\n", __FILE__, __func__, __LINE__, (int)count);
	if (!_hdb_dev)
		return -ENODEV;

	if (count > HDB_BULK_BUFFER_SIZE) {
		printk("%s %s %d: count > ADB_BULK_BUFFER_SIZE\n", __FILE__, __func__, __LINE__);
		return -EINVAL;
	}


	if (hdb_lock(&dev->read_excl)) {
		if (__ratelimit(&ratelimit)) {
			printk("%s %s %d: Failed due to lock busy\n", __FILE__, __func__, __LINE__);
		}
		return -EBUSY;
	}

	/* we will block until we're online */
	while (!(dev->online || dev->error)) {
		pr_debug("%s %s %d: waiting for online state\n", __FILE__, __func__, __LINE__);
		ret = wait_event_interruptible(dev->read_wq, (dev->online || dev->error));
		if (ret < 0) {
			hdb_unlock(&dev->read_excl);
			return ret;
		}
	}
	if (dev->error) {
		r = -EIO;
		goto done;
	}

 requeue_req:
	/* queue a request */
	req = dev->rx_req;
	req->length = count;
	dev->rx_done = 0;

	/*
	 * The MAX_PAYLOAD of adb is 4096bytes defined in system/core/adb/adb.h
	 * So when meet the request has to read 4096bytes long payload,
	 * set short_not_ok is 1. Use musb dma mode 1 to speed up the write
	 * throughput.
	 */
	if (count == 65536)
		req->short_not_ok = 1;
	else
		req->short_not_ok = 0;

	if (bitdebug_enabled == 1) {
		hdb_debug_read_copy_from_user(buf, req);
	}

	ret = usb_ep_queue(dev->hdb_ep_out, req, GFP_ATOMIC);
	if (ret < 0) {
		/* FIXME */
		/* Process adbd would try to reconnect when usb has been reset. */
		/* It should not send data after endpoint has shutdown. */
		/* It is a workaround to reduce adb retry times. */
		if (ret == -ESHUTDOWN) {
			msleep(150);
		}

		if (bitdebug_enabled == 1) {
			if (ret == -EINPROGRESS) {
				hdb_debug_read_copy_to_user(buf, req);
				goto done;
			}
		}

		printk("%s %s %d: failed to queue req %p (%d)\n",
		       __FILE__, __func__, __LINE__, req, ret);
		r = -EIO;
		dev->error = 1;
		goto done;
	} else {
		pr_debug("%s %s %d: rx %p queue\n", __FILE__, __func__, __LINE__, req);
	}

	/* wait for a request to complete */
	ret = wait_event_interruptible(dev->read_wq, dev->rx_done);
	if (ret < 0) {
		if (ret != -ERESTARTSYS)
			dev->error = 1;
		r = ret;
		usb_ep_dequeue(dev->hdb_ep_out, req);
		goto done;
	}
	if (!dev->error) {
		/* If we got a 0-len packet, throw it back and try again. */
		if (req->actual == 0)
			goto requeue_req;

		pr_debug("%s %s %d: rx %p %d\n", __FILE__, __func__, __LINE__, req, req->actual);
		xfer = (req->actual < count) ? req->actual : count;

		/* __ADB_DEBUG__ start */
		if (bitdebug_enabled == 1) {
			if (count == sizeof(hdb_amessage)) {
				hdb_amessage *msg = (hdb_amessage *) req->buf;
				if (msg != NULL) {
					switch (msg->command) {
					case A_SYNC:
					case A_CNXN:
					case A_OPEN:
					case A_OKAY:
					case A_CLSE:
					case A_WRTE:
					case A_AUTH:
						/* printk(KERN_INFO "adb: adb_read (0x%x) (0x%x) (0x%x) (0x%x) (0x%x) (0x%x)\n", msg->command, msg->arg0, msg->arg1, */
						/* msg->data_length, msg->data_check, msg->magic); */
						break;
					default:
						/* printk(KERN_INFO "adb_read msg A_DATA\n"); */
						break;
					}
				}
			}
		}
		/* __ADB_DEBUG__ end */

		if (copy_to_user(buf, req->buf, xfer))
			r = -EFAULT;

	} else
		r = -EIO;

 done:
	hdb_unlock(&dev->read_excl);
	printk("%s %s %d: returning %d\n", __FILE__, __func__, __LINE__, r);

	return r;
}

static ssize_t hdb_write(struct file *fp, const char __user *buf, size_t count, loff_t *pos)
{
	struct hdb_dev *dev = fp->private_data;
	struct usb_request *req = 0;
	int r = count, xfer;
	int ret;
	static int flow_state;
	bool data;

	if (!_hdb_dev)
		return -ENODEV;
	pr_debug("%s %s %d:(%d)\n", __FILE__, __func__, __LINE__, (int)count);

	if (hdb_lock(&dev->write_excl)) {
		return -EBUSY;
	}

	while (count > 0) {
		if (dev->error) {
			pr_debug("%s %s %d: dev->error\n", __FILE__, __func__, __LINE__);
			r = -EIO;
			break;
		}

		/* get an idle tx request to use */
		req = 0;
		ret = wait_event_interruptible(dev->write_wq,
					       (req = hdb_req_get(dev, &dev->tx_idle))
					       || dev->error);

		if (ret < 0) {
			r = ret;
			break;
		}

		if (req != 0) {
			if (count > HDB_BULK_BUFFER_SIZE)
				xfer = HDB_BULK_BUFFER_SIZE;
			else
				xfer = count;
			if (copy_from_user(req->buf, buf, xfer)) {
				r = -EFAULT;
				break;
			}
			/* __ADB_DEBUG__ start */
			data = true;
			if (bitdebug_enabled == 1) {
				if (count == sizeof(hdb_amessage)) {
					hdb_amessage *msg = (hdb_amessage *) req->buf;
					if (msg != NULL) {
						switch (msg->command) {
						case A_SYNC:
						case A_CNXN:
						case A_OPEN:
						case A_OKAY:
						case A_CLSE:
						case A_WRTE:
						case A_AUTH:
							 printk(KERN_INFO "adb_write msg (0x%x) (0x%x) (0x%x) (0x%x) (0x%x) (0x%x)\n", msg->command, msg->arg0, msg->arg1, 
							 msg->data_length, msg->data_check, msg->magic); 
							if (flow_state == 0) {
								flow_state = 1;
								/* no data packet */
								if (msg->data_length == 0) {
									flow_state = 2;
								}
							} else {
								pr_debug
								    ("adb_write flow state msg warning\n");
								pr_debug
								    ("adb_write msg (0x%x) (0x%x) (0x%x) (0x%x) (0x%x) (0x%x)\n",
								     msg->command, msg->arg0,
								     msg->arg1, msg->data_length,
								     msg->data_check, msg->magic);
							}
							data = false;
							break;
						}
					}
				} else {
					data = true;
				}

				if (count == sizeof(hdb_debuginfo)) {
					hdb_debuginfo *dbg = (hdb_debuginfo *) req->buf;
					if (dbg != NULL && dbg->command == A_DBUG
					    && dbg->headtoken == DBGHEADTOKEN
					    && dbg->tailtoken == DBGTAILTOKEN) {
						/* printk(KERN_INFO "adb_write dbg (0x%x) (0x%x) (0x%x)\n", dbg->command, dbg->msg_check, dbg->data_check); */
						if (flow_state == 2) {
							flow_state = 0;
						} else {
							pr_debug
							    ("adb_write flow state debug warning\n");
							pr_debug
							    ("adb_write dbg (0x%x) (0x%x) (0x%x)\n",
							     dbg->command, dbg->msg_check,
							     dbg->data_check);
						}
						data = false;
						if (dbg->count == -1) {
							bitdebug_enabled = 0;
							hdb_bitdebug_writeCnt = 1;
							hdb_bitdebug_readCnt = 0;
							msleep(150);
							break;
						}
					}
				}

				if (data == true && bitdebug_enabled == 1) {
					if (flow_state == 1) {
						flow_state = 2;
					} else {
						pr_debug("hdb_write flow state data warning\n");
					}
					/* printk(KERN_INFO "adb_write data\n"); */
				}
			} else {
				if (count == sizeof(hdb_debuginfo)) {
					hdb_debuginfo *dbg = (hdb_debuginfo *) req->buf;
					if (dbg != NULL && dbg->command == A_DBUG
					    && dbg->headtoken == DBGHEADTOKEN
					    && dbg->tailtoken == DBGTAILTOKEN) {
						if (dbg->count == 0) {
							bitdebug_enabled = 1;
							flow_state = 0;
							/* req->length = sizeof(hdb_debuginfo); */
							msleep(150);
							break;
						}
					}
				}
			}
			/* __ADB_DEBUG__ end */

			req->length = xfer;
			ret = usb_ep_queue(dev->hdb_ep_in, req, GFP_ATOMIC);
			if (ret < 0) {
				pr_debug("%s %s %d: xfer error %d\n", __FILE__, __func__, __LINE__,
					 ret);
				dev->error = 1;
				r = -EIO;
				break;
			}

			buf += xfer;
			count -= xfer;

			/* zero this so we don't try to free it on error exit */
			req = 0;
		}
	}

	if (req)
		hdb_req_put(dev, &dev->tx_idle, req);

	hdb_unlock(&dev->write_excl);
	printk("%s %s %d: returning %d\n", __FILE__, __func__, __LINE__, r);

	return r;
}

static int hdb_open(struct inode *ip, struct file *fp);
static int hdb_release(struct inode *ip, struct file *fp);

/* file operations for hDB device /dev/android_hdb */
static struct file_operations hdb_fops = {
	.owner = THIS_MODULE,
	.read = hdb_read,
	.write = hdb_write,
	.open = hdb_open,
	.release = hdb_release,
};

static struct miscdevice hdb_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = hdb_shortname,
	.fops = &hdb_fops,
};

static int open_release_pair;

static spinlock_t open_lock;

static int hdb_open(struct inode *ip, struct file *fp)
{
	int ret = 0;
	unsigned long flags;
	bitdebug_enabled = 0;

	spin_lock_irqsave(&open_lock, flags);

	pr_debug("[adb]adb_open start, adb_open: %p check adb_release %p, open_release_pair: %d\n",
		 adb_fops.open, adb_fops.release, open_release_pair);

	if (!_hdb_dev) {
		printk("[hdb]hdb_open _hdb_dev is NULL, open_release_pair: %d\n",
		       open_release_pair);
		open_release_pair = 0;
		ret = ENODEV;
		goto OPEN_END;
	}

	/*Workaround for being unable to call adb_release from adbd */
	if (open_release_pair > 0) {
		pr_debug("[XLOG_WARN][adb]open twice, %s %s %d: open_release_pair count: %d\n",
			 __FILE__, __func__, __LINE__, open_release_pair);
		ret = 0;
		fp->private_data = _hdb_dev;
		goto OPEN_END;
	}

	open_release_pair++;
	fp->private_data = _hdb_dev;
	/* clear the error latch */
	_hdb_dev->error = 0;

	hdb_ready_callback();

 OPEN_END:
	pr_debug("[adb]adb_open end, open_release_pair: %d", open_release_pair);

	spin_unlock_irqrestore(&open_lock, flags);

	return ret;
}

static int hdb_release(struct inode *ip, struct file *fp)
{
	int ret = 0;
	unsigned long flags;

	spin_lock_irqsave(&open_lock, flags);

	pr_debug
	    ("[adb]adb_release start, adb_open: %p check adb_release %p, open_release_pair: %d\n",
	     adb_fops.open, adb_fops.release, open_release_pair);

	if (open_release_pair < 1) {
		pr_debug
		    ("[XLOG_WARN][adb][adb] close an unopened device, %s %s %d: open_release_pair count: %d\n",
		     __FILE__, __func__, __LINE__, open_release_pair);
		ret = -1;
		goto RELEASE_END;
	}

	hdb_closed_callback();

	open_release_pair--;

 RELEASE_END:

	pr_debug("[adb]adb_release end, open_release_pair: %d", open_release_pair);

	spin_unlock_irqrestore(&open_lock, flags);

	return ret;
}

static int hdb_function_bind(struct usb_configuration *c, struct usb_function *f)
{
	struct usb_composite_dev *cdev = c->cdev;
	struct hdb_dev *dev = func_to_hdb(f);
	int id;
	int ret;

	dev->cdev = cdev;
	DBG(cdev, "%s %s %d: dev: %p\n", __FILE__, __func__, __LINE__, dev);

	/* allocate interface ID(s) */
	id = usb_interface_id(c, f);
	if (id < 0)
		return id;
	hdb_interface_desc.bInterfaceNumber = id;

	/* allocate endpoints */
	ret = hdb_create_bulk_endpoints(dev, &hdb_fullspeed_in_desc, &hdb_fullspeed_out_desc);
	if (ret)
		return ret;

	/* support high speed hardware */
	if (gadget_is_dualspeed(c->cdev->gadget)) {
		hdb_highspeed_in_desc.bEndpointAddress = hdb_fullspeed_in_desc.bEndpointAddress;
		hdb_highspeed_out_desc.bEndpointAddress = hdb_fullspeed_out_desc.bEndpointAddress;
	}
	/* support super speed hardware */
	if (gadget_is_superspeed(c->cdev->gadget)) {
		hdb_superspeed_in_desc.bEndpointAddress = adb_fullspeed_in_desc.bEndpointAddress;
		hdb_superspeed_out_desc.bEndpointAddress = adb_fullspeed_out_desc.bEndpointAddress;
	}

	DBG(cdev, "%s %s %d: %s speed %s: IN/%s, OUT/%s\n", __FILE__, __func__, __LINE__,
	    gadget_is_dualspeed(c->cdev->gadget) ? "dual" : "full",
	    f->name, dev->hdb_ep_in->name, dev->hdb_ep_out->name);
	return 0;
}

static void hdb_function_unbind(struct usb_configuration *c, struct usb_function *f)
{
	struct hdb_dev *dev = func_to_hdb(f);
	struct usb_request *req;


	dev->online = 0;
	dev->error = 1;

	wake_up(&dev->read_wq);

	hdb_request_free(dev->rx_req, dev->hdb_ep_out);
	while ((req = hdb_req_get(dev, &dev->tx_idle)))
		hdb_request_free(req, dev->hdb_ep_in);
}

static int hdb_function_set_alt(struct usb_function *f, unsigned intf, unsigned alt)
{
	struct hdb_dev *dev = func_to_hdb(f);
	struct usb_composite_dev *cdev = f->config->cdev;
	int ret;

	DBG(cdev, "hdb_function_set_alt intf: %d alt: %d\n", intf, alt);

#ifdef CONFIG_USBIF_COMPLIANCE
	if (dev->online){
		return 0 ;
	}
#endif

	ret = config_ep_by_speed(cdev->gadget, f, dev->hdb_ep_in);
	if (ret)
		return ret;

	ret = usb_ep_enable(dev->hdb_ep_in);
	if (ret)
		return ret;

	ret = config_ep_by_speed(cdev->gadget, f, dev->hdb_ep_out);
	if (ret) {
		printk("%s %s %d: usb_ep_enable in failed\n", __FILE__, __func__, __LINE__);
		return ret;
	}

	ret = usb_ep_enable(dev->hdb_ep_out);
	if (ret) {
		usb_ep_disable(dev->hdb_ep_in);
		printk("%s %s %d: usb_ep_enable out failed\n", __FILE__, __func__, __LINE__);
		return ret;
	}
	dev->online = 1;

	/* readers may be blocked waiting for us to go online */
	wake_up(&dev->read_wq);
	return 0;
}

static void hdb_function_disable(struct usb_function *f)
{
	struct hdb_dev *dev = func_to_hdb(f);
	//struct usb_composite_dev *cdev = dev->cdev;

	//DBG(cdev, "%s %s %d: cdev %p\n", __FILE__, __func__, __LINE__, cdev);
	dev->online = 0;
	dev->error = 1;
	usb_ep_disable(dev->hdb_ep_in);
	usb_ep_disable(dev->hdb_ep_out);

	/* readers may be blocked waiting for us to go online */
	wake_up(&dev->read_wq);

	//VDBG(cdev, "%s %s %d: %s disabled\n", __FILE__, __func__, __LINE__, dev->function.name);
}

static int hdb_bind_config(struct usb_configuration *c)
{
	struct hdb_dev *dev = _hdb_dev;

	printk(KERN_INFO "%s %s %d\n", __FILE__, __func__, __LINE__);

	dev->cdev = c->cdev;
	dev->function.name = "hdb";
	dev->function.fs_descriptors = fs_hdb_descs;
	dev->function.hs_descriptors = hs_hdb_descs;
	if (gadget_is_superspeed(c->cdev->gadget)) {
		printk("[hDB] SS hDB DESCS!!\n");
		dev->function.ss_descriptors = ss_hdb_descs;
	}
	dev->function.bind = hdb_function_bind;
	dev->function.unbind = hdb_function_unbind;
	dev->function.set_alt = hdb_function_set_alt;
	dev->function.disable = hdb_function_disable;

	return usb_add_function(c, &dev->function);
}

static int hdb_setup(void)
{
	struct hdb_dev *dev;
	int ret;

	dev = kzalloc(sizeof(*dev), GFP_KERNEL);
	if (!dev)
		return -ENOMEM;

	spin_lock_init(&dev->lock);

	spin_lock_init(&open_lock);

	init_waitqueue_head(&dev->read_wq);
	init_waitqueue_head(&dev->write_wq);

	atomic_set(&dev->open_excl, 0);
	atomic_set(&dev->read_excl, 0);
	atomic_set(&dev->write_excl, 0);

	INIT_LIST_HEAD(&dev->tx_idle);

	_hdb_dev = dev;

	ret = misc_register(&hdb_device);
	if (ret)
		goto err;

	return 0;

 err:
	kfree(dev);
	printk(KERN_ERR "%s %s %d: adb gadget driver failed to initialize\n", __FILE__, __func__,
	       __LINE__);
	return ret;
}

static void hdb_cleanup(void)
{
	misc_deregister(&hdb_device);

	kfree(_hdb_dev);
	_hdb_dev = NULL;
}
