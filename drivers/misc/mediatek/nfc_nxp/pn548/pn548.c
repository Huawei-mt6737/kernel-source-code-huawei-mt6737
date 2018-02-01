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
#include <linux/miscdevice.h>
#include <linux/spinlock.h>
#include <linux/dma-mapping.h>
#include <linux/proc_fs.h> 
#include <linux/regulator/consumer.h>
#include <pn548.h>

#ifdef CONFIG_HQ_HWINFO
#include "hwinfo.h"
#endif

#ifdef CONFIG_OF
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/of_address.h>
#endif

#define PN544_DRVNAME		"pn544"
#define MAX_BUFFER_SIZE		512
#define I2C_ID_NAME			"pn544"

#define PN544_MAGIC			0xE9
#define PN544_SET_PWR		_IOW(PN544_MAGIC, 0x01, unsigned int)


#define NFC_TRY_NUM 3
#define UICC_SUPPORT_CARD_EMULATION (1<<0)
#define eSE_SUPPORT_CARD_EMULATION (1<<1)
#define CARD_UNKNOWN	0
#define CARD1_SELECT    1
#define CARD2_SELECT    2
#define MAX_ATTRIBUTE_BUFFER_SIZE 128

#define ENABLE_START			0
#define ENABLE_END				1
#define MAX_CONFIG_NAME_SIZE	64
//#define	MAX_BRCM_CONFIG_NAME_SIZE	64
#define MAX_NFC_CHIP_TYPE_SIZE  32
static char g_nfc_nxp_config_name[MAX_CONFIG_NAME_SIZE];
static char g_nfc_brcm_config_name[MAX_CONFIG_NAME_SIZE];
static char nfc_chip_type[MAX_NFC_CHIP_TYPE_SIZE];


static int firmware_update = 0;
//static int lcd_status = UNKNOWN_LCD;

#define NFC_PINCTRL_STATE_ACTIVE	"nfc_active"
#define NFC_PINCTRL_STATE_SUSPEND	"nfc_suspend"

#define _read_sample_test_size  		40

#ifdef CONFIG_HQ_HWINFO
struct hwinfo * nfc_hwinfo = NULL;

static char *nfc_firmware_kbuf = NULL;

static char *nfc_firmware = NULL;
static char *nfc_devinfo = "PN548C2,NXP";
#endif

extern void mt_eint_registration(unsigned int eint_num, unsigned int flow, void (EINT_FUNC_PTR)(void), unsigned int is_auto_umask);

struct pn544_dev	
{
	wait_queue_head_t	read_wq;
	struct mutex		read_mutex;
	struct i2c_client	*client;
	struct miscdevice	pn544_device;
	bool			irq_enabled;
	spinlock_t		irq_enabled_lock;

	int			irq_gpio;
	struct regulator	*reg;

	//struct device		*dev;
	unsigned int		sim_status;
	unsigned int		sim_switch;
	unsigned int		enable_status;
};

//struct pn544_i2c_platform_data pn544_platform_data;

struct pinctrl *gpctrl = NULL;
struct pinctrl_state *st_ven_h = NULL;
struct pinctrl_state *st_ven_l = NULL;
struct pinctrl_state *st_dwn_h = NULL;
struct pinctrl_state *st_dwn_l = NULL;

struct pinctrl_state *st_eint_int = NULL;

/* For DMA */
#ifdef CONFIG_MTK_I2C_EXTENSION
static char *I2CDMAWriteBuf;	/*= NULL;*//* unnecessary initialise */
static unsigned int I2CDMAWriteBuf_pa;	/* = NULL; */
static char *I2CDMAReadBuf;	/*= NULL;*//* unnecessary initialise */
static unsigned int I2CDMAReadBuf_pa;	/* = NULL; */
#else
static char I2CDMAWriteBuf[MAX_BUFFER_SIZE];
static char I2CDMAReadBuf[MAX_BUFFER_SIZE];
#endif

static int rgb_nfc_debug_mask = 1;
#define NFC_ERR(x...) do {\
    if (rgb_nfc_debug_mask >= 0) \
        printk(KERN_ERR x);\
    } while (0)

#define NFC_INFO(x...) do {\
    if (rgb_nfc_debug_mask >= 1) \
        printk(KERN_ERR x);\
    } while (0)
#define NFC_FLOW(x...) do {\
    if (rgb_nfc_debug_mask >= 2) \
        printk(KERN_ERR x);\
    } while (0)

	
 #if 1
#include <dsm/dsm_pub.h>

#define DSM_NFC_MOD_NAME        "dsm_nfc"
#define DSM_NFC_BUF_SIZE		2048

static int dsm_nfc_errno = 0;

#ifdef CONFIG_HUAWEI_DSM
static struct dsm_dev nfc_dsm_info = {
    .name = DSM_NFC_MOD_NAME,
    .fops = NULL,
    .buff_size = DSM_NFC_BUF_SIZE,
};

static struct dsm_client *nfc_dclient = NULL;

int nfc_dsm_register(void)
{
    if (NULL != nfc_dclient)
    {
        printk(KERN_INFO "nfc_dclient had been register!\n");
        return 0;
    }

    nfc_dclient = dsm_register_client(&nfc_dsm_info);
    if(NULL == nfc_dclient)
    {
        printk(KERN_ERR "nfc_dclient register failed!\n");
    }

    return 0;
}
EXPORT_SYMBOL(nfc_dsm_register);

int nfc_dsm_report_num(int dsm_err_no, char *err_msg, int err_code)
{
    int err = 0;

    if(NULL == nfc_dclient)
    {
        printk(KERN_ERR "%s nfc_dclient did not register!\n", __func__);
        return 0;
    }

    err = dsm_client_ocuppy(nfc_dclient);
    if(0 != err)
    {
        printk(KERN_ERR "%s user buffer is busy!\n", __func__);
        return 0;
    }

    printk(KERN_ERR "%s user buffer apply successed, dsm_err_no=%d, err_code=%d!\n",
        __func__, dsm_err_no, err_code);

    err = dsm_client_record(nfc_dclient, "err_msg:%s;err_code:%d;\n",err_msg,err_code);
    dsm_client_notify(nfc_dclient, dsm_err_no);

    return 0;
}
EXPORT_SYMBOL(nfc_dsm_report_num);

int nfc_dsm_report_info(int error_no, void *log, int size)
{
    int err = 0;
    int rsize = 0;

    if(NULL == nfc_dclient)
    {
        printk(KERN_ERR "%s nfc_dclient did not register!\n", __func__);
        return 0;
    }

    if((error_no < DSM_NFC_I2C_WRITE_ERROR_NO) || (error_no > DSM_NFC_SIM_CHECK_ERROR_NO) || (NULL == log) || (size < 0))
    {
        printk(KERN_ERR "%s input param error!\n", __func__);
        return 0;
    }

    err = dsm_client_ocuppy(nfc_dclient);
    if(0 != err)
    {
        printk(KERN_ERR "%s user buffer is busy!\n", __func__);
        return 0;
    }

    if(size > DSM_NFC_BUF_SIZE)
    {
        rsize = DSM_NFC_BUF_SIZE;
    }
    else
    {
        rsize = size;
    }
    err = dsm_client_copy(nfc_dclient, log, rsize);

    dsm_client_notify(nfc_dclient, error_no);

    return 0;
}
EXPORT_SYMBOL(nfc_dsm_report_info);

#else
int inline nfc_dsm_register()
{
    return 0;
}
EXPORT_SYMBOL(nfc_dsm_register);

int nfc_dsm_report_num(int dsm_err_no, char *err_msg, int err_code)
{
    return 0;
}
EXPORT_SYMBOL(nfc_dsm_report_num);

int inline nfc_dsm_report_info(int error_no, void *log, int size)
{
    return 0;
}
EXPORT_SYMBOL(nfc_dsm_report_info);
#endif

#endif

static void get_nfc_config_name(void)
{
	int ret=-1;
	struct device_node *node;
	const char *out_value = NULL;

	memset(g_nfc_nxp_config_name, 0, MAX_CONFIG_NAME_SIZE);
	memset(g_nfc_brcm_config_name, 0, MAX_CONFIG_NAME_SIZE);

	/*get huawei_nfc_info node*/
	node = of_find_compatible_node(NULL, NULL, "mediatek,nfc-pn548");

	if(node)
	{
		/*get nfc_brcm_conf_name*/
		ret=of_property_read_string(node, "nfc_nxp_conf_name", &out_value);
		strncpy(g_nfc_nxp_config_name, out_value, MAX_CONFIG_NAME_SIZE-1);
		if (ret != 0){
			memset(g_nfc_nxp_config_name, 0, MAX_CONFIG_NAME_SIZE);
			printk("%s: can't get nfc nxp config name\n", __func__);
		}
		printk("%s: nfc nxp config name:%s\n", __func__, g_nfc_nxp_config_name);

		//get nfc_brcm_conf_name
		ret=of_property_read_string(node, "nfc_brcm_conf_name", &out_value);
		strncpy(g_nfc_brcm_config_name, out_value, MAX_CONFIG_NAME_SIZE-1);
		if (ret != 0){
			memset(g_nfc_brcm_config_name, 0, MAX_CONFIG_NAME_SIZE);
			printk("%s: can't get nfc brcm config name\n", __func__);
		}
		printk("%s: nfc brcm config name:%s\n", __func__, g_nfc_brcm_config_name);
			
	}
	
	return ;
}



static void pn544_enable_irq(struct pn544_dev *pn544_dev)
{
	unsigned long flags;

	spin_lock_irqsave(&pn544_dev->irq_enabled_lock, flags);
	enable_irq(pn544_dev->client->irq);
	pn544_dev->irq_enabled = true;
	spin_unlock_irqrestore(&pn544_dev->irq_enabled_lock, flags);
}

static void pn544_disable_irq(struct pn544_dev *pn544_dev)
{
	unsigned long flags;
	
	spin_lock_irqsave(&pn544_dev->irq_enabled_lock, flags);
	if (pn544_dev->irq_enabled) 
	{
		disable_irq_nosync(pn544_dev->client->irq);
		pn544_dev->irq_enabled = false;
	}
	spin_unlock_irqrestore(&pn544_dev->irq_enabled_lock, flags);
}

static irqreturn_t pn544_dev_irq_handler(int irq, void *dev)
{
	struct pn544_dev *pn544_dev = dev;		

	pn544_disable_irq(pn544_dev);

	/* Wake up waiting readers */
	wake_up(&pn544_dev->read_wq);

	return IRQ_HANDLED;
}

static int pn544_platform_pinctrl_select(struct pinctrl *p, struct pinctrl_state *s)
{
	int ret = 0;

	if (p != NULL && s != NULL) {
		ret = pinctrl_select_state(p, s);
	} else {
		printk("%s: pinctrl_select err\n", __func__);
		ret = -1;
	}

	return ret;
}

static ssize_t pn544_dev_read(struct file *filp, char __user *buf,
			       size_t count, loff_t *offset)
{
	struct pn544_dev *pn544_dev = filp->private_data;
	int ret=0;

	if (count > MAX_BUFFER_SIZE)
		count = MAX_BUFFER_SIZE;

	//NFC_INFO("pn544 %s : reading %zu bytes.\n", __func__, count);
	if (!gpio_get_value(pn544_dev->irq_gpio)) {
		//NFC_INFO("pn544 read no event\n");		
		if (filp->f_flags & O_NONBLOCK) 
		{
			ret = -EAGAIN;
			goto fail;
		}
		
		//NFC_INFO("pn544 read wait event\n");		
		pn544_enable_irq(pn544_dev);				
		ret = wait_event_interruptible(pn544_dev->read_wq, gpio_get_value(pn544_dev->irq_gpio));
		pn544_disable_irq(pn544_dev);
		if (ret) 
		{
			NFC_ERR("pn544 read wait event error\n");
			goto fail;
		}
	}
    mutex_lock(&pn544_dev->read_mutex);

#ifdef CONFIG_MTK_I2C_EXTENSION
	if (count < 8) {
		pn544_dev->client->addr = ((pn544_dev->client->addr & I2C_MASK_FLAG) | (I2C_ENEXT_FLAG));
		pn544_dev->client->timing = 400;
		ret = i2c_master_recv(pn544_dev->client, I2CDMAReadBuf, count);
	}
	else
	{
		pn544_dev->client->addr = (((pn544_dev->client->addr & I2C_MASK_FLAG) | (I2C_DMA_FLAG)) | (I2C_ENEXT_FLAG));
		pn544_dev->client->timing = 400;
		ret = i2c_master_recv(pn544_dev->client, (unsigned char *)(uintptr_t)I2CDMAReadBuf_pa, count);
	}

#else
	if (count < 8) {
	      ret = i2c_master_recv(pn544_dev->client, I2CDMAReadBuf, count);
	} else {
		ret = i2c_master_recv(pn544_dev->client, (unsigned char *)(uintptr_t)I2CDMAReadBuf, count);
	}
#endif     /* CONFIG_MTK_I2C_EXTENSION */
	mutex_unlock(&pn544_dev->read_mutex);

	if (ret != (int)count) {
		NFC_ERR("%s : i2c_master_recv returned %d\n", __func__, ret);
		#ifdef CONFIG_HUAWEI_DSM
		if (-EOPNOTSUPP == ret) {
			nfc_dsm_report_num(DSM_NFC_I2C_READ_EOPNOTSUPP_ERROR_NO,
				"I2C Read Error, Error Type: Operation not supported on transport endpoint\n",ret);
		} else if (-EREMOTEIO == ret) {
			nfc_dsm_report_num(DSM_NFC_I2C_READ_EREMOTEIO_ERROR_NO,
				"I2C Read Error, Error Type: Remote I/O error \n",ret);
		} else {
			nfc_dsm_report_num(DSM_NFC_I2C_READ_ERROR_NO,
				"I2C Read Error, Error Type: Others \n",ret);
		}
		#endif
		//ret = -EIO;
	}

	if (ret < 0) 
	{
		NFC_ERR("pn544 %s: i2c_master_recv returned %d\n", __func__, ret);
		return ret;
	}
	if (ret > count) 
	{
		NFC_ERR("pn544 %s: received too many bytes from i2c (%d)\n", __func__, ret);
		return -EIO;
	}
	if (copy_to_user(buf, I2CDMAReadBuf, ret)) {
		NFC_ERR("%s : failed to copy to user space\n", __func__);
		return -EFAULT;
	}
	return ret;

fail:
	return ret;
}


static ssize_t pn544_dev_write(struct file *filp, const char __user *buf,
				size_t count, loff_t *offset)
{
	struct pn544_dev *pn544_dev;
	int ret=0, idx = 0;

	pn544_dev = filp->private_data;

	if (count > MAX_BUFFER_SIZE)
		count = MAX_BUFFER_SIZE;

	if (copy_from_user(I2CDMAWriteBuf, &buf[(idx*MAX_BUFFER_SIZE)], count)) 
	{
		NFC_ERR(KERN_DEBUG "%s : failed to copy from user space\n", __func__);
		return -EFAULT;
	}
    mutex_lock(&pn544_dev->read_mutex);

#ifdef CONFIG_MTK_I2C_EXTENSION
	if (count < 8) {
	    	pn544_dev->client->addr = ((pn544_dev->client->addr & I2C_MASK_FLAG) | (I2C_ENEXT_FLAG));
	     	pn544_dev->client->timing = 400;
	     	ret = i2c_master_send(pn544_dev->client, I2CDMAWriteBuf, count);
	}
	else
	{
		    pn544_dev->client->addr = (((pn544_dev->client->addr & I2C_MASK_FLAG) | (I2C_DMA_FLAG)) | (I2C_ENEXT_FLAG));
		    pn544_dev->client->timing = 400;
		    ret = i2c_master_send(pn544_dev->client, (unsigned char *)(uintptr_t)I2CDMAWriteBuf_pa, count);
	}
#else
	if (count < 8)
	{
		 ret = i2c_master_send(pn544_dev->client, I2CDMAWriteBuf, count);
	}
	else
	{
		ret = i2c_master_send(pn544_dev->client, (unsigned char *)(uintptr_t)I2CDMAWriteBuf, count);
	}
#endif    /* CONFIG_MTK_I2C_EXTENSION */

	mutex_unlock(&pn544_dev->read_mutex);

	if(ret != (int)count) {
		NFC_ERR("%s : i2c_master_send returned %d\n", __func__, ret);
		#ifdef CONFIG_HUAWEI_DSM
		if (-EOPNOTSUPP == ret) {
			nfc_dsm_report_num(DSM_NFC_I2C_WRITE_EOPNOTSUPP_ERROR_NO,
				"I2C Write Error, Error Type: Operation not supported on transport endpoint\n",ret);
		} else if (-EREMOTEIO == ret) {
			nfc_dsm_report_num(DSM_NFC_I2C_WRITE_EREMOTEIO_ERROR_NO,
				"I2C Write Error, Error Type: Remote I/O error \n",ret);
		} else {
			nfc_dsm_report_num(DSM_NFC_I2C_WRITE_ERROR_NO,
				"I2C Write Error, Error Type: Others \n",ret);
		}
		#endif
		ret = -EIO;
	}

	return ret;
}

/*
*	name:	pn544_i2c_read
*	use:	Internal read function
*/
static ssize_t pn544_i2c_read(struct pn544_dev *pn544_dev, unsigned char *buf, int count)
{
	int ret=0;

	if (count > MAX_BUFFER_SIZE)
		count = MAX_BUFFER_SIZE;
	
	if (!gpio_get_value(pn544_dev->irq_gpio)) {
		pn544_enable_irq(pn544_dev);
		ret = wait_event_interruptible(pn544_dev->read_wq, gpio_get_value(pn544_dev->irq_gpio));
		pn544_disable_irq(pn544_dev);
		if (ret) 
		{
			NFC_ERR("pn544 read wait event error\n");
			goto fail;
		}
	}

	mutex_lock(&pn544_dev->read_mutex);

#ifdef CONFIG_MTK_I2C_EXTENSION
	if (count < 8) {
         pn544_dev->client->addr = ((pn544_dev->client->addr & I2C_MASK_FLAG) | (I2C_ENEXT_FLAG));
         pn544_dev->client->timing = 400;
         ret = i2c_master_recv(pn544_dev->client, I2CDMAReadBuf, count);
	} else {
   		 pn544_dev->client->addr = (((pn544_dev->client->addr & I2C_MASK_FLAG) | (I2C_DMA_FLAG)) | (I2C_ENEXT_FLAG));
   		 pn544_dev->client->timing = 400;
         ret = i2c_master_recv(pn544_dev->client, (unsigned char *)(uintptr_t)I2CDMAReadBuf_pa, count);
	}
#else
	if (count < 8) {
         ret = i2c_master_recv(pn544_dev->client, I2CDMAReadBuf, count);
    } else {
         ret = i2c_master_recv(pn544_dev->client, (unsigned char *)(uintptr_t)I2CDMAReadBuf, count);
    }
#endif     /* CONFIG_MTK_I2C_EXTENSION */
	mutex_unlock(&pn544_dev->read_mutex);

	if(ret != (int)count) {
		NFC_ERR("%s : i2c_master_recv returned %d\n", __func__, ret);
		#ifdef CONFIG_HUAWEI_DSM
		if (-EOPNOTSUPP == ret) {
			nfc_dsm_report_num(DSM_NFC_I2C_READ_EOPNOTSUPP_ERROR_NO,
				"I2C Read Error, Error Type: Operation not supported on transport endpoint\n",ret);
		} else if (-EREMOTEIO == ret) {
			nfc_dsm_report_num(DSM_NFC_I2C_READ_EREMOTEIO_ERROR_NO,
				"I2C Read Error, Error Type: Remote I/O error \n",ret);
		} else {
			nfc_dsm_report_num(DSM_NFC_I2C_READ_ERROR_NO,
				"I2C Read Error, Error Type: Others \n",ret);
		}
		#endif
		//ret = -EIO;
	}

	if (ret < 0) 
	{
		NFC_ERR("pn544 %s: i2c_master_recv returned %d\n", __func__, ret);
		return ret;
	}
	if (ret > count) 
	{
		NFC_ERR("pn544 %s: received too many bytes from i2c (%d)\n", __func__, ret);
		return -EIO;
	}
	memcpy(buf, I2CDMAReadBuf, count);
	return ret;

fail:
	return ret;
}


/*
*	name:	pn544_i2c_write
*	use:	Internal read function
*/
static ssize_t pn544_i2c_write(struct pn544_dev *pn544_dev, char *buf, int count)
{
	int ret=0; 
    if (count > MAX_BUFFER_SIZE)
		count = MAX_BUFFER_SIZE;

	mutex_lock(&pn544_dev->read_mutex);

	memcpy(I2CDMAWriteBuf,  buf, count);

#ifdef CONFIG_MTK_I2C_EXTENSION
	if (count < 8) {
		pn544_dev->client->addr = ((pn544_dev->client->addr & I2C_MASK_FLAG) | (I2C_ENEXT_FLAG));
		pn544_dev->client->timing = 400;
		ret = i2c_master_send(pn544_dev->client, I2CDMAWriteBuf, count);
	}
	else
	{
		pn544_dev->client->addr = (((pn544_dev->client->addr & I2C_MASK_FLAG) | (I2C_DMA_FLAG)) | (I2C_ENEXT_FLAG));
		pn544_dev->client->timing = 400;
		ret = i2c_master_send(pn544_dev->client, (unsigned char *)(uintptr_t)I2CDMAWriteBuf_pa, count);
	}
#else
	if (count < 8)
	{
		 ret = i2c_master_send(pn544_dev->client, I2CDMAWriteBuf, count);
	}
	else
	{
	    	 ret = i2c_master_send(pn544_dev->client, (unsigned char *)(uintptr_t)I2CDMAWriteBuf, count);
	}
#endif    /* CONFIG_MTK_I2C_EXTENSION */
	mutex_unlock(&pn544_dev->read_mutex);
	if(ret != (int)count) {
		NFC_ERR("%s : i2c_master_send returned %d\n", __func__, ret);
		#ifdef CONFIG_HUAWEI_DSM
		if (-EOPNOTSUPP == ret) {
			nfc_dsm_report_num(DSM_NFC_I2C_WRITE_EOPNOTSUPP_ERROR_NO,
				"I2C Write Error, Error Type: Operation not supported on transport endpoint\n",ret);
		} else if (-EREMOTEIO == ret) {
			nfc_dsm_report_num(DSM_NFC_I2C_WRITE_EREMOTEIO_ERROR_NO,
				"I2C Write Error, Error Type: Remote I/O error \n",ret);
		} else {
			nfc_dsm_report_num(DSM_NFC_I2C_WRITE_ERROR_NO,
				"I2C Write Error, Error Type: Others \n",ret);
		}
		#endif
		ret = -EIO;
	}

	return ret;
}

static int pn544_dev_open(struct inode *inode, struct file *filp)
{
	int ret = 0;
	struct pn544_dev *pn544_dev = container_of(filp->private_data, struct pn544_dev, pn544_device);
	
	NFC_INFO("%s:pn544_dev=%p\n", __func__, pn544_dev);

	filp->private_data = pn544_dev;
	
	pr_debug("pn544 %s : %d,%d\n", __func__, imajor(inode), iminor(inode));
	
	return ret;
}

static long pn544_dev_unlocked_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	int ret;
	//struct pn544_dev *pn544_dev = filp->private_data;

	//printk("%s:cmd=%d, arg=%ld, pn544_dev=%p\n", __func__, cmd, arg, pn544_dev);

	switch (cmd) 
	{
		case PN544_SET_PWR:
			if (arg == 2) {
				/* power on with firmware download (requires hw reset) */
				printk("pn544 %s power on with firmware\n", __func__);
				ret = pn544_platform_pinctrl_select(gpctrl, st_ven_h);
				ret = pn544_platform_pinctrl_select(gpctrl, st_dwn_h);
				msleep(10);
				ret = pn544_platform_pinctrl_select(gpctrl, st_ven_l);
				msleep(10);//50
				ret = pn544_platform_pinctrl_select(gpctrl, st_ven_h);
				msleep(10);
			} else if (arg == 1) {
				/* power on */
				//printk("pn544 %s power on\n", __func__);
				ret = pn544_platform_pinctrl_select(gpctrl, st_dwn_l);
				ret = pn544_platform_pinctrl_select(gpctrl, st_ven_h); 
				msleep(10);
			} else  if (arg == 0) {
				/* power off */
				//printk("pn544 %s power off\n", __func__);
				ret = pn544_platform_pinctrl_select(gpctrl, st_dwn_l);
				ret = pn544_platform_pinctrl_select(gpctrl, st_ven_l);
				msleep(10);//50
			} else {
				printk("pn544 %s bad arg %lu\n", __func__, arg);
				return -EINVAL;
			}
			break;
		default:
			printk("pn544 %s bad ioctl %u\n", __func__, cmd);
			return -EINVAL;
	}

	return ret;
}


static const struct file_operations pn544_dev_fops = {
	.owner = THIS_MODULE,
	.llseek = no_llseek,
	.read = pn544_dev_read,
	.write = pn544_dev_write,
	.open = pn544_dev_open,
#ifdef CONFIG_COMPAT
	.compat_ioctl = pn544_dev_unlocked_ioctl,
#endif
	.unlocked_ioctl = pn544_dev_unlocked_ioctl,
};

/*FUNCTION: pn544_enable_nfc
  *DESCRIPTION: reset cmd sequence to enable pn547
  *Parameters
  * struct  pn544_dev *pdev: device structure
  *RETURN VALUE
  * none */
static void pn544_enable_nfc(struct  pn544_dev *pdev)
{
	/*hardware reset*/
	pn544_platform_pinctrl_select(gpctrl, st_ven_h);
	msleep(10);
	pn544_platform_pinctrl_select(gpctrl, st_ven_l);
	msleep(10);//50
	pn544_platform_pinctrl_select(gpctrl, st_ven_h);
	msleep(10);
}

static int check_sim_status(struct i2c_client *client, struct  pn544_dev *pdev)
{
	int ret;

	unsigned char recvBuf[40];
	unsigned char send_reset[] = {0x20,0x00,0x01,0x01};//CORE_RESET_CMD
	unsigned char init_cmd[] = {0x20,0x01,0x00}; //CORE_INIT_CMD

	unsigned char read_config[] = {0x2F,0x02,0x00}; //SYSTEM_PROPRIETARY_ACT_CMD
	unsigned char read_config_UICC[] = {0x2F,0x3E,0x01,0x00};//SYSTEM_TEST_SWP_INTERFACE_CMD(swp1)
	unsigned char read_config_eSE[] = {0x2F,0x3E,0x01,0x01};	//SYSTEM_TEST_SWP_INTERFACE_CMD(eSE or swp2)

	printk("pn547 - %s : enter\n", __func__);

	pdev->sim_status = 0;
	/*hardware reset*/
	//gpio_set_value(pdev->firm_gpio, 0);
	pn544_enable_nfc(pdev);

	/*write CORE_RESET_CMD*/
	ret = pn544_i2c_write(pdev, send_reset, sizeof(send_reset));
	if (ret < 0) {
		NFC_ERR("%s: CORE_RESET_CMD pn547_i2c_write failed, ret:%d\n", __func__, ret);
		goto failed;
	}
	/*read response*/
	ret = pn544_i2c_read(pdev, recvBuf, _read_sample_test_size);
	if (ret < 0) {
		NFC_ERR("%s: CORE_RESET_RSP pn547_i2c_read failed, ret:%d\n", __func__, ret);
		goto failed;
	}

	udelay(500);
	/*write CORE_INIT_CMD*/
	ret = pn544_i2c_write(pdev, init_cmd, sizeof(init_cmd));
	if (ret < 0) {
		NFC_ERR("%s: CORE_INIT_CMD pn547_i2c_write failed, ret:%d\n", __func__, ret);
		goto failed;
	}
	/*read response*/
	ret = pn544_i2c_read(pdev, recvBuf, _read_sample_test_size);
	if (ret < 0) {
		NFC_ERR("%s: CORE_INIT_RSP pn547_i2c_read failed, ret:%d\n", __func__, ret);
		goto failed;
	}

	udelay(500);
	/*write NCI_PROPRIETARY_ACT_CMD*/
	ret = pn544_i2c_write(pdev, read_config, sizeof(read_config));
	if (ret < 0) {
		NFC_ERR("%s: PRO_ACT_CMD pn547_i2c_write failed, ret:%d\n", __func__, ret);
		goto failed;
	}
	/*read response*/
	ret = pn544_i2c_read(pdev, recvBuf, _read_sample_test_size);
	if (ret < 0) {
		NFC_ERR("%s: PRO_ACT_RSP pn547_i2c_read failed, ret:%d\n", __func__, ret);
		goto failed;
	}

	udelay(500);
	/*write TEST_SWP_CMD UICC*/
	ret = pn544_i2c_write(pdev, read_config_UICC, sizeof(read_config_UICC));
	if (ret < 0) {
		NFC_ERR("%s: TEST_UICC_CMD pn547_i2c_write failed, ret:%d\n", __func__, ret);
		goto failed;
	}
	/*read response*/
	ret = pn544_i2c_read(pdev, recvBuf, _read_sample_test_size);
	if (ret < 0) {
		NFC_ERR("%s: TEST_UICC_RSP pn547_i2c_read failed, ret:%d\n", __func__, ret);
		goto failed;
	}

	mdelay(10);
	/*read notification*/
	ret = pn544_i2c_read(pdev, recvBuf, _read_sample_test_size);
	if (ret < 0) {
		NFC_ERR("%s: TEST_UICC_NTF pn547_i2c_read failed, ret:%d\n", __func__, ret);
		goto failed;
	}

	/*NTF's format: 6F 3E 02 XX1 XX2 -> "XX1 == 0" means SWP link OK*/
	if (recvBuf[0] == 0x6F && recvBuf[1] == 0x3E && recvBuf[3] == 0x00) {
		pdev->sim_status |= UICC_SUPPORT_CARD_EMULATION;
	}

	/*write TEST_SWP_CMD eSE*/
	ret = pn544_i2c_write(pdev, read_config_eSE, sizeof(read_config_eSE));
	if (ret < 0) {
		NFC_ERR("%s: TEST_ESE_CMD pn547_i2c_write failed, ret:%d\n", __func__, ret);
		goto failed;
	}
	/*read response*/
	ret = pn544_i2c_read(pdev, recvBuf, _read_sample_test_size);
	if (ret < 0) {
		NFC_ERR("%s: TEST_ESE_RSP pn547_i2c_read failed, ret:%d\n", __func__, ret);
		goto failed;
	}

	mdelay(10);
	/*read notification*/
	ret = pn544_i2c_read(pdev, recvBuf, _read_sample_test_size);
	if (ret < 0) {
		NFC_ERR("%s: TEST_ESE_NTF pn547_i2c_read failed, ret:%d\n", __func__, ret);
		goto failed;
	}

	/*NTF's format: 6F 3E 02 XX1 XX2 -> "XX1 == 0" means SWP link OK*/
	if (recvBuf[0] == 0x6F && recvBuf[1] == 0x3E && recvBuf[3] == 0x00) {
		pdev->sim_status |= eSE_SUPPORT_CARD_EMULATION;
	}

	return pdev->sim_status;
failed:
	pdev->sim_status = ret;
	return ret;
}

/*
*	name:	pn544_read_firmware
*	use:	read nfc ic firmware for hwinfo
*/
static int pn544_read_firmware(struct i2c_client *client, struct pn544_dev *pdev)
{
	int ret = -1;
	int count = 0;
	unsigned char core_reset_cmd[] = {0x20, 0x00, 0x01, 0x00};
	unsigned char core_init_cmd[] = {0x20, 0x01, 0x00};
	unsigned char nfc_rsp[40];

	/*hardware reset*/
	pn544_enable_nfc(pdev);

	/*CORE_RESET_CMD*/
	ret = pn544_i2c_write(pdev, core_reset_cmd, sizeof(core_reset_cmd));
	if(ret < 0)
	{
		NFC_ERR("pn544 CORE_RESET_CMD send fail!\n");
		return ret;
	}

	/*read response*/
	ret = pn544_i2c_read(pdev, nfc_rsp, _read_sample_test_size);
	if(ret < 0)
	{
		NFC_ERR("pn544 CORE_RESET_CMD read fail!\n");
		return ret;
	}
	
	/*CORE_INIT_CMD*/
	ret = pn544_i2c_write(pdev, core_init_cmd, sizeof(core_init_cmd));
	if(ret < 0)
	{
		NFC_ERR("pn544 CORE_INIT_CMD send fail!\n");
		return ret;
	}

	/*read response*/
	ret = pn544_i2c_read(pdev, nfc_rsp, _read_sample_test_size);
	if(ret < 0)
	{
		NFC_ERR("pn544 CORE_INIT_CMD read fail!\n");
		return ret;
	}

	count = sizeof(nfc_rsp);

	
	if (nfc_firmware_kbuf) {
	    kfree(nfc_firmware_kbuf);
	}
	nfc_firmware_kbuf = kmalloc(PAGE_SIZE, GFP_KERNEL);
	if (!nfc_firmware_kbuf) {
	    kfree(nfc_firmware_kbuf);
	    pr_err("pn544 kmalloc buffer error");
	    return -ENOMEM;
	}

	nfc_firmware = nfc_firmware_kbuf;

	sprintf(nfc_firmware, "version:%02x.%02x.%02x", nfc_rsp[25], nfc_rsp[26], nfc_rsp[27]);

	return 1;
}

/*
*	name:	check_pn548
*	use:	check if pn548 chip is ok
*/
static int check_pn548(struct i2c_client *client, struct pn544_dev *pdev)
{
	int ret = -1;
	int count = 0;
	const char null_cmd[] = {0x00};
	//const char firm_dload_cmd[8]={0x00, 0x04, 0xD0, 0x09, 0x00, 0x00, 0xB1, 0x84};

	/*hardware reset*/
	pn544_enable_nfc(pdev);

	/*sending valid command to get ACK*/
	do {
		ret = i2c_master_send(client, null_cmd, sizeof(null_cmd));
		if(ret < 0) {
			NFC_ERR("pn544 check NFC IC failed,ret = %d, count = %d\n", ret, count);
		} else{
			//NFC_INFO("pn544 check NFC IC success,ret = %d, count = %d\n", ret, count);
			//msleep(10);
			/*hardware reset*/
			pn544_enable_nfc(pdev);
			break;
		}
		count++; 
		msleep(10);
	}while(count < 3);

	/*In case firmware dload failed, will cause host_to_pn547 cmd send failed*/
	for (count = 0; count < 3; count++) {
		pn544_platform_pinctrl_select(gpctrl, st_dwn_h);
		//msleep(10);
		pn544_enable_nfc(pdev);

		//ret = i2c_master_send(client, firm_dload_cmd, sizeof(firm_dload_cmd));
		ret = i2c_master_send(client, null_cmd, sizeof(null_cmd));
		if (ret < 0) {
			NFC_ERR("%s:pn547_i2c_write download cmd failed:%d, ret = %d\n", __func__, count, ret);
			continue;
		}
		pn544_platform_pinctrl_select(gpctrl, st_dwn_l);
		//msleep(10);
		pn544_enable_nfc(pdev);

		break;
	}

	#ifdef CONFIG_HUAWEI_DSM
	if (ret != (int)sizeof(null_cmd)) {
		if (-EOPNOTSUPP == ret) {
			nfc_dsm_report_num(DSM_NFC_I2C_WRITE_EOPNOTSUPP_ERROR_NO,
				"I2C Write Error, Error Type: Operation not supported on transport endpoint\n",ret);
		} else if (-EREMOTEIO == ret) {
			nfc_dsm_report_num(DSM_NFC_I2C_WRITE_EREMOTEIO_ERROR_NO,
				"I2C Write Error, Error Type: Remote I/O error \n",ret);
		} else {
			nfc_dsm_report_num(DSM_NFC_I2C_WRITE_ERROR_NO,
				"I2C Write Error, Error Type: Others \n",ret);
		}
		ret = -EIO;
	}
	#endif

	return ret;
}



/*FUNCTION: nfc_fwupdate_store
  *DESCRIPTION: store nfc firmware update result.
  *Parameters
  * struct device *dev:device structure
  * struct device_attribute *attr:device attribute
  * const char *buf:user buffer
  * size_t count:data count
  *RETURN VALUE
  * ssize_t:  result */
static ssize_t nfc_fwupdate_store(struct device *dev, struct device_attribute *attr,
			 const char *buf, size_t count)
{

	/*if success, store firmware update result into variate firmware_update*/
	if ('1' == buf[0]) {
		firmware_update = 1;
		pr_err("%s:firmware update success\n", __func__);
	}

	return (ssize_t)count;
}
/*FUNCTION: nfc_fwupdate_show
  *DESCRIPTION: show nfc firmware update result.
  *Parameters
  * struct device *dev:device structure
  * struct device_attribute *attr:device attribute
  * const char *buf:user buffer
  *RETURN VALUE
  * ssize_t:  result */
static ssize_t nfc_fwupdate_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	/*get firmware update result from variate firmware_update*/
	return (ssize_t)(snprintf(buf, sizeof(firmware_update)+1, "%d", firmware_update));
}


/*FUNCTION: nxp_config_name_store
  *DESCRIPTION: store nxp_config_name, infact do nothing now.
  *Parameters
  * struct device *dev:device structure
  * struct device_attribute *attr:device attribute
  * const char *buf:user buffer
  * size_t count:data count
  *RETURN VALUE
  * ssize_t:  result */
static ssize_t nxp_config_name_store(struct device *dev, struct device_attribute *attr,
			 const char *buf, size_t count)
{
	/*no need store config name, do nothing now*/
	return (ssize_t)count;
}
/*FUNCTION: nxp_config_name_show
  *DESCRIPTION: get nxp_config_name from variate g_nfc_nxp_config_name.
  *Parameters
  * struct device *dev:device structure
  * struct device_attribute *attr:device attribute
  * const char *buf:user buffer
  *RETURN VALUE
  * ssize_t:  result */
static ssize_t nxp_config_name_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	return (ssize_t)(snprintf(buf, strlen(g_nfc_nxp_config_name)+1, "%s", g_nfc_nxp_config_name));
}


/*FUNCTION: nfc_brcm_conf_name_store
  *DESCRIPTION: store nfc_brcm_conf_name, infact do nothing now.
  *Parameters
  * struct device *dev:device structure
  * struct device_attribute *attr:device attribute
  * const char *buf:user buffer
  * size_t count:data count
  *RETURN VALUE
  * ssize_t:  result */
static ssize_t nfc_brcm_conf_name_store(struct device *dev, struct device_attribute *attr,
			 const char *buf, size_t count)
{
	/*no need store config name, do nothing now*/
	return (ssize_t)count;
}
/*FUNCTION: nfc_brcm_conf_name_show
  *DESCRIPTION: get nfc_brcm_conf_name from variate g_nfc_brcm_config_name.
  *Parameters
  * struct device *dev:device structure
  * struct device_attribute *attr:device attribute
  * const char *buf:user buffer
  *RETURN VALUE
  * ssize_t:  result */
static ssize_t nfc_brcm_conf_name_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	return (ssize_t)(snprintf(buf, strlen(g_nfc_brcm_config_name)+1, "%s", g_nfc_brcm_config_name));
}


/*FUNCTION: nfc_sim_status_show
  *DESCRIPTION: get nfc-sim card support result.
  *Parameters
  * struct device *dev:device structure
  * struct device_attribute *attr:device attribute
  * const char *buf:user buffer
  *RETURN VALUE
  *		eSE		UICC	value
  *		0		0		0	(not support)
  *		0		1		1	(swp1 support)
  *		1		0		2	(swp2 support)
  *		1		1		3	(all support)
  *		<0	:error */
static ssize_t nfc_sim_status_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int status=-1;
	int retry = 0;

	struct i2c_client *i2c_client_dev = container_of(dev, struct i2c_client, dev);
	struct pn544_dev *pn544_dev;
	pn544_dev = i2c_get_clientdata(i2c_client_dev);

	if(pn544_dev == NULL)	{
		printk("%s:  pn544_dev == NULL!\n", __func__);
		return status;
	}

	printk("%s: enter!\n", __func__);
	/* if failed, we have 3 chances */
	for (retry = 0; retry < NFC_TRY_NUM; retry++){
		status = check_sim_status(i2c_client_dev,pn544_dev);
		if(status < 0){
			printk("%s: check_sim_status error!retry count=%d\n", __func__, retry);
			msleep(10);
			continue;
		}
		break;
	}

#ifdef CONFIG_HUAWEI_DSM
	if (status < 0) {
		nfc_dsm_report_num(DSM_NFC_SIM_CHECK_ERROR_NO,"Check UICC status error\n", status);
	}
#endif

	printk("%s: status=%d\n", __func__, status);
	return (ssize_t)(snprintf(buf, MAX_ATTRIBUTE_BUFFER_SIZE-1, "%d\n", status));
}

/*FUNCTION: nfc_sim_switch_store
  *DESCRIPTION: save user sim card select result.
  *Parameters
  * struct device *dev:device structure
  * struct device_attribute *attr:device attribute
  * const char *buf:user buffer
  * size_t count:data count
  *RETURN VALUE
  * ssize_t:  result */
static ssize_t nfc_sim_switch_store(struct device *dev, struct device_attribute *attr,
			 const char *buf, size_t count)
{
	struct i2c_client *i2c_client_dev = container_of(dev, struct i2c_client, dev);
	struct pn544_dev *pn544_dev;
	int val = 0;

	pn544_dev = i2c_get_clientdata(i2c_client_dev);
	if(pn544_dev == NULL)	{
		printk("%s:  pn547_dev == NULL!\n", __func__);
		return 0;
	}

	/*save card select result*/
	if (sscanf(buf, "%1d", &val) == 1) {
		printk("%s: input val = %d!\n", __func__,val);

		if (val == CARD1_SELECT){
			pn544_dev->sim_switch = CARD1_SELECT;
		}else if (val == CARD2_SELECT){
			pn544_dev->sim_switch = CARD2_SELECT;
		}else{
			return -EINVAL;
		}
	}else{
		return -EINVAL;
	}

	return (ssize_t)count;
}


/*FUNCTION: nfc_sim_switch_show
  *DESCRIPTION: get user sim card select result.
  *Parameters
  * struct device *dev:device structure
  * struct device_attribute *attr:device attribute
  * const char *buf:user buffer
  *RETURN VALUE
  * ssize_t:  result */
static ssize_t nfc_sim_switch_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct i2c_client *i2c_client_dev = container_of(dev, struct i2c_client, dev);
	struct pn544_dev *pn544_dev;
	pn544_dev = i2c_get_clientdata(i2c_client_dev);

	if(pn544_dev == NULL)	{
		printk("%s:  pn547_dev == NULL!\n", __func__);
		return 0;
	}
	return (ssize_t)(snprintf(buf,  MAX_ATTRIBUTE_BUFFER_SIZE-1, "%d\n", pn544_dev->sim_switch));
}

/*FUNCTION: rd_nfc_sim_status_show
  *DESCRIPTION: get nfc-sim card support result from variate, no need to send check commands again.
  *Parameters
  * struct device *dev:device structure
  * struct device_attribute *attr:device attribute
  * const char *buf:user buffer
  *RETURN VALUE
  *		eSE		UICC	value
  *		0		0		0	(not support)
  *		0		1		1	(swp1 support)
  *		1		0		2	(swp2 support)
  *		1		1		3	(all support)
  *		<0	:error */
static ssize_t rd_nfc_sim_status_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int status=-1;
	struct i2c_client *i2c_client_dev = container_of(dev, struct i2c_client, dev);
	struct pn544_dev *pn544_dev;
	pn544_dev = i2c_get_clientdata(i2c_client_dev);
	if(pn544_dev == NULL)	{
		printk("%s:  pn547_dev == NULL!\n", __func__);
		return status;
	}
	return (ssize_t)(snprintf(buf, MAX_ATTRIBUTE_BUFFER_SIZE-1,"%d\n", pn544_dev->sim_status));
}

/*FUNCTION: nfc_enable_status_store
  *DESCRIPTION: store nfc_enable_status for RD test.
  *Parameters
  * struct device *dev:device structure
  * struct device_attribute *attr:device attribute
  * const char *buf:user buffer
  * size_t count:data count
  *RETURN VALUE
  * ssize_t:  result */
static ssize_t nfc_enable_status_store(struct device *dev, struct device_attribute *attr,
             const char *buf, size_t count)
{
	struct i2c_client *i2c_client_dev = container_of(dev, struct i2c_client, dev);
	struct pn544_dev *pn544_dev;
	int val = 0;

	pn544_dev = i2c_get_clientdata(i2c_client_dev);
	if(pn544_dev == NULL)	{
		printk("%s:  pn547_dev == NULL!\n", __func__);
		return 0;
	}

	/*save nfc enable status*/
	if (sscanf(buf, "%1d", &val) == 1) {
		if(val == ENABLE_START){
			pn544_dev->enable_status = ENABLE_START;
		}else if(val == ENABLE_END){
			pn544_dev->enable_status = ENABLE_END;
		}else{
			return -EINVAL;
		}
	}else{
		return -EINVAL;
	}

	return (ssize_t)count;
}

/*FUNCTION: nfc_enable_status_show
  *DESCRIPTION: show nfc_enable_status for RD test.
  *Parameters
  * struct device *dev:device structure
  * struct device_attribute *attr:device attribute
  * const char *buf:user buffer
  *RETURN VALUE
  * ssize_t:  result */
static ssize_t nfc_enable_status_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct i2c_client *i2c_client_dev = container_of(dev, struct i2c_client, dev);
	struct pn544_dev *pn544_dev;
	pn544_dev = i2c_get_clientdata(i2c_client_dev);
	if(pn544_dev == NULL){
		printk("%s:  pn547_dev == NULL!\n", __func__);
		return 0;
	}
	return (ssize_t)(snprintf(buf,  MAX_ATTRIBUTE_BUFFER_SIZE-1, "%d\n", pn544_dev->enable_status));
}

/*FUNCTION: nfc_card_num_show
  *DESCRIPTION: show supported nfc_card_num, which config in device tree system.
  *Parameters
  * struct device *dev:device structure
  * struct device_attribute *attr:device attribute
  * const char *buf:user buffer
  *RETURN VALUE
  * ssize_t:  result */
static ssize_t nfc_card_num_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int temp = 0;
	int ret;
	struct device_node *node;

	/*np = of_find_node_by_name(NULL, "huawei_nfc_info");
	if(!of_device_is_available(np)){
	     printk("%s: not find node <huawei_nfc_info> !\n", __func__);
	     temp = UICC_SUPPORT_CARD_EMULATION;
	     return (ssize_t)(snprintf(buf,  MAX_ATTRIBUTE_BUFFER_SIZE-1, "%d\n", (unsigned char)temp));
	}*/
	node = of_find_compatible_node(NULL, NULL, "mediatek,nfc-pn548");
	
	if(node) {

		/*read supported nfc_card_num from device tree system.*/
		ret=of_property_read_u32(node, "nfc_card_num", &temp);
		if(ret){
			temp = UICC_SUPPORT_CARD_EMULATION;
			printk("%s: can't get nfc card num config!\n", __func__);
		}
	}
	return (ssize_t)(snprintf(buf,  MAX_ATTRIBUTE_BUFFER_SIZE-1, "%d\n", (unsigned char)temp));
}

/*FUNCTION: nfc_chip_type_show
  *DESCRIPTION: show nfc_chip_type, which config in device tree system.
  *Parameters
  * struct device *dev:device structure
  * struct device_attribute *attr:device attribute
  * const char *buf:user buffer
  *RETURN VALUE
  * ssize_t:  result */
static ssize_t nfc_chip_type_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    int ret = -1;
    struct device_node *node;
    const char *out_value = NULL;

    memset(nfc_chip_type, 0, MAX_NFC_CHIP_TYPE_SIZE);

   /* np = of_find_node_by_name(NULL, "huawei_nfc_info");
    if(!of_device_is_available(np)){
         printk("%s: not find node <huawei_nfc_info> !\n", __func__);
         strncpy(nfc_chip_type, "pn547", MAX_NFC_CHIP_TYPE_SIZE-1);
         return (ssize_t)(snprintf(buf,  MAX_ATTRIBUTE_BUFFER_SIZE-1, "%s", nfc_chip_type));
    }*/
    
	node = of_find_compatible_node(NULL, NULL, "mediatek,nfc-pn548");
	
	if(node) {

	    /*read nfc_chip_type from device tree system.*/
	    ret=of_property_read_string(node, "nfc_chip_type", &out_value);
	    strncpy(nfc_chip_type, out_value, MAX_NFC_CHIP_TYPE_SIZE-1);
	    if(ret != 0){
	        printk("%s: can't get nfc nfc_chip_type, default pn547\n", __func__);
	        strncpy(nfc_chip_type, "pn547", MAX_NFC_CHIP_TYPE_SIZE-1);
    	}
	}

    return (ssize_t)(snprintf(buf,  MAX_ATTRIBUTE_BUFFER_SIZE-1, "%s", nfc_chip_type));
}

#ifdef CONFIG_HUAWEI_DSM
/*FUNCTION: nfc_dmd_notify_show
  *DESCRIPTION: nfc_dmd_notify_show, show dsm_nfc_errno
  *Parameters
  * struct device *dev:device structure
  * struct device_attribute *attr:device attribute
  * const char *buf:user buffer
  *RETURN VALUE
  * ssize_t:  result */
static ssize_t nfc_dmd_notify_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	NFC_INFO("%s: enter\n", __func__);
	return (ssize_t)(snprintf(buf, MAX_ATTRIBUTE_BUFFER_SIZE-1, "dsm_nfc_errno: %d\n", dsm_nfc_errno));
}

/*FUNCTION: nfc_dmd_notify_store
  *DESCRIPTION: nfc_dmd_notify_store, debug for nfc dsm.
  *Parameters
  * struct device *dev:device structure
  * struct device_attribute *attr:device attribute
  * const char *buf:user buffer
  * size_t count:data count
  *RETURN VALUE
  * ssize_t:  result */
static ssize_t nfc_dmd_notify_store(struct device *dev, struct device_attribute *attr,
             const char *buf, size_t count)
{
	NFC_INFO("%s: enter\n", __func__);

	if (1 != sscanf(buf, "%10d", &dsm_nfc_errno)) {
		printk("%s : get buf failed: %s\n", __func__, buf);
		return -EINVAL;
	}
	//printk("nfc dsm test\n");
	nfc_dsm_report_num(dsm_nfc_errno,"nfc dsm test\n",-1);
	return count;
}
#endif


/*register device node to communication with user space*/
static struct device_attribute pn544_attr[] ={
	__ATTR(nfc_fwupdate, 0664, nfc_fwupdate_show, nfc_fwupdate_store),
	__ATTR(nxp_config_name, 0664, nxp_config_name_show, nxp_config_name_store),
	__ATTR(nfc_brcm_conf_name, 0664, nfc_brcm_conf_name_show, nfc_brcm_conf_name_store),
	__ATTR(nfc_sim_switch, 0664, nfc_sim_switch_show, nfc_sim_switch_store),
	__ATTR(nfc_sim_status, 0444, nfc_sim_status_show, NULL),
	__ATTR(rd_nfc_sim_status, 0444, rd_nfc_sim_status_show, NULL),
	__ATTR(nfc_enable_status, 0664, nfc_enable_status_show, nfc_enable_status_store),
	__ATTR(nfc_card_num, 0444, nfc_card_num_show, NULL),
	__ATTR(nfc_chip_type, 0444, nfc_chip_type_show, NULL),
#ifdef CONFIG_HUAWEI_DSM
	__ATTR(nfc_dmd_notify, 0664, nfc_dmd_notify_show, nfc_dmd_notify_store),
#endif
};

/*FUNCTION: create_sysfs_interfaces
  *DESCRIPTION: create_sysfs_interfaces.
  *Parameters
  * struct device *dev:device structure
  *RETURN VALUE
  * int:  result */
static int create_sysfs_interfaces(struct device *dev)
{
	int i;
	for (i = 0; i < ARRAY_SIZE(pn544_attr); i++) {
		if (device_create_file(dev, pn544_attr + i)) {
			goto error;
		}
	}

	return 0;
error:
	for ( ; i >= 0; i--) {
		device_remove_file(dev, pn544_attr + i);
	}

	printk("%s:pn547 unable to create sysfs interface.\n", __func__ );
	return -1;
}
/*FUNCTION: remove_sysfs_interfaces
  *DESCRIPTION: remove_sysfs_interfaces.
  *Parameters
  * struct device *dev:device structure
  *RETURN VALUE
  * int:  result */
static int remove_sysfs_interfaces(struct device *dev)
{
	int i;
	for (i = 0; i < ARRAY_SIZE(pn544_attr); i++) {
		device_remove_file(dev, pn544_attr + i);
	}

	return 0;
}

static int pn544_remove(struct i2c_client *client)
{
	struct pn544_dev *pn544_dev;
	
#ifdef CONFIG_HUAWEI_DSM
	dsm_unregister_client(nfc_dclient,&nfc_dsm_info);
#endif

#ifdef CONFIG_MTK_I2C_EXTENSION
	if (I2CDMAWriteBuf) {
		#if 1//def CONFIG_64BIT
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
		#if 1//def CONFIG_64BIT
		dma_free_coherent(&client->dev, MAX_BUFFER_SIZE, I2CDMAReadBuf,
				  I2CDMAReadBuf_pa);
		#else
		dma_free_coherent(NULL, MAX_BUFFER_SIZE, I2CDMAReadBuf,
				  I2CDMAReadBuf_pa);
		#endif
		I2CDMAReadBuf = NULL;
		I2CDMAReadBuf_pa = 0;
	}
#endif

	pn544_dev = i2c_get_clientdata(client);
	misc_deregister(&pn544_dev->pn544_device);
	mutex_destroy(&pn544_dev->read_mutex);
	
	remove_sysfs_interfaces(&client->dev);

	regulator_put(pn544_dev->reg);
	kfree(pn544_dev);
	return 0;
}

static int pn544_probe(struct i2c_client *client, const struct i2c_device_id *id)
{

	int ret=0;
	struct pn544_dev *pn544_dev;
	struct device_node *node;
	u32 ints[2] = { 0, 0 };

	printk("%s: start...\n", __func__);
	
	#ifdef CONFIG_HUAWEI_DSM
      	nfc_dsm_register();
	#endif
	
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) 
	{
		pr_err("%s: need I2C_FUNC_I2C\n", __func__);
		return  -ENODEV;
	}

	printk("%s: step02 is ok\n", __func__);

	pn544_dev = kzalloc(sizeof(*pn544_dev), GFP_KERNEL);
	printk("pn544_dev=%p\n", pn544_dev);

	if (pn544_dev == NULL) 
	{
		dev_err(&client->dev, "pn544 failed to allocate memory for module data\n");
		return -ENOMEM;
	}

	memset(pn544_dev, 0, sizeof(struct pn544_dev));

	printk("%s: step03 is ok\n", __func__);

	pn544_dev->client = client;

	/*check the PCB have NFC IC or not*/
	ret = check_pn548(client,pn544_dev);
	if(ret < 0) 
	{
		pr_err("check pn544 nfc chip fail\n");
		return ret;
	}
	pn544_dev->sim_switch = CARD1_SELECT;/*sim_select = 1,UICC select*/
	pn544_dev->sim_status = CARD_UNKNOWN;
	pn544_dev->enable_status = ENABLE_START;
	
	/*create interface node*/
	ret = create_sysfs_interfaces(&client->dev);
	if (ret < 0) {
		pr_err("Failed to create_sysfs_interfaces\n");
		return -ENODEV;
	}
	ret = sysfs_create_link(NULL,&client->dev.kobj,"nfc");
	if(ret < 0) {
		pr_err("Failed to sysfs_create_link\n");
		return -ENODEV;
	}

	/* init mutex and queues */
	init_waitqueue_head(&pn544_dev->read_wq);
	mutex_init(&pn544_dev->read_mutex);
	spin_lock_init(&pn544_dev->irq_enabled_lock);
#if 1
	pn544_dev->pn544_device.minor = MISC_DYNAMIC_MINOR;
	pn544_dev->pn544_device.name = PN544_DRVNAME;
	pn544_dev->pn544_device.fops = &pn544_dev_fops;

	ret = misc_register(&pn544_dev->pn544_device);
	if (ret) 
	{
		pr_err("%s: misc_register failed\n", __func__);
		goto err_misc_register;
	}
#endif    
	printk("%s: step04 is ok\n", __func__);

#ifdef CONFIG_HQ_HWINFO
	nfc_hwinfo = hwinfo_create("nfc");
	if(!nfc_hwinfo)
		printk("hwinfo create fail\r\n");
#endif
   
#ifdef CONFIG_MTK_I2C_EXTENSION
	client->dev.coherent_dma_mask = DMA_BIT_MASK(32);
#if 1//def CONFIG_64BIT
	I2CDMAWriteBuf =
	    (char *)dma_alloc_coherent(&client->dev, MAX_BUFFER_SIZE,
				       (dma_addr_t *) &I2CDMAWriteBuf_pa,
				       GFP_KERNEL);
#else
	I2CDMAWriteBuf =
	    (char *)dma_alloc_coherent(NULL, MAX_BUFFER_SIZE,
				       (dma_addr_t *) &I2CDMAWriteBuf_pa,
				       GFP_KERNEL);
#endif

	if (I2CDMAWriteBuf == NULL) {
		pr_err("%s : failed to allocate dma buffer\n", __func__);
		mutex_destroy(&pn544_dev->read_mutex);
		//gpio_free(platform_data->sysrstb_gpio);
		return ret;
	}
#if 1//def CONFIG_64BIT
	I2CDMAReadBuf =
	    (char *)dma_alloc_coherent(&client->dev, MAX_BUFFER_SIZE,
				       (dma_addr_t *) &I2CDMAReadBuf_pa,
				       GFP_KERNEL);
#else
	I2CDMAReadBuf =
	    (char *)dma_alloc_coherent(NULL, MAX_BUFFER_SIZE,
				       (dma_addr_t *) &I2CDMAReadBuf_pa,
				       GFP_KERNEL);
#endif

	if (I2CDMAReadBuf == NULL) {
		pr_err("%s : failed to allocate dma buffer\n", __func__);
		mutex_destroy(&pn544_dev->read_mutex);
		//gpio_free(platform_data->sysrstb_gpio);
		return ret;
	}
	pr_debug("%s :I2CDMAWriteBuf_pa %d, I2CDMAReadBuf_pa,%d\n", __func__,
		 I2CDMAWriteBuf_pa, I2CDMAReadBuf_pa);
#else
	memset(I2CDMAWriteBuf, 0x00, sizeof(I2CDMAWriteBuf));
	memset(I2CDMAReadBuf, 0x00, sizeof(I2CDMAReadBuf));
#endif

	printk("%s: step05 is ok\n", __func__);
	/*  NFC IRQ settings     */	
	
	/* request irq.  the irq is set whenever the chip has data available
	* for reading.  it is cleared when all data has been read.
	*/  

	node = of_find_compatible_node(NULL, NULL, "mediatek,nfc-pn548");
	if(node) {
		
		of_property_read_u32_array(node, "debounce",ints, ARRAY_SIZE(ints));
		gpio_set_debounce(ints[0], ints[1]);
		pn544_dev->irq_gpio = ints[0];
		client->irq = irq_of_parse_and_map(node, 0);
	
		ret = request_irq(client->irq, pn544_dev_irq_handler, IRQF_TRIGGER_HIGH, "nfc", pn544_dev);
	
		if (ret) {
			pr_err("%s: EINT IRQ LINE NOT AVAILABLE, ret = %d\n", __func__, ret);
		} else {
	
			printk("%s: set EINT finished, client->irq=%d\n", __func__, client->irq);
			pn544_dev->irq_enabled = true;
			pn544_disable_irq(pn544_dev);
		}
		
	} else {		
			pr_err("%s: can not find NFC eint compatible node\n",__func__);
	}

	pn544_platform_pinctrl_select(gpctrl, st_ven_h);

	i2c_set_clientdata(client, pn544_dev);
    printk("%s: step06 success\n", __func__);

#ifdef CONFIG_HQ_HWINFO
	hwinfo_set_devinfo(nfc_hwinfo,"%s",nfc_devinfo);

	pn544_read_firmware(client, pn544_dev);
	hwinfo_set_firmware(nfc_hwinfo,"%s",nfc_firmware);

	if (nfc_firmware_kbuf) {
		kfree(nfc_firmware_kbuf);//free buffer
	}
	
	if(hwinfo_register(nfc_hwinfo))
		printk("hwinfo_register fail\r\n");
#endif

    printk("%s: step07 success\n", __func__);

	/* get and save configure name*/
	get_nfc_config_name();
	
	return 0;

err_misc_register:
	mutex_destroy(&pn544_dev->read_mutex);
	kfree(pn544_dev);
	return ret;
}

static const struct i2c_device_id pn544_id[] = {
	{I2C_ID_NAME, 0},
	{}
};

static const struct of_device_id nfc_i2c_of_match[] = {
	{.compatible = "mediatek,nfc"},
	{},
};

static struct i2c_driver pn544_i2c_driver = 
{
	.id_table	= pn544_id,
	.probe		= pn544_probe,
	.remove		= pn544_remove,
	/* .detect	= pn544_detect, */
	.driver		= {
		.name = "pn544",
		.owner = THIS_MODULE,
#ifdef CONFIG_OF
		.of_match_table = nfc_i2c_of_match,
#endif
	},
};

static int pn544_platform_pinctrl_init(struct platform_device *pdev)
{
	int ret = 0;

	printk("%s\n", __func__);

	gpctrl = devm_pinctrl_get(&pdev->dev);
	if (IS_ERR(gpctrl)) {
		dev_err(&pdev->dev, "Cannot find pinctrl!");
		ret = PTR_ERR(gpctrl);
		goto end;
	}

	st_ven_h = pinctrl_lookup_state(gpctrl, "ven_high");
	if (IS_ERR(st_ven_h)) {
		ret = PTR_ERR(st_ven_h);
		printk("%s: pinctrl err, ven_high\n", __func__);
	}

	st_ven_l = pinctrl_lookup_state(gpctrl, "ven_low");
	if (IS_ERR(st_ven_l)) {
		ret = PTR_ERR(st_ven_l);
		printk("%s: pinctrl err, ven_low\n", __func__);
	}
	
    st_dwn_h = pinctrl_lookup_state(gpctrl, "eint_high");
    if (IS_ERR(st_dwn_h)) {
    	ret = PTR_ERR(st_dwn_h);
    	printk("%s: pinctrl err, dwn_high\n", __func__);
    }
    
    
    st_dwn_l = pinctrl_lookup_state(gpctrl, "eint_low");
    if (IS_ERR(st_dwn_l)) {
    	ret = PTR_ERR(st_dwn_l);
    	printk("%s: pinctrl err, dwn_low\n", __func__);
    }
    
    
    st_eint_int = pinctrl_lookup_state(gpctrl, "irq_init");
    if (IS_ERR(st_eint_int)) {
    	ret = PTR_ERR(st_eint_int);
    	printk("%s: pinctrl err, st_eint_int\n", __func__);
    }
    pn544_platform_pinctrl_select(gpctrl, st_eint_int);

end:
	return ret;
}


static int pn544_platform_probe(struct platform_device *pdev)
{
	int ret = 0;

	printk("%s: &pdev=%p\n", __func__, pdev);

	/* pinctrl init */
	ret = pn544_platform_pinctrl_init(pdev);

	return ret;
}

static int pn544_platform_remove(struct platform_device *pdev)
{
	printk("%s: &pdev=%p\n", __func__, pdev);

	return 0;
}

/*  platform driver */
static const struct of_device_id pn544_platform_of_match[] = {
	{.compatible = "mediatek,nfc-pn548",},
	{},
};

static struct platform_driver pn544_platform_driver = {
	.probe		= pn544_platform_probe,
	.remove		= pn544_platform_remove,
	.driver		= {
		.name = I2C_ID_NAME,
		.owner = THIS_MODULE,
#ifdef CONFIG_OF
		.of_match_table = pn544_platform_of_match,
#endif
	},
};

/*
 * module load/unload record keeping
 */
static int __init pn544_dev_init(void)
{
	int ret;
	printk("pn544_dev_init\n");

	platform_driver_register(&pn544_platform_driver);

	ret = i2c_add_driver(&pn544_i2c_driver);
	printk("[pn544] i2c_add_driver  ret = %d \n", ret);
	printk("pn544_dev_init success\n");

	return 0;
}
module_init(pn544_dev_init);

static void __exit pn544_dev_exit(void)
{
	printk("pn544_dev_exit\n");

	i2c_del_driver(&pn544_i2c_driver);
}
module_exit(pn544_dev_exit);

MODULE_AUTHOR("XXX");
MODULE_DESCRIPTION("NFC PN544 driver");
MODULE_LICENSE("GPL");

