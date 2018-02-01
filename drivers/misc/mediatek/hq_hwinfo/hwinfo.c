#include <linux/init.h>
#include <linux/module.h>
#include <linux/cdev.h>
#include <linux/fs.h>
#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/miscdevice.h>
#include <linux/platform_device.h>
#include <linux/gfp.h>
#include <linux/slab.h>

#include "hwinfo.h"

#define HWINFO_NAME		"hwinfo"
#define HWINFO_TAG		"[hwinfo]"
#define HWINFO_FUNC(f)              printk(KERN_INFO HWINFO_TAG" %s\n", __FUNCTION__)
#define HWINFO_DBG(fmt, args...)    printk(KERN_DEBUG HWINFO_TAG" %s %d : "fmt, __FUNCTION__, __LINE__, ##args)

struct hwinfo_dev
{
	struct device *dev;
	struct class *cls;
	struct miscdevice* mdev;
	atomic_t hwinfo_count; 	
	struct list_head hwinfo_list;
	int state;
};

static struct hwinfo_dev * g_hwinfo_dev = NULL ;

static int hwinfo_open (struct inode *inode, struct file *file)
{
    HWINFO_FUNC();
    return 0;
}
static ssize_t hwinfo_read (struct file *file, char __user *data, size_t count, loff_t * offset)
{
    
    HWINFO_FUNC();
    return 0;
}
static ssize_t hwinfo_write (struct file *file, const char __user *data, size_t count, loff_t *offset)
{
    HWINFO_FUNC();
    return 0;
}
static long hwinfo_ioctl (struct file *file , unsigned int cmd, unsigned long arg)
{
    HWINFO_FUNC();
    return 0;
}
static int hwinfo_release (struct inode *node, struct file *file)
{
	HWINFO_FUNC();
	return 0;
}

static struct file_operations hwinfo_fops =
{
    .owner	= THIS_MODULE,
    .open	= hwinfo_open,
    .read	= hwinfo_read,
    .write	= hwinfo_write,
    .unlocked_ioctl = hwinfo_ioctl,
    .release = hwinfo_release,
};
static char * hwinfo_dev_str[]=
{
	"unknow",
	"lcm",
	"touchpanel",
	"camera",
	"sensor",
	"battery",
	"charge",
	"led",
	"memory",
	"flash",
	"arg",
	NULL,
};
static char * hwinfo_bus_str[]=
{
	"unknow",
	"I2C",
	"SPI",
	"USB",
	"MIPI",
	"UART",
	"I2S",
	"VIRTUAL",
	NULL,
};
static char * __hwinfo_dev2string(enum hwinfo_device_type type)
{
	if(type > HWINFO_DEVICE_MAX)
		return NULL;
	return hwinfo_dev_str[type];
}
static char * __hwinfo_bus2string(enum hwinfo_bus_type type)
{
	if(type > HWINFO_BUS_MAX)
		return NULL;
	return hwinfo_dev_str[type];
}

static ssize_t hwinfo_show_devinfo(struct device *dev, struct device_attribute *attr,char *buf)
{
	int count = 0 ;
	struct hwinfo *hwinfo = dev_get_drvdata(dev);
	if(!hwinfo)
	{
		HWINFO_DBG("get drv data fail\r\n");
		return -1;
	}
	if(hwinfo->devinfo)
		count = sprintf(buf, "%s\n", hwinfo->devinfo);
	return count ;
}

static ssize_t hwinfo_store_devinfo(struct device *dev, struct device_attribute *attr,
			    const char *buf, size_t size)
{
	return size;
}

static ssize_t hwinfo_show_firmware(struct device *dev, struct device_attribute *attr,char *buf)
{
	int count = 0 ;
	struct hwinfo *hwinfo = dev_get_drvdata(dev);
	if(!hwinfo)
	{
		HWINFO_DBG("get drv data fail\r\n");
		return -1;
	}
	if(hwinfo->firmware)
		count = sprintf(buf, "%s\n", hwinfo->firmware);
	return count ;
}

static ssize_t hwinfo_store_firmware(struct device *dev, struct device_attribute *attr,
			    const char *buf, size_t size)
{
	return size;
}

static ssize_t hwinfo_show_businfo(struct device *dev, struct device_attribute *attr,
			   char *buf)
{	
	int count = 0 ;
	struct hwinfo *hwinfo = dev_get_drvdata(dev);
	if(!hwinfo)
	{
		HWINFO_DBG("get drv data fail\r\n");
		return -1;
	}
	count = sprintf(buf, "%s\n", __hwinfo_bus2string(hwinfo->bus_type));
	return count;
}

static ssize_t hwinfo_store_businfo(struct device *dev, struct device_attribute *attr,
			    const char *buf, size_t size)
{
	return size;
}
static ssize_t hwinfo_show_version(struct device *dev, struct device_attribute *attr,
			   char *buf)
{
	int count = 0 ;
	struct hwinfo *hwinfo = dev_get_drvdata(dev);
	if(!hwinfo)
	{
		HWINFO_DBG("get drv data fail\r\n");
		return -1;
	}
	if(hwinfo->version)
		count = sprintf(buf, "version=0x%x\n", hwinfo->version);
	return count ;
}

static ssize_t hwinfo_store_version(struct device *dev, struct device_attribute *attr,
			    const char *buf, size_t size)
{
	return size;
}
static ssize_t hwinfo_show_uptime(struct device *dev, struct device_attribute *attr,
			   char *buf)
{
	int count = 0 ;
	struct hwinfo *hwinfo = dev_get_drvdata(dev);
	if(!hwinfo)
	{
		HWINFO_DBG("get drv data fail\r\n");
		return -1;
	}
	if(hwinfo->update_times)
		count = sprintf(buf, "uptime=0x%x\n", hwinfo->update_times);
	return count ;
}

static ssize_t hwinfo_store_uptime(struct device *dev, struct device_attribute *attr,
			    const char *buf, size_t size)
{
	return size;
}
static ssize_t hwinfo_show_other(struct device *dev, struct device_attribute *attr,
			   char *buf)
{
	int count = 0 ;
	struct hwinfo *hwinfo = dev_get_drvdata(dev);
	if(!hwinfo)
	{
		HWINFO_DBG("get drv data fail\r\n");
		return -1;
	}
	if(hwinfo->other)
		count = sprintf(buf, "%s\n", hwinfo->other);
	return count ;

}

static ssize_t hwinfo_store_other(struct device *dev, struct device_attribute *attr,
			    const char *buf, size_t size)
{
	return size;
}

DEVICE_ATTR(devinfo,	S_IWUSR | S_IRUGO, hwinfo_show_devinfo, hwinfo_store_devinfo);
DEVICE_ATTR(firmware,S_IWUSR | S_IRUGO, hwinfo_show_firmware, hwinfo_store_firmware);
DEVICE_ATTR(businfo,	S_IWUSR | S_IRUGO, hwinfo_show_businfo, hwinfo_store_businfo);
DEVICE_ATTR(version,	S_IWUSR | S_IRUGO, hwinfo_show_version, hwinfo_store_version);
DEVICE_ATTR(uptime,	S_IWUSR | S_IRUGO, hwinfo_show_uptime, hwinfo_store_uptime);
DEVICE_ATTR(other,	S_IWUSR | S_IRUGO, hwinfo_show_other, hwinfo_store_other);


static struct attribute *hwinfo_commom_attributes[] = {
	&dev_attr_devinfo.attr,
	&dev_attr_firmware.attr,
	&dev_attr_businfo.attr,
	&dev_attr_version.attr,
	&dev_attr_uptime.attr,
	&dev_attr_other.attr,
	NULL,
};

static const struct attribute_group hwinfo_commom_attribute_group = {
	.attrs = hwinfo_commom_attributes
};

struct hwinfo * hwinfo_create(const char *device_name,...)
{
	struct hwinfo * hwinfo = NULL;
	hwinfo = (struct hwinfo *)kzalloc(sizeof(struct hwinfo),GFP_KERNEL);
	if(!hwinfo)
	{
		HWINFO_DBG("hwinfo alloc fail\r\n");
		return NULL;
	}
	va_list ap;
	if(!device_name)
	{
		HWINFO_DBG("hwinfo_create  device_name argument fail\r\n");
		return NULL;
	}
	va_start(ap, device_name);
	hwinfo->device_name = kvasprintf(GFP_KERNEL,device_name,ap);
	va_end(ap);
	INIT_LIST_HEAD(&(hwinfo->hwinfo_list));
	HWINFO_DBG("hwinfo create device name = %s\r\n",hwinfo->device_name);
	return hwinfo;
}
EXPORT_SYMBOL(hwinfo_create);

int hwinfo_register(struct hwinfo *hwinfo)
{
	int err;
	if(!hwinfo || !hwinfo->device_name)
	{
		HWINFO_DBG("hwinfo is null\r\n");
		return -1;
	}
	hwinfo->index = atomic_inc_return(&(g_hwinfo_dev->hwinfo_count));
	hwinfo->dev = device_create(g_hwinfo_dev->cls,NULL,MKDEV(0,hwinfo->index),hwinfo,"%s",hwinfo->device_name);
	if(!hwinfo->dev)
	{
		HWINFO_DBG("hwinfo device_create fail\r\n");
		return -1;
	}
	err = sysfs_create_group(&(hwinfo->dev->kobj),&hwinfo_commom_attribute_group);
	if(err)
	{
		HWINFO_DBG("sysfs create grop fail\r\n");
		device_destroy(g_hwinfo_dev->cls,MKDEV(0,hwinfo->index));
		atomic_dec(&(g_hwinfo_dev->hwinfo_count));
		return -1;
	}
	if(hwinfo->attr_group)
	{
		HWINFO_DBG("create pri attr_group start \r\n");
		err = sysfs_create_group(&(hwinfo->dev->kobj),hwinfo->attr_group);
		if (err)
		{
			HWINFO_DBG("sysfs create grop fail\r\n");
			device_destroy(g_hwinfo_dev->cls,MKDEV(0,hwinfo->index));
			atomic_dec(&(g_hwinfo_dev->hwinfo_count));
			return -1;
		}
		HWINFO_DBG("create pri attr_group success; \r\n");		
	}
	if(!hwinfo->version)
		hwinfo->version = 0xffff;
	if(!hwinfo->update_times)
		hwinfo->update_times = 0xffff;
	list_add_tail(&hwinfo->hwinfo_list,&g_hwinfo_dev->hwinfo_list);
	hwinfo->state = 1;
	return 0;
}
EXPORT_SYMBOL(hwinfo_register);
int hwinfo_deregister(struct hwinfo *hwinfo)
{
	if(!hwinfo)
	{
		HWINFO_DBG("hwinfo is null\r\n");
		return -1;
	}
	device_destroy(g_hwinfo_dev->cls,MKDEV(0,hwinfo->index));
	list_del(&hwinfo->hwinfo_list);
	atomic_dec(&(g_hwinfo_dev->hwinfo_count));

	if(hwinfo->device_name)
		kfree(hwinfo->device_name);
	if(hwinfo->firmware)
		kfree(hwinfo->firmware);
	if(hwinfo->devinfo);
		kfree(hwinfo->devinfo);
	if(hwinfo->other);
		kfree(hwinfo->other);
	if(hwinfo)
		kfree(hwinfo);
	return 0;
	
}
EXPORT_SYMBOL(hwinfo_deregister);

int hwinfo_set_devinfo(struct hwinfo *hwinfo,const char *fmt,...)
{
	va_list ap;
	if(!hwinfo || !fmt )
	{
		HWINFO_DBG("value check fail \r\n");
		return -1;
	}
	if(hwinfo->devinfo)
		kfree(hwinfo->devinfo);
	va_start(ap, fmt);
	hwinfo->devinfo = kvasprintf(GFP_KERNEL,fmt,ap);
	va_end(ap);
	if(!hwinfo->devinfo)
	{
		HWINFO_DBG("kzalloc fail\r\n");
		return -1;
	}
	return 0;
}
EXPORT_SYMBOL(hwinfo_set_devinfo);
int hwinfo_set_firmware(struct hwinfo *hwinfo,const char *fmt,...)
{
	va_list ap;
	if(!hwinfo || !fmt )
	{
		HWINFO_DBG("value check fail \r\n");
		return -1;
	}
	if(hwinfo->firmware)
		kfree(hwinfo->firmware);
	va_start(ap, fmt);
	hwinfo->firmware = kvasprintf(GFP_KERNEL,fmt,ap);
	va_end(ap);
	if(!hwinfo->firmware)
	{
		HWINFO_DBG("kzalloc fail\r\n");
		return -1;
	}
	return 0;
}
EXPORT_SYMBOL(hwinfo_set_firmware);
int hwinfo_set_other(struct hwinfo *hwinfo,const char *fmt,...)
{
	va_list ap;
	if(!hwinfo || !fmt )
	{
		HWINFO_DBG("value check fail \r\n");
		return -1;
	}
	if(hwinfo->other)
		kfree(hwinfo->other);
	va_start(ap, fmt);
	hwinfo->other = kvasprintf(GFP_KERNEL,fmt,ap);
	va_end(ap);
	if(!hwinfo->other)
	{
		HWINFO_DBG("kzalloc fail\r\n");
		return -1;
	}
	return 0;
}
EXPORT_SYMBOL(hwinfo_set_other);


static int hwinfo_local_init(void)
{
	HWINFO_FUNC();
	g_hwinfo_dev = (struct hwinfo_dev *)kzalloc(sizeof(struct hwinfo_dev),GFP_KERNEL);
	if(!g_hwinfo_dev)
	{
		HWINFO_DBG("g_hwinfo_dev kzalloc fail\r\n");
		return -1;
	}
	INIT_LIST_HEAD(&g_hwinfo_dev->hwinfo_list);
	atomic_set(&g_hwinfo_dev->hwinfo_count,0);
	g_hwinfo_dev->cls = class_create(THIS_MODULE,HWINFO_NAME);
	if(!g_hwinfo_dev->cls)
	{
		HWINFO_DBG("class_create fail\r\n");
		return -1;
	}
	HWINFO_DBG("hwinfo_local_init success\r\n");
	return 0;
}
static int hwinfo_local_uninit(void)
{
	HWINFO_FUNC();
	if(atomic_read(&g_hwinfo_dev->hwinfo_count) != 0 )
	{
		
	}
	if(!g_hwinfo_dev->cls)		
		class_destroy(g_hwinfo_dev->cls);
	if(!g_hwinfo_dev)
		kfree(g_hwinfo_dev);
	return 0;
}

static int hwinfo_probe(struct platform_device *pdev)
{
	int err = 0;
	HWINFO_FUNC();
	err = hwinfo_local_init(); //0 success , < 0 fail;
	if(err)
	{
		HWINFO_DBG("local init fail\r\n");
		return err;
	}
#if 0
	// it`s only for test starts
	struct hwinfo *hwinfo_sensor ,*hwinfo_lcm;
	hwinfo_sensor = hwinfo_create("sensor");
	hwinfo_lcm = hwinfo_create("lcm");
	hwinfo_set_devinfo(hwinfo_lcm,"lcm %s","syna");
	hwinfo_set_devinfo(hwinfo_sensor,"sensor %s","gsensor");
	hwinfo_lcm->version = 0x1234;
	hwinfo_register(hwinfo_lcm);
	hwinfo_register(hwinfo_sensor);
	//it`s only for test end
#endif 
	g_hwinfo_dev->state = 1 ;
	return 0;
}
static int hwinfo_remove(struct platform_device *pdev)
{
	HWINFO_FUNC();
	hwinfo_local_uninit();
	return 0;
};

#ifdef CONFIG_OF
static struct of_device_id hwinfo_of_match[] = {
	{ .compatible = "huaqin,hwinfo", },
	{},
};
#endif

static struct platform_device hwinfo_device= {
	.name = HWINFO_NAME,
	.id = -1,
};

static struct platform_driver hwinfo_driver = {
	.probe		= hwinfo_probe,
	.remove		= hwinfo_remove,
	.driver		= {
		.name		= HWINFO_NAME,
		.owner		= THIS_MODULE,
#ifdef CONFIG_OF
		.of_match_table = hwinfo_of_match,
#endif
	},
};

static int __init hwinfo_init(void)
{
	HWINFO_FUNC();
//#ifndef CONFIG_OF
	platform_device_register(&hwinfo_device);
//#endif
	platform_driver_register(&hwinfo_driver);
    return 0;
}
static void __exit hwinfo_exit(void)
{
	HWINFO_FUNC();
	hwinfo_local_uninit();
	platform_driver_unregister(&hwinfo_driver);
}

//late_init(hwinfo_init);
subsys_initcall(hwinfo_init);
module_exit(hwinfo_exit);
MODULE_LICENSE("GPL");  
MODULE_AUTHOR("WangYang <wangyang@huaqin.com>");                    
MODULE_DESCRIPTION("Hardware Info for Android Phone!");            
MODULE_VERSION("1.0.0");    

