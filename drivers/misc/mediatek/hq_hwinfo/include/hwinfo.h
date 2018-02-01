#ifndef __DEVICE_INFO_H__
#define __DEVICE_INFO_H__

#include <linux/miscdevice.h>
#include <linux/list.h>
#include <linux/device.h>

enum hwinfo_device_type
{
	HWINFO_DEVICE_UNKNOW=0x00,
	HWINFO_DEVICE_LCM,
	HWINFO_DEVICE_TP,
	HWINFO_DEVICE_CAMERA,
	HWINFO_DEVICE_SENSOR,
	HWINFO_DEVICE_BATTERY,
	HWINFO_DEVICE_CHARGE,
	HWINFO_DEVICE_LED,
	HWINFO_DEVICE_MEM,
	HWINFO_DEVICE_FLASH,
	HWINFO_DEVICE_ARG,
	HWINFO_DEVICE_OTHER,
	HWINFO_DEVICE_MAX,
};

enum hwinfo_bus_type
{
	HWINFO_BUS_UNKNOW=0x00,
	HWINFO_BUS_I2C,
	HWINFO_BUS_SPI,
	HWINFO_BUS_USB,
	HWINFO_BUS_MIPI,
	HWINFO_BUS_UART,
	HWINFO_BUS_I2S,
	HWINFO_BUS_VIRTUAL,
	HWINFO_BUS_MAX,
};

struct hwinfo_time
{
	unsigned int year;
	unsigned int month;
	unsigned int day;
	unsigned int hour;
	unsigned int min;
};

struct hwinfo
{
	struct device *dev;
	char *device_name;			//sysfs node name 			
	char *devinfo;
	char *firmware;
	char *other;
	int update_times;	
	enum hwinfo_device_type device_type;
	enum hwinfo_bus_type bus_type;
	int index;
	int state;
	int version;
	struct hwinfo_time create_time;
	struct hwinfo_time update_time;
	struct list_head hwinfo_list;
	struct attribute_group * attr_group;
	void * pri_data;
};

extern struct hwinfo * hwinfo_create(const char *device_name,...);
extern int hwinfo_register(struct hwinfo *hwinfo);
extern int hwinfo_deregister(struct hwinfo *hwinfo);
extern int hwinfo_set_devinfo(struct hwinfo *hwinfo,const char *fmt,...);
extern int hwinfo_set_firmware(struct hwinfo *hwinfo,const char *fmt,...);
extern int hwinfo_set_other(struct hwinfo *hwinfo,const char *fmt,...);


#endif

