/*
 *
 * Copyright (C) 2013 Huawei Device Co.Ltd
 * License terms: GNU General Public License (GPL) version 2
 *
 */
#include <linux/module.h>
#include <linux/init.h>
#include <linux/ctype.h>
#include <linux/delay.h>
#include <linux/input/mt.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <linux/gpio.h>
#include <linux/debugfs.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <linux/regulator/consumer.h>
#include <linux/string.h>
#include <linux/of_gpio.h>
#include <linux/kthread.h>
#include <huawei_platform/log/hw_log.h>
#include <linux/of.h>
#include <dsm/dsm_pub.h>
#include <linux/platform_device.h>

#define HWLOG_TAG DSM_EARLYBOOT_CLIENT
HWLOG_REGIST();


int poll (void)
{
	unsigned int random;
	int result;

	random = 10;//get_random_int();
	result = (random%2) ? 0 : 5;
	//DSM_LOG_INFO("%s enter random num %u, poll result %d\n", __func__, random, result);
	return result;
}
int dump (int type, void *buff, int size)
{
	int used_size = 0;
	char *ptr = buff;

	//DSM_LOG_INFO("%s called, type %d\n", __func__, type);
	used_size += snprintf(ptr, (size - used_size), "dump called:\n");
	ptr = buff + used_size;
	used_size += snprintf(ptr, (size - used_size), "Today is saturday\n");
	ptr = buff + used_size;
	used_size += snprintf(ptr, (size - used_size), "Tempeature is 10 degree\n");
	ptr = buff + used_size;
	used_size += snprintf(ptr, (size - used_size), "Wheather is sunny\n");
	ptr = buff + used_size;
	used_size += snprintf(ptr, (size - used_size), "Fit to do sports or have a picnic\n");

	return used_size;
}

struct dsm_client_ops ops = {
	.poll_state = poll,
	.dump_func = dump,
};

static struct dsm_dev public_client_devices[] = {
    {
		.name = "dsm_apcp",
		.device_name = NULL,
		.ic_name = NULL,
		.module_name = NULL,
		.fops = NULL,
		.buff_size = 1024,
    },
    {
		.name = "dsm_spi",
		.device_name = NULL,
		.ic_name = NULL,
		.module_name = NULL,
		.fops = NULL,
		.buff_size = 1024,
    },
	{
		.name = "smartpa",
		.device_name = NULL,
		.ic_name = NULL,
		.module_name = NULL,
		.fops = NULL,
		.buff_size = 1024,
	},
	{
		.name = "dsm_selinux",
		.device_name = NULL,
		.ic_name = NULL,
		.module_name = NULL,
		.fops = NULL,
		.buff_size = 1024,
    },
#ifdef CONFIG_HUAWEI_SDCARD_VOLD
	{
		.name = "sdcard_vold",
		.device_name = NULL,
		.ic_name = NULL,
		.module_name = NULL,
		.fops = NULL,
		.buff_size = 1024,
	},
#endif
    {
		.name = "dsm_e4defrag",
		.device_name = NULL,
		.ic_name = NULL,
		.module_name = NULL,
		.fops = NULL,
		.buff_size = 1024,
    },
    {
		.name = "dsm_uart",
		.device_name = NULL,
		.ic_name = NULL,
		.module_name = NULL,
		.fops = NULL,
		.buff_size = 1024,
    },
    {
		.name = "dsm_lcd",
		.device_name = NULL,
		.ic_name = NULL,
		.module_name = NULL,
		.fops = &ops,
		.buff_size = 1024,
    },
};

int get_spi_client(struct dsm_client **dev)
{
	*dev = dsm_find_client("dsm_spi");

    return 0;
}
int get_selinux_client(struct dsm_client **dev)
{
	*dev = dsm_find_client("dsm_selinux");

    return 0;
}
int dsm_earlyboot_client_remove(struct platform_device *dev)
{
    hwlog_info("called remove %s\n", __func__);
    return 0;
}

static int __init dsm_earlyboot_client_init(void)
{
    int i = 0;

    for (i = 0; i < ARRAY_SIZE(public_client_devices); i++) {
		dsm_register_client(&public_client_devices[i]);
    }

    return 0;
}

static void __exit dsm_earlyboot_client_exit(void)
{
	return;
}

late_initcall(dsm_earlyboot_client_init);
module_exit(dsm_earlyboot_client_exit);

MODULE_AUTHOR("Huawei Device Company");
MODULE_DESCRIPTION("Huawei DSM EARLYBOOT CLIENT");
MODULE_LICENSE("GPL");
