/* Himax Android Driver Sample Code for Himax chipset
*
* Copyright (C) 2014 Himax Corporation.
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

#include "himax_platform.h"

#define CONFIG_TOUCHSCREEN_HIMAX_DEBUG_PLAT 

#if defined(CONFIG_TOUCHSCREEN_HIMAX_DEBUG_PLAT)
#define D(x...) printk("[HXTP] " x)
#define I(x...) printk("[HXTP] " x)
#define W(x...) printk("[HXTP][WARNING] " x)
#define E(x...) printk("[HXTP][ERROR] " x)
#endif

int irq_enable_count = 0;
struct himax_i2c_platform_data *private_pdata;


DEFINE_MUTEX(hx_wr_access);

#ifdef MTK
u8 *gpDMABuf_va = NULL;
u8 *gpDMABuf_pa = NULL;
static int himax_tpd_int_gpio = 5;
extern int himax_tpd_rst_gpio_number;
extern int himax_tpd_int_gpio_number;

int i2c_himax_read(struct i2c_client *client, uint8_t command, uint8_t *data, uint8_t length, uint8_t toRetry)
{
	int ret=0;
	s32 retry = 0;
	u8 buffer[1];

	struct i2c_msg msg[] =
	{
		{
			.addr = (client->addr & I2C_MASK_FLAG),
			.flags = 0,
			.buf = buffer,
			.len = 1,
			.timing = 400
		},
		{
			.addr = (client->addr & I2C_MASK_FLAG),
			.ext_flag = (client->ext_flag | I2C_ENEXT_FLAG | I2C_DMA_FLAG),
			.flags = I2C_M_RD,
			.buf = gpDMABuf_pa,
			.len = length,
			.timing = 400
		},
	};
	mutex_lock(&hx_wr_access);
	buffer[0] = command;

	if (data == NULL){
		mutex_unlock(&hx_wr_access);
		return -1;
	}
	for (retry = 0; retry < toRetry; ++retry)
	{
		ret = i2c_transfer(client->adapter, &msg[0], 2);
		if (ret < 0)
		{
			continue;
		}
		memcpy(data, gpDMABuf_va, length);
		mutex_unlock(&hx_wr_access);
		return 0;
	}
	E("Dma I2C Read Error: %d byte(s), err-code: %d", length, ret);
#ifdef CONFIG_HUAWEI_DSM
	tpd_himax_dsm_report_err(DSM_TP_I2C_RW_ERROR_NO, ret);
#endif
	mutex_unlock(&hx_wr_access);
	return ret;
}

int i2c_himax_write(struct i2c_client *client, uint8_t command, uint8_t *buf, uint8_t len, uint8_t toRetry)
{
	int rc=0,retry=0;
	u8 *pWriteData = gpDMABuf_va;

	struct i2c_msg msg[]={
		{
		.addr = (client->addr & I2C_MASK_FLAG),
		.ext_flag = (client->ext_flag | I2C_ENEXT_FLAG | I2C_DMA_FLAG),
		.flags = 0,
		.buf = gpDMABuf_pa,
		.len = len+1,
		.timing = 400
		},
	};
	mutex_lock(&hx_wr_access);
	if(!pWriteData)
	{
		E("dma_alloc_coherent failed!\n");
		mutex_unlock(&hx_wr_access);
		return -1;
	}

	gpDMABuf_va[0] = command;

	memcpy(gpDMABuf_va+1, buf, len);

	for (retry = 0; retry < toRetry; ++retry)
	{
		rc = i2c_transfer(client->adapter, &msg[0], 1);
		if (rc < 0)
		{
			continue;
		}
		mutex_unlock(&hx_wr_access);
		return 0;
	}

	E("Dma I2C master write Error: %d byte(s), err-code: %d", len, rc);
#ifdef CONFIG_HUAWEI_DSM
	tpd_himax_dsm_report_err(DSM_TP_I2C_RW_ERROR_NO, rc);
#endif
	mutex_unlock(&hx_wr_access);
	return rc;
}

int i2c_himax_read_command(struct i2c_client *client, uint8_t len, uint8_t *buf, uint8_t *readlength, uint8_t toRetry)
{
	return 0;
}

int i2c_himax_write_command(struct i2c_client *client, uint8_t command, uint8_t toRetry)
{
	return i2c_himax_write(client, command, NULL, 0, toRetry);
}

int i2c_himax_master_write(struct i2c_client *client, uint8_t *buf, uint8_t len, uint8_t toRetry)
{
	int rc=0, retry=0;
	u8 *pWriteData = gpDMABuf_va;

	struct i2c_msg msg[] ={
		{
		.addr = (client->addr & I2C_MASK_FLAG),
		.ext_flag = (client->ext_flag | I2C_ENEXT_FLAG | I2C_DMA_FLAG),
		.flags = 0,
		.buf = gpDMABuf_pa,
		.len = len,
		.timing = 400
		},
	};

	mutex_lock(&hx_wr_access);
	if(!pWriteData)
	{
		E("dma_alloc_coherent failed!\n");
		mutex_unlock(&hx_wr_access);
		return -1;
	}

	memcpy(gpDMABuf_va, buf, len);
	for (retry = 0; retry < toRetry; ++retry)
	{
		rc = i2c_transfer(client->adapter, &msg[0], 1);
		if (rc < 0)
		{
			continue;
		}
		mutex_unlock(&hx_wr_access);
		return 0;
	}
	E("Dma I2C master write Error: %d byte(s), err-code: %d", len, rc);
	mutex_unlock(&hx_wr_access);
	return rc;
}

void himax_int_enable(int irqnum, int enable, int log_print)
{
	if (enable == 1 && (!atomic_read(&private_pdata->int_state))) {
		irq_enable_count=1;
		atomic_set(&private_pdata->int_state, 1);
#ifdef CONFIG_OF_TOUCH
		enable_irq(irqnum);
#else
		mt_eint_unmask(irqnum);
#endif		
	} else if (enable == 0 && (atomic_read(&private_pdata->int_state))) {
		irq_enable_count=0;
		atomic_set(&private_pdata->int_state, 0);
#ifdef CONFIG_OF_TOUCH
		disable_irq_nosync(irqnum);
#else
		mt_eint_mask(irqnum);
#endif		
	}
	
	if(log_print)
		I("enable=%d, irq_enable_count = %d, int_state =%d\n", enable, irq_enable_count,atomic_read(&private_pdata->int_state));
}


void himax_rst_gpio_set(int pinnum, uint8_t value)
{	
/*
	mt_set_gpio_mode(GPIO_CTP_RST_PIN, GPIO_CTP_RST_PIN_M_GPIO);
	mt_set_gpio_dir(GPIO_CTP_RST_PIN, GPIO_DIR_OUT);
	if(value)
		mt_set_gpio_out(GPIO_CTP_RST_PIN, GPIO_OUT_ONE);
	else
		mt_set_gpio_out(GPIO_CTP_RST_PIN, GPIO_OUT_ZERO); 
*/
/*	
    int retval = 0;
	retval = gpio_request_one(pinnum, GPIOF_OUT_INIT_LOW,"touchp_reset");
	if (retval < 0) {
		TPD_DMESG("Unable to request gpio reset_pin\n");
	}
*/
	if(value)
		tpd_gpio_output(himax_tpd_rst_gpio_number, 1);
	else
		tpd_gpio_output(himax_tpd_rst_gpio_number, 0);			
}

uint8_t himax_int_gpio_read(int pinnum)
{
	//return mt_get_gpio_in(GPIO_CTP_EINT_PIN);
	return  gpio_get_value(himax_tpd_int_gpio);
}

int himax_gpio_power_config(struct i2c_client *client,struct himax_i2c_platform_data *pdata)
{
	int retval = 0;
/*	
#ifdef CONFIG_OF_TOUCH
	struct device_node *node = NULL;
	u32 ints[2] = {0,0};
	
	node = of_find_compatible_node(NULL, NULL, "mediatek, TOUCH_PANEL-eint");
	if(node){
		of_property_read_u32_array(node , "debounce", ints, ARRAY_SIZE(ints));
		gpio_set_debounce(ints[0], ints[1]);
		client->irq = irq_of_parse_and_map(node, 0);
	}
#else
	client->irq = CUST_EINT_TOUCH_PANEL_NUM;
#endif
	//mt_set_gpio_mode(GPIO_CTP_EN_PIN, GPIO_CTP_EN_PIN_M_GPIO);
	//mt_set_gpio_dir(GPIO_CTP_EN_PIN, GPIO_DIR_OUT);
	//mt_set_gpio_out(GPIO_CTP_EN_PIN, GPIO_OUT_ONE);
	//mdelay(10);

	// TODO Interrupt / Reset Pin Setup
	//mt_set_gpio_mode(GPIO_CTP_RST_PIN, GPIO_CTP_RST_PIN_M_GPIO);
	//mt_set_gpio_dir(GPIO_CTP_RST_PIN, GPIO_DIR_OUT);

*/
	private_pdata=pdata;

#if 0
//#if	TPD_POWER_SOURCE_CUSTOM
	retval = regulator_enable(tpd->reg);
	if (retval != 0)
		TPD_DMESG("Failed to enable reg-vgp6: %d\n", retval);
	msleep(100);
#endif

	tpd_gpio_output(himax_tpd_rst_gpio_number, 1);
	msleep(30);
	tpd_gpio_output(himax_tpd_rst_gpio_number, 0);
	msleep(30);
	tpd_gpio_output(himax_tpd_rst_gpio_number, 1);
	msleep(120);

	TPD_DMESG("mtk_tpd: himax reset over \n");
	
	/* set INT mode */
	tpd_gpio_as_int(himax_tpd_int_gpio_number);

/* 
	retval = gpio_request_one(pdata->gpio_reset, GPIOF_OUT_INIT_LOW,"touchp_reset");
	if (retval < 0) {
		TPD_DMESG("Unable to request gpio reset_pin\n");
	}

	//Himax: SET Interrupt GPIO, no setting PULL LOW or PULL HIGH  
	//mt_set_gpio_mode(GPIO_CTP_EINT_PIN, GPIO_CTP_EINT_PIN_M_EINT);
	//mt_set_gpio_dir(GPIO_CTP_EINT_PIN, GPIO_DIR_IN);
	//mt_set_gpio_pull_enable(GPIO_CTP_EINT_PIN, GPIO_PULL_DISABLE);
	//mt_set_gpio_pull_select(GPIO_CTP_EINT_PIN, GPIO_PULL_UP);
	retval = gpio_request_one(pdata->gpio_irq, GPIOF_IN,"tpd_int");
	if (retval < 0) {
		TPD_DMESG("Unable to request gpio int_pin\n");
		gpio_free(pdata->gpio_irq);
	}
	
	// TODO Power Pin Setup
	//hwPowerOn(MT65XX_POWER_LDO_VGP4, VOL_2800, "TP");
	//hwPowerOn(MT65XX_POWER_LDO_VGP5, VOL_1800, "TP_EINT");
	msleep(10);

	// HW Reset
	//mt_set_gpio_out(GPIO_CTP_RST_PIN, GPIO_OUT_ONE);
	gpio_direction_output(pdata->gpio_reset, 1);
	msleep(20);
	//mt_set_gpio_out(GPIO_CTP_RST_PIN, GPIO_OUT_ZERO);
	gpio_direction_output(pdata->gpio_reset, 0);
	msleep(20);
	//mt_set_gpio_out(GPIO_CTP_RST_PIN, GPIO_OUT_ONE);
	gpio_direction_output(pdata->gpio_reset, 1);	
	msleep(20);  
*/
return 0;
}

#endif
