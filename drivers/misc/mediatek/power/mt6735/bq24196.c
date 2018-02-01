#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/irq.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/workqueue.h>
#include <linux/platform_device.h>
#include <linux/gpio.h>
#include <linux/reboot.h>
#include <linux/switch.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/regulator/consumer.h>
#include "bq24196.h"
#include "mach/mt_charging.h"
#include <mt-plat/upmu_common.h>
#include <mt-plat/mt_reboot.h>
#include <mt-plat/mt_boot.h>
#include <mt-plat/charging.h>
#include <mt-plat/battery_common.h>
#include <linux/init.h>//
#include <linux/module.h>//
#include <linux/power_supply.h>//
#include <linux/slab.h>//
#include <linux/kernel.h>//
#include <linux/sched.h>//
#include <linux/kthread.h>//
#include <linux/types.h>
#include <mach/upmu_hw.h>
#include <mt-plat/mt_gpio.h>
#include "hwinfo.h"

/**********************************************************
 *
 *    [Define]
 *
 **********************************************************/

#ifdef CONFIG_HUAWEI_DSM
#include <dsm/dsm_pub.h>
#include <linux/kthread.h>

static struct dsm_dev dsm_bq24196_dev = {
       .name           = "dsm_charger_bq24196",
       .fops           = NULL,
       .buff_size      = DSM_SENSOR_BUF_MAX,
};
 
static struct dsm_client *dsm_bq24196_client = NULL;

static int bq24196_dsm_report_err(int errno)
{
	int size = 0;

	if(dsm_client_ocuppy(dsm_bq24196_client)){
		/* buffer is busy */
		pr_err("%s: buffer is busy!, errno = %d\n", __func__,errno);
		return -EBUSY;
	}

	pr_err("dsm bq24196 error, errno = %d \n", errno);

	switch(errno)
	{
		case DSM_BQ24196_I2C_WR_ERROR:
			dsm_client_record(dsm_bq24196_client,"BQ24196 I2C read/write error\n");
			break;
		case DSM_BQ24196_WATCHDOG_FAULT_ERROR:
			dsm_client_record(dsm_bq24196_client,"BQ24196 watchdog timer expiration\n");
			break;
		case DSM_BQ24196_POWER_SUPPLY_OVP_ERROR:
			dsm_client_record(dsm_bq24196_client,"BQ24196 power supply vbus ovp\n");
			break;
		case DSM_BQ24196_THERMAL_SHUTDOWN_ERROR:
			dsm_client_record(dsm_bq24196_client,"BQ24196 thermal shutdown\n");
			break;
		case DSM_BQ24196_CHRG_TIMER_EXPIRED_ERROR:
			dsm_client_record(dsm_bq24196_client,"BQ24196 charge safety time expiration\n");
			break;
		case DSM_BQ24196_BAT_FAULT_OVP_ERROR:
			dsm_client_record(dsm_bq24196_client,"BQ24196 battery fault batovp\n");
			break;
		default:
			break;
	}
    dsm_client_notify(dsm_bq24196_client, errno);

	return size;
}

#endif


#define bq24196_REG_NUM 11
#define bq24196_SLAVE_ADDR_WRITE   0xD6
#define bq24196_SLAVE_ADDR_READ    0xD7
#define BQ24196_BUSNUM 1

/**********************************************************
  *
  *   [Global Variable]
  *
  *********************************************************/

static struct i2c_client *new_client;
int g_bq24196_hw_exist=0;
static unsigned char bq24196_reg[bq24196_REG_NUM] = { 0 };

static unsigned char g_reg_value_bq24196;
static DEFINE_MUTEX(bat_mutex);

/**********************************************************
  *
  *   [I2C Function For Read/Write bq24196]
  *
  *********************************************************/
kal_bool chargin_hw_init_done = KAL_FALSE;
int bq24196_read_byte(unsigned char cmd, unsigned char *data)
{
	int ret;

	struct i2c_msg msg[2];

	if (!new_client) {
		pr_err("error: access bq24196 before driver ready\n");
		return 0;
	}

	msg[0].addr = new_client->addr ;
	msg[0].ext_flag = new_client->ext_flag;
	msg[0].buf = &cmd;
	msg[0].flags = 0;
	msg[0].len = 1;
#ifdef CONFIG_MTK_I2C_EXTENSION
	msg[0].timing = 300;
#endif
	msg[1].addr = new_client->addr;
	msg[1].ext_flag = new_client->ext_flag;
	msg[1].buf = data;
	msg[1].flags = I2C_M_RD;
	msg[1].len = 1;
#ifdef CONFIG_MTK_I2C_EXTENSION
	msg[1].timing = 300;
#endif

	ret = i2c_transfer(new_client->adapter, msg, 2);
	if (ret != 2)
	{
		pr_err("%s: err=%d\n", __func__, ret);
	#ifdef CONFIG_HUAWEI_DSM
		bq24196_dsm_report_err(DSM_BQ24196_I2C_WR_ERROR);
	#endif
	}
	return ret == 2 ? 1 : 0;
}

int bq24196_write_byte(unsigned char cmd, unsigned char data)
{
	char buf[2];
	int ret;

	if (!new_client) {
		pr_err("error: access bq24196 before driver ready\n");
		return 0;
	}

	buf[0] = cmd;
	buf[1] = data;

	ret = i2c_master_send(new_client, buf, 2);

	if (ret != 2)
	{
		pr_err("%s: err=%d\n", __func__, ret);
	#ifdef CONFIG_HUAWEI_DSM
		bq24196_dsm_report_err(DSM_BQ24196_I2C_WR_ERROR);
	#endif
	}
	return ret == 2 ? 1 : 0;
}

unsigned int bq24196_read_interface(unsigned char RegNum, unsigned char *val, unsigned char MASK, unsigned char SHIFT)
{
	unsigned char bq24196_reg = 0;
	int ret = 0;

	ret = bq24196_read_byte(RegNum, &bq24196_reg);

	bq24196_reg &= (MASK << SHIFT);
	*val = (bq24196_reg >> SHIFT);

	return ret;
}

unsigned int bq24196_config_interface(unsigned char RegNum, unsigned char val, unsigned char MASK, unsigned char SHIFT)
{
	unsigned char bq24196_reg = 0;
	int ret = 0;

	ret = bq24196_read_byte(RegNum, &bq24196_reg);

	bq24196_reg &= ~(MASK << SHIFT);
	bq24196_reg |= (val << SHIFT);

	ret = bq24196_write_byte(RegNum, bq24196_reg);
	return ret;
}

/**********************************************************
  *
  *   [Internal Function]
  *
  *********************************************************/
/* CON0---------------------------------------------------- */

void bq24196_set_en_hiz(unsigned int val)
{
	unsigned int ret = 0;

	ret = bq24196_config_interface((unsigned char) (bq24196_CON0),
				       (unsigned char) (val), (unsigned char) (CON0_EN_HIZ_MASK), (unsigned char) (CON0_EN_HIZ_SHIFT)
	    );
}

void bq24196_set_vindpm(unsigned int val)
{
	unsigned int ret = 0;

	ret = bq24196_config_interface((unsigned char) (bq24196_CON0),
				       (unsigned char) (val), (unsigned char) (CON0_VINDPM_MASK), (unsigned char) (CON0_VINDPM_SHIFT)
	    );
}

void bq24196_set_iinlim(unsigned int val)
{
	unsigned int ret = 0;

	ret = bq24196_config_interface((unsigned char) (bq24196_CON0),
				       (unsigned char) (val), (unsigned char) (CON0_IINLIM_MASK), (unsigned char) (CON0_IINLIM_SHIFT)
	    );
}

unsigned int bq24196_get_iinlim(void)
{
	unsigned int ret = 0;
	unsigned char val = 0;

	ret = bq24196_read_interface((unsigned char) (bq24196_CON0),
				     (&val), (unsigned char) (CON0_IINLIM_MASK), (unsigned char) (CON0_IINLIM_SHIFT)
	    );
	return val;
}

/* CON1---------------------------------------------------- */

void bq24196_set_reg_rst(unsigned int val)
{
	unsigned int ret = 0;

	ret = bq24196_config_interface((unsigned char) (bq24196_CON1),
				       (unsigned char) (val),
				       (unsigned char) (CON1_REG_RST_MASK), (unsigned char) (CON1_REG_RST_SHIFT)
	    );
}

void bq24196_set_wdt_rst(unsigned int val)
{
	unsigned int ret = 0;

	ret = bq24196_config_interface((unsigned char) (bq24196_CON1),
				       (unsigned char) (val),
				       (unsigned char) (CON1_WDT_RST_MASK), (unsigned char) (CON1_WDT_RST_SHIFT)
	    );
}

void bq24196_set_otg_config(unsigned int val)
{
	unsigned int ret = 0;

	ret = bq24196_config_interface((unsigned char) (bq24196_CON1),
				       (unsigned char) (val),
				       (unsigned char) (CON1_OTG_CONFIG_MASK), (unsigned char) (CON1_OTG_CONFIG_SHIFT)
	    );
}

void bq24196_set_chg_config(unsigned int val)
{
	unsigned int ret = 0;

	ret = bq24196_config_interface((unsigned char) (bq24196_CON1),
				       (unsigned char) (val),
				       (unsigned char) (CON1_CHG_CONFIG_MASK), (unsigned char) (CON1_CHG_CONFIG_SHIFT)
	    );
}

void bq24196_set_sys_min(unsigned int val)
{
	unsigned int ret = 0;

	ret = bq24196_config_interface((unsigned char) (bq24196_CON1),
				       (unsigned char) (val),
				       (unsigned char) (CON1_SYS_MIN_MASK), (unsigned char) (CON1_SYS_MIN_SHIFT)
	    );
}

void bq24196_set_boost_lim(unsigned int val)
{
	unsigned int ret = 0;

	ret = bq24196_config_interface((unsigned char) (bq24196_CON1),
				       (unsigned char) (val),
				       (unsigned char) (CON1_BOOST_LIM_MASK), (unsigned char) (CON1_BOOST_LIM_SHIFT)
	    );
}

/* CON2---------------------------------------------------- */

void bq24196_set_ichg(unsigned int val)
{
	unsigned int ret = 0;

	ret = bq24196_config_interface((unsigned char) (bq24196_CON2),
				       (unsigned char) (val), (unsigned char) (CON2_ICHG_MASK), (unsigned char) (CON2_ICHG_SHIFT)
	    );
}

void bq24196_set_force_20pct(unsigned int val)
{
	unsigned int ret = 0;

	ret = bq24196_config_interface((unsigned char) (bq24196_CON2),
				       (unsigned char) (val),
				       (unsigned char) (CON2_FORCE_20PCT_MASK), (unsigned char) (CON2_FORCE_20PCT_SHIFT)
	    );
}

/* CON3---------------------------------------------------- */

void bq24196_set_iprechg(unsigned int val)
{
	unsigned int ret = 0;

	ret = bq24196_config_interface((unsigned char) (bq24196_CON3),
				       (unsigned char) (val),
				       (unsigned char) (CON3_IPRECHG_MASK), (unsigned char) (CON3_IPRECHG_SHIFT)
	    );
}

void bq24196_set_iterm(unsigned int val)
{
	unsigned int ret = 0;

	ret = bq24196_config_interface((unsigned char) (bq24196_CON3),
				       (unsigned char) (val), (unsigned char) (CON3_ITERM_MASK), (unsigned char) (CON3_ITERM_SHIFT)
	    );
}

/* CON4---------------------------------------------------- */

void bq24196_set_vreg(unsigned int val)
{
	unsigned int ret = 0;

	ret = bq24196_config_interface((unsigned char) (bq24196_CON4),
				       (unsigned char) (val), (unsigned char) (CON4_VREG_MASK), (unsigned char) (CON4_VREG_SHIFT)
	    );
}

void bq24196_set_batlowv(unsigned int val)
{
	unsigned int ret = 0;

	ret = bq24196_config_interface((unsigned char) (bq24196_CON4),
				       (unsigned char) (val),
				       (unsigned char) (CON4_BATLOWV_MASK), (unsigned char) (CON4_BATLOWV_SHIFT)
	    );
}

void bq24196_set_vrechg(unsigned int val)
{
	unsigned int ret = 0;

	ret = bq24196_config_interface((unsigned char) (bq24196_CON4),
				       (unsigned char) (val), (unsigned char) (CON4_VRECHG_MASK), (unsigned char) (CON4_VRECHG_SHIFT)
	    );
}

/* CON5---------------------------------------------------- */

void bq24196_set_en_term(unsigned int val)
{
	unsigned int ret = 0;

	ret = bq24196_config_interface((unsigned char) (bq24196_CON5),
				       (unsigned char) (val),
				       (unsigned char) (CON5_EN_TERM_MASK), (unsigned char) (CON5_EN_TERM_SHIFT)
	    );
}

void bq24196_set_term_stat(unsigned int val)
{
	unsigned int ret = 0;

	ret = bq24196_config_interface((unsigned char) (bq24196_CON5),
				       (unsigned char) (val),
				       (unsigned char) (CON5_TERM_STAT_MASK), (unsigned char) (CON5_TERM_STAT_SHIFT)
	    );
}

void bq24196_set_watchdog(unsigned int val)
{
	unsigned int ret = 0;

	ret = bq24196_config_interface((unsigned char) (bq24196_CON5),
				       (unsigned char) (val),
				       (unsigned char) (CON5_WATCHDOG_MASK), (unsigned char) (CON5_WATCHDOG_SHIFT)
	    );
}

void bq24196_set_en_timer(unsigned int val)
{
	unsigned int ret = 0;

	ret = bq24196_config_interface((unsigned char) (bq24196_CON5),
				       (unsigned char) (val),
				       (unsigned char) (CON5_EN_TIMER_MASK), (unsigned char) (CON5_EN_TIMER_SHIFT)
	    );
}

void bq24196_set_chg_timer(unsigned int val)
{
	unsigned int ret = 0;

	ret = bq24196_config_interface((unsigned char) (bq24196_CON5),
				       (unsigned char) (val),
				       (unsigned char) (CON5_CHG_TIMER_MASK), (unsigned char) (CON5_CHG_TIMER_SHIFT)
	    );
}

/* CON6---------------------------------------------------- */

void bq24196_set_treg(unsigned int val)
{
	unsigned int ret = 0;

	ret = bq24196_config_interface((unsigned char) (bq24196_CON6),
				       (unsigned char) (val), (unsigned char) (CON6_TREG_MASK), (unsigned char) (CON6_TREG_SHIFT)
	    );
}

/* CON7---------------------------------------------------- */
unsigned int bq24196_get_dpdm_status(void)
{
	unsigned int ret = 0;
	unsigned char val = 0;

	ret = bq24196_read_interface((unsigned char) (bq24196_CON7),
				     (&val), (unsigned char) (CON7_DPDM_EN_MASK), (unsigned char) (CON7_DPDM_EN_SHIFT)
	    );
	return val;
}

void bq24196_set_dpdm_en(unsigned int val)
{
	unsigned int ret = 0;

	ret = bq24196_config_interface((unsigned char) (bq24196_CON7),
				       (unsigned char) (val),
				       (unsigned char) (CON7_DPDM_EN_MASK), (unsigned char) (CON7_DPDM_EN_SHIFT)
	    );
}

void bq24196_set_tmr2x_en(unsigned int val)
{
	unsigned int ret = 0;

	ret = bq24196_config_interface((unsigned char) (bq24196_CON7),
				       (unsigned char) (val),
				       (unsigned char) (CON7_TMR2X_EN_MASK), (unsigned char) (CON7_TMR2X_EN_SHIFT)
	    );
}

void bq24196_set_batfet_disable(unsigned int val)
{
	unsigned int ret = 0;

	ret = bq24196_config_interface((unsigned char) (bq24196_CON7),
				       (unsigned char) (val),
				       (unsigned char) (CON7_BATFET_Disable_MASK),
				       (unsigned char) (CON7_BATFET_Disable_SHIFT)
	    );
}

void bq24196_set_int_mask(unsigned int val)
{
	unsigned int ret = 0;

	ret = bq24196_config_interface((unsigned char) (bq24196_CON7),
				       (unsigned char) (val),
				       (unsigned char) (CON7_INT_MASK_MASK), (unsigned char) (CON7_INT_MASK_SHIFT)
	    );
}

/* CON8---------------------------------------------------- */

unsigned int bq24196_get_system_status(void)
{
	unsigned int ret = 0;
	unsigned char val = 0;

	ret = bq24196_read_interface((unsigned char) (bq24196_CON8), (&val), (unsigned char) (0xFF), (unsigned char) (0x0)
	    );
	return val;
}

unsigned int bq24196_get_vbus_stat(void)
{
	unsigned int ret = 0;
	unsigned char val = 0;

	ret = bq24196_read_interface((unsigned char) (bq24196_CON8),
				     (&val), (unsigned char) (CON8_VBUS_STAT_MASK), (unsigned char) (CON8_VBUS_STAT_SHIFT)
	    );
	return val;
}

unsigned int bq24196_get_chrg_stat(void)
{
	unsigned int ret = 0;
	unsigned char val = 0;

	ret = bq24196_read_interface((unsigned char) (bq24196_CON8),
				     (&val), (unsigned char) (CON8_CHRG_STAT_MASK), (unsigned char) (CON8_CHRG_STAT_SHIFT)
	    );
	return val;
}

unsigned int bq24196_get_pg_stat(void)
{
	unsigned int ret = 0;
	unsigned char val = 0;

	ret = bq24196_read_interface((unsigned char) (bq24196_CON8),
				     (&val), (unsigned char) (CON8_PG_STAT_MASK), (unsigned char) (CON8_PG_STAT_SHIFT)
	    );
	return val;
}

unsigned int bq24196_get_vsys_stat(void)
{
	unsigned int ret = 0;
	unsigned char val = 0;

	ret = bq24196_read_interface((unsigned char) (bq24196_CON8),
				     (&val), (unsigned char) (CON8_VSYS_STAT_MASK), (unsigned char) (CON8_VSYS_STAT_SHIFT)
	    );
	return val;
}
/* CON9---------------------------------------------------- */

unsigned int bq24196_get_watchdog_fault(void)
{
	unsigned int ret = 0;
	unsigned char val = 0;

	ret = bq24196_read_interface((unsigned char) (bq24196_CON9),
				     (&val), (unsigned char) (CON9_WATCHDOG_FAULT_MASK), (unsigned char) (CON9_WATCHDOG_FAULT_SHIFT)
	    );
#ifdef CONFIG_HUAWEI_DSM
	if(val)
		bq24196_dsm_report_err(DSM_BQ24196_WATCHDOG_FAULT_ERROR);
#endif
	return val;
}

unsigned int bq24196_get_chrg_fault(void)
{
	unsigned int ret = 0;
	unsigned char val = 0;

	ret = bq24196_read_interface((unsigned char) (bq24196_CON9),
				     (&val), (unsigned char) (CON9_CHRG_FAULT_MASK), (unsigned char) (CON9_CHRG_FAULT_SHIFT)
	    );
#ifdef CONFIG_HUAWEI_DSM
	if(val==1)
		bq24196_dsm_report_err(DSM_BQ24196_POWER_SUPPLY_OVP_ERROR);
	else if(val==1)
		bq24196_dsm_report_err(DSM_BQ24196_THERMAL_SHUTDOWN_ERROR);
	else if(val==3)
		bq24196_dsm_report_err(DSM_BQ24196_CHRG_TIMER_EXPIRED_ERROR);
#endif
	return val;
}

unsigned int bq24196_get_bat_fault(void)
{
	unsigned int ret = 0;
	unsigned char val = 0;

	ret = bq24196_read_interface((unsigned char) (bq24196_CON9),
				     (&val), (unsigned char) (CON9_BAT_FAULT_MASK), (unsigned char) (CON9_BAT_FAULT_SHIFT)
	    );
#ifdef CONFIG_HUAWEI_DSM
	if(val)
		bq24196_dsm_report_err(DSM_BQ24196_BAT_FAULT_OVP_ERROR);
#endif
	return val;
}

/* CON10---------------------------------------------------- */

unsigned int bq24196_get_pn(void)
{
	unsigned int ret = 0;
	unsigned char val = 0;

	ret = bq24196_read_interface((unsigned char) (bq24196_CON10),
				     (&val), (unsigned char) (CON10_PN_MASK), (unsigned char) (CON10_PN_SHIFT)
	    );
	return val;
}

/**********************************************************
  *
  *   [Internal Function]
  *
  *********************************************************/
void bq24196_hw_component_detect(void)
{
	unsigned int ret=0;
	unsigned char val=0;

	ret=bq24196_read_interface(0x0A, &val, 0xFF, 0x0);
    
	if(val == 0)
		g_bq24196_hw_exist=0;
	else
		g_bq24196_hw_exist=1;

	printk("[bq24196_hw_component_detect] exist=%d, Reg[0x0A]=0x%x\n", 
	g_bq24196_hw_exist, val);
}
int is_bq24196_exist(void)
{
	printk("[is_bq24196_exist] g_bq24196_hw_exist=%d\n", g_bq24196_hw_exist);
    
	return g_bq24196_hw_exist;
}

void bq24196_dump_register(void)
{
	int i = 0;

	for (i = 0; i < bq24196_REG_NUM; i++) {
		bq24196_read_byte(i, &bq24196_reg[i]);
		battery_log(BAT_LOG_CRTI,
			    "[bq24196_dump_register] Reg[0x%X]=0x%X\n", i, bq24196_reg[i]);
	}
}



static int bq24196_driver_suspend(struct i2c_client *client, pm_message_t mesg)
{
	pr_debug("[bq24196_driver_suspend] client->irq(%d)\n", client->irq);
	if (client->irq > 0)
		disable_irq(client->irq);

	return 0;
}

static int bq24196_driver_resume(struct i2c_client *client)
{
	pr_debug("[bq24196_driver_resume] client->irq(%d)\n", client->irq);
	if (client->irq > 0)
		enable_irq(client->irq);

	return 0;
}

static void bq24196_driver_shutdown(struct i2c_client *client)
{
	pr_debug("[bq24196_driver_shutdown] client->irq(%d)\n", client->irq);
	bq24196_set_en_hiz(0x0);
	if (client->irq > 0)
		disable_irq(client->irq);
}

static int bq24196_driver_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	int err=0;

	pr_notice("[bq24196_driver_probe] \n");

	if (!(new_client = kmalloc(sizeof(struct i2c_client), GFP_KERNEL))) {
		err = -ENOMEM;
		goto exit;
	}
	memset(new_client, 0, sizeof(struct i2c_client));

	new_client = client;

    //---------------------
	bq24196_hw_component_detect();
	bq24196_dump_register();
	chargin_hw_init_done = KAL_TRUE;

	return 0;

exit:
	return err;
}

static const struct i2c_device_id bq24196_i2c_id[] = { {"bq24196", 0}, {} };

#ifdef CONFIG_OF
static const struct of_device_id bq24196_id[] = {
	{.compatible = "mediatek,SWITHING_CHARGER"},
	{},
};

MODULE_DEVICE_TABLE(of, bq24196_id);
#endif

static struct i2c_driver bq24196_driver = {
	.driver = {
		   .name = "bq24196",
#ifdef CONFIG_OF
		   .of_match_table = of_match_ptr(bq24196_id),
#endif
		   },
	.probe = bq24196_driver_probe,
	.shutdown = bq24196_driver_shutdown,
	.suspend = bq24196_driver_suspend,
	.resume = bq24196_driver_resume,
	.id_table = bq24196_i2c_id,
};

static ssize_t show_bq24196_access(struct device *dev, struct device_attribute *attr, char *buf)
{
	pr_notice("[show_bq24196_access] 0x%x\n", g_reg_value_bq24196);
	return sprintf(buf, "0x%x\n", g_reg_value_bq24196);
}

static ssize_t store_bq24196_access(struct device *dev, struct device_attribute *attr,
				    const char *buf, size_t size)
{
	int ret = 0;
	char *pvalue, temp_buf[16];
	unsigned int reg_value = 0;
	unsigned int reg_address = 0;

	if (buf != NULL && size != 0) {

		strcpy(temp_buf, buf);
		pvalue = temp_buf;
		if (size > 4) {
			ret = kstrtouint(strsep(&pvalue, " "), 0, &reg_address);
			if (ret) {
				pr_err("wrong format!\n");
				return size;
			}
			ret = kstrtouint(pvalue, 0, &reg_value);
			if (ret) {
				pr_err("wrong format!\n");
				return size;
			}
			pr_notice("[store_bq24196_access] write bq24196 reg 0x%x with value 0x%x !\n",
				reg_address, reg_value);
			bq24196_config_interface(reg_address, reg_value, 0xFF, 0x0);
		} else {
			ret = kstrtouint(pvalue, 0, &reg_address);
			if (ret) {
				pr_err("wrong format!\n");
				return size;
			}
			bq24196_read_interface(reg_address, &g_reg_value_bq24196, 0xFF, 0x0);
			pr_notice("[store_bq24196_access] read bq24196 reg 0x%x with value 0x%x !\n",
				reg_address, g_reg_value_bq24196);
			pr_notice
			    ("[store_bq24196_access] Please use \"cat bq24196_access\" to get value\r\n");
		}
	}
	return size;

}

static DEVICE_ATTR(bq24196_access, S_IWUSR | S_IRUGO, show_bq24196_access, store_bq24196_access);

static int bq24196_user_space_probe(struct platform_device *dev)
{
	int ret_device_file = 0;

	pr_debug("bq24196_user_space_probe!\n");
	ret_device_file = device_create_file(&(dev->dev), &dev_attr_bq24196_access);

	return 0;
}

struct platform_device bq24196_user_space_device = {
	.name = "bq24196-user",
	.id = -1,
};

static struct platform_driver bq24196_user_space_driver = {
	.probe = bq24196_user_space_probe,
	.driver = {
		   .name = "bq24196-user",
		   },
};

static struct i2c_board_info __initdata i2c_bq24196 = { I2C_BOARD_INFO("bq24196", 0x6B),};

static int __init bq24196_init(void)
{
	int ret = 0;
	pr_notice("[bq24196_init] init start.\n");
#if 0
	ret =i2c_register_board_info(BQ24196_BUSNUM, &i2c_bq24196, 1);
	if(ret)
		{
			pr_notice("i2c_register_board_info error.\n");
		}
#else
	struct i2c_adapter *adap = i2c_get_adapter(1);
	i2c_new_device(adap, &i2c_bq24196);
	i2c_put_adapter(adap);
#endif
	if (i2c_add_driver(&bq24196_driver) != 0)
		pr_err("[bq24196_init] failed to register bq24196 i2c driver.\n");
	else
		pr_err("[bq24196_init] Success to register bq24196 i2c driver.\n");

	/* bq24196 user space access interface */
	ret = platform_device_register(&bq24196_user_space_device);
	if (ret) {
		pr_err("[bq24196_init] Unable to device register(%d)\n", ret);
		return ret;
	}
	ret = platform_driver_register(&bq24196_user_space_driver);
	if (ret) {
		pr_err("[bq24196_init] Unable to register driver (%d)\n", ret);
		return ret;
	}

#ifdef CONFIG_HUAWEI_DSM
       dsm_bq24196_client= dsm_register_client(&dsm_bq24196_dev);
       if(!dsm_bq24196_client) {
               pr_err("%s: register dsm_dclient failed!!, line = %d\n", __func__,__LINE__);
               }
#endif


	return 0;
}

static void __exit bq24196_exit(void)
{
	i2c_del_driver(&bq24196_driver);
#ifdef CONFIG_HUAWEI_DSM
    dsm_unregister_client(dsm_bq24196_client,&dsm_bq24196_dev);
#endif
}
module_init(bq24196_init);
module_exit(bq24196_exit);


MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("I2C bq24196 Driver");
MODULE_AUTHOR("Tank Hung<tank.hung@mediatek.com>");

