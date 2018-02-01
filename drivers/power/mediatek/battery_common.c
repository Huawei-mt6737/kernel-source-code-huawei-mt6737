/*****************************************************************************
 *
 * Filename:
 * ---------
 *    battery_common.c
 *
 * Project:
 * --------
 *   Android_Software
 *
 * Description:
 * ------------
 *   This Module defines functions of mt6323 Battery charging algorithm
 *   and the Anroid Battery service for updating the battery status
 *
 * Author:
 * -------
 * Oscar Liu
 *
 ****************************************************************************/
#include <linux/init.h>		/* For init/exit macros */
#include <linux/module.h>	/* For MODULE_ marcros  */
#include <linux/fs.h>
#include <linux/device.h>
#include <linux/interrupt.h>
#include <linux/spinlock.h>
#include <linux/platform_device.h>
#include <linux/device.h>
#include <linux/kdev_t.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/delay.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/types.h>
#include <linux/wait.h>
#include <linux/slab.h>
#include <linux/fs.h>
#include <linux/sched.h>
#include <linux/poll.h>
#include <linux/power_supply.h>
#include <linux/wakelock.h>
#include <linux/time.h>
#include <linux/mutex.h>
#include <linux/kthread.h>
#include <linux/proc_fs.h>
#include <linux/platform_device.h>
#include <linux/seq_file.h>
#include <linux/scatterlist.h>
#ifdef CONFIG_OF
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/of_address.h>
#endif
#include <linux/suspend.h>
#include <linux/reboot.h>


#include <asm/scatterlist.h>
#include <asm/uaccess.h>
#include <asm/io.h>
#include <asm/irq.h>

#include <mt-plat/mt_boot.h>
#include <mt-plat/mtk_rtc.h>

#include <mach/mt_charging.h>
#include <mt-plat/upmu_common.h>

#include <mt-plat/charging.h>
#include <mt-plat/battery_meter.h>
#include <mt-plat/battery_common.h>
#include <mach/mt_battery_meter.h>
#include <mach/mt_charging.h>
#include <mach/mt_pmic.h>
#ifdef CONFIG_HQ_HWINFO
#include "hwinfo.h"
struct hwinfo *battery_hwinfo = NULL;
#endif

#ifdef HUAWEI_DYNAMIC_CHARGER //huawei dpm
kal_bool dpm_first_select_cc= KAL_TRUE;
kal_bool find_dpm_cc= KAL_FALSE;
kal_bool up_status= KAL_FALSE;
kal_bool down_status= KAL_FALSE;
unsigned int cc_rank_value= 2; 
#endif

#if defined(CONFIG_MTK_DUAL_INPUT_CHARGER_SUPPORT)
#include <mach/diso.h>
#endif

#if defined(CONFIG_MTK_PUMP_EXPRESS_PLUS_SUPPORT)
#include <mach/mt_pe.h>
#endif

#ifdef CONFIG_HUAWEI_DSM
#include <dsm/dsm_pub.h>
#include <linux/delay.h>

static struct dsm_dev dsm_mt_charger = {
       .name           = "dsm_charger",
       .fops           = NULL,
       .buff_size      = DSM_SENSOR_BUF_MAX,
};

static struct dsm_client *bq_power_dclient = NULL;
static int dms_chg_monitor_report_err(int errno);
int bat_update_thread_dsm(void);
#endif

#ifdef CONFIG_LOG_JANK
#include <linux/log_jank.h>
#endif
/* ////////////////////////////////////////////////////////////////////////////// */
/* Battery Logging Entry */
/* ////////////////////////////////////////////////////////////////////////////// */
int Enable_BATDRV_LOG = BAT_LOG_CRTI;
/* static struct proc_dir_entry *proc_entry; */
char proc_bat_data[32];

/* ///////////////////////////////////////////////////////////////////////////////////////// */
/* // Smart Battery Structure */
/* ///////////////////////////////////////////////////////////////////////////////////////// */
PMU_ChargerStruct BMT_status;
static int InstantICharging=0;

#if defined(CONFIG_MTK_DUAL_INPUT_CHARGER_SUPPORT)
DISO_ChargerStruct DISO_data;
/* Debug Msg */
static char *DISO_state_s[8] = {
	"IDLE",
	"OTG_ONLY",
	"USB_ONLY",
	"USB_WITH_OTG",
	"DC_ONLY",
	"DC_WITH_OTG",
	"DC_WITH_USB",
	"DC_USB_OTG",
};
#endif
#define CHARGE_FULL_DESIGN          (3000000)
extern kal_bool gFG_Is_Charging;
#ifdef CONFIG_HUAWEI_DSM
extern int dsm_batt_mt_id_error;
extern signed int gFG_current;
#endif

/*
 * Thermal related flags
 */
int g_battery_thermal_throttling_flag = 3;
/*  0:nothing,
 *	1:enable batTT&chrTimer,
 *	2:disable batTT&chrTimer,
 *	3:enable batTT,
 *	disable chrTimer
 */
int battery_cmd_thermal_test_mode = 0;
int battery_cmd_thermal_test_mode_value = 0;
int g_battery_tt_check_flag = 0;
/* 0:default enable check batteryTT, 1:default disable check batteryTT */
int three_percent_check_point = 3;
int three_percent_zcv = 3653;
int fifty_percent_check_point =65;//65% calibration 12-8
int fifty_percent_zcv = 4000;
int first_insert_charger;

/*
 *  Global Variable
 */
//zcy_add_20160711
int g_runin_test_enter = 0;
int g_runin_battery_test_flag = 0;
extern void pchr_turn_on_charging(void);

struct wake_lock battery_suspend_lock;
struct wake_lock battery_fg_lock;
CHARGING_CONTROL battery_charging_control;
unsigned int g_BatteryNotifyCode = 0x0000;
unsigned int g_BN_TestMode = 0x0000;
kal_bool g_bat_init_flag = 0;
unsigned int g_call_state = CALL_IDLE;
kal_bool g_charging_full_reset_bat_meter = KAL_FALSE;
int g_platform_boot_mode = 0;
struct timespec g_bat_time_before_sleep;
int g_smartbook_update = 0;

#if defined(CONFIG_MTK_DUAL_INPUT_CHARGER_SUPPORT)
kal_bool g_vcdt_irq_delay_flag = 0;
#endif

kal_bool skip_battery_update = KAL_FALSE;

unsigned int g_batt_temp_status = TEMP_POS_NORMAL;


kal_bool battery_suspended = KAL_FALSE;
int charge_full = CHARGE_FULL_DESIGN;
/*
#ifdef MTK_ENABLE_AGING_ALGORITHM
extern unsigned int suspend_time;
#endif 
*/

#if defined(CUST_SYSTEM_OFF_VOLTAGE)
#define SYSTEM_OFF_VOLTAGE CUST_SYSTEM_OFF_VOLTAGE
#endif
#define POST_99TO100_CHARGING_TIME
#if defined(POST_99TO100_CHARGING_TIME)
#define POST_99_CHARGING_TIME (15*60)
#endif

#if !defined(CONFIG_POWER_EXT)
static unsigned int V_0Percent_Tracking = V_0PERCENT_TRACKING;
#endif

struct battery_custom_data batt_cust_data;

/*
 * Integrate with NVRAM
 */
#define ADC_CALI_DEVNAME "MT_pmic_adc_cali"
#define TEST_ADC_CALI_PRINT _IO('k', 0)
#define SET_ADC_CALI_Slop _IOW('k', 1, int)
#define SET_ADC_CALI_Offset _IOW('k', 2, int)
#define SET_ADC_CALI_Cal _IOW('k', 3, int)
#define ADC_CHANNEL_READ _IOW('k', 4, int)
#define BAT_STATUS_READ _IOW('k', 5, int)
#define Set_Charger_Current _IOW('k', 6, int)
/* add for meta tool----------------------------------------- */
#define Get_META_BAT_VOL _IOW('k', 10, int)
#define Get_META_BAT_SOC _IOW('k', 11, int)
/* add for meta tool----------------------------------------- */

static struct class *adc_cali_class;
static int adc_cali_major;
static dev_t adc_cali_devno;
static struct cdev *adc_cali_cdev;

int adc_cali_slop[14] = { 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000 };

int adc_cali_offset[14] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
int adc_cali_cal[1] = { 0 };
int battery_in_data[1] = { 0 };
int battery_out_data[1] = { 0 };
int charging_level_data[1] = { 0 };

kal_bool g_ADC_Cali = KAL_FALSE;
kal_bool g_ftm_battery_flag = KAL_FALSE;
#if !defined(CONFIG_POWER_EXT)
static int g_wireless_state;
#endif

/*
 *  Thread related
 */
#define BAT_MS_TO_NS(x) (x * 1000 * 1000)
static kal_bool bat_thread_timeout = KAL_FALSE;
static kal_bool chr_wake_up_bat = KAL_FALSE;	/* charger in/out to wake up battery thread */
static kal_bool fg_wake_up_bat = KAL_FALSE;
static kal_bool bat_meter_timeout = KAL_FALSE;
static DEFINE_MUTEX(bat_mutex);
static DEFINE_MUTEX(charger_type_mutex);
static DECLARE_WAIT_QUEUE_HEAD(bat_thread_wq);
static struct hrtimer charger_hv_detect_timer;
static struct task_struct *charger_hv_detect_thread;
static kal_bool charger_hv_detect_flag = KAL_FALSE;
static DECLARE_WAIT_QUEUE_HEAD(charger_hv_detect_waiter);
static struct hrtimer battery_kthread_timer;
kal_bool g_battery_soc_ready = KAL_FALSE;
/*extern BOOL bat_spm_timeout;
extern unsigned int _g_bat_sleep_total_time;*/

/*
 * FOR ADB CMD
 */
/* Dual battery */
int g_status_smb = POWER_SUPPLY_STATUS_NOT_CHARGING;
int g_capacity_smb = 50;
int g_present_smb = 0;
/* ADB charging CMD */
static int cmd_discharging = -1;
static int adjust_power = -1;
static int suspend_discharging = -1;

#if !defined(CONFIG_POWER_EXT)
static int is_uisoc_ever_100 = KAL_FALSE;
#endif
#define HQ_CHARGER_FOR_HUAWEI   //add current test start
#if defined(HQ_CHARGER_FOR_HUAWEI)
static kal_bool g_charger_should_not_open = KAL_FALSE; /* for runtime & current test */
#define HQ_Get_BAT_VOL _IOW('k',7,int)
#define BAT_THREAD_CTRL _IOW('k',9,int)
#endif   //end
/* ////////////////////////////////////////////////////////////////////////////// */
/* FOR ANDROID BATTERY SERVICE */
/* ////////////////////////////////////////////////////////////////////////////// */

struct wireless_data {
	struct power_supply psy;
	int WIRELESS_ONLINE;
};

struct ac_data {
	struct power_supply psy;
	int AC_ONLINE;
};

struct usb_data {
	struct power_supply psy;
	int USB_ONLINE;
};

struct battery_data {
	struct power_supply psy;
	int BAT_STATUS;
	int BAT_HEALTH;
	int BAT_PRESENT;
	int BAT_TECHNOLOGY;
	int BAT_CAPACITY;
	/* Add for Battery Service */
	int BAT_batt_vol;
	int BAT_batt_temp;
	/* Add for EM */
	int BAT_TemperatureR;
	int BAT_TempBattVoltage;
	int BAT_InstatVolt;
	int BAT_BatteryAverageCurrent;
	int BAT_BatteryInstantCurrent;
	int BAT_BatterySenseVoltage;
	int BAT_ISenseVoltage;
	int BAT_ChargerVoltage;
	/* Dual battery */
	int status_smb;
	int capacity_smb;
	int present_smb;
	int adjust_power;
	int capacity_fcc;  //wangyang add CPM2016092700202
	int capacity_rm; 
};

static enum power_supply_property wireless_props[] = {
	POWER_SUPPLY_PROP_ONLINE,
};

static enum power_supply_property ac_props[] = {
	POWER_SUPPLY_PROP_ONLINE,
};

static enum power_supply_property usb_props[] = {
	POWER_SUPPLY_PROP_ONLINE,
};

static enum power_supply_property battery_props[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_TECHNOLOGY,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	/* Add for Battery Service */
	POWER_SUPPLY_PROP_batt_vol,
	POWER_SUPPLY_PROP_batt_temp,
	/* Add for EM */
	POWER_SUPPLY_PROP_TemperatureR,
	POWER_SUPPLY_PROP_TempBattVoltage,
	POWER_SUPPLY_PROP_InstatVolt,
	POWER_SUPPLY_PROP_BatteryAverageCurrent,
	POWER_SUPPLY_PROP_BatteryInstantCurrent,//wcl add 12 8
	POWER_SUPPLY_PROP_BatterySenseVoltage,
	POWER_SUPPLY_PROP_ISenseVoltage,
	POWER_SUPPLY_PROP_ChargerVoltage,
	/* Dual battery */
	POWER_SUPPLY_PROP_status_smb,
	POWER_SUPPLY_PROP_capacity_smb,
	POWER_SUPPLY_PROP_present_smb,
	/* ADB CMD Discharging */
	POWER_SUPPLY_PROP_adjust_power,
	POWER_SUPPLY_PROP_capacity_fcc,//wangyang add CPM2016092700202
	POWER_SUPPLY_PROP_capacity_rm, 
};

/*void check_battery_exist(void);*/
void charging_suspend_enable(void)
{
	unsigned int charging_enable = true;

	suspend_discharging = 0;
	battery_charging_control(CHARGING_CMD_ENABLE, &charging_enable);
}

void charging_suspend_disable(void)
{
	unsigned int charging_enable = false;

	suspend_discharging = 1;
	battery_charging_control(CHARGING_CMD_ENABLE, &charging_enable);
}

int read_tbat_value(void)
{
	return BMT_status.temperature;
}

int get_charger_detect_status(void)
{
	kal_bool chr_status;

	battery_charging_control(CHARGING_CMD_GET_CHARGER_DET_STATUS, &chr_status);
	return chr_status;
}

#if defined(CONFIG_MTK_POWER_EXT_DETECT)
kal_bool bat_is_ext_power(void)
{
	kal_bool pwr_src = 0;

	battery_charging_control(CHARGING_CMD_GET_POWER_SOURCE, &pwr_src);
	battery_log(BAT_LOG_FULL, "[BAT_IS_EXT_POWER] is_ext_power = %d\n", pwr_src);
	return pwr_src;
}
#endif
/* ///////////////////////////////////////////////////////////////////////////////////////// */
/* // PMIC PCHR Related APIs */
/* ///////////////////////////////////////////////////////////////////////////////////////// */
bool __attribute__((weak)) mt_usb_is_device(void)
{
	return 1;
}

kal_bool upmu_is_chr_det(void)
{
#if !defined(CONFIG_POWER_EXT)
	unsigned int tmp32;
#endif

	if (battery_charging_control == NULL)
		battery_charging_control = chr_control_interface;

#if defined(CONFIG_POWER_EXT)
	/* return KAL_TRUE; */
	return get_charger_detect_status();
#else
	if (suspend_discharging == 1)
		return KAL_FALSE;

	tmp32 = get_charger_detect_status();

#ifdef CONFIG_MTK_POWER_EXT_DETECT
	if (KAL_TRUE == bat_is_ext_power())
		return tmp32;
#endif

	if (tmp32 == 0)
		return KAL_FALSE;
	/*else {*/
#if !defined(CONFIG_MTK_DUAL_INPUT_CHARGER_SUPPORT)
		if (mt_usb_is_device()) {
			battery_log(BAT_LOG_FULL,
				    "[upmu_is_chr_det] Charger exist and USB is not host\n");

			return KAL_TRUE;
		} /*else {*/
			battery_log(BAT_LOG_CRTI,
				    "[upmu_is_chr_det] Charger exist but USB is host\n");

			return KAL_FALSE;
		/*}*/
#else
		return KAL_TRUE;
#endif
	/*}*/
#endif
}
EXPORT_SYMBOL(upmu_is_chr_det);


void wake_up_bat(void)
{
	battery_log(BAT_LOG_CRTI, "[BATTERY] wake_up_bat. \r\n");

	chr_wake_up_bat = KAL_TRUE;
	bat_thread_timeout = KAL_TRUE;
#ifdef MTK_ENABLE_AGING_ALGORITHM
	suspend_time = 0;
#endif
	battery_meter_reset_sleep_time();

	wake_up(&bat_thread_wq);
}
EXPORT_SYMBOL(wake_up_bat);


#ifdef FG_BAT_INT
void wake_up_bat2(void)
{
	battery_log(BAT_LOG_CRTI, "[BATTERY] wake_up_bat2. \r\n");

	wake_lock(&battery_fg_lock);
	fg_wake_up_bat = KAL_TRUE;
	bat_thread_timeout = KAL_TRUE;
#ifdef MTK_ENABLE_AGING_ALGORITHM
	suspend_time = 0;
#endif
	battery_meter_reset_sleep_time();
	wake_up(&bat_thread_wq);
}
EXPORT_SYMBOL(wake_up_bat2);
#endif				/* #ifdef FG_BAT_INT */

void wake_up_bat3(void)
{
	battery_log(BAT_LOG_CRTI, "[BATTERY] wake_up_bat3. \r\n");

	bat_thread_timeout = KAL_TRUE;
#ifdef MTK_ENABLE_AGING_ALGORITHM
	suspend_time = 0;
#endif
	battery_meter_reset_sleep_time();
	wake_up(&bat_thread_wq);
}
EXPORT_SYMBOL(wake_up_bat3);




static ssize_t bat_log_write(struct file *filp, const char __user *buff, size_t len, loff_t *data)
{
	if (copy_from_user(&proc_bat_data, buff, len)) {
		battery_log(BAT_LOG_FULL, "bat_log_write error.\n");
		return -EFAULT;
	}

	if (proc_bat_data[0] == '1') {
		battery_log(BAT_LOG_CRTI, "enable battery driver log system\n");
		Enable_BATDRV_LOG = 1;
	} else if (proc_bat_data[0] == '2') {
		battery_log(BAT_LOG_CRTI, "enable battery driver log system:2\n");
		Enable_BATDRV_LOG = 2;
	} else {
		battery_log(BAT_LOG_CRTI, "Disable battery driver log system\n");
		Enable_BATDRV_LOG = 0;
	}

	return len;
}

static const struct file_operations bat_proc_fops = {
	.write = bat_log_write,
};

int init_proc_log(void)
{
	int ret = 0;

#if 1
	proc_create("batdrv_log", 0644, NULL, &bat_proc_fops);
	battery_log(BAT_LOG_CRTI, "proc_create bat_proc_fops\n");
#else
	proc_entry = create_proc_entry("batdrv_log", 0644, NULL);

	if (proc_entry == NULL) {
		ret = -ENOMEM;
		battery_log(BAT_LOG_FULL, "init_proc_log: Couldn't create proc entry\n");
	} else {
		proc_entry->write_proc = bat_log_write;
		battery_log(BAT_LOG_CRTI, "init_proc_log loaded.\n");
	}
#endif

	return ret;
}


static int wireless_get_property(struct power_supply *psy,
				 enum power_supply_property psp, union power_supply_propval *val)
{
	int ret = 0;
	struct wireless_data *data = container_of(psy, struct wireless_data, psy);

	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
		val->intval = data->WIRELESS_ONLINE;
		break;
	default:
		ret = -EINVAL;
		break;
	}
	return ret;
}

static int ac_get_property(struct power_supply *psy,
			   enum power_supply_property psp, union power_supply_propval *val)
{
	int ret = 0;
	struct ac_data *data = container_of(psy, struct ac_data, psy);

	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
		val->intval = data->AC_ONLINE;
		break;
	default:
		ret = -EINVAL;
		break;
	}
	return ret;
}

static int usb_get_property(struct power_supply *psy,
			    enum power_supply_property psp, union power_supply_propval *val)
{
	int ret = 0;
	struct usb_data *data = container_of(psy, struct usb_data, psy);

	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
#if defined(CONFIG_POWER_EXT)
		/* #if 0 */
		data->USB_ONLINE = 1;
		val->intval = data->USB_ONLINE;
#else
#if defined(CONFIG_MTK_POWER_EXT_DETECT)
		if (KAL_TRUE == bat_is_ext_power())
			data->USB_ONLINE = 1;
#endif
		val->intval = data->USB_ONLINE;
#endif
		break;
	default:
		ret = -EINVAL;
		break;
	}
	return ret;
}

static int battery_get_property(struct power_supply *psy,
				enum power_supply_property psp, union power_supply_propval *val)
{
	int ret = 0;
	struct battery_data *data = container_of(psy, struct battery_data, psy);

	switch (psp) {
	case POWER_SUPPLY_PROP_STATUS:
		val->intval = data->BAT_STATUS;
		break;
	case POWER_SUPPLY_PROP_HEALTH:
		val->intval = data->BAT_HEALTH;
		break;
	case POWER_SUPPLY_PROP_PRESENT:
		val->intval = data->BAT_PRESENT;
		break;
	case POWER_SUPPLY_PROP_TECHNOLOGY:
		val->intval = data->BAT_TECHNOLOGY;
		break;
	case POWER_SUPPLY_PROP_CAPACITY:
		//zcy_add for runin battery test start
            	if(g_runin_battery_test_flag)
            	{
                	battery_xlog_printk(BAT_LOG_CRTI, "[BATTERY] battery_get_property Battery SOC:%d!!\n\r",BMT_status.SOC);  
                	val->intval = BMT_status.SOC;
            	}
               	else
         //add end
		val->intval = data->BAT_CAPACITY;
		break;
	case POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN:
		val->intval = charge_full;
		break;
	case POWER_SUPPLY_PROP_CURRENT_NOW:
		if(gFG_Is_Charging)
		{
			val->intval = (0-battery_meter_get_battery_current())*100;
		}else{
			val->intval = battery_meter_get_battery_current()*100;
		}
		break;
	case POWER_SUPPLY_PROP_batt_vol:
		val->intval = data->BAT_batt_vol;
		break;
	case POWER_SUPPLY_PROP_batt_temp:
		val->intval = data->BAT_batt_temp;
		break;
	case POWER_SUPPLY_PROP_TemperatureR:
		val->intval = data->BAT_TemperatureR;
		break;
	case POWER_SUPPLY_PROP_TempBattVoltage:
		val->intval = data->BAT_TempBattVoltage;
		break;
	case POWER_SUPPLY_PROP_InstatVolt:
		val->intval = data->BAT_InstatVolt;
		break;
	case POWER_SUPPLY_PROP_BatteryAverageCurrent:
		val->intval = data->BAT_BatteryAverageCurrent;
		break;
	case POWER_SUPPLY_PROP_BatteryInstantCurrent:
#ifdef CONFIG_MTK_BQ24196_SUPPORT
		val->intval = battery_meter_get_battery_current()/10;
#else
		val->intval = data->BAT_BatteryInstantCurrent;
#endif
		break;
	case POWER_SUPPLY_PROP_BatterySenseVoltage:
		val->intval = data->BAT_BatterySenseVoltage;
		break;
	case POWER_SUPPLY_PROP_ISenseVoltage:
		val->intval = data->BAT_ISenseVoltage;
		break;
	case POWER_SUPPLY_PROP_ChargerVoltage:
		val->intval = data->BAT_ChargerVoltage;
		break;
		/* Dual battery */
	case POWER_SUPPLY_PROP_status_smb:
		val->intval = data->status_smb;
		break;
	case POWER_SUPPLY_PROP_capacity_smb:
		val->intval = data->capacity_smb;
		break;
	case POWER_SUPPLY_PROP_present_smb:
		val->intval = data->present_smb;
		break;
	case POWER_SUPPLY_PROP_adjust_power:
		val->intval = data->adjust_power;
		break;
	case POWER_SUPPLY_PROP_capacity_fcc:  //wangyang add CPM2016092700202 
		val->intval = data->capacity_fcc;
		break;
	case POWER_SUPPLY_PROP_capacity_rm:  //wangyang add CPM2016092700202 
		val->intval = data->capacity_rm;
		break;
	default:
		ret = -EINVAL;
		break;
	}

	return ret;
}

/* wireless_data initialization */
static struct wireless_data wireless_main = {
	.psy = {
		.name = "wireless",
		.type = POWER_SUPPLY_TYPE_WIRELESS,
		.properties = wireless_props,
		.num_properties = ARRAY_SIZE(wireless_props),
		.get_property = wireless_get_property,
		},
	.WIRELESS_ONLINE = 0,
};

/* ac_data initialization */
static struct ac_data ac_main = {
	.psy = {
		.name = "ac",
		.type = POWER_SUPPLY_TYPE_MAINS,
		.properties = ac_props,
		.num_properties = ARRAY_SIZE(ac_props),
		.get_property = ac_get_property,
		},
	.AC_ONLINE = 0,
};

/* usb_data initialization */
static struct usb_data usb_main = {
	.psy = {
		.name = "usb",
		.type = POWER_SUPPLY_TYPE_USB,
		.properties = usb_props,
		.num_properties = ARRAY_SIZE(usb_props),
		.get_property = usb_get_property,
		},
	.USB_ONLINE = 0,
};

/* battery_data initialization */
static struct battery_data battery_main = {
	.psy = {
		.name = "battery",
		.type = POWER_SUPPLY_TYPE_BATTERY,
		.properties = battery_props,
		.num_properties = ARRAY_SIZE(battery_props),
		.get_property = battery_get_property,
		},
/* CC: modify to have a full power supply status */
#if defined(CONFIG_POWER_EXT)
	.BAT_STATUS = POWER_SUPPLY_STATUS_FULL,
	.BAT_HEALTH = POWER_SUPPLY_HEALTH_GOOD,
	.BAT_PRESENT = 1,
	.BAT_TECHNOLOGY = POWER_SUPPLY_TECHNOLOGY_LION,
	.BAT_CAPACITY = 100,
	.BAT_batt_vol = 4200,
	.BAT_batt_temp = 22,
	/* Dual battery */
	.status_smb = POWER_SUPPLY_STATUS_NOT_CHARGING,
	.capacity_smb = 50,
	.capacity_fcc=1600, //wangyang add CPM2016092700202 s
	.capacity_rm=800, //wangyang add CPM2016092700202 s
	.present_smb = 0,
	/* ADB CMD discharging */
	.adjust_power = -1,
#else
	.BAT_STATUS = POWER_SUPPLY_STATUS_NOT_CHARGING,
	.BAT_HEALTH = POWER_SUPPLY_HEALTH_GOOD,
	.BAT_PRESENT = 1,
	.BAT_TECHNOLOGY = POWER_SUPPLY_TECHNOLOGY_LION,
#if defined(CONFIG_MTK_PUMP_EXPRESS_SUPPORT) || defined(CONFIG_MTK_PUMP_EXPRESS_PLUS_SUPPORT)
	.BAT_CAPACITY = -1,
#else
	.BAT_CAPACITY = 50,
#endif
	.BAT_batt_vol = 0,
	.BAT_batt_temp = 0,
	/* Dual battery */
	.status_smb = POWER_SUPPLY_STATUS_NOT_CHARGING,
	.capacity_smb = 50,
	.present_smb = 0,
	/* ADB CMD discharging */
	.adjust_power = -1,
	.capacity_fcc=0, //wangyang add CPM2016092700202 
	.capacity_rm=0, //wangyang add CPM2016092700202 s
#endif
};


#if !defined(CONFIG_POWER_EXT)
/* ///////////////////////////////////////////////////////////////////////////////////////// */
/* // Create File For EM : ADC_Charger_Voltage */
/* ///////////////////////////////////////////////////////////////////////////////////////// */
static ssize_t show_ADC_Charger_Voltage(struct device *dev, struct device_attribute *attr,
					char *buf)
{
	battery_log(BAT_LOG_CRTI, "[EM] show_ADC_Charger_Voltage : %d\n", BMT_status.charger_vol);
	return sprintf(buf, "%d\n", BMT_status.charger_vol);
}

static ssize_t store_ADC_Charger_Voltage(struct device *dev, struct device_attribute *attr,
					 const char *buf, size_t size)
{
	battery_log(BAT_LOG_CRTI, "[EM] Not Support Write Function\n");
	return size;
}

static DEVICE_ATTR(ADC_Charger_Voltage, 0664, show_ADC_Charger_Voltage, store_ADC_Charger_Voltage);

/* ///////////////////////////////////////////////////////////////////////////////////////// */
/* // Create File For EM : ADC_Channel_0_Slope */
/* ///////////////////////////////////////////////////////////////////////////////////////// */
static ssize_t show_ADC_Channel_0_Slope(struct device *dev, struct device_attribute *attr,
					char *buf)
{
	int ret_value = 1;

	ret_value = (*(adc_cali_slop + 0));
	battery_log(BAT_LOG_CRTI, "[EM] ADC_Channel_0_Slope : %d\n", ret_value);
	return sprintf(buf, "%u\n", ret_value);
}

static ssize_t store_ADC_Channel_0_Slope(struct device *dev, struct device_attribute *attr,
					 const char *buf, size_t size)
{
	battery_log(BAT_LOG_CRTI, "[EM] Not Support Write Function\n");
	return size;
}

static DEVICE_ATTR(ADC_Channel_0_Slope, 0664, show_ADC_Channel_0_Slope, store_ADC_Channel_0_Slope);

/* ///////////////////////////////////////////////////////////////////////////////////////// */
/* // Create File For EM : ADC_Channel_1_Slope */
/* ///////////////////////////////////////////////////////////////////////////////////////// */
static ssize_t show_ADC_Channel_1_Slope(struct device *dev, struct device_attribute *attr,
					char *buf)
{
	int ret_value = 1;

	ret_value = (*(adc_cali_slop + 1));
	battery_log(BAT_LOG_CRTI, "[EM] ADC_Channel_1_Slope : %d\n", ret_value);
	return sprintf(buf, "%u\n", ret_value);
}

static ssize_t store_ADC_Channel_1_Slope(struct device *dev, struct device_attribute *attr,
					 const char *buf, size_t size)
{
	battery_log(BAT_LOG_CRTI, "[EM] Not Support Write Function\n");
	return size;
}

static DEVICE_ATTR(ADC_Channel_1_Slope, 0664, show_ADC_Channel_1_Slope, store_ADC_Channel_1_Slope);

/* ///////////////////////////////////////////////////////////////////////////////////////// */
/* // Create File For EM : ADC_Channel_2_Slope */
/* ///////////////////////////////////////////////////////////////////////////////////////// */
static ssize_t show_ADC_Channel_2_Slope(struct device *dev, struct device_attribute *attr,
					char *buf)
{
	int ret_value = 1;

	ret_value = (*(adc_cali_slop + 2));
	battery_log(BAT_LOG_CRTI, "[EM] ADC_Channel_2_Slope : %d\n", ret_value);
	return sprintf(buf, "%u\n", ret_value);
}

static ssize_t store_ADC_Channel_2_Slope(struct device *dev, struct device_attribute *attr,
					 const char *buf, size_t size)
{
	battery_log(BAT_LOG_CRTI, "[EM] Not Support Write Function\n");
	return size;
}

static DEVICE_ATTR(ADC_Channel_2_Slope, 0664, show_ADC_Channel_2_Slope, store_ADC_Channel_2_Slope);

/* ///////////////////////////////////////////////////////////////////////////////////////// */
/* // Create File For EM : ADC_Channel_3_Slope */
/* ///////////////////////////////////////////////////////////////////////////////////////// */
static ssize_t show_ADC_Channel_3_Slope(struct device *dev, struct device_attribute *attr,
					char *buf)
{
	int ret_value = 1;

	ret_value = (*(adc_cali_slop + 3));
	battery_log(BAT_LOG_CRTI, "[EM] ADC_Channel_3_Slope : %d\n", ret_value);
	return sprintf(buf, "%u\n", ret_value);
}

static ssize_t store_ADC_Channel_3_Slope(struct device *dev, struct device_attribute *attr,
					 const char *buf, size_t size)
{
	battery_log(BAT_LOG_CRTI, "[EM] Not Support Write Function\n");
	return size;
}

static DEVICE_ATTR(ADC_Channel_3_Slope, 0664, show_ADC_Channel_3_Slope, store_ADC_Channel_3_Slope);

/* ///////////////////////////////////////////////////////////////////////////////////////// */
/* // Create File For EM : ADC_Channel_4_Slope */
/* ///////////////////////////////////////////////////////////////////////////////////////// */
static ssize_t show_ADC_Channel_4_Slope(struct device *dev, struct device_attribute *attr,
					char *buf)
{
	int ret_value = 1;

	ret_value = (*(adc_cali_slop + 4));
	battery_log(BAT_LOG_CRTI, "[EM] ADC_Channel_4_Slope : %d\n", ret_value);
	return sprintf(buf, "%u\n", ret_value);
}

static ssize_t store_ADC_Channel_4_Slope(struct device *dev, struct device_attribute *attr,
					 const char *buf, size_t size)
{
	battery_log(BAT_LOG_CRTI, "[EM] Not Support Write Function\n");
	return size;
}

static DEVICE_ATTR(ADC_Channel_4_Slope, 0664, show_ADC_Channel_4_Slope, store_ADC_Channel_4_Slope);

/* ///////////////////////////////////////////////////////////////////////////////////////// */
/* // Create File For EM : ADC_Channel_5_Slope */
/* ///////////////////////////////////////////////////////////////////////////////////////// */
static ssize_t show_ADC_Channel_5_Slope(struct device *dev, struct device_attribute *attr,
					char *buf)
{
	int ret_value = 1;

	ret_value = (*(adc_cali_slop + 5));
	battery_log(BAT_LOG_CRTI, "[EM] ADC_Channel_5_Slope : %d\n", ret_value);
	return sprintf(buf, "%u\n", ret_value);
}

static ssize_t store_ADC_Channel_5_Slope(struct device *dev, struct device_attribute *attr,
					 const char *buf, size_t size)
{
	battery_log(BAT_LOG_CRTI, "[EM] Not Support Write Function\n");
	return size;
}

static DEVICE_ATTR(ADC_Channel_5_Slope, 0664, show_ADC_Channel_5_Slope, store_ADC_Channel_5_Slope);

/* ///////////////////////////////////////////////////////////////////////////////////////// */
/* // Create File For EM : ADC_Channel_6_Slope */
/* ///////////////////////////////////////////////////////////////////////////////////////// */
static ssize_t show_ADC_Channel_6_Slope(struct device *dev, struct device_attribute *attr,
					char *buf)
{
	int ret_value = 1;

	ret_value = (*(adc_cali_slop + 6));
	battery_log(BAT_LOG_CRTI, "[EM] ADC_Channel_6_Slope : %d\n", ret_value);
	return sprintf(buf, "%u\n", ret_value);
}

static ssize_t store_ADC_Channel_6_Slope(struct device *dev, struct device_attribute *attr,
					 const char *buf, size_t size)
{
	battery_log(BAT_LOG_CRTI, "[EM] Not Support Write Function\n");
	return size;
}

static DEVICE_ATTR(ADC_Channel_6_Slope, 0664, show_ADC_Channel_6_Slope, store_ADC_Channel_6_Slope);

/* ///////////////////////////////////////////////////////////////////////////////////////// */
/* // Create File For EM : ADC_Channel_7_Slope */
/* ///////////////////////////////////////////////////////////////////////////////////////// */
static ssize_t show_ADC_Channel_7_Slope(struct device *dev, struct device_attribute *attr,
					char *buf)
{
	int ret_value = 1;

	ret_value = (*(adc_cali_slop + 7));
	battery_log(BAT_LOG_CRTI, "[EM] ADC_Channel_7_Slope : %d\n", ret_value);
	return sprintf(buf, "%u\n", ret_value);
}

static ssize_t store_ADC_Channel_7_Slope(struct device *dev, struct device_attribute *attr,
					 const char *buf, size_t size)
{
	battery_log(BAT_LOG_CRTI, "[EM] Not Support Write Function\n");
	return size;
}

static DEVICE_ATTR(ADC_Channel_7_Slope, 0664, show_ADC_Channel_7_Slope, store_ADC_Channel_7_Slope);

/* ///////////////////////////////////////////////////////////////////////////////////////// */
/* // Create File For EM : ADC_Channel_8_Slope */
/* ///////////////////////////////////////////////////////////////////////////////////////// */
static ssize_t show_ADC_Channel_8_Slope(struct device *dev, struct device_attribute *attr,
					char *buf)
{
	int ret_value = 1;

	ret_value = (*(adc_cali_slop + 8));
	battery_log(BAT_LOG_CRTI, "[EM] ADC_Channel_8_Slope : %d\n", ret_value);
	return sprintf(buf, "%u\n", ret_value);
}

static ssize_t store_ADC_Channel_8_Slope(struct device *dev, struct device_attribute *attr,
					 const char *buf, size_t size)
{
	battery_log(BAT_LOG_CRTI, "[EM] Not Support Write Function\n");
	return size;
}

static DEVICE_ATTR(ADC_Channel_8_Slope, 0664, show_ADC_Channel_8_Slope, store_ADC_Channel_8_Slope);

/* ///////////////////////////////////////////////////////////////////////////////////////// */
/* // Create File For EM : ADC_Channel_9_Slope */
/* ///////////////////////////////////////////////////////////////////////////////////////// */
static ssize_t show_ADC_Channel_9_Slope(struct device *dev, struct device_attribute *attr,
					char *buf)
{
	int ret_value = 1;

	ret_value = (*(adc_cali_slop + 9));
	battery_log(BAT_LOG_CRTI, "[EM] ADC_Channel_9_Slope : %d\n", ret_value);
	return sprintf(buf, "%u\n", ret_value);
}

static ssize_t store_ADC_Channel_9_Slope(struct device *dev, struct device_attribute *attr,
					 const char *buf, size_t size)
{
	battery_log(BAT_LOG_CRTI, "[EM] Not Support Write Function\n");
	return size;
}

static DEVICE_ATTR(ADC_Channel_9_Slope, 0664, show_ADC_Channel_9_Slope, store_ADC_Channel_9_Slope);

/* ///////////////////////////////////////////////////////////////////////////////////////// */
/* // Create File For EM : ADC_Channel_10_Slope */
/* ///////////////////////////////////////////////////////////////////////////////////////// */
static ssize_t show_ADC_Channel_10_Slope(struct device *dev, struct device_attribute *attr,
					 char *buf)
{
	int ret_value = 1;

	ret_value = (*(adc_cali_slop + 10));
	battery_log(BAT_LOG_CRTI, "[EM] ADC_Channel_10_Slope : %d\n", ret_value);
	return sprintf(buf, "%u\n", ret_value);
}

static ssize_t store_ADC_Channel_10_Slope(struct device *dev, struct device_attribute *attr,
					  const char *buf, size_t size)
{
	battery_log(BAT_LOG_CRTI, "[EM] Not Support Write Function\n");
	return size;
}

static DEVICE_ATTR(ADC_Channel_10_Slope, 0664, show_ADC_Channel_10_Slope,
		   store_ADC_Channel_10_Slope);

/* ///////////////////////////////////////////////////////////////////////////////////////// */
/* // Create File For EM : ADC_Channel_11_Slope */
/* ///////////////////////////////////////////////////////////////////////////////////////// */
static ssize_t show_ADC_Channel_11_Slope(struct device *dev, struct device_attribute *attr,
					 char *buf)
{
	int ret_value = 1;

	ret_value = (*(adc_cali_slop + 11));
	battery_log(BAT_LOG_CRTI, "[EM] ADC_Channel_11_Slope : %d\n", ret_value);
	return sprintf(buf, "%u\n", ret_value);
}

static ssize_t store_ADC_Channel_11_Slope(struct device *dev, struct device_attribute *attr,
					  const char *buf, size_t size)
{
	battery_log(BAT_LOG_CRTI, "[EM] Not Support Write Function\n");
	return size;
}

static DEVICE_ATTR(ADC_Channel_11_Slope, 0664, show_ADC_Channel_11_Slope,
		   store_ADC_Channel_11_Slope);

/* ///////////////////////////////////////////////////////////////////////////////////////// */
/* // Create File For EM : ADC_Channel_12_Slope */
/* ///////////////////////////////////////////////////////////////////////////////////////// */
static ssize_t show_ADC_Channel_12_Slope(struct device *dev, struct device_attribute *attr,
					 char *buf)
{
	int ret_value = 1;

	ret_value = (*(adc_cali_slop + 12));
	battery_log(BAT_LOG_CRTI, "[EM] ADC_Channel_12_Slope : %d\n", ret_value);
	return sprintf(buf, "%u\n", ret_value);
}

static ssize_t store_ADC_Channel_12_Slope(struct device *dev, struct device_attribute *attr,
					  const char *buf, size_t size)
{
	battery_log(BAT_LOG_CRTI, "[EM] Not Support Write Function\n");
	return size;
}

static DEVICE_ATTR(ADC_Channel_12_Slope, 0664, show_ADC_Channel_12_Slope,
		   store_ADC_Channel_12_Slope);

/* ///////////////////////////////////////////////////////////////////////////////////////// */
/* // Create File For EM : ADC_Channel_13_Slope */
/* ///////////////////////////////////////////////////////////////////////////////////////// */
static ssize_t show_ADC_Channel_13_Slope(struct device *dev, struct device_attribute *attr,
					 char *buf)
{
	int ret_value = 1;

	ret_value = (*(adc_cali_slop + 13));
	battery_log(BAT_LOG_CRTI, "[EM] ADC_Channel_13_Slope : %d\n", ret_value);
	return sprintf(buf, "%u\n", ret_value);
}

static ssize_t store_ADC_Channel_13_Slope(struct device *dev, struct device_attribute *attr,
					  const char *buf, size_t size)
{
	battery_log(BAT_LOG_CRTI, "[EM] Not Support Write Function\n");
	return size;
}

static DEVICE_ATTR(ADC_Channel_13_Slope, 0664, show_ADC_Channel_13_Slope,
		   store_ADC_Channel_13_Slope);


/* ///////////////////////////////////////////////////////////////////////////////////////// */
/* // Create File For EM : ADC_Channel_0_Offset */
/* ///////////////////////////////////////////////////////////////////////////////////////// */
static ssize_t show_ADC_Channel_0_Offset(struct device *dev, struct device_attribute *attr,
					 char *buf)
{
	int ret_value = 1;

	ret_value = (*(adc_cali_offset + 0));
	battery_log(BAT_LOG_CRTI, "[EM] ADC_Channel_0_Offset : %d\n", ret_value);
	return sprintf(buf, "%u\n", ret_value);
}

static ssize_t store_ADC_Channel_0_Offset(struct device *dev, struct device_attribute *attr,
					  const char *buf, size_t size)
{
	battery_log(BAT_LOG_CRTI, "[EM] Not Support Write Function\n");
	return size;
}

static DEVICE_ATTR(ADC_Channel_0_Offset, 0664, show_ADC_Channel_0_Offset,
		   store_ADC_Channel_0_Offset);

/* ///////////////////////////////////////////////////////////////////////////////////////// */
/* // Create File For EM : ADC_Channel_1_Offset */
/* ///////////////////////////////////////////////////////////////////////////////////////// */
static ssize_t show_ADC_Channel_1_Offset(struct device *dev, struct device_attribute *attr,
					 char *buf)
{
	int ret_value = 1;

	ret_value = (*(adc_cali_offset + 1));
	battery_log(BAT_LOG_CRTI, "[EM] ADC_Channel_1_Offset : %d\n", ret_value);
	return sprintf(buf, "%u\n", ret_value);
}

static ssize_t store_ADC_Channel_1_Offset(struct device *dev, struct device_attribute *attr,
					  const char *buf, size_t size)
{
	battery_log(BAT_LOG_CRTI, "[EM] Not Support Write Function\n");
	return size;
}

static DEVICE_ATTR(ADC_Channel_1_Offset, 0664, show_ADC_Channel_1_Offset,
		   store_ADC_Channel_1_Offset);

/* ///////////////////////////////////////////////////////////////////////////////////////// */
/* // Create File For EM : ADC_Channel_2_Offset */
/* ///////////////////////////////////////////////////////////////////////////////////////// */
static ssize_t show_ADC_Channel_2_Offset(struct device *dev, struct device_attribute *attr,
					 char *buf)
{
	int ret_value = 1;

	ret_value = (*(adc_cali_offset + 2));
	battery_log(BAT_LOG_CRTI, "[EM] ADC_Channel_2_Offset : %d\n", ret_value);
	return sprintf(buf, "%u\n", ret_value);
}

static ssize_t store_ADC_Channel_2_Offset(struct device *dev, struct device_attribute *attr,
					  const char *buf, size_t size)
{
	battery_log(BAT_LOG_CRTI, "[EM] Not Support Write Function\n");
	return size;
}

static DEVICE_ATTR(ADC_Channel_2_Offset, 0664, show_ADC_Channel_2_Offset,
		   store_ADC_Channel_2_Offset);

/* ///////////////////////////////////////////////////////////////////////////////////////// */
/* // Create File For EM : ADC_Channel_3_Offset */
/* ///////////////////////////////////////////////////////////////////////////////////////// */
static ssize_t show_ADC_Channel_3_Offset(struct device *dev, struct device_attribute *attr,
					 char *buf)
{
	int ret_value = 1;

	ret_value = (*(adc_cali_offset + 3));
	battery_log(BAT_LOG_CRTI, "[EM] ADC_Channel_3_Offset : %d\n", ret_value);
	return sprintf(buf, "%u\n", ret_value);
}

static ssize_t store_ADC_Channel_3_Offset(struct device *dev, struct device_attribute *attr,
					  const char *buf, size_t size)
{
	battery_log(BAT_LOG_CRTI, "[EM] Not Support Write Function\n");
	return size;
}

static DEVICE_ATTR(ADC_Channel_3_Offset, 0664, show_ADC_Channel_3_Offset,
		   store_ADC_Channel_3_Offset);

/* ///////////////////////////////////////////////////////////////////////////////////////// */
/* // Create File For EM : ADC_Channel_4_Offset */
/* ///////////////////////////////////////////////////////////////////////////////////////// */
static ssize_t show_ADC_Channel_4_Offset(struct device *dev, struct device_attribute *attr,
					 char *buf)
{
	int ret_value = 1;

	ret_value = (*(adc_cali_offset + 4));
	battery_log(BAT_LOG_CRTI, "[EM] ADC_Channel_4_Offset : %d\n", ret_value);
	return sprintf(buf, "%u\n", ret_value);
}

static ssize_t store_ADC_Channel_4_Offset(struct device *dev, struct device_attribute *attr,
					  const char *buf, size_t size)
{
	battery_log(BAT_LOG_CRTI, "[EM] Not Support Write Function\n");
	return size;
}

static DEVICE_ATTR(ADC_Channel_4_Offset, 0664, show_ADC_Channel_4_Offset,
		   store_ADC_Channel_4_Offset);

/* ///////////////////////////////////////////////////////////////////////////////////////// */
/* // Create File For EM : ADC_Channel_5_Offset */
/* ///////////////////////////////////////////////////////////////////////////////////////// */
static ssize_t show_ADC_Channel_5_Offset(struct device *dev, struct device_attribute *attr,
					 char *buf)
{
	int ret_value = 1;

	ret_value = (*(adc_cali_offset + 5));
	battery_log(BAT_LOG_CRTI, "[EM] ADC_Channel_5_Offset : %d\n", ret_value);
	return sprintf(buf, "%u\n", ret_value);
}

static ssize_t store_ADC_Channel_5_Offset(struct device *dev, struct device_attribute *attr,
					  const char *buf, size_t size)
{
	battery_log(BAT_LOG_CRTI, "[EM] Not Support Write Function\n");
	return size;
}

static DEVICE_ATTR(ADC_Channel_5_Offset, 0664, show_ADC_Channel_5_Offset,
		   store_ADC_Channel_5_Offset);

/* ///////////////////////////////////////////////////////////////////////////////////////// */
/* // Create File For EM : ADC_Channel_6_Offset */
/* ///////////////////////////////////////////////////////////////////////////////////////// */
static ssize_t show_ADC_Channel_6_Offset(struct device *dev, struct device_attribute *attr,
					 char *buf)
{
	int ret_value = 1;

	ret_value = (*(adc_cali_offset + 6));
	battery_log(BAT_LOG_CRTI, "[EM] ADC_Channel_6_Offset : %d\n", ret_value);
	return sprintf(buf, "%u\n", ret_value);
}

static ssize_t store_ADC_Channel_6_Offset(struct device *dev, struct device_attribute *attr,
					  const char *buf, size_t size)
{
	battery_log(BAT_LOG_CRTI, "[EM] Not Support Write Function\n");
	return size;
}

static DEVICE_ATTR(ADC_Channel_6_Offset, 0664, show_ADC_Channel_6_Offset,
		   store_ADC_Channel_6_Offset);

/* ///////////////////////////////////////////////////////////////////////////////////////// */
/* // Create File For EM : ADC_Channel_7_Offset */
/* ///////////////////////////////////////////////////////////////////////////////////////// */
static ssize_t show_ADC_Channel_7_Offset(struct device *dev, struct device_attribute *attr,
					 char *buf)
{
	int ret_value = 1;

	ret_value = (*(adc_cali_offset + 7));
	battery_log(BAT_LOG_CRTI, "[EM] ADC_Channel_7_Offset : %d\n", ret_value);
	return sprintf(buf, "%u\n", ret_value);
}

static ssize_t store_ADC_Channel_7_Offset(struct device *dev, struct device_attribute *attr,
					  const char *buf, size_t size)
{
	battery_log(BAT_LOG_CRTI, "[EM] Not Support Write Function\n");
	return size;
}

static DEVICE_ATTR(ADC_Channel_7_Offset, 0664, show_ADC_Channel_7_Offset,
		   store_ADC_Channel_7_Offset);

/* ///////////////////////////////////////////////////////////////////////////////////////// */
/* // Create File For EM : ADC_Channel_8_Offset */
/* ///////////////////////////////////////////////////////////////////////////////////////// */
static ssize_t show_ADC_Channel_8_Offset(struct device *dev, struct device_attribute *attr,
					 char *buf)
{
	int ret_value = 1;

	ret_value = (*(adc_cali_offset + 8));
	battery_log(BAT_LOG_CRTI, "[EM] ADC_Channel_8_Offset : %d\n", ret_value);
	return sprintf(buf, "%u\n", ret_value);
}

static ssize_t store_ADC_Channel_8_Offset(struct device *dev, struct device_attribute *attr,
					  const char *buf, size_t size)
{
	battery_log(BAT_LOG_CRTI, "[EM] Not Support Write Function\n");
	return size;
}

static DEVICE_ATTR(ADC_Channel_8_Offset, 0664, show_ADC_Channel_8_Offset,
		   store_ADC_Channel_8_Offset);

/* ///////////////////////////////////////////////////////////////////////////////////////// */
/* // Create File For EM : ADC_Channel_9_Offset */
/* ///////////////////////////////////////////////////////////////////////////////////////// */
static ssize_t show_ADC_Channel_9_Offset(struct device *dev, struct device_attribute *attr,
					 char *buf)
{
	int ret_value = 1;

	ret_value = (*(adc_cali_offset + 9));
	battery_log(BAT_LOG_CRTI, "[EM] ADC_Channel_9_Offset : %d\n", ret_value);
	return sprintf(buf, "%u\n", ret_value);
}

static ssize_t store_ADC_Channel_9_Offset(struct device *dev, struct device_attribute *attr,
					  const char *buf, size_t size)
{
	battery_log(BAT_LOG_CRTI, "[EM] Not Support Write Function\n");
	return size;
}

static DEVICE_ATTR(ADC_Channel_9_Offset, 0664, show_ADC_Channel_9_Offset,
		   store_ADC_Channel_9_Offset);

/* ///////////////////////////////////////////////////////////////////////////////////////// */
/* // Create File For EM : ADC_Channel_10_Offset */
/* ///////////////////////////////////////////////////////////////////////////////////////// */
static ssize_t show_ADC_Channel_10_Offset(struct device *dev, struct device_attribute *attr,
					  char *buf)
{
	int ret_value = 1;

	ret_value = (*(adc_cali_offset + 10));
	battery_log(BAT_LOG_CRTI, "[EM] ADC_Channel_10_Offset : %d\n", ret_value);
	return sprintf(buf, "%u\n", ret_value);
}

static ssize_t store_ADC_Channel_10_Offset(struct device *dev, struct device_attribute *attr,
					   const char *buf, size_t size)
{
	battery_log(BAT_LOG_CRTI, "[EM] Not Support Write Function\n");
	return size;
}

static DEVICE_ATTR(ADC_Channel_10_Offset, 0664, show_ADC_Channel_10_Offset,
		   store_ADC_Channel_10_Offset);

/* ///////////////////////////////////////////////////////////////////////////////////////// */
/* // Create File For EM : ADC_Channel_11_Offset */
/* ///////////////////////////////////////////////////////////////////////////////////////// */
static ssize_t show_ADC_Channel_11_Offset(struct device *dev, struct device_attribute *attr,
					  char *buf)
{
	int ret_value = 1;

	ret_value = (*(adc_cali_offset + 11));
	battery_log(BAT_LOG_CRTI, "[EM] ADC_Channel_11_Offset : %d\n", ret_value);
	return sprintf(buf, "%u\n", ret_value);
}

static ssize_t store_ADC_Channel_11_Offset(struct device *dev, struct device_attribute *attr,
					   const char *buf, size_t size)
{
	battery_log(BAT_LOG_CRTI, "[EM] Not Support Write Function\n");
	return size;
}

static DEVICE_ATTR(ADC_Channel_11_Offset, 0664, show_ADC_Channel_11_Offset,
		   store_ADC_Channel_11_Offset);

/* ///////////////////////////////////////////////////////////////////////////////////////// */
/* // Create File For EM : ADC_Channel_12_Offset */
/* ///////////////////////////////////////////////////////////////////////////////////////// */
static ssize_t show_ADC_Channel_12_Offset(struct device *dev, struct device_attribute *attr,
					  char *buf)
{
	int ret_value = 1;

	ret_value = (*(adc_cali_offset + 12));
	battery_log(BAT_LOG_CRTI, "[EM] ADC_Channel_12_Offset : %d\n", ret_value);
	return sprintf(buf, "%u\n", ret_value);
}

static ssize_t store_ADC_Channel_12_Offset(struct device *dev, struct device_attribute *attr,
					   const char *buf, size_t size)
{
	battery_log(BAT_LOG_CRTI, "[EM] Not Support Write Function\n");
	return size;
}

static DEVICE_ATTR(ADC_Channel_12_Offset, 0664, show_ADC_Channel_12_Offset,
		   store_ADC_Channel_12_Offset);

/* ///////////////////////////////////////////////////////////////////////////////////////// */
/* // Create File For EM : ADC_Channel_13_Offset */
/* ///////////////////////////////////////////////////////////////////////////////////////// */
static ssize_t show_ADC_Channel_13_Offset(struct device *dev, struct device_attribute *attr,
					  char *buf)
{
	int ret_value = 1;

	ret_value = (*(adc_cali_offset + 13));
	battery_log(BAT_LOG_CRTI, "[EM] ADC_Channel_13_Offset : %d\n", ret_value);
	return sprintf(buf, "%u\n", ret_value);
}

static ssize_t store_ADC_Channel_13_Offset(struct device *dev, struct device_attribute *attr,
					   const char *buf, size_t size)
{
	battery_log(BAT_LOG_CRTI, "[EM] Not Support Write Function\n");
	return size;
}

static DEVICE_ATTR(ADC_Channel_13_Offset, 0664, show_ADC_Channel_13_Offset,
		   store_ADC_Channel_13_Offset);

/* ///////////////////////////////////////////////////////////////////////////////////////// */
/* // Create File For EM : ADC_Channel_Is_Calibration */
/* ///////////////////////////////////////////////////////////////////////////////////////// */
static ssize_t show_ADC_Channel_Is_Calibration(struct device *dev, struct device_attribute *attr,
					       char *buf)
{
	int ret_value = 2;

	ret_value = g_ADC_Cali;
	battery_log(BAT_LOG_CRTI, "[EM] ADC_Channel_Is_Calibration : %d\n", ret_value);
	return sprintf(buf, "%u\n", ret_value);
}

static ssize_t store_ADC_Channel_Is_Calibration(struct device *dev, struct device_attribute *attr,
						const char *buf, size_t size)
{
	battery_log(BAT_LOG_CRTI, "[EM] Not Support Write Function\n");
	return size;
}

static DEVICE_ATTR(ADC_Channel_Is_Calibration, 0664, show_ADC_Channel_Is_Calibration,
		   store_ADC_Channel_Is_Calibration);

/* ///////////////////////////////////////////////////////////////////////////////////////// */
/* // Create File For EM : Power_On_Voltage */
/* ///////////////////////////////////////////////////////////////////////////////////////// */
static ssize_t show_Power_On_Voltage(struct device *dev, struct device_attribute *attr, char *buf)
{
	int ret_value = 1;

	ret_value = 3400;
	battery_log(BAT_LOG_CRTI, "[EM] Power_On_Voltage : %d\n", ret_value);
	return sprintf(buf, "%u\n", ret_value);
}

static ssize_t store_Power_On_Voltage(struct device *dev, struct device_attribute *attr,
				      const char *buf, size_t size)
{
	battery_log(BAT_LOG_CRTI, "[EM] Not Support Write Function\n");
	return size;
}

static DEVICE_ATTR(Power_On_Voltage, 0664, show_Power_On_Voltage, store_Power_On_Voltage);

/* ///////////////////////////////////////////////////////////////////////////////////////// */
/* // Create File For EM : Power_Off_Voltage */
/* ///////////////////////////////////////////////////////////////////////////////////////// */
static ssize_t show_Power_Off_Voltage(struct device *dev, struct device_attribute *attr, char *buf)
{
	int ret_value = 1;

	ret_value = 3400;
	battery_log(BAT_LOG_CRTI, "[EM] Power_Off_Voltage : %d\n", ret_value);
	return sprintf(buf, "%u\n", ret_value);
}

static ssize_t store_Power_Off_Voltage(struct device *dev, struct device_attribute *attr,
				       const char *buf, size_t size)
{
	battery_log(BAT_LOG_CRTI, "[EM] Not Support Write Function\n");
	return size;
}

static DEVICE_ATTR(Power_Off_Voltage, 0664, show_Power_Off_Voltage, store_Power_Off_Voltage);

/* ///////////////////////////////////////////////////////////////////////////////////////// */
/* // Create File For EM : Charger_TopOff_Value */
/* ///////////////////////////////////////////////////////////////////////////////////////// */
static ssize_t show_Charger_TopOff_Value(struct device *dev, struct device_attribute *attr,
					 char *buf)
{
	int ret_value = 1;

	ret_value = 4110;
	battery_log(BAT_LOG_CRTI, "[EM] Charger_TopOff_Value : %d\n", ret_value);
	return sprintf(buf, "%u\n", ret_value);
}

static ssize_t store_Charger_TopOff_Value(struct device *dev, struct device_attribute *attr,
					  const char *buf, size_t size)
{
	battery_log(BAT_LOG_CRTI, "[EM] Not Support Write Function\n");
	return size;
}

static DEVICE_ATTR(Charger_TopOff_Value, 0664, show_Charger_TopOff_Value,
		   store_Charger_TopOff_Value);

/* ///////////////////////////////////////////////////////////////////////////////////////// */
/* // Create File For EM : FG_Battery_CurrentConsumption */
/* ///////////////////////////////////////////////////////////////////////////////////////// */
static ssize_t show_FG_Battery_CurrentConsumption(struct device *dev, struct device_attribute *attr,
						  char *buf)
{
	int ret_value = 8888;

	ret_value = battery_meter_get_battery_current();
	battery_log(BAT_LOG_CRTI, "[EM] FG_Battery_CurrentConsumption : %d/10 mA\n", ret_value);
	return sprintf(buf, "%u\n", ret_value);
}

static ssize_t store_FG_Battery_CurrentConsumption(struct device *dev,
						   struct device_attribute *attr, const char *buf,
						   size_t size)
{
	battery_log(BAT_LOG_CRTI, "[EM] Not Support Write Function\n");
	return size;
}

static DEVICE_ATTR(FG_Battery_CurrentConsumption, 0664, show_FG_Battery_CurrentConsumption,
		   store_FG_Battery_CurrentConsumption);

/* ///////////////////////////////////////////////////////////////////////////////////////// */
/* // Create File For EM : FG_SW_CoulombCounter */
/* ///////////////////////////////////////////////////////////////////////////////////////// */
static ssize_t show_FG_SW_CoulombCounter(struct device *dev, struct device_attribute *attr,
					 char *buf)
{
	signed int ret_value = 7777;

	ret_value = battery_meter_get_car();
	battery_log(BAT_LOG_CRTI, "[EM] FG_SW_CoulombCounter : %d\n", ret_value);
	return sprintf(buf, "%u\n", ret_value);
}

static ssize_t store_FG_SW_CoulombCounter(struct device *dev, struct device_attribute *attr,
					  const char *buf, size_t size)
{
	battery_log(BAT_LOG_CRTI, "[EM] Not Support Write Function\n");
	return size;
}

static DEVICE_ATTR(FG_SW_CoulombCounter, 0664, show_FG_SW_CoulombCounter,
		   store_FG_SW_CoulombCounter);


static ssize_t show_Charging_CallState(struct device *dev, struct device_attribute *attr, char *buf)
{
	battery_log(BAT_LOG_CRTI, "call state = %d\n", g_call_state);
	return sprintf(buf, "%u\n", g_call_state);
}

static ssize_t store_Charging_CallState(struct device *dev, struct device_attribute *attr,
					const char *buf, size_t size)
{
	int rv;

	/*rv = sscanf(buf, "%u", &g_call_state);*/
	rv = kstrtouint(buf, 0, &g_call_state);
	if (rv != 0)
		return -EINVAL;
	battery_log(BAT_LOG_CRTI, "call state = %d\n", g_call_state);
	return size;
}

static DEVICE_ATTR(Charging_CallState, 0664, show_Charging_CallState, store_Charging_CallState);

/* V_0Percent_Tracking */
static ssize_t show_V_0Percent_Tracking(struct device *dev, struct device_attribute *attr,
					char *buf)
{
	battery_log(BAT_LOG_CRTI, "V_0Percent_Tracking = %d\n", V_0Percent_Tracking);
	return sprintf(buf, "%u\n", V_0Percent_Tracking);
}

static ssize_t store_V_0Percent_Tracking(struct device *dev, struct device_attribute *attr,
					 const char *buf, size_t size)
{
	int rv;

	/*rv = sscanf(buf, "%u", &V_0Percent_Tracking);*/
	rv = kstrtouint(buf, 0, &V_0Percent_Tracking);
	if (rv != 0)
		return -EINVAL;
	battery_log(BAT_LOG_CRTI, "V_0Percent_Tracking = %d\n", V_0Percent_Tracking);
	return size;
}

static DEVICE_ATTR(V_0Percent_Tracking, 0664, show_V_0Percent_Tracking, store_V_0Percent_Tracking);


static ssize_t show_Charger_Type(struct device *dev, struct device_attribute *attr, char *buf)
{
	unsigned int chr_ype = CHARGER_UNKNOWN;

	chr_ype = BMT_status.charger_exist ? BMT_status.charger_type : CHARGER_UNKNOWN;

	battery_log(BAT_LOG_CRTI, "CHARGER_TYPE = %d\n", chr_ype);
	return sprintf(buf, "%u\n", chr_ype);
}

static ssize_t store_Charger_Type(struct device *dev, struct device_attribute *attr,
				  const char *buf, size_t size)
{
	battery_log(BAT_LOG_CRTI, "[EM] Not Support Write Function\n");
	return size;
}

static DEVICE_ATTR(Charger_Type, 0664, show_Charger_Type, store_Charger_Type);

#if defined(CONFIG_MTK_PUMP_EXPRESS_SUPPORT) || defined(CONFIG_MTK_PUMP_EXPRESS_PLUS_SUPPORT)
static ssize_t show_Pump_Express(struct device *dev, struct device_attribute *attr, char *buf)
{
#if defined(CONFIG_MTK_PUMP_EXPRESS_PLUS_SUPPORT)
	int icount = 20;	/* max debouncing time 20 * 0.2 sec */

	if (KAL_TRUE == ta_check_chr_type &&
	    STANDARD_CHARGER == BMT_status.charger_type &&
	    BMT_status.SOC >= batt_cust_data.ta_start_battery_soc &&
	    BMT_status.SOC < batt_cust_data.ta_stop_battery_soc) {
		battery_log(BAT_LOG_CRTI, "[%s]Wait for PE detection\n", __func__);
		do {
			icount--;
			msleep(200);
		} while (icount && ta_check_chr_type);
	}
#endif

#if defined(CONFIG_MTK_PUMP_EXPRESS_SUPPORT)

	if ((KAL_TRUE == ta_check_chr_type) && (STANDARD_CHARGER == BMT_status.charger_type)) {
		battery_log(BAT_LOG_CRTI, "[%s]Wait for PE detection\n", __func__);
		do {
			msleep(200);
		} while (ta_check_chr_type);
	}
#endif


	battery_log(BAT_LOG_CRTI, "Pump express = %d\n", is_ta_connect);
	return sprintf(buf, "%u\n", is_ta_connect);
}

static ssize_t store_Pump_Express(struct device *dev, struct device_attribute *attr,
				  const char *buf, size_t size)
{
	int rv;

	/*rv = sscanf(buf, "%u", &is_ta_connect);*/
	rv = kstrtouint(buf, 0, &is_ta_connect);
	if (rv != 0)
		return -EINVAL;
	battery_log(BAT_LOG_CRTI, "Pump express= %d\n", is_ta_connect);
	return size;
}

static DEVICE_ATTR(Pump_Express, 0664, show_Pump_Express, store_Pump_Express);
#endif
extern signed int gFG_BATT_CAPACITY_aging;  //wangyang add CPM2016092700202 
static void mt_battery_update_EM(struct battery_data *bat_data)
{
	bat_data->BAT_CAPACITY = BMT_status.UI_SOC;
	bat_data->BAT_TemperatureR = BMT_status.temperatureR;	/* API */
	bat_data->BAT_TempBattVoltage = BMT_status.temperatureV;	/* API */
	bat_data->BAT_InstatVolt = BMT_status.bat_vol;	/* VBAT */
	bat_data->BAT_BatteryAverageCurrent = BMT_status.ICharging;
	bat_data->BAT_BatteryInstantCurrent = InstantICharging;
	bat_data->BAT_BatterySenseVoltage = BMT_status.bat_vol;
	bat_data->BAT_ISenseVoltage = BMT_status.Vsense;	/* API */
	bat_data->BAT_ChargerVoltage = BMT_status.charger_vol;
	/* Dual battery */
	bat_data->status_smb = g_status_smb;
	bat_data->capacity_smb = g_capacity_smb;
	bat_data->present_smb = g_present_smb;
	battery_log(BAT_LOG_FULL, "status_smb = %d, capacity_smb = %d, present_smb = %d\n",
		    bat_data->status_smb, bat_data->capacity_smb, bat_data->present_smb);
	if ((BMT_status.UI_SOC == 100) && (BMT_status.charger_exist == KAL_TRUE))
		bat_data->BAT_STATUS = POWER_SUPPLY_STATUS_FULL;

#ifdef CONFIG_MTK_DISABLE_POWER_ON_OFF_VOLTAGE_LIMITATION
	if (bat_data->BAT_CAPACITY <= 0)
		bat_data->BAT_CAPACITY = 1;

	battery_log(BAT_LOG_CRTI,
		    "BAT_CAPACITY=1, due to define CONFIG_MTK_DISABLE_POWER_ON_OFF_VOLTAGE_LIMITATION\r\n");
#endif

	bat_data->capacity_fcc = gFG_BATT_CAPACITY_aging;  //wangyang add CPM2016092700202 
	bat_data->capacity_rm=BMT_status.UI_SOC*gFG_BATT_CAPACITY_aging/100;
	}
#ifdef POST_99TO100_CHARGING_TIME
	static unsigned int post_charging_time = 0;
#endif

static kal_bool mt_battery_100Percent_tracking_check(void)
{
	kal_bool resetBatteryMeter = KAL_FALSE;

#if defined(CONFIG_MTK_JEITA_STANDARD_SUPPORT)
	unsigned int cust_sync_time = CUST_SOC_JEITA_SYNC_TIME;
	static unsigned int timer_counter = (CUST_SOC_JEITA_SYNC_TIME / BAT_TASK_PERIOD);
#else
	unsigned int cust_sync_time = batt_cust_data.onehundred_percent_tracking_time;
	/* = (batt_cust_data.onehundred_percent_tracking_time / BAT_TASK_PERIOD); */
	static unsigned int timer_counter;
	static int timer_counter_init;
#endif


#if defined(CONFIG_MTK_JEITA_STANDARD_SUPPORT)
#else
	/* Init timer_counter for 1st time */
	if (timer_counter_init == 0) {
		timer_counter = (batt_cust_data.onehundred_percent_tracking_time / BAT_TASK_PERIOD);
		timer_counter_init = 1;
	}
#endif


	if (BMT_status.bat_full == KAL_TRUE) {	/* charging full first, UI tracking to 100% */
		if (BMT_status.UI_SOC >= 100) {
			BMT_status.UI_SOC = 100;

			if ((g_charging_full_reset_bat_meter == KAL_TRUE)
			    && (BMT_status.bat_charging_state == CHR_BATFULL)) {
				resetBatteryMeter = KAL_TRUE;
				g_charging_full_reset_bat_meter = KAL_FALSE;
			} else {
				resetBatteryMeter = KAL_FALSE;
			}
		} else {
			/* increase UI percentage every xxs */
			if (timer_counter >= (cust_sync_time / BAT_TASK_PERIOD)) {
				timer_counter = 1;
				BMT_status.UI_SOC++;
			} else {
				timer_counter++;

				return resetBatteryMeter;
			}

			resetBatteryMeter = KAL_TRUE;
		}

		if (BMT_status.UI_SOC == 100)
			is_uisoc_ever_100 = KAL_TRUE;

		if ((BMT_status.UI_SOC - BMT_status.SOC) > 10 && is_uisoc_ever_100 == KAL_TRUE) {
			is_uisoc_ever_100 = KAL_FALSE;
			BMT_status.bat_full = KAL_FALSE;
		}

		battery_log(BAT_LOG_CRTI, "[100percent], UI_SOC(%d), reset(%d) bat_full(%d) ever100(%d)\n",
			    BMT_status.UI_SOC, resetBatteryMeter, BMT_status.bat_full, is_uisoc_ever_100);
	} else if (is_uisoc_ever_100 == KAL_TRUE) {
			battery_log(BAT_LOG_CRTI, "[100percent-ever100],UI_SOC=%d SOC=%d\n",
			BMT_status.UI_SOC, BMT_status.UI_SOC);
	} else {
	#ifdef POST_99TO100_CHARGING_TIME
		if (BMT_status.UI_SOC == 99)
		{
			if ( post_charging_time >= POST_99_CHARGING_TIME ) {
				BMT_status.UI_SOC = 100 ; 
				battery_log(BAT_LOG_CRTI, "UI_SOC = 99, charging time over %d s , force ui_soc = %d\n", 
					POST_99_CHARGING_TIME, BMT_status.UI_SOC);
				resetBatteryMeter = KAL_TRUE;
			} else {
				post_charging_time += 10;			
			}
		} else {
			post_charging_time = 0;
		}
	#else
		/* charging is not full,  UI keep 99% if reaching 100%, */

		if (BMT_status.UI_SOC >= 99) {
			BMT_status.UI_SOC = 99;
			resetBatteryMeter = KAL_FALSE;

			battery_log(BAT_LOG_CRTI, "[100percent],UI_SOC = %d\n", BMT_status.UI_SOC);
		}
	#endif

		timer_counter = (cust_sync_time / BAT_TASK_PERIOD);

	}

	return resetBatteryMeter;
}


static kal_bool mt_battery_nPercent_tracking_check(void)
{
	kal_bool resetBatteryMeter = KAL_FALSE;
#if defined(SOC_BY_HW_FG)
	static unsigned int timer_counter;
	static int timer_counter_init;

	if (timer_counter_init == 0) {
		timer_counter_init = 1;
		timer_counter = (batt_cust_data.npercent_tracking_time / BAT_TASK_PERIOD);
	}


	if (BMT_status.nPrecent_UI_SOC_check_point == 0)
		return KAL_FALSE;
	/* 65% 15% 3% ;fuel gauge ZCV < 15%, but UI > 15%,  15% can be customized */
	if ((BMT_status.UI_SOC > fifty_percent_check_point) && (BMT_status.ZCV <= fifty_percent_zcv)) 
	{
		printk("[50percent]timer_counter=%d \n",timer_counter);
		if (timer_counter >= 10) {
			BMT_status.UI_SOC--;
			timer_counter = 1;
		} else {
			timer_counter++;
			return resetBatteryMeter;
		}
		resetBatteryMeter = KAL_TRUE;
		 battery_log(BAT_LOG_CRTI, "[50Percent] ZCV %d <= 50Percent_ZCV %d, UI_SOC=%d., tracking UI_SOC=%d\n",
                    BMT_status.ZCV, fifty_percent_zcv, BMT_status.UI_SOC, fifty_percent_check_point);
	} else if ((BMT_status.UI_SOC == fifty_percent_check_point) && (BMT_status.ZCV > fifty_percent_zcv)) 
	{
		timer_counter = (NPERCENT_TRACKING_TIME / BAT_TASK_PERIOD);
		resetBatteryMeter = KAL_TRUE;
                battery_log(BAT_LOG_CRTI,"[50Percent] hold 50percent ZCV= %d > fifty_percent_zcv= %d and UI SOC=%d, then keep %d.\n",BMT_status.ZCV, fifty_percent_zcv, BMT_status.UI_SOC, fifty_percent_check_point);
	}else if((BMT_status.ZCV <= BMT_status.nPercent_ZCV)&& (BMT_status.UI_SOC > BMT_status.nPrecent_UI_SOC_check_point)) {
		//if (timer_counter >= (batt_cust_data.npercent_tracking_time / BAT_TASK_PERIOD)){
		if((timer_counter >= 6)&&(BMT_status.UI_SOC >= 30)){
			BMT_status.UI_SOC--;
			timer_counter = 1;
		}else if(timer_counter >= 12){
			BMT_status.UI_SOC--;
			timer_counter = 1;
		} else {
			timer_counter++;
			printk("[15percent]timer_counter=%d \n",timer_counter);
			return resetBatteryMeter;
		}
		resetBatteryMeter = KAL_TRUE;
		battery_log(BAT_LOG_CRTI, "[nPercent] ZCV %d <= nPercent_ZCV %d, UI_SOC=%d., tracking UI_SOC=%d\n",BMT_status.ZCV, BMT_status.nPercent_ZCV, BMT_status.UI_SOC,BMT_status.nPrecent_UI_SOC_check_point);
	} else if ((BMT_status.ZCV > BMT_status.nPercent_ZCV)&& (BMT_status.UI_SOC == BMT_status.nPrecent_UI_SOC_check_point)) {
		/* UI less than 15 , but fuel gague is more than 15, hold UI 15% */
		timer_counter = (batt_cust_data.npercent_tracking_time / BAT_TASK_PERIOD);
		resetBatteryMeter = KAL_TRUE;
		battery_log(BAT_LOG_CRTI, "[nPercent] ZCV %d > BMT_status.nPercent_ZCV %d and UI SOC=%d, then keep %d.\n",BMT_status.ZCV, BMT_status.nPercent_ZCV, BMT_status.UI_SOC,BMT_status.nPrecent_UI_SOC_check_point);

	} else if ((BMT_status.UI_SOC > three_percent_check_point) && (BMT_status.ZCV <= three_percent_zcv)) 
	{
		printk("[3percent]timer_counter=%d \n",timer_counter);
		if (timer_counter >= 2) {
			/* every x sec decrease UI percentage */
			BMT_status.UI_SOC--;
			timer_counter = 1;
		} else {
			timer_counter++;
			return resetBatteryMeter;
		}
		resetBatteryMeter = KAL_TRUE;
		battery_log(BAT_LOG_CRTI,"[3Percent] sync 3percent ZCV %d <= BMT_status.nPercent_ZCV %d and UI SOC=%d, then keep %d.\n",BMT_status.ZCV, three_percent_zcv, BMT_status.UI_SOC, three_percent_check_point);
	} else if ((BMT_status.UI_SOC == three_percent_check_point) && (BMT_status.ZCV > three_percent_zcv)){
		timer_counter = (NPERCENT_TRACKING_TIME / BAT_TASK_PERIOD);
		resetBatteryMeter = KAL_TRUE;
		battery_log(BAT_LOG_CRTI,
				    "[3Percent] hold 3percent ZCV %d > BMT_status.nPercent_ZCV %d and UI SOC=%d, then keep %d.\n",
				    BMT_status.ZCV, three_percent_zcv, BMT_status.UI_SOC, three_percent_check_point);
	} else {
		timer_counter = (batt_cust_data.npercent_tracking_time / BAT_TASK_PERIOD);
	}
#endif
	return resetBatteryMeter;

}
extern signed int fgauge_read_capacity_by_v(signed int voltage);
static int power_off_percent_cnt =0;

static kal_bool mt_battery_65_runinPercent_tracking_check(void)
{
	unsigned int i;
	static int runin_soc =0;
	kal_bool resetBatteryMeter = KAL_FALSE;
	static unsigned int runing_timer_counter=0;
	static kal_bool zcvBufferFirst = KAL_TRUE;
	static signed int batVolBuffer[10];
	static signed int zcv_sum;
	static unsigned char zcvIndex;
	static unsigned int avgzcv;
	
	battery_log(BAT_LOG_CRTI,"get avgzcv =%d.\n",avgzcv);
	if (zcvBufferFirst == KAL_TRUE) {
		for (i = 0; i < 10; i++)
			batVolBuffer[i] = BMT_status.ZCV;
		zcv_sum = BMT_status.ZCV * 10;
		zcvBufferFirst = KAL_FALSE;
	}
	zcv_sum -= batVolBuffer[zcvIndex];
	zcv_sum += BMT_status.ZCV;
	batVolBuffer[zcvIndex] = BMT_status.ZCV;
	avgzcv = zcv_sum/10;
	battery_log(BAT_LOG_CRTI, "set avgzcv =%d,zcv=batVolBuffer[%d]= (%d)\n",avgzcv,zcvIndex, batVolBuffer[zcvIndex]);
	zcvIndex++;
	if (zcvIndex >= 10){
		zcvIndex = 0;
		}
	if ((BMT_status.UI_SOC > fifty_percent_check_point) && (BMT_status.ZCV <= fifty_percent_zcv)) {
		if (runing_timer_counter >= 3) {
			BMT_status.UI_SOC--;
			runing_timer_counter = 1;
		}else {
			runing_timer_counter++;
			return resetBatteryMeter;
		}
		resetBatteryMeter = KAL_TRUE;
		printk("[65_runinPercent] ZCV %d <= 50Percent_ZCV %d, UI_SOC=%d., tracking UI_SOC=%d\n",
		BMT_status.ZCV, fifty_percent_zcv, BMT_status.UI_SOC, fifty_percent_check_point);
	}else if((BMT_status.UI_SOC == fifty_percent_check_point)||(BMT_status.UI_SOC == 70)){
		runin_soc=fgauge_read_capacity_by_v(avgzcv);
		printk("runin_soc=%d \n",runin_soc);
		if(abs(runin_soc-BMT_status.UI_SOC)>=1){
			BMT_status.UI_SOC=runin_soc;
			resetBatteryMeter = KAL_TRUE;
		}
		printk("[65_runniPercent] ZCV= %d > fifty_percent_zcv=%d, UI SOC=%d,runin_soc=%d.\n",BMT_status.ZCV, fifty_percent_zcv, BMT_status.UI_SOC,runin_soc);
	}else
		runing_timer_counter = 3;
	printk("[65_runniPercent] ZCV= %d ,fifty_percent_zcv= %d ,UI SOC=%d,check_point=%d.\n",BMT_status.ZCV, fifty_percent_zcv, BMT_status.UI_SOC, fifty_percent_check_point);
	return resetBatteryMeter;
}

static kal_bool mt_battery_0Percent_tracking_check(void)
{
	kal_bool resetBatteryMeter = KAL_TRUE;
	static int timer_0counter=0;
	if (BMT_status.UI_SOC <= 0) {
		BMT_status.UI_SOC = 0;
	} else{
		if(BMT_status.bat_vol > SYSTEM_OFF_VOLTAGE && BMT_status.UI_SOC > 1){
			if(timer_0counter == 2){
				BMT_status.UI_SOC--;
				timer_0counter= 1;
				//resetBatteryMeter = KAL_TRUE;
			}else{
				timer_0counter++;
				resetBatteryMeter = KAL_FALSE;
				return resetBatteryMeter;
			}
		}else if (BMT_status.bat_vol <= SYSTEM_OFF_VOLTAGE){
			BMT_status.UI_SOC--; 
			battery_log(BAT_LOG_CRTI, "0Percent, VBAT < %d UI_SOC=%d\r\n", SYSTEM_OFF_VOLTAGE,BMT_status.UI_SOC);
		}
		 battery_log(BAT_LOG_CRTI,"[0Percent] wcl UI--,UI_SOC=%d,BMT_status.bat_volt=%d\n", BMT_status.UI_SOC,BMT_status.bat_vol);
	}
	if( (BMT_status.charger_exist == KAL_TRUE) && (BMT_status.UI_SOC == 0)&&(gFG_Is_Charging == KAL_FALSE))
	{
		if (power_off_percent_cnt++  >  3 ) {
			battery_log(BAT_LOG_CRTI, "0Percent, shutdown ,VBAT < %d UI_SOC=%d \r\n", SYSTEM_OFF_VOLTAGE,BMT_status.UI_SOC);
			battery_charging_control(CHARGING_CMD_SET_POWER_OFF, NULL);
 		}
	} else {
		power_off_percent_cnt = 0;
	}
	return resetBatteryMeter;
}


static void mt_battery_Sync_UI_Percentage_to_Real(void)
{
	static unsigned int timer_counter;

	if ((BMT_status.UI_SOC > BMT_status.SOC) && ((BMT_status.UI_SOC != 1))) {
#if !defined(SYNC_UI_SOC_IMM)
		/* reduce after xxs */
	if(chr_wake_up_bat==KAL_FALSE)
	{
		if(BMT_status.SOC>94)
		{
			if (timer_counter == 12) 
			{
			BMT_status.UI_SOC--;
			timer_counter = 0;
			}
			else {
			timer_counter++;
		    }
			battery_log(BAT_LOG_CRTI, "[Sync_Real] UI_SOC=%d, SOC=%d, counter = %d\n",
				    BMT_status.UI_SOC, BMT_status.SOC, timer_counter);
		}
		else
		{
			if (timer_counter >= (SYNC_TO_REAL_TRACKING_TIME / BAT_TASK_PERIOD)) 
			{
			BMT_status.UI_SOC--;
			timer_counter = 0;
			} 
#ifdef FG_BAT_INT
			else if (fg_wake_up_bat == KAL_TRUE)
				BMT_status.UI_SOC--;

#endif				/* #ifdef FG_BAT_INT */
			else
				timer_counter++;

			battery_log(BAT_LOG_CRTI, "[Sync_Real] UI_SOC=%d, SOC=%d, counter = %d\n",
				    BMT_status.UI_SOC, BMT_status.SOC, timer_counter);
		}
		} else {
			battery_log(BAT_LOG_CRTI,
				    "[Sync_Real] chr_wake_up_bat=1 , do not update UI_SOC\n");
		}
#else
		BMT_status.UI_SOC--;
#endif
		battery_log(BAT_LOG_CRTI, "[Sync_Real] UI_SOC=%d, SOC=%d, counter = %d\n",
			    BMT_status.UI_SOC, BMT_status.SOC, timer_counter);
	} else {
		timer_counter = 0;

#if !defined(CUST_CAPACITY_OCV2CV_TRANSFORM)
		BMT_status.UI_SOC = BMT_status.SOC;
#else
		if (BMT_status.UI_SOC == -1)
			BMT_status.UI_SOC = BMT_status.SOC;
		else if (BMT_status.charger_exist && BMT_status.bat_charging_state != CHR_ERROR) {
			if (BMT_status.UI_SOC < BMT_status.SOC
			    && (BMT_status.SOC - BMT_status.UI_SOC > 1))
				BMT_status.UI_SOC++;
			else
				BMT_status.UI_SOC = BMT_status.SOC;
		}
#endif
	}

	if (BMT_status.UI_SOC <= 0) {
		BMT_status.UI_SOC = 1;
		battery_log(BAT_LOG_CRTI, "[Battery]UI_SOC get 0 first (%d)\r\n",
			    BMT_status.UI_SOC);
	}
}

static void battery_update(struct battery_data *bat_data)
{
	struct power_supply *bat_psy = &bat_data->psy;
	kal_bool resetBatteryMeter = KAL_FALSE;

	bat_data->BAT_TECHNOLOGY = POWER_SUPPLY_TECHNOLOGY_LION;
	bat_data->BAT_HEALTH = POWER_SUPPLY_HEALTH_GOOD;
	bat_data->BAT_batt_vol = BMT_status.bat_vol;
	bat_data->BAT_batt_temp = BMT_status.temperature * 10;
	bat_data->BAT_PRESENT = BMT_status.bat_exist;
#ifdef HQ_CHARGER_FOR_HUAWEI
    if( (BMT_status.charger_exist == KAL_TRUE) && (BMT_status.bat_charging_state != CHR_ERROR) && !g_charger_should_not_open)
#else
    if( (BMT_status.charger_exist == KAL_TRUE) && (BMT_status.bat_charging_state != CHR_ERROR) )
#endif
       {  
       if (BMT_status.bat_exist) {     /* charging */
	   		printk("[runi_65_test]bat_vol=%d,UI_SOC=%d,g_runin_battery_test_flag=%d \n",BMT_status.bat_vol,BMT_status.UI_SOC,g_runin_battery_test_flag);
			if(BMT_status.bat_vol <= batt_cust_data.v_0percent_tracking){
				resetBatteryMeter = mt_battery_0Percent_tracking_check();
			}else{
				if (g_runin_battery_test_flag ==1){
					printk("[runi_65_test]UI_SOC=%d,g_runin_battery_test_flag=%d \n",BMT_status.UI_SOC,g_runin_battery_test_flag);
					resetBatteryMeter = mt_battery_65_runinPercent_tracking_check();
				}else{
					resetBatteryMeter = mt_battery_100Percent_tracking_check();
				}
			}
			bat_data->BAT_STATUS = POWER_SUPPLY_STATUS_CHARGING;
		} else {	/* No Battery, Only Charger */

			bat_data->BAT_STATUS = POWER_SUPPLY_STATUS_UNKNOWN;
			BMT_status.UI_SOC = 0;
		}

	} else {		/* Only Battery */

		bat_data->BAT_STATUS = POWER_SUPPLY_STATUS_NOT_CHARGING;
		if (BMT_status.bat_vol <= batt_cust_data.v_0percent_tracking)
			resetBatteryMeter = mt_battery_0Percent_tracking_check();
		else if (BMT_status.temperature > -1)
			resetBatteryMeter = mt_battery_nPercent_tracking_check();
	}

	if (resetBatteryMeter == KAL_TRUE) {
		battery_meter_reset();
	} else {
		if (BMT_status.bat_full == KAL_TRUE && is_uisoc_ever_100 == KAL_TRUE) {
			BMT_status.UI_SOC = 100;
			battery_log(BAT_LOG_CRTI, "[recharging] UI_SOC=%d, SOC=%d\n",
				    BMT_status.UI_SOC, BMT_status.SOC);
		} else {
			mt_battery_Sync_UI_Percentage_to_Real();
		}
	}

	battery_log(BAT_LOG_CRTI, "UI_SOC=(%d), resetBatteryMeter=(%d)\n",
		    BMT_status.UI_SOC, resetBatteryMeter);

#if !defined(CUST_CAPACITY_OCV2CV_TRANSFORM)
	/* set RTC SOC to 1 to avoid SOC jump in charger boot.*/
	if (BMT_status.UI_SOC <= 1)
		set_rtc_spare_fg_value(1);
	else{
			//zcy_add for runin battery test start
					 if(g_runin_battery_test_flag)
					 {
						 battery_xlog_printk(BAT_LOG_CRTI, "[BATTERY] set_rtc_spare_fg_value and Set Battery SOC:%d!!\n\r",BMT_status.SOC);  
						 set_rtc_spare_fg_value(BMT_status.SOC);
					 }
					 else //add end
			 {
				 set_rtc_spare_fg_value(BMT_status.UI_SOC);
			 }
		}
	battery_log(BAT_LOG_CRTI, "RTC_SOC=(%d)\n", get_rtc_spare_fg_value());

#else
	/* We store capacity before loading compenstation in RTC */
	if (battery_meter_get_battery_soc() <= 1)
		set_rtc_spare_fg_value(1);
	else
		{
				 //gyg_add for runin battery test start
					 if(g_runin_battery_test_flag)
					 {
						 battery_xlog_printk(BAT_LOG_CRTI, "[BATTERY] set_rtc_spare_fg_value and Set Battery SOC:%d!!\n\r",BMT_status.SOC);  
						 set_rtc_spare_fg_value(BMT_status.SOC);
					 }
					 else//add end
			 {
				set_rtc_spare_fg_value(battery_meter_get_battery_soc()); /*use battery_soc */
			 }
		}
		battery_log(BAT_LOG_CRTI, "RTC_SOC=(%d)\n", get_rtc_spare_fg_value());

#endif

	mt_battery_update_EM(bat_data);

	if (cmd_discharging == 1)
		bat_data->BAT_STATUS = POWER_SUPPLY_STATUS_CMD_DISCHARGING;

	if (adjust_power != -1) {
		bat_data->adjust_power = adjust_power;
		battery_log(BAT_LOG_CRTI, "adjust_power=(%d)\n", adjust_power);
	}
#ifdef DLPT_POWER_OFF_EN
	/*extern int dlpt_check_power_off(void);*/
	if (bat_data->BAT_CAPACITY <= DLPT_POWER_OFF_THD) {
		static signed char cnt;

		battery_log(BAT_LOG_FULL, "[DLPT_POWER_OFF_EN] run\n");

		if (dlpt_check_power_off() == 1) {
			bat_data->BAT_CAPACITY = 0;
			cnt++;
			battery_log(BAT_LOG_CRTI, "[DLPT_POWER_OFF_EN] SOC=%d to power off\n",
				    bat_data->BAT_CAPACITY);
			if (cnt >= 2)
				kernel_restart("DLPT reboot system");

		} else
			cnt = 0;
	} else {
		battery_log(BAT_LOG_FULL, "[DLPT_POWER_OFF_EN] disable(%d)\n",
			    bat_data->BAT_CAPACITY);
	}
#endif

	power_supply_changed(bat_psy);
}

void update_charger_info(int wireless_state)
{
#if defined(CONFIG_POWER_VERIFY)
	battery_log(BAT_LOG_CRTI, "[update_charger_info] no support\n");
#else
	g_wireless_state = wireless_state;
	battery_log(BAT_LOG_CRTI, "[update_charger_info] get wireless_state=%d\n", wireless_state);

	wake_up_bat();
#endif
}

static void wireless_update(struct wireless_data *wireless_data)
{
	struct power_supply *wireless_psy = &wireless_data->psy;

	if (BMT_status.charger_exist == KAL_TRUE || g_wireless_state) {
		if ((BMT_status.charger_type == WIRELESS_CHARGER) || g_wireless_state) {
			wireless_data->WIRELESS_ONLINE = 1;
			wireless_psy->type = POWER_SUPPLY_TYPE_WIRELESS;
		} else {
			wireless_data->WIRELESS_ONLINE = 0;
		}
	} else {
		wireless_data->WIRELESS_ONLINE = 0;
	}

	power_supply_changed(wireless_psy);
}

static void ac_update(struct ac_data *ac_data)
{
	struct power_supply *ac_psy = &ac_data->psy;

	if (BMT_status.charger_exist == KAL_TRUE) {
#if !defined(CONFIG_MTK_DUAL_INPUT_CHARGER_SUPPORT)
		if ((BMT_status.charger_type == NONSTANDARD_CHARGER) ||
		    (BMT_status.charger_type == STANDARD_CHARGER) ||
		    (BMT_status.charger_type == APPLE_2_1A_CHARGER) ||
		    (BMT_status.charger_type == APPLE_1_0A_CHARGER) ||
		    (BMT_status.charger_type == APPLE_0_5A_CHARGER)) {
#else
		if ((BMT_status.charger_type == NONSTANDARD_CHARGER) ||
		    (BMT_status.charger_type == STANDARD_CHARGER) ||
		    (BMT_status.charger_type == APPLE_2_1A_CHARGER) ||
		    (BMT_status.charger_type == APPLE_1_0A_CHARGER) ||
		    (BMT_status.charger_type == APPLE_0_5A_CHARGER) ||
		    (DISO_data.diso_state.cur_vdc_state == DISO_ONLINE)) {
#endif
			ac_data->AC_ONLINE = 1;
			ac_psy->type = POWER_SUPPLY_TYPE_MAINS;
		} else {
			ac_data->AC_ONLINE = 0;
		}
	} else {
		ac_data->AC_ONLINE = 0;
	}

	power_supply_changed(ac_psy);
}

static void usb_update(struct usb_data *usb_data)
{
	struct power_supply *usb_psy = &usb_data->psy;

	if (BMT_status.charger_exist == KAL_TRUE) {
		if ((BMT_status.charger_type == STANDARD_HOST) ||
		    (BMT_status.charger_type == CHARGING_HOST)) {
			usb_data->USB_ONLINE = 1;
			usb_psy->type = POWER_SUPPLY_TYPE_USB;
		} else {
			usb_data->USB_ONLINE = 0;
		}
	} else {
		usb_data->USB_ONLINE = 0;
	}

	power_supply_changed(usb_psy);
}

#endif

/* ///////////////////////////////////////////////////////////////////////////////////////// */
/* // Battery Temprature Parameters and functions */
/* ///////////////////////////////////////////////////////////////////////////////////////// */
kal_bool pmic_chrdet_status(void)
{
	if (upmu_is_chr_det() == KAL_TRUE)
		return KAL_TRUE;
	/*else {*/
		battery_log(BAT_LOG_CRTI, "[pmic_chrdet_status] No charger\r\n");
		return KAL_FALSE;
	/*}*/
}

/* ///////////////////////////////////////////////////////////////////////////////////////// */
/* // Pulse Charging Algorithm */
/* ///////////////////////////////////////////////////////////////////////////////////////// */
kal_bool bat_is_charger_exist(void)
{
	return get_charger_detect_status();
}


kal_bool bat_is_charging_full(void)
{
	if ((BMT_status.bat_full == KAL_TRUE) && (BMT_status.bat_in_recharging_state == KAL_FALSE))
		return KAL_TRUE;
	else
		return KAL_FALSE;
}


unsigned int bat_get_ui_percentage(void)
{
	/* for plugging out charger in recharge phase, using SOC as UI_SOC */

#if defined(CONFIG_POWER_EXT)
	battery_log(BAT_LOG_CRTI, "[BATTERY] bat_get_ui_percentage return 100 !!\n\r");
	return 100;
#else
	if (chr_wake_up_bat == KAL_TRUE)
		return BMT_status.SOC;
	else
		return BMT_status.UI_SOC;
#endif
}

/* Full state --> recharge voltage --> full state */
unsigned int bat_is_recharging_phase(void)
{
	return BMT_status.bat_in_recharging_state || BMT_status.bat_full == KAL_TRUE;
}


int get_bat_charging_current_level(void)
{
	CHR_CURRENT_ENUM charging_current;

	battery_charging_control(CHARGING_CMD_GET_CURRENT, &charging_current);

	return charging_current;
}


PMU_STATUS do_batt_temp_state_machine(void)
{
	if (BMT_status.temperature == batt_cust_data.err_charge_temperature)
		return PMU_STATUS_FAIL;



	if (batt_cust_data.bat_low_temp_protect_enable) {
		if (BMT_status.temperature < batt_cust_data.min_charge_temperature) {
			battery_log(BAT_LOG_CRTI,
				    "[BATTERY] Battery Under Temperature or NTC fail !!\n\r");
			g_batt_temp_status = TEMP_POS_LOW;
			return PMU_STATUS_FAIL;
		} else if (g_batt_temp_status == TEMP_POS_LOW) {
			if (BMT_status.temperature >=
			    batt_cust_data.min_charge_temperature_plus_x_degree) {
				battery_log(BAT_LOG_CRTI,
					    "[BATTERY] Battery Temperature raise from %d to %d(%d), allow charging!!\n\r",
					    batt_cust_data.min_charge_temperature,
					    BMT_status.temperature,
					    batt_cust_data.min_charge_temperature_plus_x_degree);
				g_batt_temp_status = TEMP_POS_NORMAL;
				BMT_status.bat_charging_state = CHR_PRE;
				return PMU_STATUS_OK;
			} else {
				return PMU_STATUS_FAIL;
			}
		}
	}

	if (BMT_status.temperature >= batt_cust_data.max_charge_temperature) {
		battery_log(BAT_LOG_CRTI, "[BATTERY] Battery Over Temperature !!\n\r");
		g_batt_temp_status = TEMP_POS_HIGH;
		return PMU_STATUS_FAIL;
	} else if (g_batt_temp_status == TEMP_POS_HIGH) {
		if (BMT_status.temperature < batt_cust_data.max_charge_temperature_minus_x_degree) {
			battery_log(BAT_LOG_CRTI,
				    "[BATTERY] Battery Temperature down from %d to %d(%d), allow charging!!\n\r",
				    batt_cust_data.max_charge_temperature, BMT_status.temperature,
				    batt_cust_data.max_charge_temperature_minus_x_degree);
			g_batt_temp_status = TEMP_POS_NORMAL;
			BMT_status.bat_charging_state = CHR_PRE;
			return PMU_STATUS_OK;
		} else {
			return PMU_STATUS_FAIL;
		}
	} else {
		g_batt_temp_status = TEMP_POS_NORMAL;
	}
	return PMU_STATUS_OK;
}


unsigned long BAT_Get_Battery_Voltage(int polling_mode)
{
	unsigned long ret_val = 0;

#if defined(CONFIG_POWER_EXT)
	ret_val = 4000;
#else
	ret_val = battery_meter_get_battery_voltage(KAL_FALSE);
#endif

	return ret_val;
}


static void mt_battery_average_method_init(BATTERY_AVG_ENUM type, unsigned int *bufferdata,
					   unsigned int data, signed int *sum)
{
	unsigned int i;
	static kal_bool batteryBufferFirst = KAL_TRUE;
	static kal_bool previous_charger_exist = KAL_FALSE;
	static kal_bool previous_in_recharge_state = KAL_FALSE;

	static unsigned char index;

	/* reset charging current window while plug in/out { */
	if (type == BATTERY_AVG_CURRENT) {
		if (BMT_status.charger_exist == KAL_TRUE) {
			if (previous_charger_exist == KAL_FALSE) {
				batteryBufferFirst = KAL_TRUE;
				previous_charger_exist = KAL_TRUE;
#if !defined(CONFIG_MTK_DUAL_INPUT_CHARGER_SUPPORT)
				if (BMT_status.charger_type == STANDARD_CHARGER) {
#else
				if ((BMT_status.charger_type == STANDARD_CHARGER) ||
				    (DISO_data.diso_state.cur_vdc_state == DISO_ONLINE)) {
#endif
					data = batt_cust_data.ac_charger_current / 100;
				} else if (BMT_status.charger_type == CHARGING_HOST) {
					data = batt_cust_data.charging_host_charger_current / 100;
				} else if (BMT_status.charger_type == NONSTANDARD_CHARGER)
					data = batt_cust_data.non_std_ac_charger_current / 100;	/* mA */
				else	/* USB */
					data = batt_cust_data.usb_charger_current / 100;	/* mA */
#ifdef AVG_INIT_WITH_R_SENSE
				data = AVG_INIT_WITH_R_SENSE(data);
#endif
			} else if ((previous_in_recharge_state == KAL_FALSE)
				   && (BMT_status.bat_in_recharging_state == KAL_TRUE)) {
				batteryBufferFirst = KAL_TRUE;
#if !defined(CONFIG_MTK_DUAL_INPUT_CHARGER_SUPPORT)
				if (BMT_status.charger_type == STANDARD_CHARGER) {
#else
				if ((BMT_status.charger_type == STANDARD_CHARGER) ||
				    (DISO_data.diso_state.cur_vdc_state == DISO_ONLINE)) {
#endif
					data = batt_cust_data.ac_charger_current / 100;
				} else if (BMT_status.charger_type == CHARGING_HOST) {
					data = batt_cust_data.charging_host_charger_current / 100;
				} else if (BMT_status.charger_type == NONSTANDARD_CHARGER)
					data = batt_cust_data.non_std_ac_charger_current / 100;	/* mA */
				else	/* USB */
					data = batt_cust_data.usb_charger_current / 100;	/* mA */
#ifdef AVG_INIT_WITH_R_SENSE
				data = AVG_INIT_WITH_R_SENSE(data);
#endif
			}

			previous_in_recharge_state = BMT_status.bat_in_recharging_state;
		} else {
			if (previous_charger_exist == KAL_TRUE) {
				batteryBufferFirst = KAL_TRUE;
				previous_charger_exist = KAL_FALSE;
				data = 0;
			}
		}
	}
	/* reset charging current window while plug in/out } */

	battery_log(BAT_LOG_FULL, "batteryBufferFirst =%d, data= (%d)\n", batteryBufferFirst, data);

	if (batteryBufferFirst == KAL_TRUE) {
		for (i = 0; i < BATTERY_AVERAGE_SIZE; i++)
			bufferdata[i] = data;


		*sum = data * BATTERY_AVERAGE_SIZE;
	}

	index++;
	if (index >= BATTERY_AVERAGE_DATA_NUMBER) {
		index = BATTERY_AVERAGE_DATA_NUMBER;
		batteryBufferFirst = KAL_FALSE;
	}
}


static unsigned int mt_battery_average_method(BATTERY_AVG_ENUM type, unsigned int *bufferdata,
					    unsigned int data, signed int *sum,
					    unsigned char batteryIndex)
{
	unsigned int avgdata;

	mt_battery_average_method_init(type, bufferdata, data, sum);

	*sum -= bufferdata[batteryIndex];
	*sum += data;
	bufferdata[batteryIndex] = data;
	avgdata = (*sum) / BATTERY_AVERAGE_SIZE;

	battery_log(BAT_LOG_FULL, "bufferdata[%d]= (%d)\n", batteryIndex, bufferdata[batteryIndex]);
	return avgdata;
}

void mt_battery_GetBatteryData(void)
{
	unsigned int bat_vol, charger_vol, Vsense, ZCV;
	signed int ICharging, temperature, temperatureR, temperatureV, SOC;
	static signed int bat_sum, icharging_sum, temperature_sum;
	static signed int batteryVoltageBuffer[BATTERY_AVERAGE_SIZE];
	static signed int batteryCurrentBuffer[BATTERY_AVERAGE_SIZE];
	static signed int batteryTempBuffer[BATTERY_AVERAGE_SIZE];
	static unsigned char batteryIndex;
	static signed int previous_SOC = -1;

	bat_vol = battery_meter_get_battery_voltage(KAL_TRUE);
	battery_log(BAT_LOG_CRTI,"huaqin mt_battery_GetBatteryData GET_ADC_V_BAT_SENSE bat_vol=%d \n",bat_vol);
	Vsense = battery_meter_get_VSense();
	if (upmu_is_chr_det() == KAL_TRUE)
		ICharging = battery_meter_get_charging_current();
	else
		ICharging = 0;
	InstantICharging=ICharging;
	printk("InstantICharging=%d. \n",InstantICharging);
	charger_vol = battery_meter_get_charger_voltage();
	temperature = battery_meter_get_battery_temperature();
	temperatureV = battery_meter_get_tempV();
	temperatureR = battery_meter_get_tempR(temperatureV);

	if (bat_meter_timeout == KAL_TRUE || bat_spm_timeout == TRUE || fg_wake_up_bat == KAL_TRUE) {
		SOC = battery_meter_get_battery_percentage();
		/* if (bat_spm_timeout == true) */
		/* BMT_status.UI_SOC = battery_meter_get_battery_percentage(); */

		bat_meter_timeout = KAL_FALSE;
		bat_spm_timeout = FALSE;
	} else {
		if (previous_SOC == -1)
			SOC = battery_meter_get_battery_percentage();
		else
			SOC = previous_SOC;
	}

	ZCV = battery_meter_get_battery_zcv();

	BMT_status.ICharging =
	    mt_battery_average_method(BATTERY_AVG_CURRENT, &batteryCurrentBuffer[0], ICharging,
				      &icharging_sum, batteryIndex);


	if (previous_SOC == -1 && bat_vol <= batt_cust_data.v_0percent_tracking) {
		battery_log(BAT_LOG_CRTI,
			    "battery voltage too low, use ZCV to init average data.\n");
		BMT_status.bat_vol =
		    mt_battery_average_method(BATTERY_AVG_VOLT, &batteryVoltageBuffer[0], ZCV,
					      &bat_sum, batteryIndex);
	} else {
		BMT_status.bat_vol =
		    mt_battery_average_method(BATTERY_AVG_VOLT, &batteryVoltageBuffer[0], bat_vol,
					      &bat_sum, batteryIndex);
	}


	if (battery_cmd_thermal_test_mode == 1) {
		battery_log(BAT_LOG_CRTI, "test mode , battery temperature is fixed.\n");
	} else {
		BMT_status.temperature =
		    mt_battery_average_method(BATTERY_AVG_TEMP, &batteryTempBuffer[0], temperature,
					      &temperature_sum, batteryIndex);
	}


	BMT_status.Vsense = Vsense;
	BMT_status.charger_vol = charger_vol;
	BMT_status.temperatureV = temperatureV;
	BMT_status.temperatureR = temperatureR;
	BMT_status.SOC = SOC;
	BMT_status.ZCV = ZCV;

#if !defined(CUST_CAPACITY_OCV2CV_TRANSFORM)
	if (BMT_status.charger_exist == KAL_FALSE) {
		if (BMT_status.SOC > previous_SOC && previous_SOC >= 0)
			BMT_status.SOC = previous_SOC;
	}
#endif

	previous_SOC = BMT_status.SOC;

	batteryIndex++;
	if (batteryIndex >= BATTERY_AVERAGE_SIZE)
		batteryIndex = 0;


	if (g_battery_soc_ready == KAL_FALSE)
		g_battery_soc_ready = KAL_TRUE;

	battery_log(BAT_LOG_CRTI,
	"AvgVbat=(%d,%d),AvgI=(%d,%d),VChr=%d,AvgT=(%d,%d),SOC=(%d,%d),UI_SOC=%d,ZCV=%d bcct:%d:%d I:%d\n",
		BMT_status.bat_vol, bat_vol, BMT_status.ICharging, ICharging,
		BMT_status.charger_vol, BMT_status.temperature, temperature,
		previous_SOC, BMT_status.SOC, BMT_status.UI_SOC, BMT_status.ZCV,
		g_bcct_flag, get_usb_current_unlimited(), get_bat_charging_current_level());

}



static PMU_STATUS mt_battery_CheckBatteryTemp(void)
{
	PMU_STATUS status = PMU_STATUS_OK;

#if defined(CONFIG_MTK_JEITA_STANDARD_SUPPORT)

	battery_log(BAT_LOG_CRTI, "[BATTERY] support JEITA, temperature=%d\n",
		    BMT_status.temperature);

	if (do_jeita_state_machine() == PMU_STATUS_FAIL) {
		battery_log(BAT_LOG_CRTI, "[BATTERY] JEITA : fail\n");
		status = PMU_STATUS_FAIL;
	}
#else


	if (batt_cust_data.mtk_temperature_recharge_support) {
		if (do_batt_temp_state_machine() == PMU_STATUS_FAIL) {
			battery_log(BAT_LOG_CRTI, "[BATTERY] Batt temp check : fail\n");
			status = PMU_STATUS_FAIL;
		}
	} else {
#ifdef BAT_LOW_TEMP_PROTECT_ENABLE
		if ((BMT_status.temperature < MIN_CHARGE_TEMPERATURE)
		    || (BMT_status.temperature == ERR_CHARGE_TEMPERATURE)) {
			battery_log(BAT_LOG_CRTI,
				    "[BATTERY] Battery Under Temperature or NTC fail !!\n\r");
			status = PMU_STATUS_FAIL;
		}
#endif
		if (BMT_status.temperature >= MAX_CHARGE_TEMPERATURE) {
			battery_log(BAT_LOG_CRTI, "[BATTERY] Battery Over Temperature !!\n\r");
			status = PMU_STATUS_FAIL;
		}
	}

#endif
	battery_log(BAT_LOG_CRTI, "[BATTERY] JEITA status is %d\n",status);
	return status;
}


static PMU_STATUS mt_battery_CheckChargerVoltage(void)
{
	PMU_STATUS status = PMU_STATUS_OK;
#if defined(CONFIG_MTK_DUAL_INPUT_CHARGER_SUPPORT)
	unsigned int v_charger_max = DISO_data.hv_voltage;
#endif

	if (BMT_status.charger_exist == KAL_TRUE) {
    printk("entery CheckchargerVOl 1 \r\n");
		if (batt_cust_data.v_charger_enable) {
			if (BMT_status.charger_vol <= batt_cust_data.v_charger_min) {
				battery_log(BAT_LOG_CRTI, "[BATTERY]Charger under voltage!!\r\n");
				BMT_status.bat_charging_state = CHR_ERROR;
				status = PMU_STATUS_FAIL;
			}
		}
	printk("entery CheckchargerVOl 2 \r\n");
//#if !defined(CONFIG_MTK_DUAL_INPUT_CHARGER_SUPPORT)
		if (BMT_status.charger_vol >= batt_cust_data.v_charger_max) {
//#else
		//if (BMT_status.charger_vol >= v_charger_max) {
//#endif
			BMT_status.charger_protect_status = charger_OVER_VOL;
			BMT_status.bat_charging_state = CHR_ERROR;
			status = PMU_STATUS_FAIL;
			printk("[BATTERY]charger_vol %d >max %d Charger over voltage,protect_status=%d !!\r\n", BMT_status.charger_vol,batt_cust_data.v_charger_max,BMT_status.charger_protect_status);
		} else if ((BMT_status.charger_vol > 5600) && (BMT_status.charger_protect_status == charger_OVER_VOL)) {
			printk("hold not charge BMT_status.charger_vol=%d \r\n",BMT_status.charger_vol);
			status = PMU_STATUS_FAIL;
		} else 
			BMT_status.charger_protect_status = 0;
		printk("[BATTERY]charger_vol %d < max %d Charger over voltage,protect_status=%d,recovery chaging!!\r\n", BMT_status.charger_vol,batt_cust_data.v_charger_max,BMT_status.charger_protect_status);
	}

	return status;
}


static PMU_STATUS mt_battery_CheckChargingTime(void)
{
	PMU_STATUS status = PMU_STATUS_OK;

	if ((g_battery_thermal_throttling_flag == 2) || (g_battery_thermal_throttling_flag == 3)) {
		battery_log(BAT_LOG_FULL,
			    "[TestMode] Disable Safety Timer. bat_tt_enable=%d, bat_thr_test_mode=%d, bat_thr_test_value=%d\n",
			    g_battery_thermal_throttling_flag,
			    battery_cmd_thermal_test_mode, battery_cmd_thermal_test_mode_value);

	} else {
		/* Charging OT */
		if (BMT_status.total_charging_time >= MAX_CHARGING_TIME) {
			battery_log(BAT_LOG_CRTI, "[BATTERY] Charging Over Time.\n");

			status = PMU_STATUS_FAIL;
		}
	}

	return status;

}

#if defined(STOP_CHARGING_IN_TAKLING)
static PMU_STATUS mt_battery_CheckCallState(void)
{
	PMU_STATUS status = PMU_STATUS_OK;

	if ((g_call_state == CALL_ACTIVE)
	    && (BMT_status.bat_vol > batt_cust_data.v_cc2topoff_thres))
		status = PMU_STATUS_FAIL;

	return status;
}
#endif

static void mt_battery_CheckBatteryStatus(void)
{
	battery_log(BAT_LOG_FULL, "[mt_battery_CheckBatteryStatus] cmd_discharging=(%d)\n",
		    cmd_discharging);
	if (cmd_discharging == 1) {
		battery_log(BAT_LOG_CRTI,
			    "[mt_battery_CheckBatteryStatus] cmd_discharging=(%d)\n",
			    cmd_discharging);
		BMT_status.bat_charging_state = CHR_ERROR;
		battery_charging_control(CHARGING_CMD_SET_ERROR_STATE, &cmd_discharging);
		return;
	} else if (cmd_discharging == 0) {
		BMT_status.bat_charging_state = CHR_PRE;
		battery_charging_control(CHARGING_CMD_SET_ERROR_STATE, &cmd_discharging);
		cmd_discharging = -1;
	}
	if (mt_battery_CheckBatteryTemp() != PMU_STATUS_OK) {
		BMT_status.bat_charging_state = CHR_ERROR;
		return;
	}

	if (mt_battery_CheckChargerVoltage() != PMU_STATUS_OK) {
		BMT_status.bat_charging_state = CHR_ERROR;
		return;
	}
#if defined(STOP_CHARGING_IN_TAKLING)
	if (mt_battery_CheckCallState() != PMU_STATUS_OK) {
		BMT_status.bat_charging_state = CHR_HOLD;
		return;
	}
#endif

	if (mt_battery_CheckChargingTime() != PMU_STATUS_OK) {
		BMT_status.bat_charging_state = CHR_ERROR;
		return;
	}
	if (BMT_status.bat_charging_state == CHR_ERROR) {
		BMT_status.bat_charging_state = CHR_PRE;
		battery_log(BAT_LOG_CRTI,
				    "[mt_battery_CheckBatteryStatus] cmd_discharging=(%d)\n",
				    cmd_discharging);
	}
}


static void mt_battery_notify_TotalChargingTime_check(void)
{
#if defined(BATTERY_NOTIFY_CASE_0005_TOTAL_CHARGINGTIME)
	if ((g_battery_thermal_throttling_flag == 2) || (g_battery_thermal_throttling_flag == 3)) {
		battery_log(BAT_LOG_FULL, "[TestMode] Disable Safety Timer : no UI display\n");
	} else {
		if (BMT_status.total_charging_time >= MAX_CHARGING_TIME)
			/* if(BMT_status.total_charging_time >= 60) //test */
		{
			g_BatteryNotifyCode |= 0x0010;
			battery_log(BAT_LOG_CRTI, "[BATTERY] Charging Over Time\n");
		} else {
			g_BatteryNotifyCode &= ~(0x0010);
		}
	}

	battery_log(BAT_LOG_CRTI,
		    "[BATTERY] BATTERY_NOTIFY_CASE_0005_TOTAL_CHARGINGTIME (%x)\n",
		    g_BatteryNotifyCode);
#endif
}


static void mt_battery_notify_VBat_check(void)
{
#if defined(BATTERY_NOTIFY_CASE_0004_VBAT)
	if (BMT_status.bat_vol > 4400)
		/* if(BMT_status.bat_vol > 3800) //test */
	{
		g_BatteryNotifyCode |= 0x0008;
		battery_log(BAT_LOG_CRTI, "[BATTERY] bat_vlot(%ld) > 4400mV\n", BMT_status.bat_vol);
	} else {
		g_BatteryNotifyCode &= ~(0x0008);
	}

	battery_log(BAT_LOG_CRTI, "[BATTERY] BATTERY_NOTIFY_CASE_0004_VBAT (%x)\n",
		    g_BatteryNotifyCode);

#endif
}


static void mt_battery_notify_ICharging_check(void)
{
#if defined(BATTERY_NOTIFY_CASE_0003_ICHARGING)
	if ((BMT_status.ICharging > 1000) && (BMT_status.total_charging_time > 300)) {
		g_BatteryNotifyCode |= 0x0004;
		battery_log(BAT_LOG_CRTI, "[BATTERY] I_charging(%ld) > 1000mA\n",
			    BMT_status.ICharging);
	} else {
		g_BatteryNotifyCode &= ~(0x0004);
	}

	battery_log(BAT_LOG_CRTI, "[BATTERY] BATTERY_NOTIFY_CASE_0003_ICHARGING (%x)\n",
		    g_BatteryNotifyCode);

#endif
}


static void mt_battery_notify_VBatTemp_check(void)
{
#if defined(BATTERY_NOTIFY_CASE_0002_VBATTEMP)
if ((BMT_status.temperature>=TEMP_POS_45_THRESHOLD) && (BMT_status.temperature<=TEMP_POS_60_THRESHOLD) && (BMT_status.bat_vol >= 4100)) {
		// [45-60)
		g_BatteryNotifyCode |= 0x0002;
		printk("[BATTERY] bat_temp(%d) at [45, 60),and BMT_status.bat_vol=%d >= 4100 \n",
				    BMT_status.temperature,BMT_status.bat_vol);
	} else if (BMT_status.temperature > TEMP_POS_60_THRESHOLD) {
		g_BatteryNotifyCode |= 0x0060;
		#ifdef CONFIG_HUAWEI_DSM
		dms_chg_monitor_report_err(DSM_TEMP_HIGH_SHUTDOWN_NO);
		#endif
		battery_log(BAT_LOG_CRTI, "[BATTERY] bat_temp(%d) out of range(too high)\n",
			    BMT_status.temperature);
	}
#if defined(CONFIG_MTK_JEITA_STANDARD_SUPPORT)
	else if ((BMT_status.temperature < MIN_CHARGE_TEMPERATURE) && (BMT_status.charger_exist == KAL_TRUE)) {
		g_BatteryNotifyCode |= 0x0020;
		battery_log(BAT_LOG_CRTI, "[BATTERY] bat_temp(%d) out of range(too low)\n",
				    BMT_status.temperature);
	}else if ((BMT_status.temperature <= -20) && (BMT_status.charger_exist != KAL_TRUE)){
		g_BatteryNotifyCode |= 0x0080;
		printk("[BATTERY] bat_temp(%d) out of range (,-20] (too low)\n",
			    BMT_status.temperature);
	}
#else
#ifdef BAT_LOW_TEMP_PROTECT_ENABLE
	else if ((BMT_status.temperature < MIN_CHARGE_TEMPERATURE)&& (BMT_status.charger_exist == KAL_TRUE)) {
		g_BatteryNotifyCode |= 0x0020;
		battery_log(BAT_LOG_CRTI, "[BATTERY] bat_temp(%d) out of range(too low)\n",
			    BMT_status.temperature);
	}else if ((BMT_status.temperature <= -20) && (BMT_status.charger_exist != KAL_TRUE)){
		g_BatteryNotifyCode |= 0x0080;
		printk("[BATTERY] bat_temp(%d) out of range (,-20] (too low)\n",
				    BMT_status.temperature);
	}
#endif
#endif

	battery_log(BAT_LOG_FULL, "[BATTERY] BATTERY_NOTIFY_CASE_0002_VBATTEMP (%x)\n",
		    g_BatteryNotifyCode);

#endif
}


static void mt_battery_notify_VCharger_check(void)
{
#ifdef CONFIG_MTK_BQ24196_SUPPORT
	unsigned int charging_enable=KAL_FALSE;
#endif
#if defined(BATTERY_NOTIFY_CASE_0001_VCHARGER)
#if defined(CONFIG_MTK_DUAL_INPUT_CHARGER_SUPPORT)
	unsigned int v_charger_max = DISO_data.hv_voltage;
#endif

#if !defined(CONFIG_MTK_DUAL_INPUT_CHARGER_SUPPORT)
	if (BMT_status.charger_vol > batt_cust_data.v_charger_max) {
#else
	if (BMT_status.charger_vol > v_charger_max) {
#endif
	g_BatteryNotifyCode |= 0x0001;
#ifdef CONFIG_MTK_BQ24196_SUPPORT
		battery_charging_control(CHARGING_CMD_ENABLE, &charging_enable);
#endif
		battery_log(BAT_LOG_CRTI, "[BATTERY] BMT_status.charger_vol(%d) > %d mV\n",
			    BMT_status.charger_vol, batt_cust_data.v_charger_max);
	} else {
		g_BatteryNotifyCode &= ~(0x0001);
	}
	if (g_BatteryNotifyCode != 0x0000)
		battery_log(BAT_LOG_CRTI,
			    "[BATTERY] BATTERY_NOTIFY_CASE_0001_VCHARGER (%x)\n",
			    g_BatteryNotifyCode);
#endif
}


static void mt_battery_notify_UI_test(void)
{
	if (g_BN_TestMode == 0x0001) {
		g_BatteryNotifyCode = 0x0001;
		battery_log(BAT_LOG_CRTI, "[BATTERY_TestMode] BATTERY_NOTIFY_CASE_0001_VCHARGER\n");
	} else if (g_BN_TestMode == 0x0002) {
		g_BatteryNotifyCode = 0x0002;
		battery_log(BAT_LOG_CRTI, "[BATTERY_TestMode] BATTERY_NOTIFY_CASE_0002_VBATTEMP\n");
	} else if (g_BN_TestMode == 0x0003) {
		g_BatteryNotifyCode = 0x0004;
		battery_log(BAT_LOG_CRTI,
			    "[BATTERY_TestMode] BATTERY_NOTIFY_CASE_0003_ICHARGING\n");
	} else if (g_BN_TestMode == 0x0004) {
		g_BatteryNotifyCode = 0x0008;
		battery_log(BAT_LOG_CRTI, "[BATTERY_TestMode] BATTERY_NOTIFY_CASE_0004_VBAT\n");
	} else if (g_BN_TestMode == 0x0005) {
		g_BatteryNotifyCode = 0x0010;
		battery_log(BAT_LOG_CRTI,
			    "[BATTERY_TestMode] BATTERY_NOTIFY_CASE_0005_TOTAL_CHARGINGTIME\n");
	} else {
		battery_log(BAT_LOG_CRTI, "[BATTERY] Unknown BN_TestMode Code : %x\n",
			    g_BN_TestMode);
	}
}


void mt_battery_notify_check(void)
{
	g_BatteryNotifyCode = 0x0000;

	if (g_BN_TestMode == 0x0000) {	/* for normal case */
		battery_log(BAT_LOG_FULL, "[BATTERY] mt_battery_notify_check\n");

		mt_battery_notify_VCharger_check();

		mt_battery_notify_VBatTemp_check();

		mt_battery_notify_ICharging_check();

		mt_battery_notify_VBat_check();

		mt_battery_notify_TotalChargingTime_check();
	} else {		/* for UI test */

		mt_battery_notify_UI_test();
	}
}

static void mt_battery_thermal_check(void)
{
	if ((g_battery_thermal_throttling_flag == 1) || (g_battery_thermal_throttling_flag == 3)) {
		if (battery_cmd_thermal_test_mode == 1) {
			BMT_status.temperature = battery_cmd_thermal_test_mode_value;
			battery_log(BAT_LOG_FULL,
				    "[Battery] In thermal_test_mode , Tbat=%d\n",
				    BMT_status.temperature);
		}
#if defined(CONFIG_MTK_JEITA_STANDARD_SUPPORT)
		/* ignore default rule */
#else
		if (BMT_status.temperature >= 60) {
#if defined(CONFIG_POWER_EXT)
			battery_log(BAT_LOG_CRTI,
				    "[BATTERY] CONFIG_POWER_EXT, no update battery update power down.\n");
#else
			{
				if ((g_platform_boot_mode == META_BOOT)
				    || (g_platform_boot_mode == ADVMETA_BOOT)
				    || (g_platform_boot_mode == ATE_FACTORY_BOOT)) {
					battery_log(BAT_LOG_FULL,
						    "[BATTERY] boot mode = %d, bypass temperature check\n",
						    g_platform_boot_mode);
				} else {
					struct battery_data *bat_data = &battery_main;
					struct power_supply *bat_psy = &bat_data->psy;

					battery_log(BAT_LOG_CRTI,
						    "[Battery] Tbat(%d)>=60, system need power down.\n",
						    BMT_status.temperature);

					bat_data->BAT_CAPACITY = 0;

					power_supply_changed(bat_psy);

					if (BMT_status.charger_exist == KAL_TRUE) {
						/* can not power down due to charger exist, so need reset system */
						battery_charging_control
						    (CHARGING_CMD_SET_PLATFORM_RESET, NULL);
					}
					/* avoid SW no feedback */
					battery_charging_control(CHARGING_CMD_SET_POWER_OFF, NULL);
					/* mt_power_off(); */
				}
			}
#endif
		}
#endif

	}

}


static void mt_battery_update_status(void)
{
#if defined(CONFIG_POWER_EXT)
	battery_log(BAT_LOG_CRTI, "[BATTERY] CONFIG_POWER_EXT, no update Android.\n");
#else
	{
		if (skip_battery_update == KAL_FALSE) {
			battery_log(BAT_LOG_FULL, "mt_battery_update_status.\n");
			usb_update(&usb_main);
			ac_update(&ac_main);
			wireless_update(&wireless_main);
			battery_update(&battery_main);
		} else {
			battery_log(BAT_LOG_FULL, "skip mt_battery_update_status.\n");
			battery_update(&battery_main);
			skip_battery_update = KAL_FALSE;
		}
	}

#endif
}


CHARGER_TYPE mt_charger_type_detection(void)
{
	CHARGER_TYPE CHR_Type_num = CHARGER_UNKNOWN;

	mutex_lock(&charger_type_mutex);

#if defined(CONFIG_MTK_WIRELESS_CHARGER_SUPPORT)
	battery_charging_control(CHARGING_CMD_GET_CHARGER_TYPE, &CHR_Type_num);
	BMT_status.charger_type = CHR_Type_num;
#else
#if !defined(CONFIG_MTK_DUAL_INPUT_CHARGER_SUPPORT)
	if ((BMT_status.charger_type == CHARGER_UNKNOWN)||(BMT_status.charger_type == NONSTANDARD_CHARGER)) {
#else
	if ((BMT_status.charger_type == CHARGER_UNKNOWN) &&
	    (DISO_data.diso_state.cur_vusb_state == DISO_ONLINE)) {
#endif
		battery_charging_control(CHARGING_CMD_GET_CHARGER_TYPE, &CHR_Type_num);
		BMT_status.charger_type = CHR_Type_num;

#if defined(CONFIG_MTK_KERNEL_POWER_OFF_CHARGING) &&\
	(defined(CONFIG_MTK_PUMP_EXPRESS_SUPPORT) || defined(CONFIG_MTK_PUMP_EXPRESS_PLUS_SUPPORT))
		if (BMT_status.UI_SOC == 100) {
			BMT_status.bat_charging_state = CHR_BATFULL;
			BMT_status.bat_full = KAL_TRUE;
			g_charging_full_reset_bat_meter = KAL_TRUE;
		}

		if (g_battery_soc_ready == KAL_FALSE) {
			if (BMT_status.nPercent_ZCV == 0)
				battery_meter_initial();

			BMT_status.SOC = battery_meter_get_battery_percentage();
		}

		if (BMT_status.bat_vol > 0)
			mt_battery_update_status();

#endif
	}
#endif
	mutex_unlock(&charger_type_mutex);

	return BMT_status.charger_type;
}

CHARGER_TYPE mt_get_charger_type(void)
{
#if defined(CONFIG_POWER_EXT) || defined(CONFIG_MTK_FPGA)
	return STANDARD_HOST;
#else
	return BMT_status.charger_type;
#endif
}

static void mt_battery_charger_detect_check(void)
{
static int nonstand_cnt = 0;
static kal_bool nostandard_charger_dpm=KAL_FALSE;//wcl add bpm slow insert
#ifdef CONFIG_MTK_BQ25896_SUPPORT
/*New low power feature of MT6531: disable charger CLK without CHARIN.
* MT6351 API abstracted in charging_hw_bw25896.c. Any charger with MT6351 needs to set this.
* Compile option is not limited to CONFIG_MTK_BQ25896_SUPPORT.
* PowerDown = 0
*/
		unsigned int pwr;
#endif
	if (upmu_is_chr_det() == KAL_TRUE) {
		wake_lock(&battery_suspend_lock);

#if !defined(CONFIG_MTK_DUAL_INPUT_CHARGER_SUPPORT)
		BMT_status.charger_exist = KAL_TRUE;
#endif

#if defined(CONFIG_MTK_WIRELESS_CHARGER_SUPPORT)
		mt_charger_type_detection();

		if ((BMT_status.charger_type == STANDARD_HOST)
		    || (BMT_status.charger_type == CHARGING_HOST)) {
			mt_usb_connect();
		}
#else
#if !defined(CONFIG_MTK_DUAL_INPUT_CHARGER_SUPPORT)
		if ((BMT_status.charger_type == CHARGER_UNKNOWN) || 
			((BMT_status.charger_type == NONSTANDARD_CHARGER) && (nonstand_cnt <= 5))) {
		#else
		if ((BMT_status.charger_type == CHARGER_UNKNOWN) &&
		    (DISO_data.diso_state.cur_vusb_state == DISO_ONLINE)) {
#endif
			printk("<check before> chrg_type :%d, cnt:%d\n", BMT_status.charger_type, nonstand_cnt);
			if (nonstand_cnt++ > 6)
				nonstand_cnt = 6;
#ifdef HUAWEI_DYNAMIC_CHARGER
			if(BMT_status.charger_type == NONSTANDARD_CHARGER){
				nostandard_charger_dpm=KAL_TRUE;
			}
#endif
			mt_charger_type_detection();
#ifdef HUAWEI_DYNAMIC_CHARGER
			if((nostandard_charger_dpm == KAL_TRUE)&&(BMT_status.charger_type == STANDARD_CHARGER)){
				nostandard_charger_dpm=KAL_FALSE;
				dpm_first_select_cc= KAL_TRUE;
				find_dpm_cc= KAL_FALSE;
				up_status= KAL_FALSE;
				down_status= KAL_FALSE;
				cc_rank_value= 2; 	
				printk("entery dpm slow insert reset dpm parameter,get standard cc value;nostandard_charger_dpm=%d,BMT_status.charger_type=%d \n",nostandard_charger_dpm,BMT_status.charger_type);
			}//wcl add dpm slow insert
#endif		
			printk("<check after> chrg_type :%d, cnt:%d\n", BMT_status.charger_type, nonstand_cnt);

			if(BMT_status.charger_type==CHARGING_HOST)//ADD
				mt_charger_type_detection();//ADD 

			if ((BMT_status.charger_type == STANDARD_HOST)
			    || (BMT_status.charger_type == CHARGING_HOST)) {
				mt_usb_connect();
			}
		}
#endif

#ifdef CONFIG_MTK_BQ25896_SUPPORT
/*New low power feature of MT6531: disable charger CLK without CHARIN.
* MT6351 API abstracted in charging_hw_bw25896.c. Any charger with MT6351 needs to set this.
* Compile option is not limited to CONFIG_MTK_BQ25896_SUPPORT.
* PowerDown = 0
*/
		pwr = 0;
		battery_charging_control(CHARGING_CMD_SET_CHRIND_CK_PDN, &pwr);
#endif

		battery_log(BAT_LOG_CRTI, "[BAT_thread]Cable in, CHR_Type_num=%d\r\n",
			    BMT_status.charger_type);

#if defined(CONFIG_MTK_PUMP_EXPRESS_PLUS_SUPPORT)
		/* is_ta_connect = KAL_FALSE; */
		/* ta_check_chr_type = KAL_TRUE; */
		ta_cable_out_occur = KAL_FALSE;
		battery_log(BAT_LOG_CRTI, "[PE+] Cable In\n");
#endif

	} else {
		wake_unlock(&battery_suspend_lock);

		BMT_status.charger_exist = KAL_FALSE;
		BMT_status.charger_type = CHARGER_UNKNOWN;
		BMT_status.bat_full = KAL_FALSE;
		BMT_status.bat_in_recharging_state = KAL_FALSE;
		BMT_status.bat_charging_state = CHR_PRE;
		BMT_status.total_charging_time = 0;
		BMT_status.PRE_charging_time = 0;
		BMT_status.CC_charging_time = 0;
		BMT_status.TOPOFF_charging_time = 0;
		BMT_status.POSTFULL_charging_time = 0;
		nonstand_cnt = 0;
		battery_log(BAT_LOG_FULL, "[BAT_thread]Cable out \r\n");

		mt_usb_disconnect();

#if defined(CONFIG_MTK_PUMP_EXPRESS_PLUS_SUPPORT)
		is_ta_connect = KAL_FALSE;
		ta_check_chr_type = KAL_TRUE;
		ta_cable_out_occur = KAL_TRUE;
		battery_log(BAT_LOG_FULL, "[PE+] Cable OUT\n");
#endif

#ifdef CONFIG_MTK_BQ25896_SUPPORT
/*New low power feature of MT6531: disable charger CLK without CHARIN.
* MT6351 API abstracted in charging_hw_bw25896.c. Any charger with MT6351 needs to set this.
* Compile option is not limited to CONFIG_MTK_BQ25896_SUPPORT.
* PowerDown = 1
*/
		pwr = 1;
		battery_charging_control(CHARGING_CMD_SET_CHRIND_CK_PDN, &pwr);
#endif
	}
}

static void mt_kpoc_power_off_check(void)
{
#ifdef CONFIG_MTK_KERNEL_POWER_OFF_CHARGING
	battery_log(BAT_LOG_FULL,
		    "[mt_kpoc_power_off_check] , chr_vol=%d, boot_mode=%d\r\n",
		    BMT_status.charger_vol, g_platform_boot_mode);
	if (g_platform_boot_mode == KERNEL_POWER_OFF_CHARGING_BOOT
	    || g_platform_boot_mode == LOW_POWER_OFF_CHARGING_BOOT) {
		if ((upmu_is_chr_det() == KAL_FALSE) && (BMT_status.charger_vol < 2500)) {	/* vbus < 2.5V */
			battery_log(BAT_LOG_CRTI,
				    "[bat_thread_kthread] Unplug Charger/USB In Kernel Power Off Charging Mode!  Shutdown OS!\r\n");
			battery_charging_control(CHARGING_CMD_SET_POWER_OFF, NULL);
		}
	}
#endif
}

void update_battery_2nd_info(int status_smb, int capacity_smb, int present_smb)
{
#if defined(CONFIG_POWER_VERIFY)
	battery_log(BAT_LOG_CRTI, "[update_battery_smb_info] no support\n");
#else
	g_status_smb = status_smb;
	g_capacity_smb = capacity_smb;
	g_present_smb = present_smb;
	battery_log(BAT_LOG_CRTI,
		    "[update_battery_smb_info] get status_smb=%d,capacity_smb=%d,present_smb=%d\n",
		    status_smb, capacity_smb, present_smb);

	wake_up_bat();
	g_smartbook_update = 1;
#endif
}
#if defined (CONFIG_MTK_BQ24196_SUPPORT)
extern void bq24196_set_en_hiz(unsigned int val);
#endif
void do_chrdet_int_task(void)
{
	if (g_bat_init_flag == KAL_TRUE) {
#if !defined(CONFIG_MTK_DUAL_INPUT_CHARGER_SUPPORT)
		if (upmu_is_chr_det() == KAL_TRUE) {
#else
		battery_charging_control(CHARGING_CMD_GET_DISO_STATE, &DISO_data);
		if ((DISO_data.diso_state.cur_vusb_state == DISO_ONLINE) ||
		    (DISO_data.diso_state.cur_vdc_state == DISO_ONLINE)) {
#endif
			first_insert_charger=2;
			battery_log(BAT_LOG_CRTI, "[do_chrdet_int_task] charger exist!\n");
#ifdef CONFIG_LOG_JANK
			LOG_JANK_D(JLID_USBCHARGING_START,"%s","JL_USBCHARGING_START");
#endif
			BMT_status.charger_exist = KAL_TRUE;

			wake_lock(&battery_suspend_lock);

#if defined(CONFIG_POWER_EXT)
			mt_usb_connect();
			battery_log(BAT_LOG_CRTI,
				    "[do_chrdet_int_task] call mt_usb_connect() in EVB\n");
#elif defined(CONFIG_MTK_POWER_EXT_DETECT)
			if (KAL_TRUE == bat_is_ext_power()) {
				mt_usb_connect();
				battery_log(BAT_LOG_CRTI,
					    "[do_chrdet_int_task] call mt_usb_connect() in EVB\n");
				return;
			}
#endif
		} else {
			battery_log(BAT_LOG_CRTI, "[do_chrdet_int_task] charger NOT exist!\n");
#ifdef CONFIG_LOG_JANK
			LOG_JANK_D(JLID_USBCHARGING_END,"%s","JLID_USBCHARGING_END");
#endif
			BMT_status.charger_exist = KAL_FALSE;
#if defined (CONFIG_MTK_BQ24196_SUPPORT)
			bq24196_set_en_hiz(0x0);
#endif
#if defined(CONFIG_MTK_DUAL_INPUT_CHARGER_SUPPORT)
			battery_log(BAT_LOG_CRTI,
				    "turn off charging for no available charging source\n");
			battery_charging_control(CHARGING_CMD_ENABLE, &BMT_status.charger_exist);
#endif

#ifdef CONFIG_MTK_KERNEL_POWER_OFF_CHARGING
			if (g_platform_boot_mode == KERNEL_POWER_OFF_CHARGING_BOOT
			    || g_platform_boot_mode == LOW_POWER_OFF_CHARGING_BOOT) {
				battery_log(BAT_LOG_CRTI,
					    "[pmic_thread_kthread] Unplug Charger/USB In Kernel Power Off Charging Mode!  Shutdown OS!\r\n");
				battery_charging_control(CHARGING_CMD_SET_POWER_OFF, NULL);
				/* mt_power_off(); */
			}
#endif

			wake_unlock(&battery_suspend_lock);

#if defined(CONFIG_POWER_EXT)
			mt_usb_disconnect();
			battery_log(BAT_LOG_CRTI,
				    "[do_chrdet_int_task] call mt_usb_disconnect() in EVB\n");
#elif defined(CONFIG_MTK_POWER_EXT_DETECT)
			if (KAL_TRUE == bat_is_ext_power()) {
				mt_usb_disconnect();
				battery_log(BAT_LOG_CRTI,
					    "[do_chrdet_int_task] call mt_usb_disconnect() in EVB\n");
				return;
			}
#endif
#if defined(CONFIG_MTK_PUMP_EXPRESS_SUPPORT) || defined(CONFIG_MTK_PUMP_EXPRESS_PLUS_SUPPORT)
			is_ta_connect = KAL_FALSE;
			ta_check_chr_type = KAL_TRUE;
			ta_cable_out_occur = KAL_TRUE;
#endif

		}

		/* Place charger detection and battery update here is used to speed up charging icon display. */

		mt_battery_charger_detect_check();
		if (BMT_status.UI_SOC == 100 && BMT_status.charger_exist == KAL_TRUE) {
			BMT_status.bat_charging_state = CHR_BATFULL;
			BMT_status.bat_full = KAL_TRUE;
			g_charging_full_reset_bat_meter = KAL_TRUE;
		}

		if (g_battery_soc_ready == KAL_FALSE) {
			if (BMT_status.nPercent_ZCV == 0)
				battery_meter_initial();

			BMT_status.SOC = battery_meter_get_battery_percentage();
		}

		if (BMT_status.bat_vol > 0) {
			mt_battery_update_status();
			skip_battery_update = KAL_TRUE;

		}
#if defined(CONFIG_MTK_DUAL_INPUT_CHARGER_SUPPORT)
		DISO_data.chr_get_diso_state = KAL_TRUE;
#endif

		wake_up_bat();
	} else {
#if defined(CONFIG_MTK_DUAL_INPUT_CHARGER_SUPPORT)
		g_vcdt_irq_delay_flag = KAL_TRUE;
#endif
		battery_log(BAT_LOG_CRTI,
			    "[do_chrdet_int_task] battery thread not ready, will do after bettery init.\n");
	}

}
extern int is_reconstruct_zcv;
extern void dodprint(void);
extern signed int fgauge_read_v_by_capacity(int bat_capacity);
unsigned int flag_call = 1;
void BAT_thread(void)
{
	static kal_bool battery_meter_initilized = KAL_FALSE;

	if (battery_meter_initilized == KAL_FALSE) {
		battery_meter_initial();	/* move from battery_probe() to decrease booting time */
		fifty_percent_zcv = fgauge_read_v_by_capacity(100 - fifty_percent_check_point);//65%
		BMT_status.nPercent_ZCV = battery_meter_get_battery_nPercent_zcv();
		three_percent_zcv = fgauge_read_v_by_capacity(100 - three_percent_check_point);
		battery_meter_initilized = KAL_TRUE;
#if defined(CONFIG_POWER_EXT)
#else
		BMT_status.SOC = battery_meter_get_battery_percentage();
		BMT_status.UI_SOC = BMT_status.SOC;
		BMT_status.ZCV = battery_meter_get_battery_zcv();

		BMT_status.temperatureV = battery_meter_get_tempV();
		BMT_status.temperatureR = battery_meter_get_tempR(BMT_status.temperatureV);
		BMT_status.bat_vol = battery_meter_get_battery_voltage(KAL_TRUE);
		BMT_status.temperature = battery_meter_get_battery_temperature();
		battery_update(&battery_main);
#endif

	}
		if (is_reconstruct_zcv) {
		BMT_status.nPercent_ZCV = fgauge_read_v_by_capacity(100 - BMT_status.nPrecent_UI_SOC_check_point);
		three_percent_zcv = fgauge_read_v_by_capacity(100 - three_percent_check_point);
		fifty_percent_zcv = fgauge_read_v_by_capacity(100 - fifty_percent_check_point);
		is_reconstruct_zcv = 0;
		}
	battery_log(BAT_LOG_CRTI,"[%s] ZCV:%d and UI SOC:%d, then keep BMT_status.nPercent_ZCV:%d and nPrecent_UI_SOC:%d\n",
						__func__, BMT_status.ZCV,  BMT_status.UI_SOC,BMT_status.nPercent_ZCV,BMT_status.nPrecent_UI_SOC_check_point);
	dodprint();
	mt_battery_charger_detect_check();
	mt_battery_GetBatteryData();
	if (BMT_status.charger_exist == KAL_TRUE)
		check_battery_exist();

	mt_battery_thermal_check();
	mt_battery_notify_check();

	if (BMT_status.charger_exist == KAL_TRUE) {
		mt_battery_CheckBatteryStatus();
#if defined(HQ_CHARGER_FOR_HUAWEI)  //add current test start 	
       if (!g_charger_should_not_open)	
#endif  //end 
		mt_battery_charging_algorithm();
#ifdef HUAWEI_DYNAMIC_CHARGER 
	}else{
		printk("[battery] not chargign Huawei dpm variable reset,charger_exist=%d \n",BMT_status.charger_exist);
		dpm_first_select_cc= KAL_TRUE;
		find_dpm_cc= KAL_FALSE;
		up_status= KAL_FALSE;
		down_status= KAL_FALSE;
		cc_rank_value= 2; 
	}
#else
	}
#endif
#ifdef CONFIG_HUAWEI_DSM
	bat_update_thread_dsm();
#endif
	if(first_insert_charger){
	mt_battery_GetBatteryData();
	first_insert_charger--;
	}
	mt_battery_update_status();
	mt_kpoc_power_off_check();
}

/* ///////////////////////////////////////////////////////////////////////////////////////// */
/* // Internal API */
/* ///////////////////////////////////////////////////////////////////////////////////////// */

#ifdef BATTERY_CDP_WORKAROUND2
/*extern kal_bool is_usb_rdy(void);*/
#endif

int bat_thread_kthread(void *x)
{
	ktime_t ktime = ktime_set(3, 0);	/* 10s, 10* 1000 ms */

#ifdef BATTERY_CDP_WORKAROUND2
	if (is_usb_rdy() == KAL_FALSE) {
		battery_log(BAT_LOG_CRTI, "CDP, block\n");
		wait_event(bat_thread_wq, (is_usb_rdy() == KAL_TRUE));
		battery_log(BAT_LOG_CRTI, "CDP, free\n");
	} else {
		battery_log(BAT_LOG_CRTI, "CDP, PASS\n");
	}
#endif


	/* Run on a process content */
	while (1) {
		mutex_lock(&bat_mutex);

		if (((chargin_hw_init_done == KAL_TRUE) && (battery_suspended == KAL_FALSE))
		    || ((chargin_hw_init_done == KAL_TRUE) && (fg_wake_up_bat == KAL_TRUE)))
			{
	                BAT_thread();
			if(first_insert_charger){
				msleep(3);
				BAT_thread();
				}
			}
		mutex_unlock(&bat_mutex);

#ifdef FG_BAT_INT
		if (fg_wake_up_bat == KAL_TRUE) {
			wake_unlock(&battery_fg_lock);
			fg_wake_up_bat = KAL_FALSE;
			battery_log(BAT_LOG_CRTI, "unlock battery_fg_lock\n");
		}
#endif				/* #ifdef FG_BAT_INT */

		battery_log(BAT_LOG_FULL, "wait event\n");

		wait_event(bat_thread_wq, (bat_thread_timeout == KAL_TRUE));

		bat_thread_timeout = KAL_FALSE;
		hrtimer_start(&battery_kthread_timer, ktime, HRTIMER_MODE_REL);
		ktime = ktime_set(BAT_TASK_PERIOD, 0);	/* 10s, 10* 1000 ms */
		if (chr_wake_up_bat == KAL_TRUE && g_smartbook_update != 1) {	/* for charger plug in/ out */
			printk("entery chr_wake_up_bat conditions1\n");
#if defined(CONFIG_MTK_DUAL_INPUT_CHARGER_SUPPORT)
			if (DISO_data.chr_get_diso_state) {
				DISO_data.chr_get_diso_state = KAL_FALSE;
				battery_charging_control(CHARGING_CMD_GET_DISO_STATE, &DISO_data);
			}
#endif

			g_smartbook_update = 0;
#if defined(CUST_CAPACITY_OCV2CV_TRANSFORM)
			battery_meter_set_reset_soc(KAL_FALSE);
#endif
			battery_meter_reset();
			printk("entery battery_meter_reset2 soc=%d \n",BMT_status.SOC);
			chr_wake_up_bat = KAL_FALSE;

			battery_log(BAT_LOG_CRTI,
				    "[BATTERY] Charger plug in/out, Call battery_meter_reset. (%d)\n",
				    BMT_status.UI_SOC);
		}

	}

	return 0;
}

#ifdef CONFIG_HUAWEI_DSM
static int battery_vbus_dsm = 0;
static int battery_vbat_dsm = 0;
static int battery_capacity_dsm = 0;
static int battery_ac_dsm = 0;
static int battery_usb_dsm = 0;
static int battery_present_dsm = 0;
static int battery_temp_dsm = 0;
static int battery_charge_current = 0;
static int battery_status_dsm = 0;
static int batt_pre_temp = 0;
static int batt_pre_capacity = 0;
static int dsm_i = 0;
static int dsm_j = 0;
static int charging_ibus = 0;

static int dms_chg_monitor_report_err(int errno)
{
       int size = 0;

       if(dsm_client_ocuppy(bq_power_dclient)){
               /* buffer is busy */
               battery_log(BAT_LOG_CRTI,"%s: buffer is busy!, errno = %d\n", __func__,errno);
               return -EBUSY;
       }
       switch (errno)
       {
               case DSM_CHARGER_MT_ERROR_NO:
                       dsm_client_record(bq_power_dclient,"Charger current too low\n");
                       break;
               case DSM_CHARGER_MT_OVP_ERROR_NO:
                       dsm_client_record(bq_power_dclient,"Charger OVP happen\n");
                       break;
               case DSM_BATT_MT_EXIST_NOERROR_NO:
                       dsm_client_record(bq_power_dclient,"No battery exist\n");
                       break;
               case DSM_BATT_MT_CAPA_JUMP_ERROR_NO:
                       dsm_client_record(bq_power_dclient,"Batt capacity jump > 10%\n");
                       break;
               case DSM_BATT_MT_CAPA_BOOT_JUMP_ERROR_NO:
                       dsm_client_record(bq_power_dclient,"System turn on  capacity jump > 10%\n");
                       break;
               case DSM_CHARGER_MT_TEMP_COLD_ERROR_NO:
                       dsm_client_record(bq_power_dclient,"Error temp < 0 but charing not stop\n");
                       break;
               case DSM_CHARGER_MT_TEMP_HOT_ERROR_NO:
                       dsm_client_record(bq_power_dclient,"Error temp > 55  but charing not stop\n");
                       break;
               case DSM_UNSTAND_CHG_DETECT_NO:
                       dsm_client_record(bq_power_dclient,"Non-stand charger detect\n");
                       break;
               case DSM_BATT_MT_VOLTAGE_OVER_NO:
                       dsm_client_record(bq_power_dclient,"Battery voltage over 4.4V\n");
                       break;
               case DSM_BATT_MT_VOLTAGE_BELOW_NO:
                       dsm_client_record(bq_power_dclient,"Battery voltage below 2.5V\n");
                       break;
               case DSM_CHARGER_MT_I2C_ERROR_NO:
                       dsm_client_record(bq_power_dclient,"BQ24196 i2c acces fail\n");
                       break;
			   case DSM_BATT_MT_ID_ERROR_NO:
                       dsm_client_record(bq_power_dclient,"Unknown battery module\n");
                       break;
               case DSM_CURRENT_MT_STATUS_ERROR_NO:
                       if(battery_status_dsm == POWER_SUPPLY_STATUS_CHARGING)
                               dsm_client_record(bq_power_dclient,"charger current > max-current 2A set\n");
                       else
                               dsm_client_record(bq_power_dclient,"Device system current > max-current set\n");
                       break;
               case DSM_BATT_MT_TEMP_JUMP_NO:
                       dsm_client_record(bq_power_dclient,"Battery temp jump over 5 degree in 30 mins\n");
                       break;
               case DSM_BATT_VOL_LIMIT_ERROR_NO:
                       dsm_client_record(bq_power_dclient,"Battery temp [45,55) cool current limit fail \n");
                       break;
               case DSM_BATT_CURRENT_LIMIT_ERROR_NO:
                       dsm_client_record(bq_power_dclient,"Battery temp (0,10] Warm voltage limit fail \n");
                       break;
               case DSM_BATT_SOC_MISMATCH_VOL_NO:
                       dsm_client_record(bq_power_dclient,"Battery UI_SOC mismatch voltage \n");
                       break;
               case DSM_DISCHARGER_CURRENT_OVER_5A_NO:
                       dsm_client_record(bq_power_dclient,"Battery discharging current over 5A \n");
                       break;
               case DSM_CHARGERIC_ERROR_NO:
                       dsm_client_record(bq_power_dclient,"Charger IC error \n");
                       break;
               case DSM_CHARGER_UI_ERROR_NO:
                       dsm_client_record(bq_power_dclient,"Charger USB UI show abnormal \n");
                       break;
               case DSM_CHARGER_STATUS_ERROR_NO:
                       dsm_client_record(bq_power_dclient,"Charging status UI show abnormal \n");
                       break;
               case DSM_CHARGER_CANTSTOP_VOL_OVER_NO:
                       dsm_client_record(bq_power_dclient,"Charging cant stop when battery_vol over \n");
                       break;
               case DSM_CHARGER_TEMP_BELOW0_ERROR_NO:
                       dsm_client_record(bq_power_dclient,"Battery temp below 0 \n");
                       break;
               case DSM_CHARGER_TEMP_OVER55_ERROR_NO:
                       dsm_client_record(bq_power_dclient,"Battery temp over 55 \n");
                       break;
               case DSM_BATTERY_VOL_HIGH_ERROR_NO:
                       dsm_client_record(bq_power_dclient,"Battery Vol over 4.45V \n");
                       break;
               case DSM_CHARGING_FULL_OVERTIME_NO:
                       dsm_client_record(bq_power_dclient,"Charging_full dont pull out usb more than 2h \n");
               case DSM_DISCHARGING_CMD_FAIL_NO:
                       dsm_client_record(bq_power_dclient,"USB discharging cmd dont work \n");
                       break;
               case DSM_TEMP_HIGH_SHUTDOWN_NO:
                       dsm_client_record(bq_power_dclient,"Battery temp over 60 and shutdown.\n");
               default:
                       break;
       }
       /*if device is not probe successfully or client is null, don't notify dsm work func*/
       dsm_client_notify(bq_power_dclient, errno);

       return size;
}

void mt_battery_update_status_dsm(void)
{
       battery_vbus_dsm = BMT_status.charger_vol;
       battery_vbat_dsm = BMT_status.bat_vol;
       battery_capacity_dsm = BMT_status.UI_SOC2;
       battery_temp_dsm = BMT_status.temperature;
       battery_present_dsm = BMT_status.bat_exist;
       battery_ac_dsm = ac_main.AC_ONLINE;
       battery_usb_dsm = usb_main.USB_ONLINE;
       battery_charge_current = battery_meter_get_battery_current()/10;
       battery_status_dsm  = battery_main.BAT_STATUS;
       battery_charging_control(CHARGING_CMD_GET_CURRENT, &charging_ibus);
}

int mt_battery_charge_status_dsm(void)
{
       int ret=0;
	   if((battery_status_dsm == POWER_SUPPLY_STATUS_CHARGING) && (battery_vbat_dsm < 4100) && (BMT_status.bat_charging_state == CHR_CC))
	   {
		   if((battery_charge_current < 25)&&(battery_charge_current >= 0)){
			   ret =  1;
           }else{
			   ret =  0;
		   }
	   }
	   return ret;
}

int bat_update_thread_dsm(void)
{

		//msleep(70*1000);
		//while(1){
             mt_battery_update_status_dsm();
               if(dsm_i == 0)
               {
                       batt_pre_capacity = battery_capacity_dsm;
					   dsm_i++;
			   }else if(dsm_i >= 6){
				   if((batt_pre_capacity >= 0) && (abs(battery_capacity_dsm - batt_pre_capacity) > 10)){
						  dms_chg_monitor_report_err(DSM_BATT_MT_CAPA_JUMP_ERROR_NO);
						  }
						  dsm_i = 0;
               }else{
                   if(batt_pre_capacity < 0)
                batt_pre_capacity = battery_capacity_dsm;
                       dsm_i++;
			   }

               /* 30 min check temp is jump 10 degree */
			   if(dsm_j == 0){
			    batt_pre_temp = battery_temp_dsm;
			   dsm_j++;
               }else if(dsm_j >= 180) {
                       if (abs(battery_temp_dsm - batt_pre_temp) > 10){
                               dms_chg_monitor_report_err(DSM_BATT_MT_TEMP_JUMP_NO);
                       }
                       dsm_j = 0;
               }else{
                   if(batt_pre_temp == 0)
                batt_pre_temp = battery_temp_dsm;
                       dsm_j++;
               }
               if(mt_battery_charge_status_dsm()) //Charging fail issue
                       dms_chg_monitor_report_err(DSM_CHARGER_MT_ERROR_NO);

               if(!battery_present_dsm) //Battery no-exist
                       dms_chg_monitor_report_err(DSM_BATT_MT_EXIST_NOERROR_NO);

               if((battery_status_dsm == POWER_SUPPLY_STATUS_CHARGING) && (battery_temp_dsm < 0))
                       dms_chg_monitor_report_err(DSM_CHARGER_MT_TEMP_COLD_ERROR_NO);
               if((battery_status_dsm == POWER_SUPPLY_STATUS_CHARGING) && (battery_temp_dsm > 55))
                       dms_chg_monitor_report_err(DSM_CHARGER_MT_TEMP_HOT_ERROR_NO);
               if(BMT_status.charger_type == NONSTANDARD_CHARGER)
                       dms_chg_monitor_report_err(DSM_UNSTAND_CHG_DETECT_NO);
               if(battery_vbat_dsm > 4400)
                       dms_chg_monitor_report_err(DSM_BATT_MT_VOLTAGE_OVER_NO);
               if(battery_vbat_dsm < 2500)
                       dms_chg_monitor_report_err(DSM_BATT_MT_VOLTAGE_BELOW_NO);
               if (((battery_status_dsm == POWER_SUPPLY_STATUS_CHARGING) && (!(battery_ac_dsm||battery_usb_dsm))) \
              ||((battery_status_dsm == POWER_SUPPLY_STATUS_NOT_CHARGING) &&(battery_ac_dsm||battery_ac_dsm)))
                       dms_chg_monitor_report_err(DSM_CHARGER_MT_STATUS_ERROR_NO);

				if((battery_status_dsm == POWER_SUPPLY_STATUS_CHARGING) && (battery_charge_current > 2050)) //Current can't > 2000mA
                        dms_chg_monitor_report_err(DSM_CURRENT_MT_STATUS_ERROR_NO);
                if ((battery_status_dsm == POWER_SUPPLY_STATUS_CHARGING) && (battery_vbus_dsm < 3900))
                       dms_chg_monitor_report_err(DSM_CHARGER_MT_VOL_LOW_ERROR_NO);
				if ((battery_status_dsm == POWER_SUPPLY_STATUS_CHARGING) && (battery_vbus_dsm > 6200)) // > 6V OVP occur
                       dms_chg_monitor_report_err(DSM_CHARGER_MT_OVP_ERROR_NO);
			    if(dsm_batt_mt_id_error)
					dms_chg_monitor_report_err(DSM_BATT_MT_ID_ERROR_NO);

			    if(((0==BMT_status.SOC)&&(BMT_status.ZCV>3600))||((BMT_status.SOC<=2)&&(BMT_status.ZCV>3700)) \
			    ||((BMT_status.SOC<90)&&(BMT_status.ZCV>4350)))
					dms_chg_monitor_report_err(DSM_BATT_SOC_MISMATCH_VOL_NO);
			    if((gFG_current >50000)&&(gFG_Is_Charging==0))
					dms_chg_monitor_report_err(DSM_DISCHARGER_CURRENT_OVER_5A_NO);
			    if(BMT_status.bat_charging_state == CHR_ERROR)
					dms_chg_monitor_report_err(DSM_CHARGER_MT_ERROR_NO);
			    /*if(((BMT_status.charger_exist == KAL_TRUE) && (BMT_status.charger_vol < 200)) ||((BMT_status.charger_exist == KAL_FALSE) && (BMT_status.charger_vol > 200)))
					dms_chg_monitor_report_err(DSM_CHARGER_UI_ERROR_NO);*/
			    if((BMT_status.charger_exist == KAL_FALSE) && (BMT_status.bat_charging_state == CHR_CC))
					dms_chg_monitor_report_err(DSM_CHARGER_STATUS_ERROR_NO);
			    if(((BMT_status.bat_charging_state == CHR_CC) && (BMT_status.ZCV > 4500)) || ((BMT_status.bat_charging_state ==CHR_TOP_OFF) && (BMT_status.ZCV > 4500)))
					dms_chg_monitor_report_err(DSM_CHARGER_CANTSTOP_VOL_OVER_NO);
			    if(battery_temp_dsm < 0)
					dms_chg_monitor_report_err(DSM_CHARGER_TEMP_BELOW0_ERROR_NO);
			    if(battery_temp_dsm > 55)
					dms_chg_monitor_report_err(DSM_CHARGER_TEMP_OVER55_ERROR_NO);
			    if(BMT_status.ZCV > 4450)
					dms_chg_monitor_report_err(DSM_BATTERY_VOL_HIGH_ERROR_NO);
			    if(BMT_status.POSTFULL_charging_time > 2*60*60)
					dms_chg_monitor_report_err(DSM_CHARGING_FULL_OVERTIME_NO);
			    if((cmd_discharging == 1) && (gFG_Is_Charging==1))
					dms_chg_monitor_report_err(DSM_DISCHARGING_CMD_FAIL_NO);
			    if((battery_charge_current > 2000) && (BMT_status.charger_exist == KAL_FALSE))
					dms_chg_monitor_report_err(DSM_DISCHARGING_CURRENT_OVER_NO);
			    if((charging_ibus < 300) && (BMT_status.bat_charging_state == CHR_CC))
					dms_chg_monitor_report_err(DSM_CHARGING_IBUS_LOW_NO);

			    if((battery_temp_dsm >= 45) && (battery_temp_dsm < 55))
					dms_chg_monitor_report_err(DSM_BATT_VOL_LIMIT_ERROR_NO);
			    if((battery_temp_dsm > 0) && (battery_temp_dsm <= 10))
					dms_chg_monitor_report_err(DSM_BATT_CURRENT_LIMIT_ERROR_NO);
				//msleep(5000);
				//}
       return 0;
}
#endif

void bat_thread_wakeup(void)
{
	battery_log(BAT_LOG_FULL, "******** battery : bat_thread_wakeup  ********\n");

	bat_thread_timeout = KAL_TRUE;
	bat_meter_timeout = KAL_TRUE;
#ifdef MTK_ENABLE_AGING_ALGORITHM
	suspend_time = 0;
#endif
	battery_meter_reset_sleep_time();
	wake_up(&bat_thread_wq);
}
#if 1//donghaijiao 160713 add for identify the flash ID 	
extern unsigned int g_emmc_raw_cid[4]; /* raw card CID */	
extern void msdc_emmc_id_check(void);	
#endif

/* ///////////////////////////////////////////////////////////////////////////////////////// */
/* // fop API */
/* ///////////////////////////////////////////////////////////////////////////////////////// */
static long adc_cali_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	int *user_data_addr;
	int *naram_data_addr;
	int i = 0;
	int ret = 0;
	int adc_in_data[2] = { 1, 1 };
    int adc_out_data[5] = { 1, 1,1,1,1 };
 #if 1//donghaijiao 160713 add for identify the flash ID
        char temp_buf[10];
 #endif
 #ifdef HQ_CHARGER_FOR_HUAWEI    //add current test start 	
    kal_bool icharging_enable = KAL_FALSE;	
#endif //end 
	//sync gyg runin test
	kal_bool charging_enable = KAL_FALSE;
	//

	mutex_lock(&bat_mutex);

	switch (cmd) {
	case TEST_ADC_CALI_PRINT:
		g_ADC_Cali = KAL_FALSE;
		break;

	case SET_ADC_CALI_Slop:
		naram_data_addr = (int *)arg;
		ret = copy_from_user(adc_cali_slop, naram_data_addr, 36);
		g_ADC_Cali = KAL_FALSE;	/* enable calibration after setting ADC_CALI_Cal */
		/* Protection */
		for (i = 0; i < 14; i++) {
			if ((*(adc_cali_slop + i) == 0) || (*(adc_cali_slop + i) == 1))
				*(adc_cali_slop + i) = 1000;

		}
		for (i = 0; i < 14; i++)
			battery_log(BAT_LOG_CRTI, "adc_cali_slop[%d] = %d\n", i,
				    *(adc_cali_slop + i));
		battery_log(BAT_LOG_FULL, "**** unlocked_ioctl : SET_ADC_CALI_Slop Done!\n");
		break;

	case SET_ADC_CALI_Offset:
		naram_data_addr = (int *)arg;
		ret = copy_from_user(adc_cali_offset, naram_data_addr, 36);
		g_ADC_Cali = KAL_FALSE;	/* enable calibration after setting ADC_CALI_Cal */
		for (i = 0; i < 14; i++)
			battery_log(BAT_LOG_CRTI, "adc_cali_offset[%d] = %d\n", i,
				    *(adc_cali_offset + i));
		battery_log(BAT_LOG_FULL, "**** unlocked_ioctl : SET_ADC_CALI_Offset Done!\n");
		break;

	case SET_ADC_CALI_Cal:
		naram_data_addr = (int *)arg;
		ret = copy_from_user(adc_cali_cal, naram_data_addr, 4);
		g_ADC_Cali = KAL_TRUE;
		if (adc_cali_cal[0] == 1)
			g_ADC_Cali = KAL_TRUE;
		else
			g_ADC_Cali = KAL_FALSE;

		for (i = 0; i < 1; i++)
			battery_log(BAT_LOG_CRTI, "adc_cali_cal[%d] = %d\n", i,
				    *(adc_cali_cal + i));
		battery_log(BAT_LOG_FULL, "**** unlocked_ioctl : SET_ADC_CALI_Cal Done!\n");
		break;

	case ADC_CHANNEL_READ:
		/* g_ADC_Cali = KAL_FALSE; *//* 20100508 Infinity */
		user_data_addr = (int *)arg;
		ret = copy_from_user(adc_in_data, user_data_addr, 8);	/* 2*int = 2*4 */

		if (adc_in_data[0] == 0) {	/* I_SENSE */
			adc_out_data[0] = battery_meter_get_VSense() * adc_in_data[1];
		} else if (adc_in_data[0] == 1) {	/* BAT_SENSE */
			adc_out_data[0] =
			    battery_meter_get_battery_voltage(KAL_TRUE) * adc_in_data[1];
		} else if (adc_in_data[0] == 3) {	/* V_Charger */
			adc_out_data[0] = battery_meter_get_charger_voltage() * adc_in_data[1];
			/* adc_out_data[0] = adc_out_data[0] / 100; */
		} else if (adc_in_data[0] == 30) {	/* V_Bat_temp magic number */
			adc_out_data[0] = battery_meter_get_battery_temperature() * adc_in_data[1];
		} else if (adc_in_data[0] == 66) {
			adc_out_data[0] = (battery_meter_get_battery_current()) / 10;

			if (battery_meter_get_battery_current_sign() == KAL_TRUE)
				adc_out_data[0] = 0 - adc_out_data[0];	/* charging */

		} else {
			battery_log(BAT_LOG_FULL, "unknown channel(%d,%d)\n",
				    adc_in_data[0], adc_in_data[1]);
		}

		if (adc_out_data[0] < 0)
			adc_out_data[1] = 1;	/* failed */
		else
			adc_out_data[1] = 0;	/* success */

		if (adc_in_data[0] == 30)
			adc_out_data[1] = 0;	/* success */

		if (adc_in_data[0] == 66)
			adc_out_data[1] = 0;	/* success */

		ret = copy_to_user(user_data_addr, adc_out_data, 8);
		battery_log(BAT_LOG_CRTI,
			    "**** unlocked_ioctl : Channel %d * %d times = %d\n",
			    adc_in_data[0], adc_in_data[1], adc_out_data[0]);
		break;

	case BAT_STATUS_READ:
		user_data_addr = (int *)arg;
		ret = copy_from_user(battery_in_data, user_data_addr, 4);
		//gyg_add_20150612 start
		//3->On  ;  2->Off
        battery_xlog_printk(BAT_LOG_CRTI, "[BATTERY] BAT_STATUS_READ adc_cali_ioctl battery_in_data[0]:%d!!\n\r", battery_in_data[0]); 
		if(battery_in_data[0]==3)
		{
            battery_xlog_printk(BAT_LOG_CRTI, "[BATTERY] adc_cali_ioctl cmd is 3 and turn on chg!!\n\r");  
			g_runin_test_enter = 0;
            g_runin_battery_test_flag =1;
			pchr_turn_on_charging();
		}
		else if(battery_in_data[0]==2)
		{
            battery_xlog_printk(BAT_LOG_CRTI, "[BATTERY] adc_cali_ioctl cmd is 2 and turn off chg!!\n\r");  
			g_runin_test_enter = 1;
            g_runin_battery_test_flag =1;
			battery_charging_control(CHARGING_CMD_ENABLE,&charging_enable);
		}
		else if(battery_in_data[0]==1)
		{
            battery_xlog_printk(BAT_LOG_CRTI, "[BATTERY] adc_cali_ioctl cmd is 1 and exit runin battery test!!\n\r");  
			g_runin_test_enter = 0;
            g_runin_battery_test_flag =1;
			pchr_turn_on_charging();
		}
		else
		{
            battery_xlog_printk(BAT_LOG_CRTI, "[BATTERY] Not in runin battery test mode!!\n\r");  
			g_runin_test_enter = 0;
            g_runin_battery_test_flag =0;
		}
		//gyg_add_20150612 end

		/* [0] is_CAL */
		if (g_ADC_Cali)
			battery_out_data[0] = 1;
		else
			battery_out_data[0] = 0;

		ret = copy_to_user(user_data_addr, battery_out_data, 4);
		battery_log(BAT_LOG_CRTI, "**** unlocked_ioctl : CAL:%d\n", battery_out_data[0]);
		break;

	case Set_Charger_Current:	/* For Factory Mode */
		user_data_addr = (int *)arg;
		ret = copy_from_user(charging_level_data, user_data_addr, 4);
		g_ftm_battery_flag = KAL_TRUE;
		if (charging_level_data[0] == 0)
			charging_level_data[0] = CHARGE_CURRENT_70_00_MA;
		else if (charging_level_data[0] == 1)
			charging_level_data[0] = CHARGE_CURRENT_200_00_MA;
		else if (charging_level_data[0] == 2)
			charging_level_data[0] = CHARGE_CURRENT_400_00_MA;
		else if (charging_level_data[0] == 3)
			charging_level_data[0] = CHARGE_CURRENT_450_00_MA;
		else if (charging_level_data[0] == 4)
			charging_level_data[0] = CHARGE_CURRENT_550_00_MA;
		else if (charging_level_data[0] == 5)
			charging_level_data[0] = CHARGE_CURRENT_650_00_MA;
		else if (charging_level_data[0] == 6)
			charging_level_data[0] = CHARGE_CURRENT_700_00_MA;
		else if (charging_level_data[0] == 7)
			charging_level_data[0] = CHARGE_CURRENT_800_00_MA;
		else if (charging_level_data[0] == 8)
			charging_level_data[0] = CHARGE_CURRENT_900_00_MA;
		else if (charging_level_data[0] == 9)
			charging_level_data[0] = CHARGE_CURRENT_1000_00_MA;
		else if (charging_level_data[0] == 10)
			charging_level_data[0] = CHARGE_CURRENT_1100_00_MA;
		else if (charging_level_data[0] == 11)
			charging_level_data[0] = CHARGE_CURRENT_1200_00_MA;
		else if (charging_level_data[0] == 12)
			charging_level_data[0] = CHARGE_CURRENT_1300_00_MA;
		else if (charging_level_data[0] == 13)
			charging_level_data[0] = CHARGE_CURRENT_1400_00_MA;
		else if (charging_level_data[0] == 14)
			charging_level_data[0] = CHARGE_CURRENT_1500_00_MA;
		else if (charging_level_data[0] == 15)
			charging_level_data[0] = CHARGE_CURRENT_1600_00_MA;
		else
			charging_level_data[0] = CHARGE_CURRENT_450_00_MA;

		wake_up_bat();
		battery_log(BAT_LOG_CRTI, "**** unlocked_ioctl : set_Charger_Current:%d\n",
			    charging_level_data[0]);
		break;
		/* add for meta tool------------------------------- */
	case Get_META_BAT_VOL:
               #if 1//donghaijiao  160713 add for identify the flash ID
               msdc_emmc_id_check();
               #endif
		user_data_addr = (int *)arg;
		ret = copy_from_user(adc_in_data, user_data_addr, 8);
		adc_out_data[0] = BMT_status.bat_vol;
 #if 1//donghaijiao  160713 add for identify the flash ID
                        printk("**** unlocked_ioctl : BAT_VOL:%d\n", adc_out_data[0]);   
                        printk("****g_emmc_raw_cid 00 :0x%x\n", g_emmc_raw_cid[0]);   
                        printk("****g_emmc_raw_cid 01 :0x%x\n", g_emmc_raw_cid[1]);   
                        printk("****g_emmc_raw_cid 02 :0x%x\n", g_emmc_raw_cid[2]);   
                        printk("****g_emmc_raw_cid 03 :0x%x\n", g_emmc_raw_cid[3]);   
                               temp_buf[0] = (g_emmc_raw_cid[0]) & 0xFF; /* Manufacturer ID */
                               temp_buf[1] = (g_emmc_raw_cid[0] >> 16) & 0xFF; /* Reserved(6)+Card/BGA(2) */
                               temp_buf[2] = (g_emmc_raw_cid[0] >> 8 ) & 0xFF; /* OEM/Application ID */
                               temp_buf[3] = (g_emmc_raw_cid[0] >> 0 ) & 0xFF; /* Product name [0] */
                               temp_buf[4] = (g_emmc_raw_cid[1] >> 24) & 0xFF; /* Product name [1] */
                               temp_buf[5] = (g_emmc_raw_cid[1] >> 16) & 0xFF; /* Product name [2] */
                               temp_buf[6] = (g_emmc_raw_cid[1] >> 8 ) & 0xFF; /* Product name [3] */
                               temp_buf[7] = (g_emmc_raw_cid[1] >> 0 ) & 0xFF; /* Product name [4] */
                               temp_buf[8] = (g_emmc_raw_cid[2] >> 24) & 0xFF; /* Product name [5] */
                               temp_buf[9] = (g_emmc_raw_cid[0] >> 24) & 0xFF;
                        printk("****g_emmc_raw_cid 01 after :0x%x\n", temp_buf[0]);   
                        printk("****g_emmc_raw_cid 02 after :0x%x\n", temp_buf[1]);   
                        printk("****g_emmc_raw_cid 03 after :0x%x\n", temp_buf[2]);  
                        printk("****g_emmc_raw_cid 04 after :0x%x\n", temp_buf[3]);   
                        printk("****g_emmc_raw_cid 05 after :0x%x\n", temp_buf[4]);   
                        printk("****g_emmc_raw_cid 06 after :0x%x\n", temp_buf[5]);    
                        printk("****g_emmc_raw_cid 07 after :0x%x\n", temp_buf[6]);  
                        printk("****g_emmc_raw_cid 08 after :0x%x\n", temp_buf[7]);   
                        printk("****g_emmc_raw_cid 09 after :0x%x\n", temp_buf[8]); 
                        printk("****g_emmc_raw_cid 10 after :0x%x\n", temp_buf[4]<<8);                   
                        printk("****g_emmc_raw_cid 11 after :0x%x\n", temp_buf[4]<<8 | temp_buf[5]);   
                        printk("****g_emmc_raw_cid 12 after :0x%x\n", temp_buf[9]);
                                
                               //adc_out_data[1] = (temp_buf[0]<<8 | temp_buf[9]);
                               adc_out_data[1]=(g_emmc_raw_cid[0] >> 16) & 0xffff;
                               adc_out_data[2]=g_emmc_raw_cid[0] & 0xffff;
                               adc_out_data[3]=(g_emmc_raw_cid[1] >> 16) & 0xffff;
                               adc_out_data[4]=g_emmc_raw_cid[1] & 0xffff;     
                               
                               //printk("****g_emmc_raw_cid adc_out_data[0] :0x%x\n", adc_out_data[0]);                                
                               //printk("****g_emmc_raw_cid adc_out_data[1] :0x%x\n", adc_out_data[1]);        
                               //printk("****g_emmc_raw_cid adc_out_data[2] :0x%x\n", adc_out_data[2]);        
                               //printk("****g_emmc_raw_cid adc_out_data[3] :0x%x\n", adc_out_data[3]);        
                               //printk("****g_emmc_raw_cid adc_out_data[4] :0x%x\n", adc_out_data[4]);
                                       
                               //xlog_printk(ANDROID_LOG_DEBUG, "Power/Battery", "****g_emmc_raw_cid all123 :0x%x\n", ((g_emmc_raw_cid[0]<<10) | (g_emmc_raw_cid[1]<<2) | (g_emmc_raw_cid[2]&0xff000000>>6)));   
   #endif

               ret = copy_to_user(user_data_addr, adc_out_data, 20);

		break;
	case Get_META_BAT_SOC:
		user_data_addr = (int *)arg;
		ret = copy_from_user(adc_in_data, user_data_addr, 8);
		adc_out_data[0] = BMT_status.UI_SOC;
		ret = copy_to_user(user_data_addr, adc_out_data, 8);

		break;
		/* add bing meta tool------------------------------- */
#if defined(HQ_CHARGER_FOR_HUAWEI) // add current test start
               case BAT_THREAD_CTRL: /* for runtime test */
                       user_data_addr = (int *)arg;
                       if (*user_data_addr == 2) {
                               icharging_enable = KAL_FALSE;
                               g_charger_should_not_open = KAL_TRUE;                   /* close charger */
                       }
                       else if (*user_data_addr == 1 || *user_data_addr == 3) {
                               icharging_enable = KAL_TRUE;
                               g_charger_should_not_open = KAL_FALSE;                  /* open charger */
                       }
                       battery_charging_control(CHARGING_CMD_ENABLE, &icharging_enable);       
                       mt_battery_update_status();
               wake_up_bat();
                       break;
               case HQ_Get_BAT_VOL:
                       user_data_addr = (int *)arg;
                       adc_out_data[0] = battery_meter_get_battery_voltage(KAL_TRUE);
                       ret = copy_to_user(user_data_addr, adc_out_data, 4); 
                       break;
#endif //end

	default:
		g_ADC_Cali = KAL_FALSE;
		break;
	}

	mutex_unlock(&bat_mutex);

	return 0;
}

static int adc_cali_open(struct inode *inode, struct file *file)
{
	return 0;
}

static int adc_cali_release(struct inode *inode, struct file *file)
{
	return 0;
}


static const struct file_operations adc_cali_fops = {
	.owner = THIS_MODULE,
	.unlocked_ioctl = adc_cali_ioctl,
	.open = adc_cali_open,
	.release = adc_cali_release,
};


void check_battery_exist(void)
{
#if defined(CONFIG_DIS_CHECK_BATTERY)
	battery_log(BAT_LOG_CRTI, "[BATTERY] Disable check battery exist.\n");
#else
	unsigned int baton_count = 0;
	unsigned int charging_enable = KAL_FALSE;
	unsigned int battery_status;
	unsigned int i;

	for (i = 0; i < 3; i++) {
		battery_charging_control(CHARGING_CMD_GET_BATTERY_STATUS, &battery_status);
		baton_count += battery_status;

	}

	if (baton_count >= 3) {
		if ((g_platform_boot_mode == META_BOOT) || (g_platform_boot_mode == ADVMETA_BOOT)
		    || (g_platform_boot_mode == ATE_FACTORY_BOOT)) {
			battery_log(BAT_LOG_FULL,
				    "[BATTERY] boot mode = %d, bypass battery check\n",
				    g_platform_boot_mode);
		} else {
			battery_log(BAT_LOG_CRTI,
				    "[BATTERY] Battery is not exist, power off FAN5405 and system (%d)\n",
				    baton_count);

			battery_charging_control(CHARGING_CMD_ENABLE, &charging_enable);
			#ifdef CONFIG_MTK_POWER_PATH_MANAGEMENT_SUPPORT
			battery_charging_control(CHARGING_CMD_SET_PLATFORM_RESET, NULL);
			#else
			battery_charging_control(CHARGING_CMD_SET_POWER_OFF, NULL);
			#endif
		}
	}
#endif
}


#if defined(MTK_PLUG_OUT_DETECTION)

void charger_plug_out_sw_mode(void)
{
	signed int ICharging = 0;
	signed short i;
	signed short cnt = 0;
	kal_bool enable;
	unsigned int charging_enable;
	signed int VCharger = 0;

	if (BMT_status.charger_exist == KAL_TRUE) {
		if (chargin_hw_init_done && upmu_is_chr_det() == KAL_TRUE) {

			for (i = 0; i < 4; i++) {
				enable = pmic_get_register_value(PMIC_RG_CHR_EN);

				if (enable == 1) {

					ICharging = battery_meter_get_charging_current_imm();
					VCharger = battery_meter_get_charger_voltage();
					if (ICharging < 70 && VCharger < 4400) {
						cnt++;
						battery_log(BAT_LOG_CRTI,
							    "[charger_hv_detect_sw_thread_handler] fail ICharging=%d , VCHR=%d cnt=%d\n",
							    ICharging, VCharger, cnt);
					} else {
						/* battery_log(BAT_LOG_CRTI,
						 * "[charger_hv_detect_sw_thread_handler] success ICharging=%d,
						 * VCHR=%d cnt=%d\n",ICharging,VCharger,cnt);
						 */
						break;
					}
				} else {
					break;
				}
			}

			if (cnt >= 3) {
				charging_enable = KAL_FALSE;
				battery_charging_control(CHARGING_CMD_ENABLE, &charging_enable);
				battery_log(BAT_LOG_CRTI,
					    "[charger_hv_detect_sw_thread_handler] ICharging=%d VCHR=%d cnt=%d turn off charging\n",
					    ICharging, VCharger, cnt);
			}

		}
	}

}


/*extern unsigned int upmu_get_reg_value(unsigned int reg);*/
void hv_sw_mode(void)
{
	kal_bool hv_status;
	unsigned int charging_enable;

	if (upmu_is_chr_det() == KAL_TRUE)
		check_battery_exist();



	if (chargin_hw_init_done)
		battery_charging_control(CHARGING_CMD_GET_HV_STATUS, &hv_status);

	if (hv_status == KAL_TRUE) {
		battery_log(BAT_LOG_CRTI, "[charger_hv_detect_sw_thread_handler] charger hv\n");

		charging_enable = KAL_FALSE;
		if (chargin_hw_init_done)
			battery_charging_control(CHARGING_CMD_ENABLE, &charging_enable);
	} else
		battery_log(BAT_LOG_FULL,
			    "[charger_hv_detect_sw_thread_handler] upmu_chr_get_vcdt_hv_det() != 1\n");


	/* battery_log(BAT_LOG_CRTI,
	 * "[PMIC_BIAS_GEN_EN & PMIC_BIAS_GEN_EN_SEL] 0xa=0x%x\n",
	 * upmu_get_reg_value(0x000a));
	 */
	if (pmic_get_register_value(PMIC_BIAS_GEN_EN) == 1
	    || pmic_get_register_value(PMIC_BIAS_GEN_EN_SEL) == 0) {
		battery_log(BAT_LOG_CRTI,
			    "[PMIC_BIAS_GEN_EN & PMIC_BIAS_GEN_EN_SEL] be writen 0xa=0x%x\n",
			    upmu_get_reg_value(0x000a));
		BUG_ON(1);
	}

	if (chargin_hw_init_done)
		battery_charging_control(CHARGING_CMD_RESET_WATCH_DOG_TIMER, NULL);

}

int charger_hv_detect_sw_thread_handler(void *unused)
{
	ktime_t ktime;
	unsigned int hv_voltage = V_CHARGER_MAX * 1000;


	unsigned char cnt = 0;

#if defined(CONFIG_MTK_DUAL_INPUT_CHARGER_SUPPORT)
	hv_voltage = DISO_data.hv_voltage;
#endif

	do {

		if (BMT_status.charger_exist == KAL_TRUE)
			ktime = ktime_set(0, BAT_MS_TO_NS(200));
		else
			ktime = ktime_set(0, BAT_MS_TO_NS(1000));



		if (chargin_hw_init_done)
			battery_charging_control(CHARGING_CMD_SET_HV_THRESHOLD, &hv_voltage);

		wait_event_interruptible(charger_hv_detect_waiter,
					 (charger_hv_detect_flag == KAL_TRUE));

		if (BMT_status.charger_exist == KAL_TRUE) {
			if (cnt >= 5) {
				/* battery_log(BAT_LOG_CRTI, */
				/* "[charger_hv_detect_sw_thread_handler] charger in do hv_sw_mode\n"); */
				hv_sw_mode();
				cnt = 0;
			} else {
				cnt++;
			}

			/* battery_log(BAT_LOG_CRTI, */
			/* "[charger_hv_detect_sw_thread_handler] charger in cnt=%d\n",cnt); */
			charger_plug_out_sw_mode();
		} else {
			/* battery_log(BAT_LOG_CRTI, */
			/* "[charger_hv_detect_sw_thread_handler] charger out do hv_sw_mode\n"); */
			hv_sw_mode();
		}


		charger_hv_detect_flag = KAL_FALSE;
		hrtimer_start(&charger_hv_detect_timer, ktime, HRTIMER_MODE_REL);

	} while (!kthread_should_stop());

	return 0;
}

#else
int charger_hv_detect_sw_thread_handler(void *unused)
{
	ktime_t ktime;
	unsigned int charging_enable;
	unsigned int hv_voltage = batt_cust_data.v_charger_max * 1000;
	kal_bool hv_status;

#if defined(CONFIG_MTK_DUAL_INPUT_CHARGER_SUPPORT)
	hv_voltage = DISO_data.hv_voltage;
#endif

	do {
#ifdef CONFIG_MTK_BQ25896_SUPPORT
		/*this annoying SW workaround wakes up bat_thread. 10 secs is set instead of 1 sec*/
		ktime = ktime_set(10, 0);
#else
		ktime = ktime_set(0, BAT_MS_TO_NS(1000));
#endif
		if (chargin_hw_init_done)
			battery_charging_control(CHARGING_CMD_SET_HV_THRESHOLD, &hv_voltage);

		wait_event_interruptible(charger_hv_detect_waiter,
					 (charger_hv_detect_flag == KAL_TRUE));

		if (upmu_is_chr_det() == KAL_TRUE)
			check_battery_exist();




		charger_hv_detect_flag = KAL_FALSE;

		if (chargin_hw_init_done)
			battery_charging_control(CHARGING_CMD_GET_HV_STATUS, &hv_status);

		if (hv_status == KAL_TRUE) {
			battery_log(BAT_LOG_CRTI,
				    "[charger_hv_detect_sw_thread_handler] charger hv\n");

			charging_enable = KAL_FALSE;
			if (chargin_hw_init_done)
				battery_charging_control(CHARGING_CMD_ENABLE, &charging_enable);
		} else {
			battery_log(BAT_LOG_FULL,
				    "[charger_hv_detect_sw_thread_handler] upmu_chr_get_vcdt_hv_det() != 1\n");
		}



		if (chargin_hw_init_done)
			battery_charging_control(CHARGING_CMD_RESET_WATCH_DOG_TIMER, NULL);

		hrtimer_start(&charger_hv_detect_timer, ktime, HRTIMER_MODE_REL);

	} while (!kthread_should_stop());

	return 0;
}
#endif				/* #if defined(MTK_PLUG_OUT_DETECTION) */

enum hrtimer_restart charger_hv_detect_sw_workaround(struct hrtimer *timer)
{
	charger_hv_detect_flag = KAL_TRUE;
	wake_up_interruptible(&charger_hv_detect_waiter);

	battery_log(BAT_LOG_FULL, "[charger_hv_detect_sw_workaround]\n");

	return HRTIMER_NORESTART;
}

void charger_hv_detect_sw_workaround_init(void)
{
	ktime_t ktime;

	ktime = ktime_set(0, BAT_MS_TO_NS(2000));
	hrtimer_init(&charger_hv_detect_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	charger_hv_detect_timer.function = charger_hv_detect_sw_workaround;
	hrtimer_start(&charger_hv_detect_timer, ktime, HRTIMER_MODE_REL);

	charger_hv_detect_thread =
	    kthread_run(charger_hv_detect_sw_thread_handler, 0,
			"mtk charger_hv_detect_sw_workaround");
	if (IS_ERR(charger_hv_detect_thread)) {
		battery_log(BAT_LOG_FULL,
			    "[%s]: failed to create charger_hv_detect_sw_workaround thread\n",
			    __func__);
	}
	check_battery_exist();
	battery_log(BAT_LOG_CRTI, "charger_hv_detect_sw_workaround_init : done\n");
}


enum hrtimer_restart battery_kthread_hrtimer_func(struct hrtimer *timer)
{
	bat_thread_wakeup();

	return HRTIMER_NORESTART;
}

void battery_kthread_hrtimer_init(void)
{
	ktime_t ktime;
#ifdef CONFIG_MTK_BQ25896_SUPPORT
/*watchdog timer before 40 secs*/
	ktime = ktime_set(10, 0);	/* 3s, 10* 1000 ms */
#else
	ktime = ktime_set(1, 0);
#endif
	hrtimer_init(&battery_kthread_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	battery_kthread_timer.function = battery_kthread_hrtimer_func;
	hrtimer_start(&battery_kthread_timer, ktime, HRTIMER_MODE_REL);

	battery_log(BAT_LOG_CRTI, "battery_kthread_hrtimer_init : done\n");
}


static void get_charging_control(void)
{
	battery_charging_control = chr_control_interface;
}

#if defined(CONFIG_MTK_DUAL_INPUT_CHARGER_SUPPORT)
static irqreturn_t diso_auxadc_irq_thread(int irq, void *dev_id)
{
	int pre_diso_state = (DISO_data.diso_state.pre_otg_state |
			      (DISO_data.diso_state.pre_vusb_state << 1) |
			      (DISO_data.diso_state.pre_vdc_state << 2)) & 0x7;

	battery_log(BAT_LOG_CRTI,
		    "[DISO]auxadc IRQ threaded handler triggered, pre_diso_state is %s\n",
		    DISO_state_s[pre_diso_state]);

	switch (pre_diso_state) {
#ifdef MTK_DISCRETE_SWITCH	/*for DSC DC plugin handle */
	case USB_ONLY:
#endif
	case OTG_ONLY:
		BMT_status.charger_exist = KAL_TRUE;
		wake_lock(&battery_suspend_lock);
		wake_up_bat();
		break;
	case DC_WITH_OTG:
		BMT_status.charger_exist = KAL_FALSE;
		/* need stop charger quickly */
		battery_charging_control(CHARGING_CMD_ENABLE, &BMT_status.charger_exist);
		BMT_status.charger_exist = KAL_FALSE;	/* reset charger status */
		BMT_status.charger_type = CHARGER_UNKNOWN;
		wake_unlock(&battery_suspend_lock);
		wake_up_bat();
		break;
	case DC_WITH_USB:
		/*usb delayed work will reflact BMT_staus , so need update state ASAP */
		if ((BMT_status.charger_type == STANDARD_HOST)
		    || (BMT_status.charger_type == CHARGING_HOST))
			mt_usb_disconnect();	/* disconnect if connected */
		BMT_status.charger_type = CHARGER_UNKNOWN;	/* reset chr_type */
		wake_up_bat();
		break;
	case DC_ONLY:
		BMT_status.charger_type = CHARGER_UNKNOWN;
		mt_battery_charger_detect_check();	/* plug in VUSB, check if need connect usb */
		break;
	default:
		battery_log(BAT_LOG_CRTI,
			    "[DISO]VUSB auxadc threaded handler triggered ERROR OR TEST\n");
		break;
	}
	return IRQ_HANDLED;
}

static void dual_input_init(void)
{
	DISO_data.irq_callback_func = diso_auxadc_irq_thread;
	battery_charging_control(CHARGING_CMD_DISO_INIT, &DISO_data);
}
#endif

int __batt_init_cust_data_from_cust_header(void)
{
	/* mt_charging.h */
	/* stop charging while in talking mode */
#if defined(STOP_CHARGING_IN_TAKLING)
	batt_cust_data.stop_charging_in_takling = 1;
#else				/* #if defined(STOP_CHARGING_IN_TAKLING) */
	batt_cust_data.stop_charging_in_takling = 0;
#endif				/* #if defined(STOP_CHARGING_IN_TAKLING) */

#if defined(TALKING_RECHARGE_VOLTAGE)
	batt_cust_data.talking_recharge_voltage = TALKING_RECHARGE_VOLTAGE;
#endif

#if defined(TALKING_SYNC_TIME)
	batt_cust_data.talking_sync_time = TALKING_SYNC_TIME;
#endif

	/* Battery Temperature Protection */
#if defined(MTK_TEMPERATURE_RECHARGE_SUPPORT)
	batt_cust_data.mtk_temperature_recharge_support = 1;
#else				/* #if defined(MTK_TEMPERATURE_RECHARGE_SUPPORT) */
	batt_cust_data.mtk_temperature_recharge_support = 0;
#endif				/* #if defined(MTK_TEMPERATURE_RECHARGE_SUPPORT) */

#if defined(MAX_CHARGE_TEMPERATURE)
	batt_cust_data.max_charge_temperature = MAX_CHARGE_TEMPERATURE;
#endif

#if defined(MAX_CHARGE_TEMPERATURE_MINUS_X_DEGREE)
	batt_cust_data.max_charge_temperature_minus_x_degree =
	    MAX_CHARGE_TEMPERATURE_MINUS_X_DEGREE;
#endif

#if defined(MIN_CHARGE_TEMPERATURE)
	batt_cust_data.min_charge_temperature = MIN_CHARGE_TEMPERATURE;
#endif

#if defined(MIN_CHARGE_TEMPERATURE_PLUS_X_DEGREE)
	batt_cust_data.min_charge_temperature_plus_x_degree = MIN_CHARGE_TEMPERATURE_PLUS_X_DEGREE;
#endif

#if defined(ERR_CHARGE_TEMPERATURE)
	batt_cust_data.err_charge_temperature = ERR_CHARGE_TEMPERATURE;
#endif

	/* Linear Charging Threshold */
#if defined(V_PRE2CC_THRES)
	batt_cust_data.v_pre2cc_thres = V_PRE2CC_THRES;
#endif
#if defined(V_CC2TOPOFF_THRES)
	batt_cust_data.v_cc2topoff_thres = V_CC2TOPOFF_THRES;
#endif
#if defined(RECHARGING_VOLTAGE)
	batt_cust_data.recharging_voltage = RECHARGING_VOLTAGE;
#endif
#if defined(CHARGING_FULL_CURRENT)
	batt_cust_data.charging_full_current = CHARGING_FULL_CURRENT;
#endif

	/* Charging Current Setting */
#if defined(CONFIG_USB_IF)
	batt_cust_data.config_usb_if = 1;
#else				/* #if defined(CONFIG_USB_IF) */
	batt_cust_data.config_usb_if = 0;
#endif				/* #if defined(CONFIG_USB_IF) */

#if defined(USB_CHARGER_CURRENT_SUSPEND)
	batt_cust_data.usb_charger_current_suspend = USB_CHARGER_CURRENT_SUSPEND;
#endif
#if defined(USB_CHARGER_CURRENT_UNCONFIGURED)
	batt_cust_data.usb_charger_current_unconfigured = USB_CHARGER_CURRENT_UNCONFIGURED;
#endif
#if defined(USB_CHARGER_CURRENT_CONFIGURED)
	batt_cust_data.usb_charger_current_configured = USB_CHARGER_CURRENT_CONFIGURED;
#endif
#if defined(USB_CHARGER_CURRENT)
	batt_cust_data.usb_charger_current = USB_CHARGER_CURRENT;
#endif
#if defined(AC_CHARGER_INPUT_CURRENT)
	batt_cust_data.ac_charger_input_current = AC_CHARGER_INPUT_CURRENT;
#endif
#if defined(AC_CHARGER_CURRENT)
	batt_cust_data.ac_charger_current = AC_CHARGER_CURRENT;
#endif
#if defined(NON_STD_AC_CHARGER_INPUT_CURRENT)
	batt_cust_data.non_std_ac_charger_input_current = NON_STD_AC_CHARGER_INPUT_CURRENT;
#endif
#if defined(NON_STD_AC_CHARGER_CURRENT)
	batt_cust_data.non_std_ac_charger_current = NON_STD_AC_CHARGER_CURRENT;
#endif
#if defined(CHARGING_HOST_CHARGER_CURRENT)
	batt_cust_data.charging_host_charger_current = CHARGING_HOST_CHARGER_CURRENT;
#endif
#if defined(APPLE_0_5A_CHARGER_CURRENT)
	batt_cust_data.apple_0_5a_charger_current = APPLE_0_5A_CHARGER_CURRENT;
#endif
#if defined(APPLE_1_0A_CHARGER_CURRENT)
	batt_cust_data.apple_1_0a_charger_current = APPLE_1_0A_CHARGER_CURRENT;
#endif
#if defined(APPLE_2_1A_CHARGER_CURRENT)
	batt_cust_data.apple_2_1a_charger_current = APPLE_2_1A_CHARGER_CURRENT;
#endif

	/* Precise Tunning
	   batt_cust_data.battery_average_data_number =
	   BATTERY_AVERAGE_DATA_NUMBER;
	   batt_cust_data.battery_average_size = BATTERY_AVERAGE_SIZE;
	 */


	/* charger error check */
#if defined(BAT_LOW_TEMP_PROTECT_ENABLE)
	batt_cust_data.bat_low_temp_protect_enable = 1;
#else				/* #if defined(BAT_LOW_TEMP_PROTECT_ENABLE) */
	batt_cust_data.bat_low_temp_protect_enable = 0;
#endif				/* #if defined(BAT_LOW_TEMP_PROTECT_ENABLE) */

#if defined(V_CHARGER_ENABLE)
	batt_cust_data.v_charger_enable = V_CHARGER_ENABLE;
#endif
#if defined(V_CHARGER_MAX)
	batt_cust_data.v_charger_max = V_CHARGER_HIGHT_WARNING;
#endif
#if defined(V_CHARGER_MIN)
	batt_cust_data.v_charger_min = V_CHARGER_MIN;
#endif


	/* Tracking TIME */
#if defined(ONEHUNDRED_PERCENT_TRACKING_TIME)
	batt_cust_data.onehundred_percent_tracking_time = ONEHUNDRED_PERCENT_TRACKING_TIME;
#endif
#if defined(NPERCENT_TRACKING_TIME)
	batt_cust_data.npercent_tracking_time = NPERCENT_TRACKING_TIME;
#endif
#if defined(SYNC_TO_REAL_TRACKING_TIME)
	batt_cust_data.sync_to_real_tracking_time = SYNC_TO_REAL_TRACKING_TIME;
#endif
#if defined(V_0PERCENT_TRACKING)
	batt_cust_data.v_0percent_tracking = V_0PERCENT_TRACKING;
#endif

	/* High battery support */
#if defined(HIGH_BATTERY_VOLTAGE_SUPPORT)
	batt_cust_data.high_battery_voltage_support = 1;
#else				/* #if defined(HIGH_BATTERY_VOLTAGE_SUPPORT) */
	batt_cust_data.high_battery_voltage_support = 0;
#endif				/* #if defined(HIGH_BATTERY_VOLTAGE_SUPPORT) */

#if	defined(CONFIG_MTK_PUMP_EXPRESS_PLUS_SUPPORT)
	batt_cust_data.mtk_pump_express_plus_support = 1;

	#if defined(TA_START_BATTERY_SOC)
	batt_cust_data.ta_start_battery_soc = TA_START_BATTERY_SOC;
	#endif
	#if defined(TA_STOP_BATTERY_SOC)
	batt_cust_data.ta_stop_battery_soc = TA_STOP_BATTERY_SOC;
	#endif
	#if defined(TA_AC_12V_INPUT_CURRENT)
	batt_cust_data.ta_ac_12v_input_current = TA_AC_12V_INPUT_CURRENT;
	#endif
	#if defined(TA_AC_9V_INPUT_CURRENT)
	batt_cust_data.ta_ac_9v_input_current = TA_AC_9V_INPUT_CURRENT;
	#endif
	#if defined(TA_AC_7V_INPUT_CURRENT)
	batt_cust_data.ta_ac_7v_input_current = TA_AC_7V_INPUT_CURRENT;
	#endif
	#if defined(TA_AC_CHARGING_CURRENT)
	batt_cust_data.ta_ac_charging_current = TA_AC_CHARGING_CURRENT;
	#endif
	#if defined(TA_9V_SUPPORT)
	batt_cust_data.ta_9v_support = 1;
	#endif
	#if defined(TA_12V_SUPPORT)
	batt_cust_data.ta_12v_support = 1;
	#endif
#endif

	return 0;
}
#if 0
#if defined(BATTERY_DTS_SUPPORT) && defined(CONFIG_OF)
static void __batt_parse_node(const struct device_node *np,
				const char *node_srting, int *cust_val)
{
	u32 val;

	if (of_property_read_u32(np, node_srting, &val) == 0) {
		(*cust_val) = (int)val;
		battery_log(BAT_LOG_FULL, "Get %s: %d\n", node_srting, (*cust_val));
	} else {
		battery_log(BAT_LOG_CRTI, "Get %s failed\n", node_srting);
	}
}

static int __batt_init_cust_data_from_dt(void)
{
	/* struct device_node *np = dev->dev.of_node; */
	struct device_node *np;

	/* check customer setting */
	np = of_find_compatible_node(NULL, NULL, "mediatek,battery");
	if (!np) {
		/* printk(KERN_ERR "(E) Failed to find device-tree node: %s\n", path); */
		battery_log(BAT_LOG_CRTI, "Failed to find device-tree node: bat_comm\n");
		return -ENODEV;
	}

	__batt_parse_node(np, "stop_charging_in_takling",
		&batt_cust_data.stop_charging_in_takling);

	__batt_parse_node(np, "talking_recharge_voltage",
		&batt_cust_data.talking_recharge_voltage);

	__batt_parse_node(np, "talking_sync_time",
		&batt_cust_data.talking_sync_time);

	__batt_parse_node(np, "mtk_temperature_recharge_support",
		&batt_cust_data.mtk_temperature_recharge_support);

	__batt_parse_node(np, "max_charge_temperature",
		&batt_cust_data.max_charge_temperature);

	__batt_parse_node(np, "max_charge_temperature_minus_x_degree",
		&batt_cust_data.max_charge_temperature_minus_x_degree);

	__batt_parse_node(np, "min_charge_temperature",
		&batt_cust_data.min_charge_temperature);

	__batt_parse_node(np, "min_charge_temperature_plus_x_degree",
		&batt_cust_data.min_charge_temperature_plus_x_degree);

	__batt_parse_node(np, "err_charge_temperature",
		&batt_cust_data.err_charge_temperature);

	__batt_parse_node(np, "v_pre2cc_thres",
		&batt_cust_data.v_pre2cc_thres);

	__batt_parse_node(np, "v_cc2topoff_thres",
		&batt_cust_data.v_cc2topoff_thres);

	__batt_parse_node(np, "recharging_voltage",
		&batt_cust_data.recharging_voltage);

	__batt_parse_node(np, "charging_full_current",
		&batt_cust_data.charging_full_current);

	__batt_parse_node(np, "config_usb_if",
		&batt_cust_data.config_usb_if);

	__batt_parse_node(np, "usb_charger_current_suspend",
		&batt_cust_data.usb_charger_current_suspend);

	__batt_parse_node(np, "usb_charger_current_unconfigured",
		&batt_cust_data.usb_charger_current_unconfigured);

	__batt_parse_node(np, "usb_charger_current_configured",
		&batt_cust_data.usb_charger_current_configured);

	__batt_parse_node(np, "usb_charger_current",
		&batt_cust_data.usb_charger_current);

	__batt_parse_node(np, "ac_charger_input_current",
		&batt_cust_data.ac_charger_input_current);

	__batt_parse_node(np, "ac_charger_current",
		&batt_cust_data.ac_charger_current);

	__batt_parse_node(np, "non_std_ac_charger_current",
		&batt_cust_data.non_std_ac_charger_current);

	__batt_parse_node(np, "charging_host_charger_current",
		&batt_cust_data.charging_host_charger_current);

	__batt_parse_node(np, "apple_0_5a_charger_current",
		&batt_cust_data.apple_0_5a_charger_current);

	__batt_parse_node(np, "apple_1_0a_charger_current",
		&batt_cust_data.apple_1_0a_charger_current);

	__batt_parse_node(np, "apple_2_1a_charger_current",
		&batt_cust_data.apple_2_1a_charger_current);

	__batt_parse_node(np, "bat_low_temp_protect_enable",
		&batt_cust_data.bat_low_temp_protect_enable);

	__batt_parse_node(np, "v_charger_enable",
		&batt_cust_data.v_charger_enable);

	__batt_parse_node(np, "v_charger_max",
		&batt_cust_data.v_charger_max);

	__batt_parse_node(np, "v_charger_min",
		&batt_cust_data.v_charger_min);

	__batt_parse_node(np, "onehundred_percent_tracking_time",
		&batt_cust_data.onehundred_percent_tracking_time);

	__batt_parse_node(np, "npercent_tracking_time",
		&batt_cust_data.npercent_tracking_time);

	__batt_parse_node(np, "sync_to_real_tracking_time",
		&batt_cust_data.sync_to_real_tracking_time);

	__batt_parse_node(np, "v_0percent_tracking",
		&batt_cust_data.v_0percent_tracking);

	__batt_parse_node(np, "high_battery_voltage_support",
		&batt_cust_data.high_battery_voltage_support);

	__batt_parse_node(np, "mtk_jeita_standard_support",
		&batt_cust_data.mtk_jeita_standard_support);

	__batt_parse_node(np, "cust_soc_jeita_sync_time",
		&batt_cust_data.cust_soc_jeita_sync_time);

	__batt_parse_node(np, "jeita_recharge_voltage",
		&batt_cust_data.jeita_recharge_voltage);

	__batt_parse_node(np, "jeita_temp_above_pos_60_cv_voltage",
		&batt_cust_data.jeita_temp_above_pos_60_cv_voltage);

	__batt_parse_node(np, "jeita_temp_pos_10_to_pos_45_cv_voltage",
		&batt_cust_data.jeita_temp_pos_10_to_pos_45_cv_voltage);

	__batt_parse_node(np, "jeita_temp_pos_0_to_pos_10_cv_voltage",
		&batt_cust_data.jeita_temp_pos_0_to_pos_10_cv_voltage);

	__batt_parse_node(np, "jeita_temp_neg_10_to_pos_0_cv_voltage",
		&batt_cust_data.jeita_temp_neg_10_to_pos_0_cv_voltage);

	__batt_parse_node(np, "jeita_temp_below_neg_10_cv_voltage",
		&batt_cust_data.jeita_temp_below_neg_10_cv_voltage);

	__batt_parse_node(np, "jeita_neg_10_to_pos_0_full_current",
		&batt_cust_data.jeita_neg_10_to_pos_0_full_current);

	__batt_parse_node(np, "jeita_temp_pos_45_to_pos_60_recharge_voltage",
		&batt_cust_data.jeita_temp_pos_45_to_pos_60_recharge_voltage);

	__batt_parse_node(np, "jeita_temp_pos_10_to_pos_45_recharge_voltage",
		&batt_cust_data.jeita_temp_pos_10_to_pos_45_recharge_voltage);

	__batt_parse_node(np, "jeita_temp_pos_0_to_pos_10_recharge_voltage",
		&batt_cust_data.jeita_temp_pos_0_to_pos_10_recharge_voltage);

	__batt_parse_node(np, "jeita_temp_neg_10_to_pos_0_recharge_voltage",
		&batt_cust_data.jeita_temp_neg_10_to_pos_0_recharge_voltage);

	__batt_parse_node(np, "jeita_temp_pos_45_to_pos_60_cc2topoff_threshold",
		&batt_cust_data.jeita_temp_pos_45_to_pos_60_cc2topoff_threshold);

	__batt_parse_node(np, "jeita_temp_pos_10_to_pos_45_cc2topoff_threshold",
		&batt_cust_data.jeita_temp_pos_10_to_pos_45_cc2topoff_threshold);

	__batt_parse_node(np, "jeita_temp_pos_0_to_pos_10_cc2topoff_threshold",
		&batt_cust_data.jeita_temp_pos_0_to_pos_10_cc2topoff_threshold);

	__batt_parse_node(np, "jeita_temp_neg_10_to_pos_0_cc2topoff_threshold",
		&batt_cust_data.jeita_temp_neg_10_to_pos_0_cc2topoff_threshold);

#if	defined(CONFIG_MTK_PUMP_EXPRESS_PLUS_SUPPORT)
	__batt_parse_node(np, "mtk_pump_express_plus_support",
		&batt_cust_data.mtk_pump_express_plus_support);

	__batt_parse_node(np, "ta_start_battery_soc",
		&batt_cust_data.ta_start_battery_soc);

	__batt_parse_node(np, "ta_stop_battery_soc",
		&batt_cust_data.ta_stop_battery_soc);

	__batt_parse_node(np, "ta_ac_12v_input_current",
		&batt_cust_data.ta_ac_12v_input_current);

	__batt_parse_node(np, "ta_ac_9v_input_current",
		&batt_cust_data.ta_ac_9v_input_current);

	__batt_parse_node(np, "ta_ac_7v_input_current",
		&batt_cust_data.ta_ac_7v_input_current);

	__batt_parse_node(np, "ta_ac_charging_current",
		&batt_cust_data.ta_ac_charging_current);

	__batt_parse_node(np, "ta_9v_support",
		&batt_cust_data.ta_9v_support);

	__batt_parse_node(np, "ta_12v_support",
		&batt_cust_data.ta_12v_support);
#endif

	of_node_put(np);
	return 0;
}
#endif
#endif
int batt_init_cust_data(void)
{
	__batt_init_cust_data_from_cust_header();

/* 
#if defined(BATTERY_DTS_SUPPORT) && defined(CONFIG_OF)
	battery_log(BAT_LOG_CRTI, "battery custom init by DTS\n");
	__batt_init_cust_data_from_dt();
#endif 
*/
	return 0;
}

static int battery_probe(struct platform_device *dev)
{
	struct class_device *class_dev = NULL;
	int ret = 0;

	battery_log(BAT_LOG_CRTI, "******** battery driver probe!! ********\n");

	/* Integrate with NVRAM */
	ret = alloc_chrdev_region(&adc_cali_devno, 0, 1, ADC_CALI_DEVNAME);
	if (ret)
		battery_log(BAT_LOG_CRTI, "Error: Can't Get Major number for adc_cali\n");
	adc_cali_cdev = cdev_alloc();
	adc_cali_cdev->owner = THIS_MODULE;
	adc_cali_cdev->ops = &adc_cali_fops;
	ret = cdev_add(adc_cali_cdev, adc_cali_devno, 1);
	if (ret)
		battery_log(BAT_LOG_CRTI, "adc_cali Error: cdev_add\n");
	adc_cali_major = MAJOR(adc_cali_devno);
	adc_cali_class = class_create(THIS_MODULE, ADC_CALI_DEVNAME);
	class_dev = (struct class_device *)device_create(adc_cali_class,
							 NULL,
							 adc_cali_devno, NULL, ADC_CALI_DEVNAME);
	battery_log(BAT_LOG_CRTI, "[BAT_probe] adc_cali prepare : done !!\n ");

	get_charging_control();

	batt_init_cust_data();
#if defined(BATTERY_SW_INIT)
		battery_charging_control(CHARGING_CMD_SW_INIT, NULL);
#endif


	battery_charging_control(CHARGING_CMD_GET_PLATFORM_BOOT_MODE, &g_platform_boot_mode);
	battery_log(BAT_LOG_CRTI, "[BAT_probe] g_platform_boot_mode = %d\n ", g_platform_boot_mode);

	wake_lock_init(&battery_fg_lock, WAKE_LOCK_SUSPEND, "battery fg wakelock");

	wake_lock_init(&battery_suspend_lock, WAKE_LOCK_SUSPEND, "battery suspend wakelock");
#if defined(CONFIG_MTK_PUMP_EXPRESS_SUPPORT) || defined(CONFIG_MTK_PUMP_EXPRESS_PLUS_SUPPORT)
	wake_lock_init(&TA_charger_suspend_lock, WAKE_LOCK_SUSPEND, "TA charger suspend wakelock");
#endif

	/* Integrate with Android Battery Service */
	ret = power_supply_register(&(dev->dev), &ac_main.psy);
	if (ret) {
		battery_log(BAT_LOG_CRTI, "[BAT_probe] power_supply_register AC Fail !!\n");
		return ret;
	}
	battery_log(BAT_LOG_CRTI, "[BAT_probe] power_supply_register AC Success !!\n");

	ret = power_supply_register(&(dev->dev), &usb_main.psy);
	if (ret) {
		battery_log(BAT_LOG_CRTI, "[BAT_probe] power_supply_register USB Fail !!\n");
		return ret;
	}
	battery_log(BAT_LOG_CRTI, "[BAT_probe] power_supply_register USB Success !!\n");

	ret = power_supply_register(&(dev->dev), &wireless_main.psy);
	if (ret) {
		battery_log(BAT_LOG_CRTI, "[BAT_probe] power_supply_register WIRELESS Fail !!\n");
		return ret;
	}
	battery_log(BAT_LOG_CRTI, "[BAT_probe] power_supply_register WIRELESS Success !!\n");

	ret = power_supply_register(&(dev->dev), &battery_main.psy);
	if (ret) {
		battery_log(BAT_LOG_CRTI, "[BAT_probe] power_supply_register Battery Fail !!\n");
		return ret;
	}
	battery_log(BAT_LOG_CRTI, "[BAT_probe] power_supply_register Battery Success !!\n");

#if !defined(CONFIG_POWER_EXT)

#ifdef CONFIG_MTK_POWER_EXT_DETECT
	if (KAL_TRUE == bat_is_ext_power()) {
		battery_main.BAT_STATUS = POWER_SUPPLY_STATUS_FULL;
		battery_main.BAT_HEALTH = POWER_SUPPLY_HEALTH_GOOD;
		battery_main.BAT_PRESENT = 1;
		battery_main.BAT_TECHNOLOGY = POWER_SUPPLY_TECHNOLOGY_LION;
		battery_main.BAT_CAPACITY = 100;
		battery_main.BAT_batt_vol = 4200;
		battery_main.BAT_batt_temp = 220;

		g_bat_init_flag = KAL_TRUE;
		return 0;
	}
#endif
	/* For EM */
	{
		int ret_device_file = 0;

		ret_device_file = device_create_file(&(dev->dev), &dev_attr_ADC_Charger_Voltage);

		ret_device_file = device_create_file(&(dev->dev), &dev_attr_ADC_Channel_0_Slope);
		ret_device_file = device_create_file(&(dev->dev), &dev_attr_ADC_Channel_1_Slope);
		ret_device_file = device_create_file(&(dev->dev), &dev_attr_ADC_Channel_2_Slope);
		ret_device_file = device_create_file(&(dev->dev), &dev_attr_ADC_Channel_3_Slope);
		ret_device_file = device_create_file(&(dev->dev), &dev_attr_ADC_Channel_4_Slope);
		ret_device_file = device_create_file(&(dev->dev), &dev_attr_ADC_Channel_5_Slope);
		ret_device_file = device_create_file(&(dev->dev), &dev_attr_ADC_Channel_6_Slope);
		ret_device_file = device_create_file(&(dev->dev), &dev_attr_ADC_Channel_7_Slope);
		ret_device_file = device_create_file(&(dev->dev), &dev_attr_ADC_Channel_8_Slope);
		ret_device_file = device_create_file(&(dev->dev), &dev_attr_ADC_Channel_9_Slope);
		ret_device_file = device_create_file(&(dev->dev), &dev_attr_ADC_Channel_10_Slope);
		ret_device_file = device_create_file(&(dev->dev), &dev_attr_ADC_Channel_11_Slope);
		ret_device_file = device_create_file(&(dev->dev), &dev_attr_ADC_Channel_12_Slope);
		ret_device_file = device_create_file(&(dev->dev), &dev_attr_ADC_Channel_13_Slope);

		ret_device_file = device_create_file(&(dev->dev), &dev_attr_ADC_Channel_0_Offset);
		ret_device_file = device_create_file(&(dev->dev), &dev_attr_ADC_Channel_1_Offset);
		ret_device_file = device_create_file(&(dev->dev), &dev_attr_ADC_Channel_2_Offset);
		ret_device_file = device_create_file(&(dev->dev), &dev_attr_ADC_Channel_3_Offset);
		ret_device_file = device_create_file(&(dev->dev), &dev_attr_ADC_Channel_4_Offset);
		ret_device_file = device_create_file(&(dev->dev), &dev_attr_ADC_Channel_5_Offset);
		ret_device_file = device_create_file(&(dev->dev), &dev_attr_ADC_Channel_6_Offset);
		ret_device_file = device_create_file(&(dev->dev), &dev_attr_ADC_Channel_7_Offset);
		ret_device_file = device_create_file(&(dev->dev), &dev_attr_ADC_Channel_8_Offset);
		ret_device_file = device_create_file(&(dev->dev), &dev_attr_ADC_Channel_9_Offset);
		ret_device_file = device_create_file(&(dev->dev), &dev_attr_ADC_Channel_10_Offset);
		ret_device_file = device_create_file(&(dev->dev), &dev_attr_ADC_Channel_11_Offset);
		ret_device_file = device_create_file(&(dev->dev), &dev_attr_ADC_Channel_12_Offset);
		ret_device_file = device_create_file(&(dev->dev), &dev_attr_ADC_Channel_13_Offset);

		ret_device_file =
		    device_create_file(&(dev->dev), &dev_attr_ADC_Channel_Is_Calibration);

		ret_device_file = device_create_file(&(dev->dev), &dev_attr_Power_On_Voltage);
		ret_device_file = device_create_file(&(dev->dev), &dev_attr_Power_Off_Voltage);
		ret_device_file = device_create_file(&(dev->dev), &dev_attr_Charger_TopOff_Value);

		ret_device_file =
		    device_create_file(&(dev->dev), &dev_attr_FG_Battery_CurrentConsumption);
		ret_device_file = device_create_file(&(dev->dev), &dev_attr_FG_SW_CoulombCounter);
		ret_device_file = device_create_file(&(dev->dev), &dev_attr_Charging_CallState);
		ret_device_file = device_create_file(&(dev->dev), &dev_attr_V_0Percent_Tracking);

		ret_device_file = device_create_file(&(dev->dev), &dev_attr_Charger_Type);
#if defined(CONFIG_MTK_PUMP_EXPRESS_SUPPORT) || defined(CONFIG_MTK_PUMP_EXPRESS_PLUS_SUPPORT)
		ret_device_file = device_create_file(&(dev->dev), &dev_attr_Pump_Express);
#endif
	}

	/* battery_meter_initial();      //move to mt_battery_GetBatteryData() to decrease booting time */

	/* Initialization BMT Struct */
	BMT_status.bat_exist = KAL_TRUE;	/* phone must have battery */
	BMT_status.charger_exist = KAL_FALSE;	/* for default, no charger */
	BMT_status.bat_vol = 0;
	BMT_status.ICharging = 0;
	BMT_status.temperature = 0;
	BMT_status.charger_vol = 0;
	BMT_status.total_charging_time = 0;
	BMT_status.PRE_charging_time = 0;
	BMT_status.CC_charging_time = 0;
	BMT_status.TOPOFF_charging_time = 0;
	BMT_status.POSTFULL_charging_time = 0;
	BMT_status.SOC = 0;
#ifdef CUST_CAPACITY_OCV2CV_TRANSFORM
	BMT_status.UI_SOC = -1;
#else
	BMT_status.UI_SOC = 0;
#endif

	BMT_status.bat_charging_state = CHR_PRE;
	BMT_status.bat_in_recharging_state = KAL_FALSE;
	BMT_status.bat_full = KAL_FALSE;
	BMT_status.nPercent_ZCV = 0;
	BMT_status.nPrecent_UI_SOC_check_point = battery_meter_get_battery_nPercent_UI_SOC();

#if defined(CONFIG_MTK_DUAL_INPUT_CHARGER_SUPPORT)
	dual_input_init();
#endif
	#ifdef CONFIG_HUAWEI_DSM
       bq_power_dclient = dsm_register_client(&dsm_mt_charger);
       if(!bq_power_dclient) {
               battery_log(BAT_LOG_CRTI, "%s: register dsm_dclient failed!!, line = %d\n", __func__,__LINE__);
       }
	#endif
	/* battery kernel thread for 10s check and charger in/out event */
	/* Replace GPT timer by hrtime */
	battery_kthread_hrtimer_init();
#ifdef CONFIG_HQ_HWINFO
		battery_hwinfo = hwinfo_create("battery_info");
		if(!battery_hwinfo)
			printk("battery hwinfo create fail\r\n");
		if(hwinfo_register(battery_hwinfo))
		printk("register battery hwinfo fail\r\n");
#endif
	kthread_run(bat_thread_kthread, NULL, "bat_thread_kthread");
#ifdef CONFIG_HUAWEI_DSM
    //kthread_run(bat_update_thread_dsm, NULL, "bat_update_thread_dsm");
#endif
	battery_log(BAT_LOG_CRTI, "[battery_probe] bat_thread_kthread Done\n");



	charger_hv_detect_sw_workaround_init();


	/*LOG System Set */
	init_proc_log();

#else
	/* keep HW alive */
	/* charger_hv_detect_sw_workaround_init(); */
#endif
	g_bat_init_flag = KAL_TRUE;

#if defined(CONFIG_MTK_DUAL_INPUT_CHARGER_SUPPORT)
	if (g_vcdt_irq_delay_flag == KAL_TRUE)
		do_chrdet_int_task();
#endif

	return 0;

}

static void battery_timer_pause(void)
{


	/* battery_log(BAT_LOG_CRTI, "******** battery driver suspend!! ********\n" ); */
#ifdef CONFIG_POWER_EXT
#else

#ifdef CONFIG_MTK_POWER_EXT_DETECT
	if (KAL_TRUE == bat_is_ext_power())
		return 0;
#endif
	mutex_lock(&bat_mutex);
	/* cancel timer */
	hrtimer_cancel(&battery_kthread_timer);
	hrtimer_cancel(&charger_hv_detect_timer);

	battery_suspended = KAL_TRUE;
	mutex_unlock(&bat_mutex);

	battery_log(BAT_LOG_FULL, "@bs=1@\n");
#endif

	get_monotonic_boottime(&g_bat_time_before_sleep);
}

static void battery_timer_resume(void)
{
#ifdef CONFIG_POWER_EXT
#else
	kal_bool is_pcm_timer_trigger = KAL_FALSE;
	struct timespec bat_time_after_sleep;
	ktime_t ktime, hvtime;

#ifdef CONFIG_MTK_POWER_EXT_DETECT
	if (KAL_TRUE == bat_is_ext_power())
		return 0;
#endif

	ktime = ktime_set(BAT_TASK_PERIOD, 0);	/* 10s, 10* 1000 ms */
	hvtime = ktime_set(0, BAT_MS_TO_NS(2000));

	get_monotonic_boottime(&bat_time_after_sleep);
	battery_charging_control(CHARGING_CMD_GET_IS_PCM_TIMER_TRIGGER, &is_pcm_timer_trigger);

	if (is_pcm_timer_trigger == KAL_TRUE || bat_spm_timeout) {
		mutex_lock(&bat_mutex);
		BAT_thread();
		mutex_unlock(&bat_mutex);
	} else {
		battery_log(BAT_LOG_CRTI, "battery resume NOT by pcm timer!!\n");
	}

	if (g_call_state == CALL_ACTIVE &&
		(bat_time_after_sleep.tv_sec - g_bat_time_before_sleep.tv_sec >= batt_cust_data.talking_sync_time)) {
		/* phone call last than x min */
		BMT_status.UI_SOC = battery_meter_get_battery_percentage();
		battery_log(BAT_LOG_CRTI, "Sync UI SOC to SOC immediately\n");
	}

	mutex_lock(&bat_mutex);

	/* restore timer */
	hrtimer_start(&battery_kthread_timer, ktime, HRTIMER_MODE_REL);
	hrtimer_start(&charger_hv_detect_timer, hvtime, HRTIMER_MODE_REL);

	battery_suspended = KAL_FALSE;
	battery_log(BAT_LOG_FULL, "@bs=0@\n");
	mutex_unlock(&bat_mutex);

#endif
}

static int battery_remove(struct platform_device *dev)
{
	battery_log(BAT_LOG_CRTI, "******** battery driver remove!! ********\n");
	#ifdef CONFIG_HUAWEI_DSM
       dsm_unregister_client(bq_power_dclient,&dsm_mt_charger);
	#endif
	return 0;
}

static void battery_shutdown(struct platform_device *dev)
{
	battery_log(BAT_LOG_CRTI, "******** battery driver shutdown!! ********\n");

}

/* ///////////////////////////////////////////////////////////////////////////////////////// */
/* // Battery Notify API */
/* ///////////////////////////////////////////////////////////////////////////////////////// */
static ssize_t show_BatteryNotify(struct device *dev, struct device_attribute *attr, char *buf)
{
	battery_log(BAT_LOG_CRTI, "[Battery] show_BatteryNotify : %x\n", g_BatteryNotifyCode);

	return sprintf(buf, "%u\n", g_BatteryNotifyCode);
}

static ssize_t store_BatteryNotify(struct device *dev, struct device_attribute *attr,
				   const char *buf, size_t size)
{
	/*char *pvalue = NULL;*/
	int rv;
	unsigned long reg_BatteryNotifyCode = 0;

	battery_log(BAT_LOG_CRTI, "[Battery] store_BatteryNotify\n");
	if (buf != NULL && size != 0) {
		battery_log(BAT_LOG_CRTI, "[Battery] buf is %s and size is %Zu\n", buf, size);
		rv = kstrtoul(buf, 0, &reg_BatteryNotifyCode);
		if (rv != 0)
			return -EINVAL;
		g_BatteryNotifyCode = reg_BatteryNotifyCode;
		battery_log(BAT_LOG_CRTI, "[Battery] store code : %x\n", g_BatteryNotifyCode);
	}
	return size;
}

static DEVICE_ATTR(BatteryNotify, 0664, show_BatteryNotify, store_BatteryNotify);

static ssize_t show_BN_TestMode(struct device *dev, struct device_attribute *attr, char *buf)
{
	battery_log(BAT_LOG_CRTI, "[Battery] show_BN_TestMode : %x\n", g_BN_TestMode);
	return sprintf(buf, "%u\n", g_BN_TestMode);
}

static ssize_t store_BN_TestMode(struct device *dev, struct device_attribute *attr, const char *buf,
				 size_t size)
{
	/*char *pvalue = NULL;*/
	int rv;
	unsigned long reg_BN_TestMode = 0;

	battery_log(BAT_LOG_CRTI, "[Battery] store_BN_TestMode\n");
	if (buf != NULL && size != 0) {
		battery_log(BAT_LOG_CRTI, "[Battery] buf is %s and size is %Zu\n", buf, size);
		rv = kstrtoul(buf, 0, &reg_BN_TestMode);
		if (rv != 0)
			return -EINVAL;
		g_BN_TestMode = reg_BN_TestMode;
		battery_log(BAT_LOG_CRTI, "[Battery] store g_BN_TestMode : %x\n", g_BN_TestMode);
	}
	return size;
}

static DEVICE_ATTR(BN_TestMode, 0664, show_BN_TestMode, store_BN_TestMode);


/* ///////////////////////////////////////////////////////////////////////////////////////// */
/* // platform_driver API */
/* ///////////////////////////////////////////////////////////////////////////////////////// */
#if 0
static int battery_cmd_read(char *buf, char **start, off_t off, int count, int *eof, void *data)
{
	int len = 0;
	char *p = buf;

	p += sprintf(p,
		     "g_battery_thermal_throttling_flag=%d,\nbattery_cmd_thermal_test_mode=%d,\nbattery_cmd_thermal_test_mode_value=%d\n",
		     g_battery_thermal_throttling_flag, battery_cmd_thermal_test_mode,
		     battery_cmd_thermal_test_mode_value);

	*start = buf + off;

	len = p - buf;
	if (len > off)
		len -= off;
	else
		len = 0;

	return len < count ? len : count;
}
#endif

static ssize_t battery_cmd_write(struct file *file, const char *buffer, size_t count, loff_t *data)
{
	int len = 0, bat_tt_enable = 0, bat_thr_test_mode = 0, bat_thr_test_value = 0;
	char desc[32];

	len = (count < (sizeof(desc) - 1)) ? count : (sizeof(desc) - 1);
	if (copy_from_user(desc, buffer, len))
		return 0;

	desc[len] = '\0';

	if (sscanf(desc, "%d %d %d", &bat_tt_enable, &bat_thr_test_mode, &bat_thr_test_value) == 3) {
		g_battery_thermal_throttling_flag = bat_tt_enable;
		battery_cmd_thermal_test_mode = bat_thr_test_mode;
		battery_cmd_thermal_test_mode_value = bat_thr_test_value;

		battery_log(BAT_LOG_CRTI,
			    "bat_tt_enable=%d, bat_thr_test_mode=%d, bat_thr_test_value=%d\n",
			    g_battery_thermal_throttling_flag,
			    battery_cmd_thermal_test_mode, battery_cmd_thermal_test_mode_value);

		return count;
	} /*else {*/
		battery_log(BAT_LOG_CRTI,
			    "  bad argument, echo [bat_tt_enable] [bat_thr_test_mode] [bat_thr_test_value] > battery_cmd\n");
	/*}*/

	return -EINVAL;
}

static int proc_utilization_show(struct seq_file *m, void *v)
{
	seq_printf(m,
		   "=> g_battery_thermal_throttling_flag=%d,\nbattery_cmd_thermal_test_mode=%d,\nbattery_cmd_thermal_test_mode_value=%d\n",
		   g_battery_thermal_throttling_flag, battery_cmd_thermal_test_mode,
		   battery_cmd_thermal_test_mode_value);

	seq_printf(m, "=> get_usb_current_unlimited=%d,\ncmd_discharging = %d\n",
		   get_usb_current_unlimited(), cmd_discharging);
	return 0;
}

static int proc_utilization_open(struct inode *inode, struct file *file)
{
	return single_open(file, proc_utilization_show, NULL);
}

static const struct file_operations battery_cmd_proc_fops = {
	.open = proc_utilization_open,
	.read = seq_read,
	.write = battery_cmd_write,
};

static ssize_t current_cmd_write(struct file *file, const char *buffer, size_t count, loff_t *data)
{
	int len = 0;
	char desc[32];
	int cmd_current_unlimited = false;
	unsigned int charging_enable = false;

	len = (count < (sizeof(desc) - 1)) ? count : (sizeof(desc) - 1);
	if (copy_from_user(desc, buffer, len))
		return 0;

	desc[len] = '\0';

	if (sscanf(desc, "%d %d", &cmd_current_unlimited, &cmd_discharging) == 2) {
		set_usb_current_unlimited(cmd_current_unlimited);
		if (cmd_discharging == 1) {
			charging_enable = false;
			adjust_power = -1;
		} else if (cmd_discharging == 0) {
			charging_enable = true;
			adjust_power = -1;
		}
		battery_charging_control(CHARGING_CMD_ENABLE, &charging_enable);

		battery_log(BAT_LOG_CRTI,
			    "[current_cmd_write] cmd_current_unlimited=%d, cmd_discharging=%d\n",
			    cmd_current_unlimited, cmd_discharging);
		return count;
	} /*else {*/
		battery_log(BAT_LOG_CRTI, "  bad argument, echo [enable] > current_cmd\n");
	/*}*/

	return -EINVAL;
}

static int current_cmd_read(struct seq_file *m, void *v)
{
	unsigned int charging_enable = false;

	cmd_discharging = 1;
	charging_enable = false;
	adjust_power = -1;

	battery_charging_control(CHARGING_CMD_ENABLE, &charging_enable);

	battery_log(BAT_LOG_CRTI, "[current_cmd_write] cmd_discharging=%d\n", cmd_discharging);

	return 0;
}

static int proc_utilization_open_cur_stop(struct inode *inode, struct file *file)
{
	return single_open(file, current_cmd_read, NULL);
}

static ssize_t discharging_cmd_write(struct file *file, const char *buffer, size_t count,
				     loff_t *data)
{
	int len = 0;
	char desc[32];
	unsigned int charging_enable = false;

	len = (count < (sizeof(desc) - 1)) ? count : (sizeof(desc) - 1);
	if (copy_from_user(desc, buffer, len))
		return 0;

	desc[len] = '\0';

	if (sscanf(desc, "%d %d", &charging_enable, &adjust_power) == 2) {
		battery_log(BAT_LOG_CRTI, "[current_cmd_write] adjust_power = %d\n", adjust_power);
		return count;
	} /*else {*/
		battery_log(BAT_LOG_CRTI, "  bad argument, echo [enable] > current_cmd\n");
	/*}*/

	return -EINVAL;
}

static const struct file_operations discharging_cmd_proc_fops = {
	.open = proc_utilization_open,
	.read = seq_read,
	.write = discharging_cmd_write,
};

static const struct file_operations current_cmd_proc_fops = {
	.open = proc_utilization_open_cur_stop,
	.read = seq_read,
	.write = current_cmd_write,
};

static int mt_batteryNotify_probe(struct platform_device *dev)
{
	int ret_device_file = 0;
	/* struct proc_dir_entry *entry = NULL; */
	struct proc_dir_entry *battery_dir = NULL;

	battery_log(BAT_LOG_CRTI, "******** mt_batteryNotify_probe!! ********\n");

	ret_device_file = device_create_file(&(dev->dev), &dev_attr_BatteryNotify);
	ret_device_file = device_create_file(&(dev->dev), &dev_attr_BN_TestMode);

	battery_dir = proc_mkdir("mtk_battery_cmd", NULL);
	if (!battery_dir) {
		pr_err("[%s]: mkdir /proc/mtk_battery_cmd failed\n", __func__);
	} else {
#if 1
		proc_create("battery_cmd", S_IRUGO | S_IWUSR, battery_dir, &battery_cmd_proc_fops);
		battery_log(BAT_LOG_CRTI, "proc_create battery_cmd_proc_fops\n");

		proc_create("current_cmd", S_IRUGO | S_IWUSR, battery_dir, &current_cmd_proc_fops);
		battery_log(BAT_LOG_CRTI, "proc_create current_cmd_proc_fops\n");
		proc_create("discharging_cmd", S_IRUGO | S_IWUSR, battery_dir,
			    &discharging_cmd_proc_fops);
		battery_log(BAT_LOG_CRTI, "proc_create discharging_cmd_proc_fops\n");


#else
		entry = create_proc_entry("battery_cmd", S_IRUGO | S_IWUSR, battery_dir);
		if (entry) {
			entry->read_proc = battery_cmd_read;
			entry->write_proc = battery_cmd_write;
		}
#endif
	}

	battery_log(BAT_LOG_CRTI, "******** mtk_battery_cmd!! ********\n");

	return 0;

}

#ifdef CONFIG_OF
static const struct of_device_id mt_battery_of_match[] = {
	{.compatible = "mediatek,battery",},
	{},
};

MODULE_DEVICE_TABLE(of, mt_battery_of_match);
#endif

static int battery_pm_suspend(struct device *device)
{
	int ret = 0;

	struct platform_device *pdev = to_platform_device(device);

	BUG_ON(pdev == NULL);

	return ret;
}

static int battery_pm_resume(struct device *device)
{
	int ret = 0;

	struct platform_device *pdev = to_platform_device(device);

	BUG_ON(pdev == NULL);

	return ret;
}

static int battery_pm_freeze(struct device *device)
{
	int ret = 0;

	struct platform_device *pdev = to_platform_device(device);

	BUG_ON(pdev == NULL);

	return ret;
}

static int battery_pm_restore(struct device *device)
{
	int ret = 0;

	struct platform_device *pdev = to_platform_device(device);

	BUG_ON(pdev == NULL);

	return ret;
}

static int battery_pm_restore_noirq(struct device *device)
{
	int ret = 0;

	struct platform_device *pdev = to_platform_device(device);

	BUG_ON(pdev == NULL);

	return ret;
}

const struct dev_pm_ops battery_pm_ops = {
	.suspend = battery_pm_suspend,
	.resume = battery_pm_resume,
	.freeze = battery_pm_freeze,
	.thaw = battery_pm_restore,
	.restore = battery_pm_restore,
	.restore_noirq = battery_pm_restore_noirq,
};

#if defined(CONFIG_OF) || defined(BATTERY_MODULE_INIT)
struct platform_device battery_device = {
	.name = "battery",
	.id = -1,
};
#endif

static struct platform_driver battery_driver = {
	.probe = battery_probe,
	.remove = battery_remove,
	.shutdown = battery_shutdown,
	.driver = {
		   .name = "battery",
		   .pm = &battery_pm_ops,
		   },
};

#ifdef CONFIG_OF
static int battery_dts_probe(struct platform_device *dev)
{
	int ret = 0;

	battery_log(BAT_LOG_CRTI, "******** battery_dts_probe!! ********\n");

	battery_device.dev.of_node = dev->dev.of_node;
	ret = platform_device_register(&battery_device);
	if (ret) {
		battery_log(BAT_LOG_CRTI,
			    "****[battery_dts_probe] Unable to register device (%d)\n", ret);
		return ret;
	}
	return 0;

}

static struct platform_driver battery_dts_driver = {
	.probe = battery_dts_probe,
	.remove = NULL,
	.shutdown = NULL,
	.driver = {
		   .name = "battery-dts",
#ifdef CONFIG_OF
		   .of_match_table = mt_battery_of_match,
#endif
		   },
};

/* -------------------------------------------------------- */

static const struct of_device_id mt_bat_notify_of_match[] = {
	{.compatible = "mediatek,bat_notify",},
	{},
};

MODULE_DEVICE_TABLE(of, mt_bat_notify_of_match);
#endif

struct platform_device MT_batteryNotify_device = {
	.name = "mt-battery",
	.id = -1,
};

static struct platform_driver mt_batteryNotify_driver = {
	.probe = mt_batteryNotify_probe,
	.driver = {
		   .name = "mt-battery",
		   },
};

#ifdef CONFIG_OF
static int mt_batteryNotify_dts_probe(struct platform_device *dev)
{
	int ret = 0;
	/* struct proc_dir_entry *entry = NULL; */

	battery_log(BAT_LOG_CRTI, "******** mt_batteryNotify_dts_probe!! ********\n");

	MT_batteryNotify_device.dev.of_node = dev->dev.of_node;
	ret = platform_device_register(&MT_batteryNotify_device);
	if (ret) {
		battery_log(BAT_LOG_CRTI,
			    "****[mt_batteryNotify_dts] Unable to register device (%d)\n", ret);
		return ret;
	}
	return 0;

}


static struct platform_driver mt_batteryNotify_dts_driver = {
	.probe = mt_batteryNotify_dts_probe,
	.driver = {
		   .name = "mt-dts-battery",
#ifdef CONFIG_OF
		   .of_match_table = mt_bat_notify_of_match,
#endif
		   },
};
#endif
/* -------------------------------------------------------- */

static int battery_pm_event(struct notifier_block *notifier, unsigned long pm_event, void *unused)
{
	switch (pm_event) {
	case PM_HIBERNATION_PREPARE:	/* Going to hibernate */
	case PM_RESTORE_PREPARE:	/* Going to restore a saved image */
	case PM_SUSPEND_PREPARE:	/* Going to suspend the system */
		pr_debug("[%s] pm_event %lu\n", __func__, pm_event);
		battery_timer_pause();
		return NOTIFY_DONE;
	case PM_POST_HIBERNATION:	/* Hibernation finished */
	case PM_POST_SUSPEND:	/* Suspend finished */
	case PM_POST_RESTORE:	/* Restore failed */
		pr_debug("[%s] pm_event %lu\n", __func__, pm_event);
		battery_timer_resume();
		return NOTIFY_DONE;
	}
	return NOTIFY_OK;
}

static struct notifier_block battery_pm_notifier_block = {
	.notifier_call = battery_pm_event,
	.priority = 0,
};

static int __init battery_init(void)
{
	int ret;

	pr_debug("battery_init\n");

#ifdef CONFIG_OF
	/*  */
#else

#ifdef BATTERY_MODULE_INIT
	ret = platform_device_register(&battery_device);
	if (ret) {
		battery_log(BAT_LOG_CRTI,
			    "****[battery_device] Unable to device register(%d)\n", ret);
		return ret;
	}
#endif
#endif

	ret = platform_driver_register(&battery_driver);
	if (ret) {
		battery_log(BAT_LOG_CRTI,
			    "****[battery_driver] Unable to register driver (%d)\n", ret);
		return ret;
	}
	/* battery notofy UI */
#ifdef CONFIG_OF
	/*  */
#else
	ret = platform_device_register(&MT_batteryNotify_device);
	if (ret) {
		battery_log(BAT_LOG_CRTI,
			    "****[mt_batteryNotify] Unable to device register(%d)\n", ret);
		return ret;
	}
#endif
	ret = platform_driver_register(&mt_batteryNotify_driver);
	if (ret) {
		battery_log(BAT_LOG_CRTI,
			    "****[mt_batteryNotify] Unable to register driver (%d)\n", ret);
		return ret;
	}
#ifdef CONFIG_OF
	ret = platform_driver_register(&battery_dts_driver);
	ret = platform_driver_register(&mt_batteryNotify_dts_driver);
#endif
	ret = register_pm_notifier(&battery_pm_notifier_block);
	if (ret)
		pr_debug("[%s] failed to register PM notifier %d\n", __func__, ret);

	battery_log(BAT_LOG_CRTI, "****[battery_driver] Initialization : DONE !!\n");
	return 0;
}

#ifdef BATTERY_MODULE_INIT
late_initcall(battery_init);
#else
static void __exit battery_exit(void)
{
}
late_initcall(battery_init);

#endif

MODULE_AUTHOR("Oscar Liu");
MODULE_DESCRIPTION("Battery Device Driver");
MODULE_LICENSE("GPL");
