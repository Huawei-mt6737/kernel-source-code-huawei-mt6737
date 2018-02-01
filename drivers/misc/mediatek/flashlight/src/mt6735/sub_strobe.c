
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/types.h>
#include <linux/wait.h>
#include <linux/slab.h>
#include <linux/fs.h>
#include <linux/sched.h>
#include <linux/poll.h>
#include <linux/device.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/cdev.h>
#include <linux/errno.h>
#include <linux/time.h>
#include <asm/io.h>
#include <asm/uaccess.h>
#include "kd_camera_typedef.h"
#include <linux/hrtimer.h>
#include <linux/ktime.h>
#include <linux/version.h>
#ifdef CONFIG_COMPAT
#include <linux/fs.h>
#include <linux/compat.h>
#endif
#include "kd_flashlight.h"
/******************************************************************************
 * Debug configuration
******************************************************************************/
/* availible parameter */
/* ANDROID_LOG_ASSERT */
/* ANDROID_LOG_ERROR */
/* ANDROID_LOG_WARNING */
/* ANDROID_LOG_INFO */
/* ANDROID_LOG_DEBUG */
/* ANDROID_LOG_VERBOSE */
#define TAG_NAME "[sub_strobe.c]"
#define PK_DBG_NONE(fmt, arg...)    do {} while (0)
#define PK_DBG_FUNC(fmt, arg...)    pr_debug(TAG_NAME "%s: " fmt, __func__ , ##arg)
#define PK_WARN(fmt, arg...)        pr_warn(TAG_NAME "%s: " fmt, __func__ , ##arg)
#define PK_NOTICE(fmt, arg...)      pr_notice(TAG_NAME "%s: " fmt, __func__ , ##arg)
#define PK_INFO(fmt, arg...)        pr_info(TAG_NAME "%s: " fmt, __func__ , ##arg)
#define PK_TRC_FUNC(f)              pr_debug(TAG_NAME "<%s>\n", __func__)
#define PK_TRC_VERBOSE(fmt, arg...) pr_debug(TAG_NAME fmt, ##arg)
#define PK_ERROR(fmt, arg...)       pr_err(TAG_NAME "%s: " fmt, __func__ , ##arg)
#include "mt_pwm.h"

#define DEBUG_LEDS_STROBE
#ifdef DEBUG_LEDS_STROBE
#define PK_DBG PK_DBG_FUNC
#define PK_VER PK_TRC_VERBOSE
#define PK_ERR PK_ERROR
#else
#define PK_DBG(a, ...)
#define PK_VER(a, ...)
#define PK_ERR(a, ...)
#endif
#define LEDS_CUSTOM_MODE_THRES 	20
struct pwm_spec_config sub_strob_pwm_config = {
    .pwm_no = PWM4,
    .mode = PWM_MODE_OLD,
    .clk_div = CLK_DIV1,//CLK_DIV16: 147.7kHz    CLK_DIV32: 73.8kHz
    .clk_src = PWM_CLK_OLD_MODE_32K,
    .pmic_pad = false,
    .PWM_MODE_OLD_REGS.IDLE_VALUE = IDLE_FALSE,
    .PWM_MODE_OLD_REGS.GUARD_VALUE = GUARD_FALSE,
    .PWM_MODE_OLD_REGS.GDURATION = 0,
    .PWM_MODE_OLD_REGS.WAVE_NUM = 0,
    .PWM_MODE_OLD_REGS.DATA_WIDTH = 4,
    .PWM_MODE_OLD_REGS.THRESH = 2,
};

static DEFINE_SPINLOCK(g_strobeSMPLock); /* cotta-- SMP proection */

static int g_timeOutTimeMs=0;
static BOOL g_is_torch_mode = 0;
static U16 g_duty = 0;
static struct work_struct workTimeOut;
static struct hrtimer g_timeOutTimer;
static u32 strobe_Res = 0;
static u32 strobe_Timeus = 0;
static BOOL g_strobe_On = 0;
static int duty_map[] = {40,32,20,24,48,64,80,96};
struct pinctrl *sub_strob_gpio_ctrl = NULL;
struct pinctrl_state *sub_strob__gpio_default = NULL;
struct pinctrl_state *sub_strob__gpio_out_h = NULL;
struct pinctrl_state *sub_strob__gpio_out_l = NULL;
struct pinctrl_state *sub_strob__pwm_out_h = NULL;
struct pinctrl_state *sub_strob__pwm_out_l = NULL;

int sub_strob_get_gpio_info(struct platform_device *pdev)
{
    int ret;
    printk("shenwuyi add sub_strob_get_gpio_info\n");
    sub_strob_gpio_ctrl = devm_pinctrl_get(&pdev->dev);
    if (IS_ERR(sub_strob_gpio_ctrl)) {
        dev_err(&pdev->dev, "Cannot find sub_strob_gpio_ctrl!");
        ret = PTR_ERR(sub_strob_gpio_ctrl);
        return ret;
    }
    sub_strob__gpio_default = pinctrl_lookup_state(sub_strob_gpio_ctrl, "sub_strob_default");
    
    sub_strob__gpio_out_h = pinctrl_lookup_state(sub_strob_gpio_ctrl, "sub_strob_gpio_out_h");   
    if (IS_ERR(sub_strob__gpio_out_h)) {
        ret = PTR_ERR(sub_strob__gpio_out_h);
        pr_debug("%s : pinctrl err, sub_strob__gpio_out_h \n", __func__);
    }

    sub_strob__gpio_out_l = pinctrl_lookup_state(sub_strob_gpio_ctrl, "sub_strob_gpio_out_l");   
    if (IS_ERR(sub_strob__gpio_out_l)) {
        ret = PTR_ERR(sub_strob__gpio_out_l);
        pr_debug("%s : pinctrl err, sub_strob_gpio_out_l \n", __func__);
    }
    
    sub_strob__pwm_out_h = pinctrl_lookup_state(sub_strob_gpio_ctrl, "sub_strob_pwm_out_h");   
    if (IS_ERR(sub_strob__pwm_out_h)) {
        ret = PTR_ERR(sub_strob__pwm_out_h);
        pr_debug("%s : pinctrl err, sub_strob__pwm_out_h \n", __func__);
    }

    sub_strob__pwm_out_l = pinctrl_lookup_state(sub_strob_gpio_ctrl, "sub_strob_pwm_out_l");   
    if (IS_ERR(sub_strob__pwm_out_l)) {
        ret = PTR_ERR(sub_strob__pwm_out_l);
        pr_debug("%s : pinctrl err, sub_strob_pwm_out_l \n", __func__);
    }
    printk("[flashlight_gpio%d] flashlight_gpio_pinctrl----------\n", pdev->id);
    return 0;
}


int FL_enable_sub(void)
{
	PK_DBG("FL_enable_subn");	
    pinctrl_select_state(sub_strob_gpio_ctrl, sub_strob__pwm_out_h);
    mdelay(1);
    pwm_set_spec_config(&sub_strob_pwm_config);  
    return 0;
    
}
int FL_disable_sub(void)
{
	PK_DBG("FL_disable_sub");
    pinctrl_select_state(sub_strob_gpio_ctrl, sub_strob__gpio_out_l);

    return 0;
}

static ssize_t restore_sub_strob_on(struct device *dev,struct device_attribute *attr, const char *buf, size_t size)
{
	kal_int32 temp = 0;
	

	sscanf(buf, "%d", &temp);
	
    PK_DBG("[restore_sub_strob_on] restore_sub_strob_on temp = %d\n",temp);
	

	if(temp > 0){
	sub_strob_pwm_config.PWM_MODE_OLD_REGS.THRESH = temp;
	FL_enable_sub();
	}else{
	FL_disable_sub();
	}
    
    return size;
}


static ssize_t show_sub_strob_on(struct device *dev,struct device_attribute *attr, char *buf)
{
      return  sprintf(buf, "show_sub_strob_on\n");
}

static DEVICE_ATTR(sub_strob_on,  0664, show_sub_strob_on, restore_sub_strob_on);

static int sub_strob_probe(struct platform_device *pdev)
{ 
 
 int ret_device_file=0;
  sub_strob_get_gpio_info(pdev);
  ret_device_file = device_create_file(&(pdev->dev), &dev_attr_sub_strob_on);
  ret_device_file = device_create_file(&(pdev->dev), &dev_attr_sub_strob_on);
  return 0;
}
/*
static int sub_strob_probe(struct platform_device *dev)
{ 

  int ret_device_file=0;
  PK_DBG("sub_strob_prob");
  hw_init();
  ret_device_file = device_create_file(&(dev->dev), &dev_attr_sgm3785_flash_mode);
  ret_device_file = device_create_file(&(dev->dev), &dev_attr_sgm3785_torch_mode);
 
  return 0;
}
*/
static int sub_strob_remove(struct platform_device *dev)    
{
    PK_DBG("sub_strob_remove");
    return 0;
}
static int sub_strob_suspend(struct platform_device *dev, pm_message_t state)    
{
    PK_DBG("sub_strob_suspend");   
    return 0;
}

static int sub_strob_resume(struct platform_device *dev)
{
	
   PK_DBG("sub_strob_resume"); 
   return 0;
}

static void sub_strob_shutdown(struct platform_device *dev)    
{
	PK_DBG("sub_strob_shutdown\n");
    return 0;
}

struct platform_device sub_strob_device = {
    .name   = "sub_strob",
    .id	    = -1,
};

static const struct of_device_id sub_strob_gpio_of_match[] = {
{.compatible = "mediatek,sub_strob",},
{}
};
static struct platform_driver sub_strob_driver = {
    .probe       = sub_strob_probe,
    .remove      = sub_strob_remove,
    .shutdown    = sub_strob_shutdown,
    .suspend     = sub_strob_suspend,
    .resume      = sub_strob_resume,
    .driver      = {
        .name = "sub_flashlight",
   
    #ifdef CONFIG_OF
       .of_match_table = sub_strob_gpio_of_match,
	#endif
	}
};

/************ sgm end*********************/


static int sub_strob_init(void)
{
 
    int ret;
/*
    ret = platform_device_register(&flash_leds_sgm3785_device);
    if (ret) {
    printk("<<-sgm3785->> Unable to register device (%d)\n", ret);
	return ret;
    }
*/   
    PK_DBG("sub_strob_init\n");
    ret = platform_driver_register(&sub_strob_driver);
    if (ret) {
    PK_DBG("<<-sub_strob_init->> Unable to register driver (%d)\n", ret);
	return ret;
    }
    return 0;    
}

static void sub_strob_exit(void)
{
	PK_DBG("sub_strob_exit");
	platform_device_unregister(&sub_strob_device);
	platform_driver_unregister(&sub_strob_driver);

}

module_init(sub_strob_init);
module_exit(sub_strob_exit);

MODULE_DESCRIPTION("Flash driver for SUB_STROB");
MODULE_AUTHOR("shenwuyi@huaqin.com>");
MODULE_LICENSE("GPL v2");


static enum hrtimer_restart ledTimeOutCallback(struct hrtimer *timer)
{
    schedule_work(&workTimeOut);
    return HRTIMER_NORESTART;
}

static void work_timeOutFunc(struct work_struct *data)
{
    FL_disable_sub();
    PK_DBG("ledTimeOut_callback\n");
    //printk(KERN_ALERT "work handler function./n");
}

static void timerInit(void)
{
  INIT_WORK(&workTimeOut, work_timeOutFunc);
	g_timeOutTimeMs=1000; //1s
	hrtimer_init( &g_timeOutTimer, CLOCK_MONOTONIC, HRTIMER_MODE_REL );
	g_timeOutTimer.function=ledTimeOutCallback;

}



int FL_dim_duty_sub(kal_uint32 duty)
{
	if(duty == 1)	{
		g_is_torch_mode = 1;
        sub_strob_pwm_config.PWM_MODE_OLD_REGS.THRESH = 2;
	}
	else{
		g_is_torch_mode = 0;	
        sub_strob_pwm_config.PWM_MODE_OLD_REGS.THRESH = 4;
	}
	if((g_timeOutTimeMs == 0) && (duty > 1))
	{
	    
		g_duty = duty_map[duty];
		PK_ERR("FL_dim_duty_sub %d > thres %d, FLASH mode but timeout %d \n", duty, LEDS_CUSTOM_MODE_THRES, g_timeOutTimeMs);	
		g_is_torch_mode = 0;	
	}	
//	upmu_set_flash_dim_duty(duty);
	PK_DBG("wangyang FL_dim_duty_sub %d, g_is_torch_mode %d \n", duty, g_is_torch_mode);

    return 0;
}

int FL_step_sub(kal_uint32 step)
{
	int sTab[8]={0,2,4,6,9,11,13,15};
	PK_DBG("FL_step_sub");
//	upmu_set_flash_sel(sTab[step]);	
    return 0;
}

int FL_init_sub(void)
{
//	upmu_set_flash_dim_duty(0);
//	upmu_set_flash_sel(0);
	PK_DBG("FL_init");
	FL_disable_sub();
	INIT_WORK(&workTimeOut, work_timeOutFunc);
	g_is_torch_mode = 0;
    return 0;
}

int FL_uninit_sub(void)
{
	PK_DBG("FL_uninit_sub");
	FL_disable_sub();
	g_is_torch_mode = 0;
    return 0;
}

/*****************************************************************************
User interface
*****************************************************************************/


static int sub_strobe_ioctl(unsigned int cmd, unsigned long arg)
{
    int i4RetValue = 0;
    int ior_shift;
    int iow_shift;
    int iowr_shift;
    int iFlashType = (int)FLASHLIGHT_NONE;
    ior_shift = cmd - (_IOR(FLASHLIGHT_MAGIC,0, int));
    iow_shift = cmd - (_IOW(FLASHLIGHT_MAGIC,0, int));
    iowr_shift = cmd - (_IOWR(FLASHLIGHT_MAGIC,0, int));
    PK_DBG("sub_strobe_ioctl() line=%d ior_shift=%d, iow_shift=%d iowr_shift=%d arg=%d\n",__LINE__, ior_shift, iow_shift, iowr_shift, arg);
    switch(cmd)
    {

        case FLASH_IOC_SET_TIME_OUT_TIME_MS:
            PK_DBG("sub_strobe_ioctl_TIME_OUT_TIME_MS: %d\n",arg);
            g_timeOutTimeMs=arg;
        break;


        case FLASH_IOC_SET_DUTY :
            PK_DBG("sub_strobe_ioctl_DUTY: %d\n",arg);
            FL_dim_duty_sub(arg);
            break;


        case FLASH_IOC_SET_STEP:
            PK_DBG("sub_strobe_ioctl_STEP: %d\n",arg);

            break;

        case FLASH_IOC_SET_ONOFF :
            PK_DBG("sub_strobe_ioctlF: %d\n",arg);
            if(arg==1)
            {
                if(g_timeOutTimeMs!=0)
                {
                    ktime_t ktime;
                    ktime = ktime_set( 0, g_timeOutTimeMs*1000000 );
                    hrtimer_start( &g_timeOutTimer, ktime, HRTIMER_MODE_REL );
                }
                FL_enable_sub();
            }
            else
            {
                FL_disable_sub();
                hrtimer_cancel( &g_timeOutTimer );
            }
            break;
        //case FLASHLIGHTIOC_G_FLASHTYPE:
        //    iFlashType = FLASHLIGHT_LED_CONSTANT;
         //   if(copy_to_user((void __user *) arg , (void*)&iFlashType , _IOC_SIZE(cmd)))
         //   {
         //       PK_DBG("[strobe_ioctl] ioctl copy to user failed\n");
         //       return -EFAULT;
         //   }
         //   break;            
        default :
            PK_DBG(" No such command \n");
            i4RetValue = -EPERM;
            break;
    }
    return i4RetValue;
}


static int sub_strobe_open(void *pArg)
{
    int i4RetValue = 0;
    PK_DBG("sub_strobe_open line=%d\n", __LINE__);

    if (0 == strobe_Res)
    {
        FL_init_sub();
        timerInit();
    }
    PK_DBG("sub_strobe_open line=%d\n", __LINE__);
    spin_lock_irq(&g_strobeSMPLock);


    if(strobe_Res)
    {
        PK_ERR(" busy!\n");
        i4RetValue = -EBUSY;
    }
    else
    {
        strobe_Res += 1;
    }


    spin_unlock_irq(&g_strobeSMPLock);
    PK_DBG("sub_strobe_open=%d\n", __LINE__);

    return i4RetValue;

}

static int sub_strobe_release(void *pArg)
{
    PK_DBG(" sub_strobe_release\n");

    if (strobe_Res)
    {
        spin_lock_irq(&g_strobeSMPLock);

        strobe_Res = 0;
        strobe_Timeus = 0;

        /* LED On Status */
        g_strobe_On = FALSE;

        spin_unlock_irq(&g_strobeSMPLock);

        FL_uninit_sub();
    }

    PK_DBG(" Done\n");

    return 0;

}

FLASHLIGHT_FUNCTION_STRUCT subStrobeFunc = {
	sub_strobe_open,
	sub_strobe_release,
	sub_strobe_ioctl
};


MUINT32 subStrobeInit(PFLASHLIGHT_FUNCTION_STRUCT *pfFunc)
{
	if (pfFunc != NULL)
		*pfFunc = &subStrobeFunc;
	return 0;
}
