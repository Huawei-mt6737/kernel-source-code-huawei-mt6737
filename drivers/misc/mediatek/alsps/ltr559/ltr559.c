/*
 * 
 * Author: MingHsien Hsieh <minghsien.hsieh@mediatek.com>
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

#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/gpio.h>
#include <linux/mutex.h>

#include "cust_alsps.h"
#include "ltr559.h"
#include "alsps.h"

/******************************************************************************
 * configuration
*******************************************************************************/
/*----------------------------------------------------------------------------*/

#define LTR559_DEV_NAME   "LTR559"

#define GN_MTK_BSP_PS_DYNAMIC_CALI
//#define HQ_PSENSOR_CALIBRATION
#define MAX_PS_NOISE 1330
/*----------------------------------------------------------------------------*/
#define APS_TAG                  "[ltr559] "
#define APS_FUN(f)               printk(KERN_INFO APS_TAG"%s\n", __FUNCTION__)

#define APS_ERR(fmt, args...)	pr_err(APS_TAG fmt, ##args)
#define APS_LOG(fmt, args...)	pr_err(APS_TAG fmt, ##args)
#define APS_DBG(fmt, args...)	pr_err(APS_TAG fmt, ##args)


/*----------------------------------------------------------------------------*/

static struct i2c_client *ltr559_i2c_client = NULL;
static  DEFINE_MUTEX (read_lock);

/*----------------------------------------------------------------------------*/
static const struct i2c_device_id ltr559_i2c_id[] = {{LTR559_DEV_NAME,0},{}};
static unsigned long long int_top_time;
static struct alsps_hw alsps_cust;
static struct alsps_hw *hw = &alsps_cust;
static struct platform_device *alspsPltFmDev;
static int ltr559_devinit(void);

/*----------------------------------------------------------------------------*/
static int ltr559_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id); 
static int ltr559_i2c_remove(struct i2c_client *client);
static int ltr559_i2c_detect(struct i2c_client *client, struct i2c_board_info *info);
/*----------------------------------------------------------------------------*/
static int ltr559_i2c_suspend(struct i2c_client *client, pm_message_t msg);
static int ltr559_i2c_resume(struct i2c_client *client);
static int ltr559_ps_enable(struct i2c_client *client, int enable);


static int ps_gainrange;
static int als_gainrange;

static int final_prox_val;

/*----------------------------------------------------------------------------*/


/*----------------------------------------------------------------------------*/
static int ltr559_als_read(struct i2c_client *client, u16* data);
static int ltr559_ps_read(struct i2c_client *client, u16* data);
static int ltr559_ps_set_thres(void);

/*----------------------------------------------------------------------------*/


typedef enum {
    CMC_BIT_ALS    = 1,
    CMC_BIT_PS     = 2,
} CMC_BIT;

/*----------------------------------------------------------------------------*/
struct ltr559_i2c_addr {    /*define a series of i2c slave address*/
    u8  write_addr;  
    u8  ps_thd;     /*PS INT threshold*/
};

/*----------------------------------------------------------------------------*/

struct ltr559_priv {
    struct alsps_hw  *hw;
    struct i2c_client *client;
    struct work_struct  eint_work;
    struct mutex lock;
	/*i2c address group*/
    struct ltr559_i2c_addr  addr;

     /*misc*/
    u16		    als_modulus;
    atomic_t    i2c_retry;
    atomic_t    als_debounce;   /*debounce time after enabling als*/
    atomic_t    als_deb_on;     /*indicates if the debounce is on*/
    atomic_t    als_deb_end;    /*the jiffies representing the end of debounce*/
    atomic_t    ps_mask;        /*mask ps: always return far away*/
    atomic_t    ps_debounce;    /*debounce time after enabling ps*/
    atomic_t    ps_deb_on;      /*indicates if the debounce is on*/
    atomic_t    ps_deb_end;     /*the jiffies representing the end of debounce*/
    atomic_t    ps_suspend;
    atomic_t    als_suspend;
	atomic_t  init_done;
	struct device_node *irq_node;
	int		irq;

    /*data*/
    u16         als;
    u16          ps;
    u8          _align;
    u16         als_level_num;
    u16         als_value_num;
    u32         als_level[C_CUST_ALS_LEVEL-1];
    u32         als_value[C_CUST_ALS_LEVEL];
	u16			ps_cali;

    atomic_t    als_cmd_val;    /*the cmd value can't be read, stored in ram*/
    atomic_t    ps_cmd_val;     /*the cmd value can't be read, stored in ram*/
    atomic_t    ps_thd_val;     /*the cmd value can't be read, stored in ram*/
	atomic_t    ps_thd_val_high;     /*the cmd value can't be read, stored in ram*/
	atomic_t    ps_thd_val_low;     /*the cmd value can't be read, stored in ram*/
	int 		init_noise;
	int 		current_xtalk;
    ulong       enable;         /*enable mask*/
    ulong       pending_intr;   /*pending interrupt*/

    /*early suspend*/
#if defined(CONFIG_HAS_EARLYSUSPEND)
    struct early_suspend    early_drv;
#endif     
};

 struct PS_CALI_DATA_STRUCT
{
    int close;
    int far_away;
    int valid;
} ;

static struct PS_CALI_DATA_STRUCT ps_cali={0,0,0};
static int intr_flag_value = 0;


static struct ltr559_priv *ltr559_obj = NULL;


static struct i2c_client *ltr559_i2c_client;

static DEFINE_MUTEX(ltr559_mutex);


static int ltr559_local_init(void);
static int ltr559_remove(void);
static int last_min_value = 2047;
static int ltr559_dynamic_calibrate(void);

#ifdef HQ_PSENSOR_CALIBRATION
static int ltr559_psensor_calibrate(void);
static int ltr559_get_init_noise(void);
#endif
static int ltr559_init_flag =  -1;
static struct alsps_init_info ltr559_init_info = {
		.name = "LTR559",
		.init = ltr559_local_init,
		.uninit = ltr559_remove,

};


#ifdef CONFIG_OF
static const struct of_device_id alsps_of_match[] = {
	{.compatible = "mediatek,alsps_ltr559"},
	{},
};
#endif

/*----------------------------------------------------------------------------*/
static struct i2c_driver ltr559_i2c_driver = {	
	.probe      = ltr559_i2c_probe,
	.remove     = ltr559_i2c_remove,
	.detect     = ltr559_i2c_detect,
	.suspend    = ltr559_i2c_suspend,
	.resume     = ltr559_i2c_resume,
	.id_table   = ltr559_i2c_id,
	.driver = {
		.name           = LTR559_DEV_NAME,
		#ifdef CONFIG_OF
		.of_match_table = alsps_of_match,
		#endif
	},
};



/* 
 * #########
 * ## I2C ##
 * #########
 */

// I2C Read
static int ltr559_i2c_read_reg(u8 regnum)
{
    u8 buffer[1],reg_value[1];
	int res = 0;
	mutex_lock(&read_lock);
	buffer[0]= regnum;
	res = i2c_master_send(ltr559_obj->client, buffer, 0x1);
	if(res <= 0)	{
	   
	   APS_ERR("read reg send res = %d\n",res);
	   mutex_unlock(&read_lock);
		return res;
	}
	res = i2c_master_recv(ltr559_obj->client, reg_value, 0x1);
	if(res <= 0)
	{
		APS_ERR("read reg recv res = %d\n",res);
		mutex_unlock(&read_lock);
		return res;
	}
	mutex_unlock(&read_lock);
	return reg_value[0];
}

// I2C Write
static int ltr559_i2c_write_reg(u8 regnum, u8 value)
{
	u8 databuf[2];    
	int res = 0;
   
	databuf[0] = regnum;   
	databuf[1] = value;
	res = i2c_master_send(ltr559_obj->client, databuf, 0x2);

	if (res < 0)
	{
			APS_ERR("addr:%d wirte reg send res = %d\n",ltr559_obj->client->addr,res);
		   	return res;
	}
	else
		return 0;
}


/*----------------------------------------------------------------------------*/
static ssize_t ltr559_show_als(struct device_driver *ddri, char *buf)
{
	int res;
	
	if(!ltr559_obj)
	{
		APS_ERR("ltr559_obj is null!!\n");
		return 0;
	}
	res = ltr559_als_read(ltr559_obj->client, &ltr559_obj->als);
    return snprintf(buf, PAGE_SIZE, "0x%04X\n", ltr559_obj->als);    
	
}
/*----------------------------------------------------------------------------*/
static ssize_t ltr559_show_ps(struct device_driver *ddri, char *buf)
{
	int  res;
	if(!ltr559_obj)
	{
		APS_ERR("ltr559_obj is null!!\n");
		return 0;
	}
	res = ltr559_ps_read(ltr559_obj->client, &ltr559_obj->ps);
	
    return sprintf(buf,"AP3426 PS val=%d\n", ltr559_obj->ps);     
}
/*----------------------------------------------------------------------------*/


/*----------------------------------------------------------------------------*/
static ssize_t ltr559_show_status(struct device_driver *ddri, char *buf)
{
	ssize_t len = 0;
	
	if(!ltr559_obj)
	{
		APS_ERR("ltr559_obj is null!!\n");
		return 0;
	}
	
	if(ltr559_obj->hw)
	{
	
		len += snprintf(buf+len, PAGE_SIZE-len, "CUST: %d, (%d %d)\n", 
			ltr559_obj->hw->i2c_num, ltr559_obj->hw->power_id, ltr559_obj->hw->power_vol);
		
	}
	else
	{
		len += snprintf(buf+len, PAGE_SIZE-len, "CUST: NULL\n");
	}


	len += snprintf(buf+len, PAGE_SIZE-len, "MISC: %d %d\n", atomic_read(&ltr559_obj->als_suspend), atomic_read(&ltr559_obj->ps_suspend));

	return len;
}

/*----------------------------------------------------------------------------*/
static int ltr559_set_PSgain(struct i2c_client *client, int val)
{
               int re_val, err;
              
               re_val = ltr559_i2c_read_reg(LTR559_PS_CONTR);
 
               re_val = re_val |(val << 2);
              
               err = ltr559_i2c_write_reg(LTR559_PS_CONTR, re_val);
 
 
               return err;
}
/*----------------------------------------------------------------------------*/
static int ltr559_get_PSgain(struct i2c_client *client)
{
               int re_val = 0;
 
               re_val = ltr559_i2c_read_reg(LTR559_PS_CONTR);
               re_val = ((re_val&0x0C)>>2);
 
               return re_val;
}

static int ltr559_set_ALSgain(struct i2c_client *client, int val)
{
               int re_val, err;
              
               re_val = ltr559_i2c_read_reg(LTR559_ALS_CONTR);
 
               re_val = re_val |(val << 2);
              
               err = ltr559_i2c_write_reg(LTR559_ALS_CONTR, re_val);
 
 
               return err;
}
 
static int ltr559_get_ALSgain(struct i2c_client *client)
{
               int re_val = 0;
 
               re_val = ltr559_i2c_read_reg(LTR559_ALS_CONTR);
               re_val = ((re_val&0x1C)>>2);
 
               return re_val;
}



/*----------------------------------------------------------------------------*/
static ssize_t ltr559_store_status(struct device_driver *ddri, const char *buf, size_t count)
{
	int status1,ret;
	if(!ltr559_obj)
	{
		APS_ERR("ltr559_obj is null!!\n");
		return 0;
	}
	
	if(1 == sscanf(buf, "%d ", &status1))
	{ 
	    //ret=ltr559_ps_enable(ps_gainrange);
		ret=ltr559_ps_enable(ltr559_obj->client, status1);
		APS_DBG("iret= %d, ps_gainrange = %d\n", ret, ps_gainrange);
	}
	else
	{
		APS_DBG("invalid content: %s, length = %zu\n", buf, count);
	}
	return count;    
}



static ssize_t ltr559_ps_gain_show(struct device_driver *ddri, char *buf)
{
	int ret_val =0;
	int count=0;
	
    if (!ltr559_obj) {
        APS_ERR("ltr559_obj is null!!\n");
		count += sprintf(buf+count, "ltr559_obj is null!!\n");
        return -1;
    }
	ret_val = ltr559_get_PSgain(ltr559_obj->client);
	count += snprintf(buf+count, PAGE_SIZE, "%u\n", ret_val);
	return count;
}

static ssize_t ltr559_ps_gain_store(struct device_driver *ddri, const char *buf, size_t count)
{
	int val=0,ret=0;

	if (!ltr559_obj) {
        APS_ERR("ltr559_obj is null!!\n");
        count += sprintf(buf+count, "ltr559_obj is null!!\n");
        return count;
    }
	ret = sscanf(buf, "%d", &val);
	if ((val < 0) || (val > 3)){
        APS_ERR("set ps gain value < 0 or value > 3 \n");
        count += sprintf(buf+count, "set ps gain value < 0 or value > 3 \n");
        return count;
    }
	ltr559_set_PSgain(ltr559_obj->client, val);
	return count;
}

static ssize_t ltr559_als_gain_show(struct device_driver *ddri, char *buf)
{
	int ret_val =0;
	int count=0;
	
    if (!ltr559_obj) {
        APS_ERR("ltr559_obj is null!!\n");
        return -1;
    }
	ret_val = ltr559_get_ALSgain(ltr559_obj->client);
	count += snprintf(buf+count, PAGE_SIZE, "%u\n", ret_val);
	return count;
}
static ssize_t ltr559_als_gain_store(struct device_driver *ddri, const char *buf, size_t count)
{
	int val=0,ret=0;

	if (!ltr559_obj) {
        APS_ERR("ltr559_obj is null!!\n");
        count += sprintf(buf+count, "ltr559_obj is null!!\n");
        return count;
    }
	ret = sscanf(buf, "%d", &val);
	if ((val < 0) || (val > 7)){
        APS_ERR("set als gain value < 0 or value > 3 \n");
        count += sprintf(buf+count, "set als gain value < 0 or value > 3 \n");
        return count;
    }
	ltr559_set_ALSgain(ltr559_obj->client, val);
	return count;
}




/*----------------------------------------------------------------------------*/
static ssize_t ltr559_show_reg(struct device_driver *ddri, char *buf)
{
	int i,len=0;
	int reg[]={0x80,0x81,0x82,0x83,0x84,0x85,0x86,0x87,0x88,0x89,0x8a,0x8b,0x8c,
		0x8d,0x8e,0x8f,0x90,0x91,0x92,0x93,0x94,0x95,0x97,0x98,0x99,0x9a,0x9e};
	for(i=0;i<27;i++)
		{
		len += snprintf(buf+len, PAGE_SIZE-len, "reg:0x%04X value: 0x%04X\n", reg[i],ltr559_i2c_read_reg(reg[i]));	

	    }
	return len;
}
/*----------------------------------------------------------------------------*/
static ssize_t ltr559_store_reg(struct device_driver *ddri, const char *buf, size_t count)
{
	int ret,value;
	u8 reg;
	if(!ltr559_obj)
	{
		APS_ERR("ltr559_obj is null!!\n");
		return 0;
	}
	
	if(2 == sscanf(buf, "%hhx %x ", &reg, &value))
	{ 
		APS_DBG("before write reg: %x, reg_value = %x  write value=%x\n", reg,ltr559_i2c_read_reg(reg),value);
	    ret=ltr559_i2c_write_reg(reg,value);
		APS_DBG("after write reg: %x, reg_value = %x\n", reg,ltr559_i2c_read_reg(reg));
	}
	else
	{
		APS_DBG("invalid content: '%s', length = %zu\n", buf, count);
	}
	return count;    
}

/*----------------------------------------------------------------------------*/
static ssize_t ltr559_show_threshold(struct device_driver *ddri, char *buf)
{
	int len=0;
	
	if(!ltr559_obj)
	{
		APS_ERR("ltr559_obj is null!!\n");
		return 0;
	}

	len += sprintf(buf,"threshold: low:%d,high:%d\n",\
		atomic_read(&ltr559_obj->ps_thd_val_low),atomic_read(&ltr559_obj->ps_thd_val_high));	

	return len;
}
/*----------------------------------------------------------------------------*/
static ssize_t ltr559_store_threshold(struct device_driver *ddri, const char *buf, size_t count)
{
	int low = 0;
	int high = 0;
	
	if(!ltr559_obj)
	{
		APS_ERR("ltr559_obj is null!!\n");
		return 0;
	}
	
	if(2 == sscanf(buf, "%d %d", &low, &high))
	{ 
		if((low < high)&&(low < 2047)&&(low > 0)&&(high < 2047)&&(high > 0)){
			atomic_set(&ltr559_obj->ps_thd_val_high,high);
			atomic_set(&ltr559_obj->ps_thd_val_low,low);
			
			APS_DBG("zqq store thd:bufl=%d,bufh=%d,low=%d,high=%d\n",low,high,\
				atomic_read(&ltr559_obj->ps_thd_val_low),atomic_read(&ltr559_obj->ps_thd_val_high));		

			if(0 == ltr559_obj->hw->polling_mode_ps)
			{	
				ltr559_ps_set_thres();
			}
		}
	}
	else
	{
		APS_DBG("invalid content: '%s', length = %zu\n", buf, count);
	}
	return count;    
}


/*----------------------------------------------------------------------------*/
static DRIVER_ATTR(als,     0444, ltr559_show_als,   NULL);
static DRIVER_ATTR(Huaqin_show_val,0444, ltr559_show_ps,    NULL);
static DRIVER_ATTR(status,  0664, ltr559_show_status,  ltr559_store_status);
static DRIVER_ATTR(reg,     0664, ltr559_show_reg,   ltr559_store_reg);
static DRIVER_ATTR(threshold,     0664, ltr559_show_threshold,ltr559_store_threshold);
static DRIVER_ATTR(ps_gain,  0660, ltr559_ps_gain_show, ltr559_ps_gain_store);
static DRIVER_ATTR(als_gain,  0660, ltr559_als_gain_show, ltr559_als_gain_store);
/*----------------------------------------------------------------------------*/
static struct driver_attribute *ltr559_attr_list[] = {
    &driver_attr_als,
    &driver_attr_Huaqin_show_val,    
    &driver_attr_status,
    &driver_attr_reg,
    &driver_attr_threshold,
    &driver_attr_ps_gain,
    &driver_attr_als_gain,
};
/*----------------------------------------------------------------------------*/
static int ltr559_create_attr(struct device_driver *driver) 
{
	int idx, err = 0;
	int num = (int)(sizeof(ltr559_attr_list)/sizeof(ltr559_attr_list[0]));

	if (driver == NULL)
	{
		return -EINVAL;
	}

	for(idx = 0; idx < num; idx++)
	{
		err = driver_create_file(driver, ltr559_attr_list[idx]);
		if(err)
		{            
			APS_ERR("driver_create_file (%s) = %d\n", ltr559_attr_list[idx]->attr.name, err);
			break;
		}
	}    
	return err;
}
/*----------------------------------------------------------------------------*/
	static int ltr559_delete_attr(struct device_driver *driver)
	{
	int idx ,err = 0;
	int num = (int)(sizeof(ltr559_attr_list)/sizeof(ltr559_attr_list[0]));

	if (!driver)
	return -EINVAL;

	for (idx = 0; idx < num; idx++) 
	{
		driver_remove_file(driver, ltr559_attr_list[idx]);
	}
	
	return err;
}

/*----------------------------------------------------------------------------*/

/* 
 * ###############
 * ## PS CONFIG ##
 * ###############

 */

static int ltr559_ps_set_thres(void)
{
	int res;
	u8 databuf[2];

	struct i2c_client *client = ltr559_obj->client;
	struct ltr559_priv *obj = ltr559_obj;	

	APS_FUN();

	//BUILD_BUG_ON_ZERO(0>1);


	APS_DBG("ps_cali.valid: %d\n", ps_cali.valid);
	if(1 == ps_cali.valid)
	{
		databuf[0] = LTR559_PS_THRES_LOW_0; 
		databuf[1] = (u8)(ps_cali.far_away & 0x00FF);
		res = i2c_master_send(client, databuf, 0x2);
		if(res <= 0)
		{
			goto EXIT_ERR;
			return ltr559_ERR_I2C;
		}
		databuf[0] = LTR559_PS_THRES_LOW_1; 
		databuf[1] = (u8)((ps_cali.far_away & 0xFF00) >> 8);
		res = i2c_master_send(client, databuf, 0x2);
		if(res <= 0)
		{
			goto EXIT_ERR;
			return ltr559_ERR_I2C;
		}
		databuf[0] = LTR559_PS_THRES_UP_0;	
		databuf[1] = (u8)(ps_cali.close & 0x00FF);
		res = i2c_master_send(client, databuf, 0x2);
		if(res <= 0)
		{
			goto EXIT_ERR;
			return ltr559_ERR_I2C;
		}
		databuf[0] = LTR559_PS_THRES_UP_1;	
		databuf[1] = (u8)((ps_cali.close & 0xFF00) >> 8);;
		res = i2c_master_send(client, databuf, 0x2);
		if(res <= 0)
		{
			goto EXIT_ERR;
			return ltr559_ERR_I2C;
		}
	}
	else
	{
		databuf[0] = LTR559_PS_THRES_LOW_0; 
		databuf[1] = (u8)((atomic_read(&obj->ps_thd_val_low)) & 0x00FF);
		res = i2c_master_send(client, databuf, 0x2);
		if(res <= 0)
		{
			goto EXIT_ERR;
			return ltr559_ERR_I2C;
		}
		databuf[0] = LTR559_PS_THRES_LOW_1; 
		databuf[1] = (u8)((atomic_read(&obj->ps_thd_val_low )>> 8) & 0x00FF);
		
		res = i2c_master_send(client, databuf, 0x2);
		if(res <= 0)
		{
			goto EXIT_ERR;
			return ltr559_ERR_I2C;
		}
		databuf[0] = LTR559_PS_THRES_UP_0;	
		databuf[1] = (u8)((atomic_read(&obj->ps_thd_val_high)) & 0x00FF);
		res = i2c_master_send(client, databuf, 0x2);
		if(res <= 0)
		{
			goto EXIT_ERR;
			return ltr559_ERR_I2C;
		}
		databuf[0] = LTR559_PS_THRES_UP_1;	
		databuf[1] = (u8)((atomic_read(&obj->ps_thd_val_high) >> 8) & 0x00FF);
		res = i2c_master_send(client, databuf, 0x2);
		if(res <= 0)
		{
			goto EXIT_ERR;
			return ltr559_ERR_I2C;
		}
	
	}
	APS_DBG("ps low: %d high: %d\n", atomic_read(&obj->ps_thd_val_low),\
		atomic_read(&obj->ps_thd_val_high));

	res = 0;
	return res;
	
	EXIT_ERR:
	APS_ERR("set thres: %d\n", res);
	return res;

}


//static int ltr559_ps_enable(int gainrange)
static int ltr559_ps_enable(struct i2c_client *client, int enable)
{
	//struct ltr559_priv *obj = ltr559_obj;
	u8 regdata;	
	int err;
	
	//int setgain;
    APS_LOG("ltr559_ps_enable() ...start!\n");
	
	err = ltr559_devinit();
	if(err<0)
	{
		APS_LOG("ltr559 ps enable devinit err\n");
		return err;
	} 
	err = ltr559_i2c_write_reg(LTR559_PS_MEAS_RATE, 0x0f); 
	if(err<0)
	{
		APS_LOG("ltr559 set ps measurement rate error\n");
		return err;
	} 
	if (enable == 1) {
		APS_LOG("PS: enable ps only \n");
		regdata = 0x03;
	} else {
		APS_LOG("PS: disable ps only \n");
		regdata = 0x00;
	}
	err = ltr559_i2c_write_reg(LTR559_PS_CONTR, regdata);
	if(err<0)
	{
		APS_ERR("PS: enable ps err: %d en: %d \n", err, enable);
		return err;
	}
	msleep(WAKEUP_DELAY);

	regdata = ltr559_i2c_read_reg(LTR559_PS_CONTR);


#ifdef GN_MTK_BSP_PS_DYNAMIC_CALI
	if (regdata & 0x02) {		
		if (ltr559_dynamic_calibrate() < 0)
			return -1;
	}
#endif
#ifdef HQ_PSENSOR_CALIBRATION
	if (regdata & 0x02) {	// enable ps	
		if (ltr559_psensor_calibrate() < 0)
			return -1;
	}
#endif
	err = ltr559_i2c_write_reg(LTR559_PS_MEAS_RATE, 0x02); 
	if(err<0)
	{
		APS_LOG("ltr559 set ps measurement rate error\n");
		return err;
	} 

	return 0;
	
}


static int ltr559_ps_read(struct i2c_client *client, u16 *data)
{
	int psval_lo, psval_hi, psdata;

	psval_lo = ltr559_i2c_read_reg(LTR559_PS_DATA_0);
	//APS_DBG("ps_rawdata_psval_lo = %d\n", psval_lo);
	if (psval_lo < 0){
	    
	    APS_DBG("psval_lo error\n");
		psdata = psval_lo;
		goto out;
	}
	psval_hi = ltr559_i2c_read_reg(LTR559_PS_DATA_1);
    //APS_DBG("ps_rawdata_psval_hi = %d\n", psval_hi);

	if (psval_hi < 0){
	    APS_DBG("psval_hi error\n");
		psdata = psval_hi;
		goto out;
	}
	
	psdata = ((psval_hi & 7)* 256) + psval_lo;
    //psdata = ((psval_hi&0x7)<<8) + psval_lo;
    APS_DBG("ps_rawdata = %d\n", psdata);

	*data = psdata;
	return 0;
    
	out:
	final_prox_val = psdata;
	
	return psdata;
}

/* 
 * ################
 * ## ALS CONFIG ##
 * ################
 */


/*---------------------------add by hongguang for dynamic calibrate-------------------------------------------------*/
#ifdef GN_MTK_BSP_PS_DYNAMIC_CALI
static int ltr559_dynamic_calibrate(void)
{
	int i = 0;
	int j = 0;
	int data = 0;
	int noise = 0;
	int max = 0;
	unsigned long data_total = 0;
	struct ltr559_priv *obj = ltr559_obj;


	APS_FUN(f);
	if (!obj) goto err;

	msleep(20);
	for (i = 0; i < 5; i++) {
		if (max++ > 5) {
			goto err;
		}
		msleep(15);
				
		ltr559_ps_read(obj->client, &obj->ps);
		data = obj->ps;
		
		if(data == 0){
			j++;
		}	
		data_total += data;
	}
	noise = data_total/(5 - j);
	//isadjust = 1;
		if((noise < last_min_value+500)){
			last_min_value = noise;
				if(noise < 100){
						atomic_set(&obj->ps_thd_val_high,  noise+106);//70
						atomic_set(&obj->ps_thd_val_low, noise+26); //50
				}else if(noise < 200){
						atomic_set(&obj->ps_thd_val_high,  noise+120);
						atomic_set(&obj->ps_thd_val_low, noise+40);
				}else if(noise < 300){
						atomic_set(&obj->ps_thd_val_high,  noise+150);
						atomic_set(&obj->ps_thd_val_low, noise+50);
				}else if(noise < 400){
						atomic_set(&obj->ps_thd_val_high,  noise+200);
						atomic_set(&obj->ps_thd_val_low, noise+50);
				}else if(noise < 600){
						atomic_set(&obj->ps_thd_val_high,  noise+200);
						atomic_set(&obj->ps_thd_val_low, noise+50);
				}else if(noise < 1000){
						atomic_set(&obj->ps_thd_val_high,  noise+200);
						atomic_set(&obj->ps_thd_val_low, noise+100);	
				}else if(noise < 1450){
						atomic_set(&obj->ps_thd_val_high,  noise+200);
						atomic_set(&obj->ps_thd_val_low, noise+150);
				}
				else{
						atomic_set(&obj->ps_thd_val_high,  1600);
						atomic_set(&obj->ps_thd_val_low, 1550);
				}

			}
	APS_DBG(" calibrate:noise=%d, thdlow= %d , thdhigh = %d\n",\
		noise,  atomic_read(&obj->ps_thd_val_low),  atomic_read(&obj->ps_thd_val_high));

	if(0 == obj->hw->polling_mode_ps)
	{	
		ltr559_ps_set_thres();
	}
	return 0;
err:
	APS_ERR("ltr559_dynamic_calibrate fail!!!\n");
	return -1;
}
#endif

/*---------------------------------add by hongguang for dynamic calibrate-------------------------------------------*/
#ifdef HQ_PSENSOR_CALIBRATION
static int ltr559_psensor_calibrate(void)
{
	int i = 0;
	int j = 0;
	unsigned long sum = 0;
	struct ltr559_priv *obj = ltr559_obj;


	APS_FUN(f);
	if (!obj) goto err;

	for (i = 0; i < 5; i++) {
		msleep(30);
				
		ltr559_ps_read(obj->client, &obj->ps);
		
		if(obj->ps == 0){
			j++;
		}	
		sum += obj->ps;
	}
	
	if(j<5)
		sum = sum/(5 - j);
	else
		sum = 0;
	
	if(sum < MAX_PS_NOISE){
		if(obj->init_noise == 0)
			obj->init_noise = sum;
		
		obj->current_xtalk = sum;
	}else{
		obj->current_xtalk = obj->init_noise;
	}

	atomic_set(&obj->ps_thd_val_high,obj->hw->ps_threshold_high+obj->current_xtalk);
	atomic_set(&obj->ps_thd_val_low,obj->hw->ps_threshold_low+obj->current_xtalk);

	APS_DBG("zqq calibrate:noise=%ld,low=%d ,high=%d\n",\
		sum,atomic_read(&obj->ps_thd_val_low),atomic_read(&obj->ps_thd_val_high));

	if(0 == obj->hw->polling_mode_ps)
	{	
		ltr559_ps_set_thres();
	}
	
	return 0;
err:
	APS_ERR("zqq ltr559_psensor_calibrate fail!!!\n");
	return -1;
}

static int ltr559_get_init_noise(void)
{
	int i = 0;
	int j = 0;
	u8 regdata;	
	int err;
	unsigned long sum = 0;
	struct ltr559_priv *obj = ltr559_obj;

	APS_FUN(f);
	if (!obj) goto err;
	//disable irq
	disable_irq_nosync(obj->irq);
	//enable
	regdata = ltr559_i2c_read_reg(LTR559_PS_CONTR);
	regdata |= 0x03;
	err = ltr559_i2c_write_reg(LTR559_PS_CONTR, regdata);
	if(err<0)
	{
		APS_ERR("PS: enable ps err: %d \n", err);
	}
	
	err = ltr559_i2c_write_reg(LTR559_PS_MEAS_RATE, 0x0f); 
	if(err<0)
	{
		APS_LOG("ltr559 set ps measurement rate error\n");
	}

	for (i = 0; i < 5; i++) {
		msleep(30);
				
		ltr559_ps_read(obj->client, &obj->ps);
		
		if(obj->ps == 0){
			j++;
		}	
		sum += obj->ps;
	}
	
	if(j<5)
		sum = sum/(5 - j);
	else
		sum = 0;
	
	if(sum < MAX_PS_NOISE){
		obj->init_noise = sum;	
	}
	
	err = ltr559_i2c_write_reg(LTR559_PS_MEAS_RATE, 0x02); 
	if(err<0)
	{
		APS_LOG("ltr559 set ps measurement rate error\n");
	} 

	//disable 
	regdata = ltr559_i2c_read_reg(LTR559_PS_CONTR);
	regdata &= 0xfc;
	err = ltr559_i2c_write_reg(LTR559_PS_CONTR, regdata);
	if(err<0)
	{
		APS_ERR("PS: disable ps err: %d\n", err);
	}
	//enable irq
	enable_irq(obj->irq);
	APS_DBG("zqq init_noise %d\n",obj->init_noise);
	
	return 0;
err:
	APS_ERR("zqq ltr559_get_init_noise fail!!!\n");
	return -1;
}
#endif

static int ltr559_als_enable(struct i2c_client *client, int enable)
{
	int err=0;	
	u8 regdata=0;	

	if (enable == 1) {
		APS_LOG("ALS(1): enable als only \n");
		regdata = 0x0D;
	} else {
		APS_LOG("ALS(1): disable als only \n");
		regdata = 0x00;
	}

	err = ltr559_i2c_write_reg(LTR559_ALS_CONTR, regdata);
	if(err<0)
	{
		APS_ERR("ALS: enable als err: %d en: %d \n", err, enable);
		return err;
	}

	//obj->als_enable = enable;
	
	mdelay(WAKEUP_DELAY);

	return 0;
	
        
}


static int ltr559_als_read(struct i2c_client *client, u16* data)
{
	int alsval_ch0_lo, alsval_ch0_hi, alsval_ch0;
	int alsval_ch1_lo, alsval_ch1_hi, alsval_ch1;
	int  luxdata_int;
	int ratio;
	
	if (client == NULL) {
		APS_DBG("ltr559 i2c_client is null!\n");
		return -1;
	}

	alsval_ch1_lo = ltr559_i2c_read_reg(LTR559_ALS_DATA_CH1_0);
	alsval_ch1_hi = ltr559_i2c_read_reg(LTR559_ALS_DATA_CH1_1);
	alsval_ch1 = (alsval_ch1_hi * 256) + alsval_ch1_lo;
	APS_DBG("alsval_ch1_lo = %d,alsval_ch1_hi=%d,alsval_ch1=%d\n",alsval_ch1_lo,alsval_ch1_hi,alsval_ch1);


	alsval_ch0_lo = ltr559_i2c_read_reg(LTR559_ALS_DATA_CH0_0);
	alsval_ch0_hi = ltr559_i2c_read_reg(LTR559_ALS_DATA_CH0_1);
	alsval_ch0 = (alsval_ch0_hi * 256) + alsval_ch0_lo;
	APS_DBG("alsval_ch0_lo = %d,alsval_ch0_hi=%d,alsval_ch0=%d\n",alsval_ch0_lo,alsval_ch0_hi,alsval_ch0);
	

    if((alsval_ch1==0)&&(alsval_ch0==0))
    {
        luxdata_int = 0;
		goto EXIT_ERR;
    }
	ratio = (alsval_ch1*1000) /(alsval_ch0+alsval_ch1);
	APS_DBG("ratio = %d  gainrange = %d\n",ratio,als_gainrange);
	if (ratio < 450){
		luxdata_int = (((17743 * alsval_ch0)+(11059 * alsval_ch1)))/8/1194;
	}
	else if ((ratio < 640) && (ratio >= 450)){
		luxdata_int = (((42785 * alsval_ch0)-(19548 * alsval_ch1)))/8/1000;
	}
	else if ((ratio < 760) && (ratio >= 640)) {
		luxdata_int = (((5926 * alsval_ch0)+(1185 * alsval_ch1)))/8/1523;
	}
	else if ((ratio < 900) && (ratio >= 760)) {
		luxdata_int = (((5926 * alsval_ch0)+(1185 * alsval_ch1)))/8/3194;
	}
	else {
		luxdata_int = 0;
	}
	
	APS_DBG("als_value_lux = %d\n", luxdata_int);

	*data = luxdata_int;
	
	return 0;

EXIT_ERR:
	*data = luxdata_int;
	APS_ERR("ltr559 read als data fail!\n");
	return 0;
	
}



/*----------------------------------------------------------------------------*/
int ltr559_get_addr(struct alsps_hw *hw, struct ltr559_i2c_addr *addr)
{
	//
	if(!hw || !addr)
	{
		return -EFAULT;
	}
	addr->write_addr= hw->i2c_addr[0];
	//printk(KERN_EMERG "[TANG]addr->write_addr is %d",addr->write_addr);
	//
	return 0;
}


/*-----------------------------------------------------------------------------*/
void ltr559_eint_func(void)
{
	struct ltr559_priv *obj = ltr559_obj;
	APS_FUN();
	if(!obj)
	{
		return;
	}
	int_top_time = sched_clock();
	schedule_work(&obj->eint_work);
	//schedule_delayed_work(&obj->eint_work);
}

#if defined(CONFIG_OF)
static irqreturn_t ltr559_eint_handler(int irq, void *desc)
{
	ltr559_eint_func();
	disable_irq_nosync(ltr559_obj->irq);

	return IRQ_HANDLED;
}
#endif


/*----------------------------------------------------------------------------*/
/*for interrup work mode support -- by liaoxl.lenovo 12.08.2011*/
int ltr559_setup_eint(struct i2c_client *client)
{

	int ret;
	struct pinctrl *pinctrl;
	//struct pinctrl_state *pins_default;
	struct pinctrl_state *pins_cfg;

	u32 ints[2] = {0, 0};

	APS_FUN();

	alspsPltFmDev = get_alsps_platformdev();
	/* gpio setting */
	pinctrl = devm_pinctrl_get(&alspsPltFmDev->dev);
	if (IS_ERR(pinctrl)) {
		ret = PTR_ERR(pinctrl);
		APS_ERR("Cannot find alsps pinctrl!\n");
	}

	pins_cfg = pinctrl_lookup_state(pinctrl, "pin_cfg");
	if (IS_ERR(pins_cfg)) {
		ret = PTR_ERR(pins_cfg);
		APS_ERR("Cannot find alsps pinctrl pin_cfg!\n");

	}
/* eint request */
	if (ltr559_obj->irq_node) {
		of_property_read_u32_array(ltr559_obj->irq_node, "debounce", ints, ARRAY_SIZE(ints));
		gpio_request(ints[0], "p-sensor");
		gpio_set_debounce(ints[0], ints[1]);
		pinctrl_select_state(pinctrl, pins_cfg);
		APS_LOG("ints[0] = %d, ints[1] = %d!!\n", ints[0], ints[1]);

		ltr559_obj->irq = irq_of_parse_and_map(ltr559_obj->irq_node, 0);
		APS_LOG("ltr559_obj->irq = %d\n", ltr559_obj->irq);
		if (!ltr559_obj->irq) {
			APS_ERR("irq_of_parse_and_map fail!!\n");
			return -EINVAL;
		}
		//APS_ERR("irq to gpio = %d \n",irq_to_gpio(ltr559_obj->irq));
		if (request_irq(ltr559_obj->irq, ltr559_eint_handler, IRQF_TRIGGER_FALLING, "ALS-eint", NULL)) {
			APS_ERR("IRQ LINE NOT AVAILABLE!!\n");
			return -EINVAL;
		}
		//enable_irq(ltr559_obj->irq);
	} else {
		APS_ERR("null irq node!!\n");
		return -EINVAL;
	}
	
    return 0;
}


/*----------------------------------------------------------------------------*/
static void ltr559_power(struct alsps_hw *hw, unsigned int on) 
{

}

/*----------------------------------------------------------------------------*/

static int ltr559_check_intr(struct i2c_client *client) 
{
	

	int res,intp,intl;
	u8 buffer[2];

	APS_FUN();
	buffer[0] = ltr559_i2c_read_reg(LTR559_ALS_PS_STATUS);
	APS_LOG("status = %x\n", buffer[0]);
	res = 1;
	intp = 0;
	intl = 0;
	if(0 != (buffer[0] & 0x02))
	{
	
		res = 0;
		intp = 1;
	}
	if(0 != (buffer[0] & 0x08))
	{
		res = 0;
		intl = 1;		
	}

		if(0 == res)
		{
			if((1 == intp) && (0 == intl))
			{
				APS_LOG("PS interrupt\n");
				buffer[1] = buffer[0] & 0xfD;
				
			}
			else if((0 == intp) && (1 == intl))
			{
				APS_LOG("ALS interrupt\n");
				buffer[1] = buffer[0] & 0xf7;
			}
			else
			{
				APS_LOG("Check ALS/PS interrup error\n");
				buffer[1] = buffer[0] & 0xf5;
			}
			//buffer[0] = LTR559_ALS_PS_STATUS	;
			//res = i2c_master_send(client, buffer, 0x2);
			//if(res <= 0)
			//{
			//	goto EXIT_ERR;
			//}
			//else
			//{
			//	res = 0;
			//}
		}

	return res;

EXIT_ERR:
	APS_ERR("ltr559_check_intr fail\n");
	return 1;
}



static int ltr559_devinit(void)
{
	int res;
	int init_ps_gain;
	int init_als_gain;
	u8 databuf[2];	

	struct i2c_client *client = ltr559_obj->client;

	struct ltr559_priv *obj = ltr559_obj;   
	
	mdelay(PON_DELAY);

	init_ps_gain = MODE_PS_ON_Gain16;
	//init_ps_gain = 0;

	APS_LOG("LTR559_PS setgain = %d!\n",init_ps_gain);
	res = ltr559_i2c_write_reg(LTR559_PS_CONTR, init_ps_gain); 
	if(res<0)
	{
	    APS_LOG("ltr559 set ps gain error\n");
	    return res;
	}
	
	mdelay(WAKEUP_DELAY);
    
	res = ltr559_i2c_write_reg(LTR559_PS_N_PULSES, 4); 
	if(res<0)
	{
		APS_LOG("ltr559 set ps pulse error\n");
		return res;
	} 
	/*for interrup work mode support */
	if(0 == obj->hw->polling_mode_ps)
	{	
		APS_LOG("eint enable");
		//ltr559_ps_set_thres();
		
		databuf[0] = LTR559_INTERRUPT;	
		databuf[1] = 0x01;
		res = i2c_master_send(client, databuf, 0x2);
		if(res <= 0)
		{
			goto EXIT_ERR;
			return ltr559_ERR_I2C;
		}

		databuf[0] = LTR559_INTERRUPT_PERSIST;	
		databuf[1] = 0x01;
		res = i2c_master_send(client, databuf, 0x2);
		if(res <= 0)
		{
			goto EXIT_ERR;
			return ltr559_ERR_I2C;
		}

	}

	

	res = 0;

	EXIT_ERR:
	APS_ERR("init dev: %d\n", res);
	return res;

}
/*----------------------------------------------------------------------------*/


static int ltr559_get_als_value(struct ltr559_priv *obj, u16 als)
{
	int idx;
	int invalid = 0;
	APS_DBG("als  = %d\n",als); 
	for(idx = 0; idx < obj->als_level_num; idx++)
	{
		if(als < obj->hw->als_level[idx])
		{
			break;
		}
	}
	
	if(idx >= obj->als_value_num)
	{
		APS_ERR("exceed range\n"); 
		idx = obj->als_value_num - 1;
	}
	
	if(1 == atomic_read(&obj->als_deb_on))
	{
		unsigned long endt = atomic_read(&obj->als_deb_end);
		if(time_after(jiffies, endt))
		{
			atomic_set(&obj->als_deb_on, 0);
		}
		
		if(1 == atomic_read(&obj->als_deb_on))
		{
			invalid = 1;
		}
	}

	if(!invalid)
	{
		APS_DBG("ALS: %05d => %05d\n", als, obj->hw->als_value[idx]);	
		return obj->hw->als_value[idx];
	}
	else
	{
		APS_ERR("ALS: %05d => %05d (-1)\n", als, obj->hw->als_value[idx]);    
		return -1;
	}
}
/*----------------------------------------------------------------------------*/
static int ltr559_get_ps_value(struct ltr559_priv *obj, u16 ps)
{
	int val, invalid = 0;

	static int val_temp = 1;
	if((ps > atomic_read(&obj->ps_thd_val_high)))
	{
		val = 0;  /*close*/
		val_temp = 0;
		intr_flag_value = 1;
	}
			//else if((ps < atomic_read(&obj->ps_thd_val_low))&&(temp_ps[0]  < atomic_read(&obj->ps_thd_val_low)))
	else if((ps < atomic_read(&obj->ps_thd_val_low)))
	{
		val = 1;  /*far away*/
		val_temp = 1;
		intr_flag_value = 0;
	}
	else
		val = val_temp;	
			
	
	if(atomic_read(&obj->ps_suspend))
	{
		invalid = 1;
	}
	else if(1 == atomic_read(&obj->ps_deb_on))
	{
		unsigned long endt = atomic_read(&obj->ps_deb_end);
		if(time_after(jiffies, endt))
		{
			atomic_set(&obj->ps_deb_on, 0);
		}
		
		if (1 == atomic_read(&obj->ps_deb_on))
		{
			invalid = 1;
		}
	}
	else if (obj->als > 50000)
	{
		//invalid = 1;
		APS_DBG("ligh too high will result to failt proximiy\n");
		return 1;  /*far away*/
	}

	if(!invalid)
	{
		APS_DBG("PS:  %05d => %05d\n", ps, val);
		return val;
	}	
	else
	{
		return -1;
	}	
}

/*----------------------------------------------------------------------------*/
/* if use  this typ of enable , Gsensor should report inputEvent(x, y, z ,stats, div) to HAL */
static int als_open_report_data(int open)
{
	/* should queuq work to report event if  is_report_input_direct=true */
	return 0;
}

/* if use  this typ of enable , Gsensor only enabled but not report inputEvent to HAL */

static int als_enable_nodata(int en)
{
	int res = 0;

	APS_LOG("ltr559_obj als enable value = %d\n", en);


	mutex_lock(&ltr559_mutex);
	if (en)
		set_bit(CMC_BIT_ALS, &ltr559_obj->enable);
	else
		clear_bit(CMC_BIT_ALS, &ltr559_obj->enable);
	mutex_unlock(&ltr559_mutex);
	if (!ltr559_obj) {
		APS_ERR("ltr559_obj is null!!\n");
		return -1;
	}
	res = ltr559_als_enable(ltr559_obj->client, en);
	if (res) {
		APS_ERR("als_enable_nodata is failed!!\n");
		return -1;
	}
	return 0;
}

static int als_set_delay(u64 ns)
{
	return 0;
}

static int als_get_data(int *value, int *status)
{
	int err = 0;

	struct ltr559_priv *obj = NULL;


	if (!ltr559_obj) {
		APS_ERR("ltr559_obj is null!!\n");
		return -1;
	}
	obj = ltr559_obj;
	err = ltr559_als_read(obj->client, &obj->als);
	if (err)
		err = -1;
	else {
// *value = ltr559_get_als_value(obj, obj->als);
        APS_ERR("als: %d\n", obj->als);
		*value = obj->als;
		if (*value < 0)
			err = -1;
		*status = SENSOR_STATUS_ACCURACY_MEDIUM;
	}

	return err;
}

/* if use  this typ of enable , Gsensor should report inputEvent(x, y, z ,stats, div) to HAL */
static int ps_open_report_data(int open)
{
	/* should queuq work to report event if  is_report_input_direct=true */
	return 0;
}

/* if use  this typ of enable , Gsensor only enabled but not report inputEvent to HAL */

static int ps_enable_nodata(int en)
{
	int res = 0;


	APS_LOG("ltr559_obj ps enable value = %d\n", en);


	mutex_lock(&ltr559_mutex);
	if (en)
		set_bit(CMC_BIT_PS, &ltr559_obj->enable);

	else
		clear_bit(CMC_BIT_PS, &ltr559_obj->enable);

	mutex_unlock(&ltr559_mutex);
	if (!ltr559_obj) {
		APS_ERR("ltr559_obj is null!!\n");
		return -1;
	}
	res = ltr559_ps_enable(ltr559_obj->client, en);

	if (res) {
		APS_ERR("als_enable_nodata is failed!!\n");
		return -1;
	}
	return 0;

}

static int ps_set_delay(u64 ns)
{
	return 0;
}

static int ps_get_data(int *value, int *status)
{
	int err = 0;



	if (!ltr559_obj) {
		APS_ERR("ltr559_obj is null!!\n");
		return -1;
	}

	err = ltr559_ps_read(ltr559_obj->client, &ltr559_obj->ps);
	if (err)
		err = -1;
	else {
		*value = ltr559_get_ps_value(ltr559_obj, ltr559_obj->ps);
		if (*value < 0)
			err = -1;
		*status = SENSOR_STATUS_ACCURACY_MEDIUM;
	}

	return err;
}


/*----------------------------------------------------------------------------*/
/*for interrup work mode support */
static void ltr559_eint_work(struct work_struct *work)
{
	struct ltr559_priv *obj = (struct ltr559_priv *)container_of(work, struct ltr559_priv, eint_work);
	int err;
	//hwm_sensor_data sensor_data;
	u8 databuf[2];
	int res = 0;
	int value = 1;
	APS_FUN();
	err = ltr559_check_intr(obj->client);
	if(err < 0)
	{
		APS_ERR("ltr559_eint_work check intrs: %d\n", err);
	}
	else
	{
		//get raw data
		ltr559_ps_read(obj->client, &obj->ps);
    	if(obj->ps < 0)
    	{
    		err = -1;
    		return;
    	}
		#ifdef HQ_PSENSOR_CALIBRATION
		APS_DBG("zqq int:init=%d,noise=%d,low=%d,high=%d\n",obj->init_noise,obj->current_xtalk,\
			atomic_read(&obj->ps_thd_val_low),atomic_read(&obj->ps_thd_val_high));
		#endif
		APS_DBG("ltr559_eint_work rawdata ps=%d als_ch0=%d!\n",obj->ps,obj->als);
		value = ltr559_get_ps_value(obj, obj->ps);
/*singal interrupt function add*/
		//intr_flag_value = value;
		APS_DBG("intr_flag_value=%d\n",intr_flag_value);
		if(intr_flag_value){// far
				APS_DBG(" interrupt value ps will < ps_threshold_low");

				databuf[0] = LTR559_PS_THRES_LOW_0;	
				databuf[1] = (u8)((atomic_read(&obj->ps_thd_val_low)) & 0x00FF);
				res = i2c_master_send(obj->client, databuf, 0x2);
				if(res <= 0)
				{
					return;
				}
				databuf[0] = LTR559_PS_THRES_LOW_1;	
				databuf[1] = (u8)(((atomic_read(&obj->ps_thd_val_low)) & 0xFF00) >> 8);
				res = i2c_master_send(obj->client, databuf, 0x2);
				if(res <= 0)
				{
					return;
				}
				databuf[0] = LTR559_PS_THRES_UP_0;	
				databuf[1] = (u8)(0x00FF);
				res = i2c_master_send(obj->client, databuf, 0x2);
				if(res <= 0)
				{
					return;
				}
				databuf[0] = LTR559_PS_THRES_UP_1; 
				databuf[1] = (u8)((0xFF00) >> 8);
				res = i2c_master_send(obj->client, databuf, 0x2);
				if(res <= 0)
				{
					return;
				}
		}
		else{	

			
#ifdef GN_MTK_BSP_PS_DYNAMIC_CALI
				if(last_min_value> 20 && obj->ps < (last_min_value - 150))
				{
					if(obj->ps < 100){
						atomic_set(&obj->ps_thd_val_high,  obj->ps+106);//70
						atomic_set(&obj->ps_thd_val_low, obj->ps+26); //50
					}else if(obj->ps < 200){
						atomic_set(&obj->ps_thd_val_high,  obj->ps+120);
						atomic_set(&obj->ps_thd_val_low, obj->ps+40);
					}else if(obj->ps < 300){
						atomic_set(&obj->ps_thd_val_high,  obj->ps+150);
						atomic_set(&obj->ps_thd_val_low, obj->ps+50);
					}else if(obj->ps < 400){
						atomic_set(&obj->ps_thd_val_high,  obj->ps+200);
						atomic_set(&obj->ps_thd_val_low, obj->ps+50);
					}else if(obj->ps < 600){
						atomic_set(&obj->ps_thd_val_high,  obj->ps+200);
						atomic_set(&obj->ps_thd_val_low, obj->ps+50);
					}else if(obj->ps < 1000){
						atomic_set(&obj->ps_thd_val_high,  obj->ps+200);
						atomic_set(&obj->ps_thd_val_low, obj->ps+100);	
					}else if(obj->ps < 1450){
						atomic_set(&obj->ps_thd_val_high,  obj->ps+200);
						atomic_set(&obj->ps_thd_val_low, obj->ps+150);
					}
					else{
						atomic_set(&obj->ps_thd_val_high,  1600);
						atomic_set(&obj->ps_thd_val_low, 1450);
					}
					
				}
#endif			
				APS_DBG(" interrupt value ps will > ps_threshold_high");
				databuf[0] = LTR559_PS_THRES_LOW_0;	
				databuf[1] = (u8)(0 & 0x00FF);
				res = i2c_master_send(obj->client, databuf, 0x2);
				if(res <= 0)
				{
					return;
				}
				databuf[0] = LTR559_PS_THRES_LOW_1;	
				databuf[1] = (u8)((0 & 0xFF00) >> 8);
				res = i2c_master_send(obj->client, databuf, 0x2);
				if(res <= 0)
				{
					return;
				}
				databuf[0] = LTR559_PS_THRES_UP_0;	
				databuf[1] = (u8)((atomic_read(&obj->ps_thd_val_high)) & 0x00FF);
				res = i2c_master_send(obj->client, databuf, 0x2);
				if(res <= 0)
				{
					return;
				}
				databuf[0] = LTR559_PS_THRES_UP_1; 
				databuf[1] = (u8)(((atomic_read(&obj->ps_thd_val_high)) & 0xFF00) >> 8);
				res = i2c_master_send(obj->client, databuf, 0x2);
				if(res <= 0)
				{
					return;
				}
		}
		//let up layer to know
		//if((err = hwmsen_get_interrupt_data(ID_PROXIMITY, &sensor_data)))
		//{
		  //APS_ERR("call hwmsen_get_interrupt_data fail = %d\n", err);
		//}
		if(ltr559_init_flag == 0)
			ps_report_interrupt_data(value);
	}
	//mt65xx_eint_unmask(CUST_EINT_ALS_NUM);      
	enable_irq(ltr559_obj->irq);

}



/****************************************************************************** 
 * Function Configuration
******************************************************************************/
static int ltr559_open(struct inode *inode, struct file *file)
{
	file->private_data = ltr559_i2c_client;

	if (!file->private_data)
	{
		APS_ERR("null pointer!!\n");
		return -EINVAL;
	}
	
	return nonseekable_open(inode, file);
}
/*----------------------------------------------------------------------------*/
static int ltr559_release(struct inode *inode, struct file *file)
{
	file->private_data = NULL;
	return 0;
}
/*----------------------------------------------------------------------------*/


static long ltr559_unlocked_ioctl(struct file *file, unsigned int cmd,
       unsigned long arg)       
{
	struct i2c_client *client = (struct i2c_client*)file->private_data;
	struct ltr559_priv *obj = i2c_get_clientdata(client);  
	int err = 0;
	void __user *ptr = (void __user*) arg;
	int dat;
	uint32_t enable;
	int ps_result;
	int ps_cali;
	int threshold[2];
	int ps_gain,als_gain;
	APS_DBG("cmd= %d\n", cmd); 
	switch (cmd)
	{
		case ALSPS_SET_PS_MODE:
			if(copy_from_user(&enable, ptr, sizeof(enable)))
			{
				err = -EFAULT;
				goto err_out;
			}
			//if(enable)
			//{
			    err = ltr559_ps_enable(obj->client, enable);
				if(err < 0)
				{
					APS_ERR("enable ps fail: %d en: %d\n", err, enable); 
					goto err_out;
				}
				set_bit(CMC_BIT_PS, &obj->enable);
			//}
			//else
			//{
			//    err = ltr559_ps_disable();
			//	if(err < 0)
			//	{
			//		APS_ERR("disable ps fail: %d\n", err); 
			//		goto err_out;
			//	}
			//	
			//	clear_bit(CMC_BIT_PS, &obj->enable);
			//}
			break;

		case ALSPS_GET_PS_MODE:
			enable = test_bit(CMC_BIT_PS, &obj->enable) ? (1) : (0);
			if(copy_to_user(ptr, &enable, sizeof(enable)))
			{
				err = -EFAULT;
				goto err_out;
			}
			break;

		case ALSPS_GET_PS_DATA:
			APS_DBG("ALSPS_GET_PS_DATA\n"); 
		    err = ltr559_ps_read(obj->client, &obj->ps);
			if(err < 0)
			{
				goto err_out;
			}
			
			dat = ltr559_get_ps_value(obj, obj->ps);
			if(copy_to_user(ptr, &dat, sizeof(dat)))
			{
				err = -EFAULT;
				goto err_out;
			}  
			break;

		case ALSPS_GET_PS_GAIN: // by baoyuanrui 2016-10-21
				ps_gain = ltr559_get_PSgain(obj->client);
				if ((ps_gain < 0) || (ps_gain > 3)) {
					goto  err_out;
				}
		
				if (copy_to_user(ptr, &ps_gain, sizeof(ps_gain))) {
					err = -EFAULT;
					goto  err_out;
				}
				break;
		
		case ALSPS_SET_PS_GAIN: // by baoyuanrui 2016-10-21
				if (copy_from_user(&ps_gain, ptr, sizeof(ps_gain))) {
					err = -EFAULT;
					goto  err_out;
				}
				err = ltr559_set_PSgain(obj->client, ps_gain);
				if (err <= 0) {
					goto err_out;
				}
				break;


		case ALSPS_GET_PS_RAW_DATA:    
			err = ltr559_ps_read(obj->client, &obj->ps);
			if(err < 0)
			{
				goto err_out;
			}
			dat = obj->ps;
			if(copy_to_user(ptr, &dat, sizeof(dat)))
			{
				err = -EFAULT;
				goto err_out;
			}  
			break;

		case ALSPS_SET_ALS_MODE:
			if(copy_from_user(&enable, ptr, sizeof(enable)))
			{
				err = -EFAULT;
				goto err_out;
			}
			//if(enable)
			//{
			    err = ltr559_als_enable(obj->client, enable);
				if(err < 0)
				{
					APS_ERR("enable als fail: %d en: %d\n", err, enable); 
					goto err_out;
				}
				set_bit(CMC_BIT_ALS, &obj->enable);
			//}
			//else
			//{
			//    err = ltr559_als_disable();
			//	if(err < 0)
			//	{
			//		APS_ERR("disable als fail: %d\n", err); 
			//		goto err_out;
			//	}
			//	clear_bit(CMC_BIT_ALS, &obj->enable);
			//}
			break;

		case ALSPS_GET_ALS_MODE:
			enable = test_bit(CMC_BIT_ALS, &obj->enable) ? (1) : (0);
			if(copy_to_user(ptr, &enable, sizeof(enable)))
			{
				err = -EFAULT;
				goto err_out;
			}
			break;

		case ALSPS_GET_ALS_DATA: 
		    err = ltr559_als_read(obj->client, &obj->als);
			if(err < 0)
			{
				goto err_out;
			}

			dat = ltr559_get_als_value(obj, obj->als);
			if(copy_to_user(ptr, &dat, sizeof(dat)))
			{
				err = -EFAULT;
				goto err_out;
			}              
			break;

		case ALSPS_GET_ALS_GAIN: // by baoyuanrui 2016-10-21
				als_gain = ltr559_get_ALSgain(obj->client);
				if ((als_gain < 0) || (als_gain > 3)) {
					goto  err_out;
				}
		
				if (copy_to_user(ptr, &als_gain, sizeof(als_gain))) {
					err = -EFAULT;
					goto  err_out;
				}
				break;
		
		case ALSPS_SET_ALS_GAIN: // by baoyuanrui 2016-10-21
				if (copy_from_user(&als_gain, ptr, sizeof(als_gain))) {
					err = -EFAULT;
					goto  err_out;
				}
				err = ltr559_set_ALSgain(obj->client, als_gain);
				if (err <= 0) {
					goto err_out;
				}
				break;


		case ALSPS_GET_ALS_RAW_DATA:    
			err = ltr559_als_read(obj->client, &obj->als);
			if(err < 0)
			{
				goto err_out;
			}

			dat = obj->als;
			if(copy_to_user(ptr, &dat, sizeof(dat)))
			{
				err = -EFAULT;
				goto err_out;
			}              
			break;
			
/*----------------------------------for factory mode test add by hongguang---------------------------------------*/
		case ALSPS_GET_PS_TEST_RESULT:
			err = ltr559_ps_read(obj->client, &obj->ps);
			if (err)
				goto err_out;
		
			if (obj->ps > atomic_read(&obj->ps_thd_val_high))
				ps_result = 0;
			else
				ps_result = 1;
		
			if (copy_to_user(ptr, &ps_result, sizeof(ps_result))) {
				err = -EFAULT;
				goto err_out;
			}
			break;
		
		case ALSPS_IOCTL_CLR_CALI:
			if (copy_from_user(&dat, ptr, sizeof(dat))) {
				err = -EFAULT;
				goto err_out;
			}
			if (dat == 0)
				obj->ps_cali = 0;
			break;
		
		case ALSPS_IOCTL_GET_CALI:
			ps_cali = obj->ps_cali;
			if (copy_to_user(ptr, &ps_cali, sizeof(ps_cali))) {
				err = -EFAULT;
				goto err_out;
			}
			break;
		
		case ALSPS_IOCTL_SET_CALI:
			if (copy_from_user(&ps_cali, ptr, sizeof(ps_cali))) {
				err = -EFAULT;
				goto err_out;
			}
		
			obj->ps_cali = ps_cali;
			break;
		
		case ALSPS_SET_PS_THRESHOLD:
			if (copy_from_user(threshold, ptr, sizeof(threshold))) {
				err = -EFAULT;
				goto err_out;
			}
			APS_ERR("%s set threshold high: 0x%x, low: 0x%x\n", __func__, threshold[0],	threshold[1]);
			atomic_set(&obj->ps_thd_val_high, (threshold[0] + obj->ps_cali));
			atomic_set(&obj->ps_thd_val_low, (threshold[1] + obj->ps_cali));	/* need to confirm */
		
			//set_psensor_threshold(obj->client);penghoubing
		
			break;
		
		case ALSPS_GET_PS_THRESHOLD_HIGH:
			threshold[0] = atomic_read(&obj->ps_thd_val_high) - obj->ps_cali;
			APS_ERR("%s get threshold high: 0x%x\n", __func__, threshold[0]);
			if (copy_to_user(ptr, &threshold[0], sizeof(threshold[0]))) {
				err = -EFAULT;
				goto err_out;
			}
			break;
		
		case ALSPS_GET_PS_THRESHOLD_LOW:
			threshold[0] = atomic_read(&obj->ps_thd_val_low) - obj->ps_cali;
			APS_ERR("%s get threshold low: 0x%x\n", __func__, threshold[0]);
			if (copy_to_user(ptr, &threshold[0], sizeof(threshold[0]))) {
				err = -EFAULT;
				goto err_out;
			}
			break;
/*-------------------------------for factory mode test add by hongguang-----------------------------------------------------------*/

		default:
			APS_ERR("%s not supported = 0x%04x", __FUNCTION__, cmd);
			err = -ENOIOCTLCMD;
			break;
	}

	err_out:
	return err;    
}

/*----------------------------------------------------------------------------*/
static struct file_operations ltr559_fops = {
	//.owner = THIS_MODULE,
	.open = ltr559_open,
	.release = ltr559_release,
	.unlocked_ioctl = ltr559_unlocked_ioctl,
};
/*----------------------------------------------------------------------------*/
static struct miscdevice ltr559_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "als_ps",
	.fops = &ltr559_fops,
};

static int ltr559_i2c_suspend(struct i2c_client *client, pm_message_t msg) 
{
#if 0
	struct ltr559_priv *obj = i2c_get_clientdata(client);    
	int err;
	APS_FUN();    

	if(msg.event == PM_EVENT_SUSPEND)
	{   
		if(!obj)
		{
			APS_ERR("null pointer!!\n");
			return -EINVAL;
		}
		
		atomic_set(&obj->als_suspend, 1);
		err = ltr559_als_enable(obj->client, 0);
		if(err < 0)
		{
			APS_ERR("disable als: %d\n", err);
			return err;
		}

	}
	#endif
	return 0;
}
/*----------------------------------------------------------------------------*/
static int ltr559_i2c_resume(struct i2c_client *client)
{
#if 0
	struct ltr559_priv *obj = i2c_get_clientdata(client);        
	int err;
	APS_FUN();

	if(!obj)
	{
		APS_ERR("null pointer!!\n");
		return -EINVAL;
	}

/*	err = ltr559_devinit();
	if(err < 0)
	{
		APS_ERR("initialize client fail!!\n");
		return err;        
	}*/
	atomic_set(&obj->als_suspend, 0);
	if(test_bit(CMC_BIT_ALS, &obj->enable))
	{
	    err = ltr559_als_enable(obj->client, 1);
	    if (err < 0)
		{
			APS_ERR("enable als fail: %d\n", err);        
		}
	}
#endif
	return 0;
}
#if defined(CONFIG_HAS_EARLYSUSPEND)
static void ltr559_early_suspend(struct early_suspend *h) 
{   /*early_suspend is only applied for ALS*/
	#if 0
	struct ltr559_priv *obj = container_of(h, struct ltr559_priv, early_drv);   
	int err;
	APS_FUN();    

	if(!obj)
	{
		APS_ERR("null pointer!!\n");
		return;
	}
	
	atomic_set(&obj->als_suspend, 1); 
	err = ltr559_als_enable(obj->client, 0);
	if(err < 0)
	{
		APS_ERR("disable als fail: %d\n", err); 
	}
	#endif
}

static void ltr559_late_resume(struct early_suspend *h)
{   /*early_suspend is only applied for ALS*/
#if 0
	struct ltr559_priv *obj = container_of(h, struct ltr559_priv, early_drv);         
	int err;
	APS_FUN();

	if(!obj)
	{
		APS_ERR("null pointer!!\n");
		return;
	}

	atomic_set(&obj->als_suspend, 0);
	if(test_bit(CMC_BIT_ALS, &obj->enable))
	{
	    err = ltr559_als_enable(obj->client, 1);
		if(err < 0)
		{
			APS_ERR("enable als fail: %d\n", err);        

		}
	}
	#endif
}
#endif
#if 0
int ltr559_ps_operate(void* self, uint32_t command, void* buff_in, int size_in,
		void* buff_out, int size_out, int* actualout)
{
	int err = 0;
	int value;
	hwm_sensor_data* sensor_data;
	struct ltr559_priv *obj = (struct ltr559_priv *)self;
	
	switch (command)
	{
		case SENSOR_DELAY:
			if((buff_in == NULL) || (size_in < sizeof(int)))
			{
				APS_ERR("Set delay parameter error!\n");
				err = -EINVAL;
			}
			// Do nothing
			break;

		case SENSOR_ENABLE:
			if((buff_in == NULL) || (size_in < sizeof(int)))
			{
				APS_ERR("Enable sensor parameter error!\n");
				err = -EINVAL;
			}
			else
			{				
				value = *(int *)buff_in;
				if(value)
				{
				    err = ltr559_ps_enable(obj->client, 1);
					if(err < 0)
					{
						APS_ERR("enable ps fail: %d\n", err); 
						return -1;
					}
					set_bit(CMC_BIT_PS, &obj->enable);
				}
				else
				{
				    err = ltr559_ps_disable(obj->client, 0);
					if(err < 0)
					{
						APS_ERR("disable ps fail: %d\n", err); 
						return -1;
					}
					clear_bit(CMC_BIT_PS, &obj->enable);
				}
			}
			break;

		case SENSOR_GET_DATA:
			if((buff_out == NULL) || (size_out< sizeof(hwm_sensor_data)))
			{
				APS_ERR("get sensor data parameter error!\n");
				err = -EINVAL;
			}
			else
			{
				APS_ERR("get sensor ps data !\n");
				sensor_data = (hwm_sensor_data *)buff_out;
				err = ltr559_ps_read(obj->client, &obj->ps);
    			if(err < 0)
    			{
    				err = -1;
    				break;
    			}
				sensor_data->values[0] = ltr559_get_ps_value(obj, obj->ps);
				sensor_data->value_divide = 1;
				sensor_data->status = SENSOR_STATUS_ACCURACY_MEDIUM;			
			}
			break;
		default:
			APS_ERR("proxmy sensor operate function no this parameter %d!\n", command);
			err = -1;
			break;
	}
	
	return err;
}

int ltr559_als_operate(void* self, uint32_t command, void* buff_in, int size_in,
		void* buff_out, int size_out, int* actualout)
{
	int err = 0;
	int value;
	hwm_sensor_data* sensor_data;
	struct ltr559_priv *obj = (struct ltr559_priv *)self;

	switch (command)
	{
		case SENSOR_DELAY:
			if((buff_in == NULL) || (size_in < sizeof(int)))
			{
				APS_ERR("Set delay parameter error!\n");
				err = -EINVAL;
			}
			// Do nothing
			break;

		case SENSOR_ENABLE:
			if((buff_in == NULL) || (size_in < sizeof(int)))
			{
				APS_ERR("Enable sensor parameter error!\n");
				err = -EINVAL;
			}
			else
			{
				value = *(int *)buff_in;				
				if(value)
				{
				    err = ltr559_als_enable(obj->client, 1);
					if(err < 0)
					{
						APS_ERR("enable als fail: %d\n", err); 
						return -1;
					}
					set_bit(CMC_BIT_ALS, &obj->enable);
				}
				else
				{
				    err = ltr559_als_disable(obj->client, 0);
					if(err < 0)
					{
						APS_ERR("disable als fail: %d\n", err); 
						return -1;
					}
					clear_bit(CMC_BIT_ALS, &obj->enable);
				}
				
			}
			break;

		case SENSOR_GET_DATA:
			if((buff_out == NULL) || (size_out< sizeof(hwm_sensor_data)))
			{
				APS_ERR("get sensor data parameter error!\n");
				err = -EINVAL;
			}
			else
			{
				APS_ERR("get sensor als data !\n");
				sensor_data = (hwm_sensor_data *)buff_out;
				obj->als = ltr559_als_read(obj->client, &obj->als);
                #if defined(MTK_AAL_SUPPORT)
				sensor_data->values[0] = obj->als;
				#else
				sensor_data->values[0] = ltr559_get_als_value(obj, obj->als);
				#endif
				sensor_data->value_divide = 1;
				sensor_data->status = SENSOR_STATUS_ACCURACY_MEDIUM;
			}
			break;
		default:
			APS_ERR("light sensor operate function no this parameter %d!\n", command);
			err = -1;
			break;
	}
	
	return err;
}
#endif

/*----------------------------------------------------------------------------*/
static int ltr559_i2c_detect(struct i2c_client *client, struct i2c_board_info *info) 
{    
	strcpy(info->type, LTR559_DEV_NAME);
	return 0;
}

//extern char *g_alsps_name;
/*----------------------------------------------------------------------------*/
static int ltr559_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct ltr559_priv *obj;
	//struct hwmsen_object obj_ps, obj_als;
	struct als_control_path als_ctl = {0};
	struct als_data_path als_data = {0};
	struct ps_control_path ps_ctl = {0};
	struct ps_data_path ps_data = {0};
	int err = 0;
	//int tang_tem = 0;
//    printk(KERN_EMERG "[TANG]ltr559_i2c_probe\n");
	APS_LOG("ltr559_i2c_probe\n");

	if(!(obj = kzalloc(sizeof(*obj), GFP_KERNEL)))
	{
		err = -ENOMEM;
		goto exit;
	}
	memset(obj, 0, sizeof(*obj));
	ltr559_obj = obj;

	obj->hw = hw;
	ltr559_get_addr(obj->hw, &obj->addr);
	//client->addr = 0x23;
	INIT_WORK(&obj->eint_work, ltr559_eint_work);
	obj->client = client;
	i2c_set_clientdata(client, obj);	
	atomic_set(&obj->als_debounce, 300);
	atomic_set(&obj->als_deb_on, 0);
	atomic_set(&obj->als_deb_end, 0);
	atomic_set(&obj->ps_debounce, 300);
	atomic_set(&obj->ps_deb_on, 0);
	atomic_set(&obj->ps_deb_end, 0);
	atomic_set(&obj->ps_mask, 0);
	atomic_set(&obj->als_suspend, 0);
	atomic_set(&obj->ps_thd_val_high,  obj->hw->ps_threshold_high);
	atomic_set(&obj->ps_thd_val_low,  obj->hw->ps_threshold_low);
	//atomic_set(&obj->als_cmd_val, 0xDF);
	//atomic_set(&obj->ps_cmd_val,  0xC1);
	atomic_set(&obj->ps_thd_val,  obj->hw->ps_threshold);

	
	obj->irq_node = of_find_compatible_node(NULL, NULL, "mediatek, als-eint");

	obj->enable = 0;
	obj->pending_intr = 0;
	obj->als_level_num = sizeof(obj->hw->als_level)/sizeof(obj->hw->als_level[0]);
	obj->als_value_num = sizeof(obj->hw->als_value)/sizeof(obj->hw->als_value[0]);   
	obj->als_modulus = (400*100)/(16*150);//(1/Gain)*(400/Tine), this value is fix after init ATIME and CONTROL register value
										//(400)/16*2.72 here is amplify *100
	BUG_ON(sizeof(obj->als_level) != sizeof(obj->hw->als_level));
	memcpy(obj->als_level, obj->hw->als_level, sizeof(obj->als_level));
	BUG_ON(sizeof(obj->als_value) != sizeof(obj->hw->als_value));
	memcpy(obj->als_value, obj->hw->als_value, sizeof(obj->als_value));
	atomic_set(&obj->i2c_retry, 3);
	set_bit(CMC_BIT_ALS, &obj->enable);
	set_bit(CMC_BIT_PS, &obj->enable);

	APS_LOG("ltr559_devinit() start...!\n");
	client->addr = 0x23;
	ltr559_i2c_client = client;
	//printk(KERN_EMERG "[TANG] ltr559 write_addr is %d,client addr is %d\n",obj->addr.write_addr,client->addr);
	err = ltr559_devinit();
	if(err)
	{
		goto exit_init_failed;
	}
	
	if((err = ltr559_setup_eint(client))!=0)
	{
		APS_ERR("setup eint: %d\n", err);
		//return err;
	}
	
	
	APS_LOG("ltr559_devinit() ...OK!\n");

	//printk("@@@@@@ manufacturer value:%x\n",ltr559_i2c_read_reg(0x87));

	err  = misc_register(&ltr559_device);
	if(err)
	{
		APS_ERR("ltr559_device register failed\n");
		goto exit_misc_device_register_failed;
	}

	
	/* Register sysfs attribute */
	err = ltr559_create_attr(&(ltr559_init_info.platform_diver_addr->driver));
	if(err)
	{
		printk(KERN_ERR "create attribute err = %d\n", err);
		goto exit_create_attr_failed;
	}
	

	als_ctl.open_report_data = als_open_report_data;
	als_ctl.enable_nodata = als_enable_nodata;
	als_ctl.set_delay  = als_set_delay;
	als_ctl.is_report_input_direct = false;
	als_ctl.is_support_batch = false;


	err = als_register_control_path(&als_ctl);
	if (err) {
		APS_ERR("register fail = %d\n", err);
		goto exit_sensor_obj_attach_fail;
	}

	als_data.get_data = als_get_data;
	als_data.vender_div = 100;
	err = als_register_data_path(&als_data);
	if (err) {
		APS_ERR("tregister fail = %d\n", err);
		goto exit_sensor_obj_attach_fail;
	}

	ps_ctl.open_report_data = ps_open_report_data;
	ps_ctl.enable_nodata = ps_enable_nodata;
	ps_ctl.set_delay  = ps_set_delay;
	ps_ctl.is_report_input_direct = false;
	ps_ctl.is_support_batch = false;
	err = ps_register_control_path(&ps_ctl);
	if (err) {
		APS_ERR("register fail = %d\n", err);
		goto exit_sensor_obj_attach_fail;
	}

	ps_data.get_data = ps_get_data;
	ps_data.vender_div = 100;
	err = ps_register_data_path(&ps_data);
	if (err) {
		APS_ERR("tregister fail = %d\n", err);
		goto exit_sensor_obj_attach_fail;
	}

	err = batch_register_support_info(ID_LIGHT, als_ctl.is_support_batch, 1, 0);
	if (err)
		APS_ERR("register light batch support err = %d\n", err);

	err = batch_register_support_info(ID_PROXIMITY, ps_ctl.is_support_batch, 1, 0);
	if (err)
		APS_ERR("register proximity batch support err = %d\n", err);

#if defined(CONFIG_HAS_EARLYSUSPEND)
	obj->early_drv.level	= EARLY_SUSPEND_LEVEL_DISABLE_FB - 1,
	obj->early_drv.suspend	= ltr559_early_suspend,
	obj->early_drv.resume	= ltr559_late_resume,	 
	register_early_suspend(&obj->early_drv);
#endif

	#ifdef HQ_PSENSOR_CALIBRATION
	ltr559_get_init_noise();
	#endif
	
	ltr559_init_flag = 0;
	//g_alsps_name = LTR559_DEV_NAME;
	APS_LOG("%s: OK\n", __func__);
	return 0;
	
exit_create_attr_failed:
exit_sensor_obj_attach_fail:
exit_misc_device_register_failed:
		misc_deregister(&ltr559_device);
exit_init_failed:
		kfree(obj);
exit:
	ltr559_i2c_client = NULL;
	APS_ERR("%s: err = %d\n", __func__, err);
	ltr559_init_flag =  -1;
	return err;

}

/*----------------------------------------------------------------------------*/

static int ltr559_i2c_remove(struct i2c_client *client)
{
	int err;
	err = ltr559_delete_attr(&ltr559_i2c_driver.driver);
	if(err)
	{
		APS_ERR("ltr559_delete_attr fail: %d\n", err);
	} 

	err = misc_deregister(&ltr559_device);
	if(err)
	{
		APS_ERR("misc_deregister fail: %d\n", err);    
	}
	
	ltr559_i2c_client = NULL;
	i2c_unregister_device(client);
	kfree(i2c_get_clientdata(client));

	return 0;
}
/*----------------------------------------------------------------------------*/
#if 0
static int ltr559_probe(struct platform_device *pdev) 
{
	struct alsps_hw *hw = get_cust_alsps_hw();

	ltr559_power(hw, 1);
	//ltr559_force[0] = hw->i2c_num;
	//ltr559_force[1] = hw->i2c_addr[0];
	//APS_DBG("I2C = %d, addr =0x%x\n",ltr559_force[0],ltr559_force[1]);
	if(i2c_add_driver(&ltr559_i2c_driver))
	{
		APS_ERR("add driver error\n");
		return -1;
	} 
	return 0;
}
#endif
/*----------------------------------------------------------------------------*/
static int ltr559_remove(void)
{
	//struct alsps_hw *hw = get_cust_alsps_hw();
	APS_FUN();    
	ltr559_power(hw, 0);    
	i2c_del_driver(&ltr559_i2c_driver);
	return 0;
}
/*----------------------------------------------------------------------------*/
#if 0
static struct platform_driver ltr559_alsps_driver = {
	.probe      = ltr559_probe,
	.remove     = ltr559_remove,    
	.driver     = {
		.name  = "als_ps",
		//.owner = THIS_MODULE,
	}
};
#endif

//#ifdef CONFIG_OF
//static const struct of_device_id alsps_of_match[] = {
//	{ .compatible = "mediatek,als_ps", },
//	{},
//};
//#endif

//static struct platform_driver ltr559_alsps_driver =
//{
//	.probe      = ltr559_probe,
//	.remove     = ltr559_remove,    
//	.driver     = 
//	{
//		.name = "als_ps",
//       #ifdef CONFIG_OF
//		.of_match_table = alsps_of_match,
//		#endif
//	}
//};

static int  ltr559_local_init(void)
{
	
	ltr559_power(hw, 1);
	//printk(KERN_EMERG "[TANG]ltr559_local_init\n");
	if (i2c_add_driver(&ltr559_i2c_driver)) {
		APS_ERR("add driver error\n");
		return -1;
	}
	if (-1 == ltr559_init_flag)
		return -1;

	return 0;
}

/*----------------------------------------------------------------------------*/
static int __init ltr559_init(void)
{
	const char *name = "mediatek,LTR559";
    //printk(KERN_EMERG "[TANG]ltr559_init\n");
	APS_FUN();

	//i2c_register_board_info(0, &i2c_ltr559, 1);
	//if(platform_driver_register(&ltr559_alsps_driver))
	//{
	//	APS_ERR("failed to register driver");
	//	return -ENODEV;
	//}
	
	hw =   get_alsps_dts_func(name, hw);
	if (!hw)
		APS_ERR("get dts info fail\n");
	alsps_driver_add(&ltr559_init_info);
	return 0;
}
/*----------------------------------------------------------------------------*/
static void __exit ltr559_exit(void)
{
	APS_FUN();
	//platform_driver_unregister(&ltr559_alsps_driver);
}
/*----------------------------------------------------------------------------*/
module_init(ltr559_init);
module_exit(ltr559_exit);
/*----------------------------------------------------------------------------*/
MODULE_AUTHOR("XX Xx");
MODULE_DESCRIPTION("LTR-559ALS Driver");
MODULE_LICENSE("GPL");

