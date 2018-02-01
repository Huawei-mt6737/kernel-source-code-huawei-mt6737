#include <linux/kernel.h>
#include <linux/module.h>  
#include <linux/fs.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/list.h>
#include <linux/i2c.h>
#include <linux/irq.h>
//#include <linux/jiffies.h>
#include <linux/uaccess.h>
//#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/platform_device.h>

#include "lp3101.h"
/***************************************************************************** 
 * Define
 *****************************************************************************/
#define I2C_ID_NAME "lp3101"

/***************************************************************************** 
 * GLobal Variable
 *****************************************************************************/
static const struct of_device_id lcm_of_match[] = {
	{.compatible = "mediatek,i2c_lcd_bias"},
	{},
};

static struct i2c_client *lp3101_i2c_client = NULL;
static struct lcd_bais *lp_pinctrl = NULL;

/***************************************************************************** 
 * Function Prototype
 *****************************************************************************/ 
static int lp3101_probe(struct i2c_client *client, const struct i2c_device_id *id);
static int lp3101_remove(struct i2c_client *client);
/***************************************************************************** 
 * Data Structure
 *****************************************************************************/
/*
static struct lp3101_dev	{	
	struct i2c_client	*client;
};*/

static int lp3101_pinctrl_probe(struct platform_device *pdev)
{
		int ret = 0;
		struct lcd_bais *lcd_bais;
		
		LP_LOG("lp3101_pinctrl_probe 0\n");

		if (!lp_pinctrl)
			lp_pinctrl = kzalloc(sizeof(struct lcd_bais), GFP_KERNEL);
	
		lcd_bais = lp_pinctrl;
	
		lcd_bais->pinctrl = devm_pinctrl_get(&pdev->dev);
		
		LP_LOG("lp3101_pinctrl_probe 1\n");
		if (IS_ERR(lcd_bais->pinctrl)) 
		{
			LP_LOG("Cannot find lcd bais pinctrl!\n");
			ret = PTR_ERR(lcd_bais->pinctrl);
			return ret;
		}
		else
		{
			lcd_bais->pin_cfg = kzalloc(sizeof(struct lcd_bais_pinctrl), GFP_KERNEL);
	
			lcd_bais->pin_cfg->enn_low = pinctrl_lookup_state(lp_pinctrl->pinctrl, "lcd_bias_enn_low");
			if (IS_ERR(lcd_bais->pin_cfg->enn_low))
			{
				ret = PTR_ERR(lcd_bais->pin_cfg->enn_low);
				LP_LOG("Cannot find lp3101 pinctrl lcd_bias_enn_low!\n");
				return ret;
			}

			lcd_bais->pin_cfg->enn_high = pinctrl_lookup_state(lp_pinctrl->pinctrl, "lcd_bias_enn_high");
			if (IS_ERR(lcd_bais->pin_cfg->enn_high))
			{
				ret = PTR_ERR(lcd_bais->pin_cfg->enn_high);
				LP_LOG("Cannot find lp3101 pinctrl lcd_bias_enn_high!\n");
				return ret;
			}

			lcd_bais->pin_cfg->enp_low = pinctrl_lookup_state(lp_pinctrl->pinctrl, "lcd_bias_enp_low");
			if (IS_ERR(lcd_bais->pin_cfg->enp_low))
			{
				ret = PTR_ERR(lcd_bais->pin_cfg->enp_low);
				LP_LOG("Cannot find lp3101 pinctrl lcd_bias_enp_low!\n");
				return ret;
			}

			lcd_bais->pin_cfg->enp_high = pinctrl_lookup_state(lp_pinctrl->pinctrl, "lcd_bias_enp_high");
			if (IS_ERR(lcd_bais->pin_cfg->enp_high))
			{
				ret = PTR_ERR(lcd_bais->pin_cfg->enp_high);
				LP_LOG("Cannot find lp3101 pinctrl lcd_bias_enp_high!\n");
				return ret;
			}
		}

		LP_LOG("lp3101_pinctrl_probe success\n"); 

	return ret;
}

static int lp3101_pinctrl_remove(struct platform_device *pdev)
{
	return 0;
}

static const struct of_device_id lp3101_pinctrl_ids[] = {
	{.compatible = "mediatek,lcd_bais_pinctrl",},
	{},
};

static struct platform_driver lp3101_pinctrl_driver = {
	.probe = lp3101_pinctrl_probe,
	.remove = lp3101_pinctrl_remove,
	.driver = {
	.name = "lp3101_pinctrl",
#ifdef CONFIG_OF
	.of_match_table = lp3101_pinctrl_ids,
#endif
	},
};

void lcd_bais_enn_enable(u8 enable)
{
	if(enable == 1)
	{
		if (IS_ERR(lp_pinctrl->pin_cfg->enn_high))
			LP_LOG("Can not find lcd_bias_enn_high, set fail!!!");
		else
		{
			pinctrl_select_state(lp_pinctrl->pinctrl,lp_pinctrl->pin_cfg->enn_high);
			LP_LOG("enn high\n");
		}
	}
	else
	{
		if (IS_ERR(lp_pinctrl->pin_cfg->enn_low))
			LP_LOG("Can not find lcd_bias_enn_low, set fail!!!");
		else
		{
			pinctrl_select_state(lp_pinctrl->pinctrl,lp_pinctrl->pin_cfg->enn_low);
			LP_LOG("enn low\n");
		}
	}
}

void lcd_bais_enp_enable(u8 enable)
{
	if(enable == 1)
	{
		if (IS_ERR(lp_pinctrl->pin_cfg->enp_high))
			LP_LOG("Can not find lcd_bias_enp_high, set fail!!!");
		else
		{
			pinctrl_select_state(lp_pinctrl->pinctrl,lp_pinctrl->pin_cfg->enp_high);
			LP_LOG("enp high\n");
		}
	}
	else
	{
		if (IS_ERR(lp_pinctrl->pin_cfg->enp_low))
			LP_LOG("Can not find lcd_bias_enp_low, set fail!!!");
		else
		{
			pinctrl_select_state(lp_pinctrl->pinctrl,lp_pinctrl->pin_cfg->enp_low);
			LP_LOG("enp low\n");
		}
	}
}

static const struct i2c_device_id lp3101_id[] = {
	{ I2C_ID_NAME, 0 },
	{ }
};

static struct i2c_driver lp3101_i2c_driver = {
	.id_table		= lp3101_id,
	.probe		= lp3101_probe,
	.remove		= lp3101_remove,
	.driver		= {
		.owner	= THIS_MODULE,
		.name	= "lp3101",
#if !defined(CONFIG_MTK_LEGACY)
		.of_match_table = lcm_of_match,
#endif
	},
};
 

/***************************************************************************** 
 * Function
 *****************************************************************************/ 
static int lp3101_probe(struct i2c_client *client, const struct i2c_device_id *id)
{  
	LP_LOG("lp3101_probe: info==>name=%s addr=0x%x\n",client->name,client->addr);
	lp3101_i2c_client = client;		

	return 0;
}

static int lp3101_remove(struct i2c_client *client)
{
	LP_LOG( "lp3101_remove\n");
	lp3101_i2c_client = NULL;
	i2c_unregister_device(client);
	return 0;
}

int lp3101_write_bytes(unsigned char addr, unsigned char value)
{	
	int ret = 0;
	struct i2c_client *client = lp3101_i2c_client;
	char write_data[2]={0};	

	if(client == NULL)
	{
		printk("lp3101 write data fail 0 !!\n");
		return -1;
	}

	write_data[0]= addr;
	write_data[1] = value;
	ret=i2c_master_send(client, write_data, 2);
	if(ret<0)
		printk("lp3101 write data fail 1 !!\n");

	return ret ;
}

/*
 * module load/unload record keeping
 */
static int __init lp3101_i2c_init(void)
{
	int ret = 0;

	ret = platform_driver_register(&lp3101_pinctrl_driver);

	if(ret)
	{
		printk("%s: failed to register lp3101 pinctrl driver", __func__);
		return ret;
	}
	else
	{
		ret = i2c_add_driver(&lp3101_i2c_driver);
		if (ret < 0)
		{
			printk("%s: failed to register lp3101 i2c driver", __func__);
			return -1;
		}
	}
	LP_LOG( "lp3101_i2c_init success\n"); 
	return 0;
}

static void __exit lp3101_i2c_exit(void)
{
	LP_LOG( "lp3101_i2c_exit\n");
	i2c_del_driver(&lp3101_i2c_driver);  
}

module_init(lp3101_i2c_init);
module_exit(lp3101_i2c_exit);

MODULE_AUTHOR("Erick.wang");
MODULE_DESCRIPTION("MTK lp3101 I2C Driver");
MODULE_LICENSE("GPL"); 
