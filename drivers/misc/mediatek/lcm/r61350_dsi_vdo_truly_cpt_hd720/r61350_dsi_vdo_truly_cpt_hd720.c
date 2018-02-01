#ifndef BUILD_LK
#include <linux/string.h>
#include <linux/kernel.h>
#endif
#include "lcm_drv.h"

#ifdef BUILD_LK
#include <platform/upmu_common.h>
#include <platform/mt_gpio.h>
#include <platform/mt_i2c.h> 
#include <platform/mt_pmic.h>
#include <string.h>
#endif

//#ifndef CONFIG_PINCTRL_MT6735
//#include <cust_gpio_usage.h>
//#endif

#ifndef CONFIG_FPGA_EARLY_PORTING
#if defined(CONFIG_MTK_LEGACY)
#include <cust_i2c.h>
#endif
#endif

#ifdef BUILD_LK
#define LCD_DEBUG  printf
#else
#define LCD_DEBUG  printk
#endif

// ---------------------------------------------------------------------------
//  Local Constants
// ---------------------------------------------------------------------------
#define FRAME_WIDTH  (720)
#define FRAME_HEIGHT (1280)

#define LCM_ID (0x98)

#ifdef BUILD_LK
#define GPIO_LP3101_ENN   GPIO_LCD_BIAS_ENN_PIN
#define GPIO_LP3101_ENP   GPIO_LCD_BIAS_ENP_PIN
#endif

#define  LCM_DSI_CMD_MODE	0

// ---------------------------------------------------------------------------
//  Local Variables
// ---------------------------------------------------------------------------

static LCM_UTIL_FUNCS lcm_util = {0};

#define SET_RESET_PIN(v)    (lcm_util.set_reset_pin((v)))

#define UDELAY(n) (lcm_util.udelay(n))
#define MDELAY(n) (lcm_util.mdelay(n))


// ---------------------------------------------------------------------------
//  Local Functions
// ---------------------------------------------------------------------------

#define dsi_set_cmdq_V2(cmd, count, ppara, force_update)	        lcm_util.dsi_set_cmdq_V2(cmd, count, ppara, force_update)
#define dsi_set_cmdq(pdata, queue_size, force_update)				lcm_util.dsi_set_cmdq(pdata, queue_size, force_update)
#define wrtie_cmd(cmd)												lcm_util.dsi_write_cmd(cmd)
#define write_regs(addr, pdata, byte_nums)							lcm_util.dsi_write_regs(addr, pdata, byte_nums)
#define read_reg(cmd)												lcm_util.dsi_dcs_read_lcm_reg(cmd)
#define read_reg_v2(cmd, buffer, buffer_size)   					lcm_util.dsi_dcs_read_lcm_reg_v2(cmd, buffer, buffer_size)   
#define dsi_swap_port(swap)				lcm_util.dsi_swap_port(swap)	//wqtao.add for build error.

#ifndef BUILD_LK
#define set_gpio_lcd_enp(cmd) lcm_util.set_gpio_lcd_enp_bias(cmd)
#define set_gpio_lcd_enn(cmd) lcm_util.set_gpio_lcd_enn_bias(cmd)
//#define set_gpio_lcd_power_enable(cmd) lcm_util.set_gpio_lcd_power_enable(cmd)
#endif
struct LCM_setting_table {
	unsigned int cmd;
	unsigned char count;
	unsigned char para_list[64];
};


#ifdef BUILD_LK
#define LP3101_SLAVE_ADDR_WRITE  0x7C  
static struct mt_i2c_t lp3101_i2c;

static int lp3101_write_bytes(kal_uint8 addr, kal_uint8 value)
{
	kal_uint32 ret_code = I2C_OK;
	kal_uint8 write_data[2];
	kal_uint16 len;

	write_data[0]= addr;
	write_data[1] = value;

	lp3101_i2c.id = I2C1;
	/* Since i2c will left shift 1 bit, we need to set FAN5405 I2C address to >>1 */
	lp3101_i2c.addr = (LP3101_SLAVE_ADDR_WRITE >> 1);
	lp3101_i2c.mode = ST_MODE;
	lp3101_i2c.speed = 100;
	len = 2;

	ret_code = i2c_write(&lp3101_i2c, write_data, len);
	//LCD_DEBUG("%s: i2c_write: ret_code: %d\n", __func__, ret_code);

	return ret_code;
}
#endif

#define REGFLAG_PORT_SWAP	0xFFFA
#define REGFLAG_DELAY		0xFFFC
#define REGFLAG_END_OF_TABLE	0xFFFD   /* END OF REGISTERS MARKER */

/*
static void push_table(struct LCM_setting_table *table, unsigned int count, unsigned char force_update)
{
	unsigned int i;

	for (i = 0; i < count; i++) {
		unsigned cmd;

		cmd = table[i].cmd;

		switch (cmd) {
		case REGFLAG_DELAY:
			if (table[i].count <= 10)
				MDELAY(table[i].count);
			else
				MDELAY(table[i].count);
			break;
		case REGFLAG_END_OF_TABLE:
			break;
		case REGFLAG_PORT_SWAP:
			dsi_swap_port(1);
			break;
		default:
			dsi_set_cmdq_V2(cmd, table[i].count, table[i].para_list, force_update);
		}
	}
}

static struct LCM_setting_table init_setting[] = {
	{REGFLAG_END_OF_TABLE, 0x00, {} }
};
*/


static void init_lcm_registers(void)
{
	unsigned int data_array[16];

data_array[0] = 0x00b02300;
dsi_set_cmdq(data_array, 1, 1);

data_array[0] = 0x00022902;
data_array[1] = 0x000001e3;
dsi_set_cmdq(data_array, 2, 1);

data_array[0] = 0x00032902;
data_array[1] = 0x002c61b6;
dsi_set_cmdq(data_array, 2, 1);

data_array[0] = 0x00072902;
data_array[1] = 0x10b223c0;
data_array[2] = 0x007f0210;
dsi_set_cmdq(data_array, 3, 1);

data_array[0] = 0x00172902;
data_array[1] = 0xa08a0bc1;
data_array[2] = 0x0000008a;
data_array[3] = 0x00030000;
data_array[4] = 0x00000000;
data_array[5] = 0x00000000;
data_array[6] = 0x00000000;
dsi_set_cmdq(data_array, 7, 1);

data_array[0] = 0x001d2902;
data_array[1] = 0x400407c5;
data_array[2] = 0x03000004;
data_array[3] = 0x00078001;
data_array[4] = 0x00000000;
data_array[5] = 0x00000000;
data_array[6] = 0x00000000;
data_array[7] = 0x00000000;
data_array[8] = 0x0000000e;
dsi_set_cmdq(data_array, 9, 1);

data_array[0] = 0x00032902;
data_array[1] = 0x000201c6;
dsi_set_cmdq(data_array, 2, 1);

data_array[0] = 0x003b2902;
data_array[1] = 0x222001c8;
data_array[2] = 0x00002624;
data_array[3] = 0x00001d1d;
data_array[4] = 0x1d1d1d00;
data_array[5] = 0x00000000;
data_array[6] = 0x02091311;
data_array[7] = 0x27252321;
data_array[8] = 0x1d1d0000;
data_array[9] = 0x1d000000;
data_array[10] = 0x00001d1d;
data_array[11] = 0x13110000;
data_array[12] = 0x0000890a;
data_array[13] = 0x00000100;
data_array[14] = 0x00000200;
data_array[15] = 0x00000300;
dsi_set_cmdq(data_array, 16, 1);

data_array[0] = 0x00272902;
data_array[1] = 0x2c1c00ca;
data_array[2] = 0x6a53483c;
data_array[3] = 0x4b978a7b;
data_array[4] = 0x847a6557;
data_array[5] = 0xbfb3a991;
data_array[6] = 0x3c2c1c00;
data_array[7] = 0x7b6a5348;
data_array[8] = 0x574b978a;
data_array[9] = 0x91847a65;
data_array[10] = 0x00bfb3a9;
dsi_set_cmdq(data_array, 11, 1);

data_array[0] = 0x00072902;
data_array[1] = 0x415202d0;
data_array[2] = 0x00993400;
dsi_set_cmdq(data_array, 3, 1);

data_array[0] = 0x00022902;
data_array[1] = 0x000003d1;
dsi_set_cmdq(data_array, 2, 1);

data_array[0] = 0x00032902;
data_array[1] = 0x000b8ed2;
dsi_set_cmdq(data_array, 2, 1);

data_array[0] = 0x00022902;
data_array[1] = 0x000002e5;
dsi_set_cmdq(data_array, 2, 1);


data_array[0] = 0x00032902;
data_array[1] = 0x002828d5;
dsi_set_cmdq(data_array, 2, 1);

data_array[0] = 0x03b02300;
dsi_set_cmdq(data_array, 1, 1);

data_array[0] = 0xff511500;
dsi_set_cmdq(data_array, 1, 1);

data_array[0] = 0x2c531500;
dsi_set_cmdq(data_array, 1, 1);

data_array[0] = 0x00551500;
dsi_set_cmdq(data_array, 1, 1);

data_array[0] = 0x00110500;
dsi_set_cmdq(data_array, 1, 1);
MDELAY(120);

data_array[0] = 0x00290500;
dsi_set_cmdq(data_array, 1, 1);
MDELAY(20);
}


// ---------------------------------------------------------------------------
//  LCM Driver Implementations
// ---------------------------------------------------------------------------

static void lcm_set_util_funcs(const LCM_UTIL_FUNCS *util)
{
	memcpy(&lcm_util, util, sizeof(LCM_UTIL_FUNCS));
}

static void lcm_get_params(LCM_PARAMS *params)
{
	memset(params, 0, sizeof(LCM_PARAMS));

	params->type   = LCM_TYPE_DSI;

	params->width  = FRAME_WIDTH;
	params->height = FRAME_HEIGHT;

	params->physical_width = 62;
	params->physical_height = 110;

	params->dsi.mode   = BURST_VDO_MODE;

	// DSI

	params->dsi.LANE_NUM				= LCM_FOUR_LANE;
	//The following defined the fomat for data coming from LCD engine.
	params->dsi.data_format.color_order = LCM_COLOR_ORDER_RGB;
	params->dsi.data_format.trans_seq   = LCM_DSI_TRANS_SEQ_MSB_FIRST;
	params->dsi.data_format.padding     = LCM_DSI_PADDING_ON_LSB;
	params->dsi.data_format.format      = LCM_DSI_FORMAT_RGB888;


	params->dsi.PS=LCM_PACKED_PS_24BIT_RGB888;
	
	params->dsi.vertical_sync_active				= 4; // 3;
	params->dsi.vertical_backporch					= 12; //7; // 10; // 16; // 20;   // zrl 2013-09-24  is important if up or down black/white line
	params->dsi.vertical_frontporch					= 16; // 20;	
	params->dsi.vertical_active_line				= FRAME_HEIGHT;

	params->dsi.horizontal_sync_active				= 10;//20
	params->dsi.horizontal_backporch				= 54;//70
	params->dsi.horizontal_frontporch				= 80;//100 

//inx  vbp=6,vfp=10,vsw=2,hbp=70,hfp=100,hsw=20
//cpt   vbp=12,vfp=16,vsw=4,hbp=70,hfp=100,hsw=20

	params->dsi.horizontal_active_pixel				= FRAME_WIDTH;

	params->dsi.PLL_CLOCK				= 214;
	params->dsi.ssc_disable = 0;
	params->dsi.ssc_range = 2;
	params->dsi.HS_TRAIL = 6;

	params->dsi.esd_check_enable = 1;
	params->dsi.customization_esd_check_enable = 1;
	params->dsi.lcm_esd_check_table[0].cmd          = 0x0A;
	params->dsi.lcm_esd_check_table[0].count        = 1;
	params->dsi.lcm_esd_check_table[0].para_list[0] = 0x9C;
//	params->dsi.lcm_esd_check_table[1].cmd          = 0x09;
//	params->dsi.lcm_esd_check_table[1].count        = 1;
//	params->dsi.lcm_esd_check_table[1].para_list[0] = 0x80;

	params->dsi.noncont_clock = 1;		
	params->dsi.noncont_clock_period = 2;

}


static void lcm_init_power(void)
{
	int ret=0;
	unsigned char cmd = 0x0;
	unsigned char data = 0xFF;

	cmd=0x00;
	data=0x0F;//5.5v    0x14: 6v

#ifdef BUILD_LK
#ifdef GPIO_LCM_PWR_EN
	mt_set_gpio_mode(GPIO_LCM_PWR_EN, GPIO_MODE_00);
	mt_set_gpio_dir(GPIO_LCM_PWR_EN, GPIO_DIR_OUT);
	mt_set_gpio_out(GPIO_LCM_PWR_EN, GPIO_OUT_ONE);
	MDELAY(5);
#endif
        
	mt_set_gpio_mode(GPIO_LP3101_ENP, GPIO_MODE_00);	//data sheet 136 page ,the first AVDD power on
	mt_set_gpio_dir(GPIO_LP3101_ENP, GPIO_DIR_OUT);
	mt_set_gpio_out(GPIO_LP3101_ENP, GPIO_OUT_ONE);
        
#else
	//set_gpio_lcd_power_enable(1);
	MDELAY(5);
	//	set_gpio_lcd_enp(1);
	lcd_bais_enp_enable(1);
#endif
	MDELAY(5);

	ret = lp3101_write_bytes(cmd, data);
	LCD_DEBUG("[r61350cpt-truly]: %s, line%d,	ret=%d\n", __func__, __LINE__, ret);
	MDELAY(2);

	cmd=0x01;
	data=0x0F;//5.5v    0x14: 6v

#ifdef BUILD_LK
	mt_set_gpio_mode(GPIO_LP3101_ENN, GPIO_MODE_00);
	mt_set_gpio_dir(GPIO_LP3101_ENN, GPIO_DIR_OUT);
	mt_set_gpio_out(GPIO_LP3101_ENN, GPIO_OUT_ONE);
#else
	//	set_gpio_lcd_enn(1);
	lcd_bais_enn_enable(1);
#endif
	// MDELAY(5);

	ret = lp3101_write_bytes(cmd, data);
	LCD_DEBUG("[r61350cpt-truly]: %s, line%d,	ret=%d\n", __func__, __LINE__, ret);

	// MDELAY(10);
}

static void lcm_suspend_power(void)
{
	//SET_RESET_PIN(0);
	MDELAY(5);
#ifdef BUILD_LK
	mt_set_gpio_mode(GPIO_LP3101_ENN, GPIO_MODE_00);
	mt_set_gpio_dir(GPIO_LP3101_ENN, GPIO_DIR_OUT);
	mt_set_gpio_out(GPIO_LP3101_ENN, GPIO_OUT_ZERO);
#else
	//	set_gpio_lcd_enn(0);
	lcd_bais_enn_enable(0);
#endif
	MDELAY(5);

#ifdef BUILD_LK
	mt_set_gpio_mode(GPIO_LP3101_ENP, GPIO_MODE_00);	//data sheet 136 page ,the first AVDD power on
	mt_set_gpio_dir(GPIO_LP3101_ENP, GPIO_DIR_OUT);
	mt_set_gpio_out(GPIO_LP3101_ENP, GPIO_OUT_ZERO);

#ifdef GPIO_LCM_PWR_EN
	mt_set_gpio_mode(GPIO_LCM_PWR_EN, GPIO_MODE_00);
	mt_set_gpio_dir(GPIO_LCM_PWR_EN, GPIO_DIR_OUT);
	mt_set_gpio_out(GPIO_LCM_PWR_EN, GPIO_OUT_ZERO);
#endif
#else
	//	set_gpio_lcd_enp(0);
	lcd_bais_enp_enable(0);
	//set_gpio_lcd_power_enable(0);
#endif
}

static void lcm_resume_power(void)
{
	lcm_init_power();
}

static void lcm_init(void)
{
	SET_RESET_PIN(1);
	MDELAY(2);   //10   
	SET_RESET_PIN(0);
	MDELAY(10);
	SET_RESET_PIN(1);
	MDELAY(50);  //120

	init_lcm_registers();
	LCD_DEBUG("[r61350cpt-truly]: %s, line%d\n", __func__, __LINE__);
}





static void lcm_suspend(void) 
{ 

	unsigned int data_array[2]; 

	SET_RESET_PIN(1); 
	MDELAY(2); //10 
	SET_RESET_PIN(0); 
	MDELAY(10); 
	SET_RESET_PIN(1); 
	MDELAY(40); //50 

	data_array[0] = 0x00280500; // Display Off 
	dsi_set_cmdq(data_array, 1, 1); 
	MDELAY(20); 
	data_array[0] = 0x00100500; // Sleep In 
	dsi_set_cmdq(data_array, 1, 1); 
	MDELAY(130); 

	data_array[0] = 0x00022902; 
	data_array[1] = 0x000000b0; 
	dsi_set_cmdq(data_array, 2, 1); 

	data_array[0] = 0x00022902; //deep standby 
	data_array[1] = 0x000001b1; 
	dsi_set_cmdq(data_array, 2, 1); 
	MDELAY(10); 
	LCD_DEBUG("[r61350cpt-truly]: %s, line%d\n", __func__, __LINE__); 
}


static void lcm_resume(void)
{
	lcm_init();
	LCD_DEBUG("[r61350cpt-truly]: %s, line%d\n", __func__, __LINE__);
}


#if (LCM_DSI_CMD_MODE)
static void lcm_update(unsigned int x, unsigned int y,
                       unsigned int width, unsigned int height)
{
	unsigned int x0 = x;
	unsigned int y0 = y;
	unsigned int x1 = x0 + width - 1;
	unsigned int y1 = y0 + height - 1;

	unsigned char x0_MSB = ((x0>>8)&0xFF);
	unsigned char x0_LSB = (x0&0xFF);
	unsigned char x1_MSB = ((x1>>8)&0xFF);
	unsigned char x1_LSB = (x1&0xFF);
	unsigned char y0_MSB = ((y0>>8)&0xFF);
	unsigned char y0_LSB = (y0&0xFF);
	unsigned char y1_MSB = ((y1>>8)&0xFF);
	unsigned char y1_LSB = (y1&0xFF);

	unsigned int data_array[16];

	data_array[0]= 0x00053902;
	data_array[1]= (x1_MSB<<24)|(x0_LSB<<16)|(x0_MSB<<8)|0x2a;
	data_array[2]= (x1_LSB);
	dsi_set_cmdq(data_array, 3, 1);
	
	data_array[0]= 0x00053902;
	data_array[1]= (y1_MSB<<24)|(y0_LSB<<16)|(y0_MSB<<8)|0x2b;
	data_array[2]= (y1_LSB);
	dsi_set_cmdq(data_array, 3, 1);

//	data_array[0]= 0x00290508; //HW bug, so need send one HS packet
//	dsi_set_cmdq(data_array, 1, 1);
	
	data_array[0]= 0x002c3909;
	dsi_set_cmdq(data_array, 1, 0);
}
#endif

static unsigned int lcm_compare_id(void)
{
	unsigned char id[5]={0};
        unsigned char id1[3]={0};
	unsigned char buffer[5];
	unsigned int data_array[16];
        unsigned int array[16];    
	SET_RESET_PIN(1);
	MDELAY(1);
	SET_RESET_PIN(0);
	MDELAY(10);
	SET_RESET_PIN(1);
	MDELAY(20);  //120

	data_array[0] = 0x00022902;
	data_array[1] = 0x000004b0;
	dsi_set_cmdq(data_array, 2, 1);
	array[0] = 0x00053700;
	dsi_set_cmdq(array, 1, 1);
	MDELAY(2);
        	
	read_reg_v2(0xBF, buffer, 5);
        
	id[0] = buffer[0];
	id[1] = buffer[1];
	id[2] = buffer[2];         
	id[3] = buffer[3];
	id[4] = buffer[4];
	array[0] = 0x00013700;
	dsi_set_cmdq(array, 1, 1);
        read_reg_v2(0xDA, buffer, 1); 
        id1[0] = buffer[0];  

	LCD_DEBUG("[r61350cpt-truly]: %s, line%d, id0=%x\n", __func__, __LINE__, id[0]);
	LCD_DEBUG("[r61350cpt-truly: %s, line%d, id-da=%x\n", __func__, __LINE__, id1[0]);
	LCD_DEBUG("[r61350cpt-truly]: %s, line%d, id1=%x\n", __func__, __LINE__, id[1]);
	LCD_DEBUG("[r61350cpt-truly]: %s, line%d, id2=%x\n", __func__, __LINE__, id[2]);
	LCD_DEBUG("[r61350cpt-truly]: %s, line%d, id3=%x\n", __func__, __LINE__, id[3]);
	LCD_DEBUG("[r61350cpt-truly]: %s, line%d, id4=%x\n", __func__, __LINE__, id[4]);


	if(id[0] == 0x02 && id1[0]== 0x1A)// && id[2] == 0x13)
		return 1;
	LCD_DEBUG("[r61350cpt-truly]: read id fail !\n");
	        return 0;
}

LCM_DRIVER r61350_dsi_vdo_truly_cpt_hd720_lcm_drv = 
{
	.name		= "r61350_dsi_vdo_truly_cpt_hd720",
	.set_util_funcs	= lcm_set_util_funcs,
	.get_params	= lcm_get_params,
	.init			= lcm_init,
	.suspend		= lcm_suspend,
	.resume		= lcm_resume,
	.compare_id	= lcm_compare_id,
	.init_power = lcm_init_power,
	.resume_power = lcm_resume_power,
	.suspend_power	= lcm_suspend_power,
#if (LCM_DSI_CMD_MODE)
	.update		= lcm_update,
#endif
};

