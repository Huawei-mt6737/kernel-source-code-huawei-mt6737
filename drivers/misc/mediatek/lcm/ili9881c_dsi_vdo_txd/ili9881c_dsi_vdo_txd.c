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

static void push_table(struct LCM_setting_table *table, unsigned int count, unsigned char force_update)
{
	unsigned int i;
	
	for(i = 0; i < count; i++) 
	{   
		unsigned cmd;
		cmd = table[i].cmd;

		switch (cmd)
		{
			case REGFLAG_DELAY :
				MDELAY(table[i].count);
				break;
			case REGFLAG_END_OF_TABLE :
				break;
			/*case 0xd9 :
				table[i].para_list[0] = vcom;
				vcom +=2;
				dsi_set_cmdq_V2(cmd, table[i].count, table[i].para_list, force_update);
				break;*/
			default:
				dsi_set_cmdq_V2(cmd, table[i].count, table[i].para_list, force_update);
		}
	}
}




static struct LCM_setting_table lcm_initialization_setting[] = {
	{0xFF,3,{0x98,0x81,0x03}},

	{0x01,1,{0x00}},

	{0x02,1,{0x00}},

	{0x03,1,{0x53}},

	{0x04,1,{0x13}},

	{0x05,1,{0x13}},

	{0x06,1,{0x06}},

	{0x07,1,{0x00}},

	{0x08,1,{0x04}},

	{0x09,1,{0x00}}, 

	{0x0a,1,{0x00}}, 

	{0x0b,1,{0x00}}, 

	{0x0C,1,{0x00}},

	{0x0D,1,{0x00}},

	{0x0E,1,{0x00}},

	{0x0f,1,{0x00}}, 

	{0x10,1,{0x00}}, 

	{0x11,1,{0x00}},

	{0x12,1,{0x00}},

	{0x13,1,{0x00}},

	{0x14,1,{0x00}},

	{0x15,1,{0x00}},

	{0x16,1,{0x00}},

	{0x17,1,{0x00}},

	{0x18,1,{0x00}},

	{0x19,1,{0x00}},

	{0x1A,1,{0x00}},

	{0x1B,1,{0x00}},

	{0x1C,1,{0x00}},

	{0x1D,1,{0x00}},

	{0x1E,1,{0xC0}},

	{0x1F,1,{0x80}},

	{0x20,1,{0x04}},

	{0x21,1,{0x0B}},

	{0x22,1,{0x00}},

	{0x23,1,{0x00}},

	{0x24,1,{0x00}},

	{0x25,1,{0x00}},

	{0x26,1,{0x00}},

	{0x27,1,{0x00}},

	{0x28,1,{0x55}},

	{0x29,1,{0x03}},

	{0x2A,1,{0x00}},

	{0x2B,1,{0x00}},

	{0x2C,1,{0x00}},

	{0x2D,1,{0x00}},

	{0x2E,1,{0x00}},

	{0x2F,1,{0x00}},

	{0x30,1,{0x00}},

	{0x31,1,{0x00}},

	{0x32,1,{0x00}},

	{0x33,1,{0x00}},

	{0x34,1,{0x04}},

	{0x35,1,{0x05}},

	{0x36,1,{0x05}},

	{0x37,1,{0x00}},

	{0x38,1,{0x3C}},

	{0x39,1,{0x00}},

	{0x3A,1,{0x40}},

	{0x3B,1,{0x40}},

	{0x3C,1,{0x00}},

	{0x3D,1,{0x00}},

	{0x3E,1,{0x00}},

	{0x3F,1,{0x00}},

	{0x40,1,{0x00}},

	{0x41,1,{0x00}},

	{0x42,1,{0x00}},

	{0x43,1,{0x00}},

	{0x44,1,{0x00}},

	{0x50,1,{0x01}},

	{0x51,1,{0x23}},

	{0x52,1,{0x45}},

	{0x53,1,{0x67}},

	{0x54,1,{0x89}},

	{0x55,1,{0xAB}},

	{0x56,1,{0x01}},

	{0x57,1,{0x23}},

	{0x58,1,{0x45}},

	{0x59,1,{0x67}},

	{0x5A,1,{0x89}},

	{0x5B,1,{0xAB}},

	{0x5C,1,{0xCD}},

	{0x5D,1,{0xEF}},

	{0x5E,1,{0x01}},

	{0x5F,1,{0x14}},

	{0x60,1,{0x15}},

	{0x61,1,{0x0C}},

	{0x62,1,{0x0D}},

	{0x63,1,{0x0E}},

	{0x64,1,{0x0F}},

	{0x65,1,{0x10}},

	{0x66,1,{0x11}},

	{0x67,1,{0x08}},

	{0x68,1,{0x02}},

	{0x69,1,{0x0A}},

	{0x6A,1,{0x02}},

	{0x6B,1,{0x02}},

	{0x6C,1,{0x02}},

	{0x6D,1,{0x02}},

	{0x6E,1,{0x02}},

	{0x6F,1,{0x02}},

	{0x70,1,{0x02}},

	{0x71,1,{0x02}},

	{0x72,1,{0x06}},

	{0x73,1,{0x02}},

	{0x74,1,{0x02}},

	{0x75,1,{0x14}},

	{0x76,1,{0x15}},

	{0x77,1,{0x11}},

	{0x78,1,{0x10}},

	{0x79,1,{0x0F}},

	{0x7A,1,{0x0E}},

	{0x7B,1,{0x0D}},

	{0x7C,1,{0x0C}},

	{0x7D,1,{0x06}},

	{0x7E,1,{0x02}},

	{0x7F,1,{0x0A}},

	{0x80,1,{0x02}},

	{0x81,1,{0x02}},

	{0x82,1,{0x02}},

	{0x83,1,{0x02}},

	{0x84,1,{0x02}},

	{0x85,1,{0x02}},

	{0x86,1,{0x02}},

	{0x87,1,{0x02}},

	{0x88,1,{0x08}},

	{0x89,1,{0x02}},

	{0x8A,1,{0x02}},

	{0xFF,3,{0x98,0x81,0x04}},

	{0x6C,1,{0x15}},

	{0x6E,1,{0x3B}},
	{0xb5,1,{0x06}},
	{0x31,1,{0x75}},


	{0x6F,1,{0x57}},                // reg vcl + pumping ratio VGH=4x VGL=-2.5x
	 
	{0x3A,1,{0xA4}},

	{0x8D,1,{0x15}},

	{0x87,1,{0xBA}},

	{0x26,1,{0x76}},

	{0XB2,1,{0XD1}},

	{0X88,1,{0X0B}},

	{0xFF,3,{0x98,0x81,0x01}},

	{0x22,1,{0x0A}},

	{0x31,1,{0x00}},

	{0x53,1,{0x8A}},

	{0x55,1,{0x88}},

	{0x50,1,{0xA0}},

	{0x51,1,{0xA0}},

	{0x60,1,{0x14}},

	{0xA0,1,{0x08}},

	{0xA1,1,{0x21}},

	{0xA2,1,{0x30}},

	{0xA3,1,{0x0F}},

	{0xA4,1,{0x11}},

	{0xA5,1,{0x27}},

	{0xA6,1,{0x1C}},

	{0xA7,1,{0x1E}},

	{0xA8,1,{0x8C}},

	{0xA9,1,{0x1B}},

	{0xAA,1,{0x28}},

	{0xAB,1,{0x74}},

	{0xAC,1,{0x1A}},

	{0xAD,1,{0x19}},

	{0xAE,1,{0x4D}},

	{0xAF,1,{0x21}},

	{0xB0,1,{0x28}},

	{0xB1,1,{0x4a}},

	{0xB2,1,{0x5b}},

	{0xB3,1,{0x2C}},

	{0xC0,1,{0x08}},

	{0xC1,1,{0x21}},

	{0xC2,1,{0x30}},

	{0xC3,1,{0x0F}},

	{0xC4,1,{0x11}},

	{0xC5,1,{0x27}},

	{0xC6,1,{0x1C}},

	{0xC7,1,{0x1E}},

	{0xC8,1,{0x8C}},

	{0xC9,1,{0x1B}},

	{0xCA,1,{0x28}},

	{0xCB,1,{0x74}},

	{0xCC,1,{0x1A}},

	{0xCD,1,{0x19}},

	{0xCE,1,{0x4D}},

	{0xCF,1,{0x21}},

	{0xD0,1,{0x28}},

	{0xD1,1,{0x4a}},

	{0xD2,1,{0x59}},

	{0xD3,1,{0x2C}},

	{0xFF,3,{0x98,0x81,0x00}},

	{0x35,1,{0x00}},

	{0x11,1,{0x00}},

	{REGFLAG_DELAY, 120, {}},

	{0x29,1,{0x00}},

	{REGFLAG_DELAY, 20, {}},

	{REGFLAG_END_OF_TABLE, 0x00, {}},
};


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


#if (LCM_DSI_CMD_MODE)
	params->dsi.mode   = CMD_MODE;
#else
	params->dsi.mode   = SYNC_PULSE_VDO_MODE; //SYNC_PULSE_VDO_MODE;//BURST_VDO_MODE; 
#endif
	
	// DSI
	/* Command mode setting */
	//1 Three lane or Four lane
		params->dsi.LANE_NUM				= LCM_FOUR_LANE;
	//The following defined the fomat for data coming from LCD engine.
	params->dsi.data_format.format      = LCM_DSI_FORMAT_RGB888;

	// Video mode setting		
	params->dsi.PS=LCM_PACKED_PS_24BIT_RGB888;
		
		params->dsi.vertical_sync_active				= 8;// 3    2
		params->dsi.vertical_backporch					= 18;// 20   1
		params->dsi.vertical_frontporch					= 20; // 1  12
		params->dsi.vertical_active_line				= FRAME_HEIGHT; 

		params->dsi.horizontal_sync_active				= 20;// 50  2
		params->dsi.horizontal_backporch				= 60;//90
		params->dsi.horizontal_frontporch				= 70;//90
		params->dsi.horizontal_active_pixel				= FRAME_WIDTH;
//		params->dsi.ssc_disable                         = 1;
		params->dsi.HS_TRAIL                             = 6;
		/*ui = 1000/(dis_params->PLL_CLOCK*2) + 0x01; 
		cycle_time = 8000/(dis_params->PLL_CLOCK*2) + 0x01;
		HS_TRAIL = (0x04*ui + 0x50)/cycle_time;*/
	//params->dsi.LPX=8; 

	// Bit rate calculation
		//1 Every lane speed
		//params->dsi.pll_select=1;
		//params->dsi.PLL_CLOCK  = LCM_DSI_6589_PLL_CLOCK_377;
		params->dsi.PLL_CLOCK=215;//208//270		
		//params->dsi.noncont_clock=1;
		//params->dsi.noncont_clock_period=2;
		//params->dsi.pll_div1=0;		// div1=0,1,2,3;div1_real=1,2,4,4 ----0: 546Mbps  1:273Mbps
		//params->dsi.pll_div2=0;		// div2=0,1,2,3;div1_real=1,2,4,4	

	//	params->dsi.fbk_div =7;    // fref=26MHz, fvco=fref*(fbk_div+1)*2/(div1_real*div2_real)	
		//params->dsi.compatibility_for_nvk = 1;		// this parameter would be set to 1 if DriverIC is NTK's and when force match DSI clock for NTK's
		params->dsi.esd_check_enable = 1;
		params->dsi.customization_esd_check_enable = 1;
		params->dsi.lcm_esd_check_table[0].cmd			= 0x09;
		params->dsi.lcm_esd_check_table[0].count		= 3;
		params->dsi.lcm_esd_check_table[0].para_list[0] = 0x80;
		params->dsi.lcm_esd_check_table[0].para_list[1] = 0x03;
		params->dsi.lcm_esd_check_table[0].para_list[2] = 0x06;


}

static void lcm_init_power(void)
{
	int ret=0;
	unsigned char cmd = 0x0;
	unsigned char data = 0xFF;

	cmd=0x00;
	data=0x0C;//5.6v    0x14: 6v

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
	LCD_DEBUG("[ili9881c-txd]: %s, line%d,	ret=%d\n", __func__, __LINE__, ret);
	MDELAY(2);

	cmd=0x01;
	data=0x0C;//5.6v    0x14: 6v

#ifdef BUILD_LK
	mt_set_gpio_mode(GPIO_LP3101_ENN, GPIO_MODE_00);
	mt_set_gpio_dir(GPIO_LP3101_ENN, GPIO_DIR_OUT);
	mt_set_gpio_out(GPIO_LP3101_ENN, GPIO_OUT_ONE);
#else
	//	set_gpio_lcd_enn(1);
	lcd_bais_enn_enable(1);
#endif
	MDELAY(5);

	ret = lp3101_write_bytes(cmd, data);
	LCD_DEBUG("[ili9881c-txd]: %s, line%d,	ret=%d\n", __func__, __LINE__, ret);

	MDELAY(10);
}

static void lcm_suspend_power(void)
{
	SET_RESET_PIN(0);
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
	MDELAY(120);  //50

	push_table(lcm_initialization_setting, sizeof(lcm_initialization_setting) / sizeof(struct LCM_setting_table), 1);
	LCD_DEBUG("[ili9881c-txd]: %s, line%d\n", __func__, __LINE__);
}
/*
static struct LCM_setting_table lcm_exit_sleep_mode_setting[] = {
	{0x11,1,{0x00}},   // Sleep out 

	{REGFLAG_DELAY, 120, {}},

	{0x29,1,{0x00}},   // Display on 

	{REGFLAG_DELAY, 20, {}},

	{REGFLAG_END_OF_TABLE, 0x00, {}},	
};
*/
static struct LCM_setting_table lcm_deep_sleep_mode_in_setting[] = {
	{0x28,1,{0x00}},   // Display off 
	
	{REGFLAG_DELAY, 20, {}},
	
	{0x10,1,{0x00}},   // Enter Sleep mode 
	
	{REGFLAG_DELAY, 120, {}},

  	{REGFLAG_END_OF_TABLE, 0x00, {}},
};


static void lcm_suspend(void)
{
	
        push_table(lcm_deep_sleep_mode_in_setting, sizeof(lcm_deep_sleep_mode_in_setting) / sizeof(struct LCM_setting_table), 1);

	LCD_DEBUG("[ili9881c-txd]: %s, line%d\n", __func__, __LINE__);
}


static void lcm_resume(void)
{
	lcm_init();
	LCD_DEBUG("[ili9881c-txd]: %s, line%d\n", __func__, __LINE__);
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
	unsigned char id[3]={0};
	unsigned char buffer[2];
	unsigned int array[16];  

	SET_RESET_PIN(1);
	MDELAY(20);
	SET_RESET_PIN(0);
	MDELAY(20);
	SET_RESET_PIN(1);
	MDELAY(100);

    array[0] = 0x00043902;
	array[1] = 0x018198FF;
	dsi_set_cmdq(array, 2, 1);

	array[0] = 0x00013700;
	dsi_set_cmdq(array, 1, 1);

	MDELAY(10);
	read_reg_v2(0x00, buffer, 1);
	id[0] = buffer[0]; 

	read_reg_v2(0x01, buffer, 1);
	id[1] = buffer[0]; 

	read_reg_v2(0x02, buffer, 1);
	id[2] = buffer[0]; 

	LCD_DEBUG("[ili9881c-txd]: %s, line%d, id0=%x\n", __func__, __LINE__, id[0]);
	LCD_DEBUG("[ili9881c-txd]: %s, line%d, id1=%x\n", __func__, __LINE__, id[1]);
	LCD_DEBUG("[ili9881c-txd]: %s, line%d, id2=%x\n", __func__, __LINE__, id[2]);

	if(id[0] == 0x98 && id[1] == 0x81 && id[2] == 0x0C)
		return 1;

	LCD_DEBUG("[ili9881c-txd]: read id fail !\n");
	return 0;
}

LCM_DRIVER ili9881c_dsi_vdo_txd_lcm_drv = 
{
	.name		= "ili9881c_dsi_vdo_txd",
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

