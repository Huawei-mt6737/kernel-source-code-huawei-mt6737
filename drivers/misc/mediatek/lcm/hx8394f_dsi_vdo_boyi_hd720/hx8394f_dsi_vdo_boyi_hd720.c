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
	/*
	SET_RESET_PIN(1);
	MDELAY(10);
	SET_RESET_PIN(0);
	MDELAY(10);
	SET_RESET_PIN(1);
	MDELAY(120);
	*/

	data_array[0]=0x00043902;
	data_array[1]=0x9483FFB9;
	dsi_set_cmdq(&data_array,2, 1);

	data_array[0]=0x000B3902;
	data_array[1]=0x721250B1;
	data_array[2]=0x71543209;
	data_array[3]=0x00433051;
	dsi_set_cmdq(&data_array,4, 1);

	data_array[0]=0x00073902;
	data_array[1]=0x680363BA;
	data_array[2]=0x00C0B26B;
	dsi_set_cmdq(&data_array,3, 1);

	data_array[0]=0x00073902;
	data_array[1]=0x648000B2;
	data_array[2]=0x002F0A0E;
	dsi_set_cmdq(&data_array,3, 1);

	data_array[0]=0x00163902;
	data_array[1]=0x1C781CB4;
	data_array[2]=0x01781C78;
	data_array[3]=0x0075860C;
	data_array[4]=0x1C781C3F;
	data_array[5]=0x01781C78;
	data_array[6]=0x0000860C;
	dsi_set_cmdq(&data_array,7, 1);

	data_array[0]=0x00223902;
	data_array[1]=0x000000D3;
	data_array[2]=0x10000000;
	data_array[3]=0x03103210;
	data_array[4]=0x13320300;
	data_array[5]=0x320000C0;
	data_array[6]=0x00000810;
	data_array[7]=0x05050437;
	data_array[8]=0x47050537;
	data_array[9]=0x0000400E;
	dsi_set_cmdq(&data_array,10, 1);

	data_array[0]=0x002D3902;
	data_array[1]=0x181818D5;
	data_array[2]=0x02010018;
	data_array[3]=0x06050403;
	data_array[4]=0x18181807;
	data_array[5]=0x18181818;
	data_array[6]=0x18181818;
	data_array[7]=0x18181818;
	data_array[8]=0x18181818;
	data_array[9]=0x18181818;
	data_array[10]=0x19191918;
	data_array[11]=0x22212019;
	data_array[12]=0x00000023;
	dsi_set_cmdq(&data_array,13, 1);

	data_array[0]=0x002D3902;
	data_array[1]=0x191818D6;
	data_array[2]=0x05060719;
	data_array[3]=0x01020304;
	data_array[4]=0x18181800;
	data_array[5]=0x18181818;
	data_array[6]=0x18181818;
	data_array[7]=0x18181818;
	data_array[8]=0x18181818;
	data_array[9]=0x18181818;
	data_array[10]=0x18191918;
	data_array[11]=0x21222318;
	data_array[12]=0x00000020;
	dsi_set_cmdq(&data_array,13, 1);

	data_array[0]=0x00033902;
	data_array[1]=0x008080B6;
	dsi_set_cmdq(&data_array,2, 1);

	data_array[0]=0x003B3902;
	data_array[1]=0x2D2300E0;
	data_array[2]=0x3A383432;
	data_array[3]=0x8D807138;
	data_array[4]=0xA3A1908B;
	data_array[5]=0xA7AEAFA4;
	data_array[6]=0x5D5FC1B3;
	data_array[7]=0x6F686561;
	data_array[8]=0x23007F7B;
	data_array[9]=0x3834322D;
	data_array[10]=0x8071383A;
	data_array[11]=0xA1908B8D;
	data_array[12]=0xAEAFA4A3;
	data_array[13]=0x5FC1B3A7;
	data_array[14]=0x6865615D;
	data_array[15]=0x007F7B6F;
	dsi_set_cmdq(&data_array,16, 1);

	data_array[0]=0x00023902;
	data_array[1]=0x00000BCC;
	dsi_set_cmdq(&data_array,2, 1);

	data_array[0]=0x00033902;
	data_array[1]=0x00311FC0;
	dsi_set_cmdq(&data_array,2, 1);

	data_array[0]=0x00023902;
	data_array[1]=0x000002D4;
	dsi_set_cmdq(&data_array,2, 1);

	data_array[0]=0x00023902;
	data_array[1]=0x000001BD;
	dsi_set_cmdq(&data_array,2, 1);

	data_array[0]=0x00023902;
	data_array[1]=0x000000B1;
	dsi_set_cmdq(&data_array,2, 1);

	data_array[0]=0x00023902;
	data_array[1]=0x000000BD;
	dsi_set_cmdq(&data_array,2, 1);

	data_array[0]=0x00023902;
	data_array[1]=0x0000EFC6;
	dsi_set_cmdq(&data_array,2, 1);

	data_array[0]=0x00110500;
	dsi_set_cmdq(&data_array,1, 1);
	MDELAY(120);

	data_array[0]=0x00290500;
	dsi_set_cmdq(&data_array,1, 1);
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

	// enable tearing-free
	params->dbi.te_mode 			= LCM_DBI_TE_MODE_VSYNC_ONLY;
	params->dbi.te_edge_polarity		= LCM_POLARITY_RISING;

#if (LCM_DSI_CMD_MODE)
	params->dsi.mode   = CMD_MODE;
#else
    	params->dsi.mode   = BURST_VDO_MODE;
#endif

#if 1
	params->dsi.esd_check_enable = 1;
	params->dsi.customization_esd_check_enable = 1;
	/*
	 * reg 0x09, return 0x80, 0x73, 0x04
	 */
	params->dsi.lcm_esd_check_table[0].cmd = 0x09;
	params->dsi.lcm_esd_check_table[0].count = 3;
	params->dsi.lcm_esd_check_table[0].para_list[0] = 0x80;
	params->dsi.lcm_esd_check_table[0].para_list[1] = 0x73;
	params->dsi.lcm_esd_check_table[0].para_list[2] = 0x06;//04
	params->dsi.lcm_esd_check_table[1].cmd = 0x00;
	/*
	 * reg 0xD9, return 0x80
	 */
	//params->dsi.lcm_esd_check_table[1].cmd = 0xD9;
	//params->dsi.lcm_esd_check_table[1].count = 1;
	//params->dsi.lcm_esd_check_table[1].para_list[0] = 0x80;
	/*
	 * reg 0x45, return 0x0f, 0xf0 || 0x0f, 0xff || 0x05, 0x1d
	 * reg:0x45 has 3 conditions, so set count=2, para_list[6]
	 */
	//params->dsi.lcm_esd_check_table[2].cmd = 0x45;
	//params->dsi.lcm_esd_check_table[2].count = 2;
	//params->dsi.lcm_esd_check_table[2].para_list[0] = 0x0f;
	//params->dsi.lcm_esd_check_table[2].para_list[1] = 0xf0;
	//params->dsi.lcm_esd_check_table[2].para_list[2] = 0x0f;
	//params->dsi.lcm_esd_check_table[2].para_list[3] = 0xff;
	//params->dsi.lcm_esd_check_table[2].para_list[4] = 0x05;
	//params->dsi.lcm_esd_check_table[2].para_list[5] = 0x1d;

#endif


	/* Command mode setting */
	params->dsi.LANE_NUM				= LCM_FOUR_LANE;
	//The following defined the fomat for data coming from LCD engine.
	params->dsi.data_format.color_order = LCM_COLOR_ORDER_RGB;
	params->dsi.data_format.trans_seq   = LCM_DSI_TRANS_SEQ_MSB_FIRST;
	params->dsi.data_format.padding     = LCM_DSI_PADDING_ON_LSB;
	params->dsi.data_format.format      = LCM_DSI_FORMAT_RGB888;

	params->dsi.PS=LCM_PACKED_PS_24BIT_RGB888;

	params->dsi.vertical_sync_active                 = 4;//4;//2;
	params->dsi.vertical_backporch                   = 12;//16;//14;
	params->dsi.vertical_frontporch                  = 15;//16;
	params->dsi.vertical_active_line                 = FRAME_HEIGHT; 

	params->dsi.horizontal_sync_active               = 30;//20;
	params->dsi.horizontal_backporch                 = 60;//40;
	params->dsi.horizontal_frontporch                = 60;//60
	params->dsi.horizontal_active_pixel              = FRAME_WIDTH;

	params->dsi.PLL_CLOCK = 214;//dsi clock customization: should config clock value directly
	params->dsi.ssc_disable = 0;
	params->dsi.ssc_range = 2;
	params->dsi.noncont_clock = 1;
	params->dsi.noncont_clock_period = 1;
	params->dsi.HS_TRAIL = 6;//16
}

static void lcm_init_power(void)
{
	int ret=0;
	unsigned char cmd = 0x0;
	unsigned char data = 0xFF;

	cmd=0x00;
	data=0x0C;//5.2v    0x14: 6v

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
	//set_gpio_lcd_enp(1);
	lcd_bais_enp_enable(1);
#endif
	MDELAY(5);

	ret = lp3101_write_bytes(cmd, data);
	LCD_DEBUG("[hx8394f-boyi]: %s, line%d,  ret=%d\n", __func__, __LINE__, ret);
	MDELAY(2);

	cmd=0x01;
	data=0x0C;//5.2v    0x14: 6v
	
#ifdef BUILD_LK
	mt_set_gpio_mode(GPIO_LP3101_ENN, GPIO_MODE_00);
	mt_set_gpio_dir(GPIO_LP3101_ENN, GPIO_DIR_OUT);
	mt_set_gpio_out(GPIO_LP3101_ENN, GPIO_OUT_ONE);
#else
	//set_gpio_lcd_enn(1);
	lcd_bais_enn_enable(1);
#endif
	MDELAY(5);

	ret = lp3101_write_bytes(cmd, data);
	LCD_DEBUG("[hx8394f-boyi]: %s, line%d,  ret=%d\n", __func__, __LINE__, ret);

	MDELAY(10);
}

static void lcm_suspend_power(void)
{
#ifdef BUILD_LK
	mt_set_gpio_mode(GPIO_LP3101_ENN, GPIO_MODE_00);
	mt_set_gpio_dir(GPIO_LP3101_ENN, GPIO_DIR_OUT);
	mt_set_gpio_out(GPIO_LP3101_ENN, GPIO_OUT_ZERO);
#else
	//set_gpio_lcd_enn(0);
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
	//set_gpio_lcd_enp(0);
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
	LCD_DEBUG("[hx8394f-boyi]: %s, line%d\n", __func__, __LINE__);
}
/*
static struct LCM_setting_table lcm_exit_sleep_mode_setting[] = {
	{0x11,1,{0x00}},   // Sleep out 

	{REGFLAG_DELAY, 120, {}},

	{0x29,1,{0x00}},   // Display on 

	{REGFLAG_DELAY, 20, {}},

	{REGFLAG_END_OF_TABLE, 0x00, {}},	
};

static struct LCM_setting_table lcm_deep_sleep_mode_in_setting[] = {
	{0x28,1,{0x00}},   // Display off 
	
	{REGFLAG_DELAY, 20, {}},
	
	{0x10,1,{0x00}},   // Enter Sleep mode 
	
	{REGFLAG_DELAY, 120, {}},

  	{REGFLAG_END_OF_TABLE, 0x00, {}},
};
*/

static void lcm_suspend(void)
{
	unsigned int data_array[2]; 

	data_array[0] = 0x00280500; // Display Off 
	dsi_set_cmdq(data_array, 1, 1); 
	MDELAY(50);
	data_array[0] = 0x00100500; // Sleep In 
	dsi_set_cmdq(data_array, 1, 1); 
	MDELAY(130); 

	/*
	data_array[0] = 0x00022902;
	data_array[1] = 0x000000b0;
	dsi_set_cmdq(data_array, 2, 1);

	data_array[0] = 0x00022902; //deep standby
	data_array[1] = 0x000001b1;
	dsi_set_cmdq(data_array, 2, 1);
	MDELAY(10);
	*/
	LCD_DEBUG("[hx8394f-boyi]: %s, line%d\n", __func__, __LINE__);
}


static void lcm_resume(void)
{
	lcm_init();
	LCD_DEBUG("[hx8394f-boyi]: %s, line%d\n", __func__, __LINE__);
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
	unsigned int data_array[16];
	unsigned int array[16];  

	SET_RESET_PIN(1);
	MDELAY(5);
	SET_RESET_PIN(0);
	MDELAY(10);
	SET_RESET_PIN(1);
	MDELAY(50);

	data_array[0] = 0x00043902;
	data_array[1] = 0x9483FFB9;
	dsi_set_cmdq(&data_array, 2, 1);
	data_array[0] = 0x00053902;
	data_array[1] = 0x001100B0;
	data_array[2] = 0x00000000;
	dsi_set_cmdq(&data_array, 3, 1);

	array[0] = 0x00013700;
	dsi_set_cmdq(array, 1, 1);

	read_reg_v2(0xDA, buffer, 1);
	id[0] = buffer[0];

	read_reg_v2(0xDB, buffer, 1);
	id[1] = buffer[0];

	//read_reg_v2(0xDC, buffer, 3);//1009
	//id[2] = buffer[0];

	LCD_DEBUG("[hx8394f-boyi]: %s, line%d, id0=%x\n", __func__, __LINE__, id[0]);
	LCD_DEBUG("[hx8394f-boyi]: %s, line%d, id1=%x\n", __func__, __LINE__, id[1]);
	LCD_DEBUG("[hx8394f-boyi]: %s, line%d, id2=%x\n", __func__, __LINE__, id[2]);

	if(id[0] == 0xB1 && id[1] == 0x94)// && id[2] == 0x1C)
		return 1;

	LCD_DEBUG("[hx8394f-boyi]: read id fail !\n");
	return 0;
}

LCM_DRIVER hx8394f_dsi_vdo_boyi_hd720_lcm_drv = 
{
	.name		= "hx8394f_dsi_vdo_boyi_hd720",
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

