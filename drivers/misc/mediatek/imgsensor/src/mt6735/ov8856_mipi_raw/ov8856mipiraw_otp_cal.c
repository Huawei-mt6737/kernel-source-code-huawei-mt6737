/*
 * Driver for CAM_CAL
 *
 *
 */
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/slab.h>
#include <linux/fs.h>
#include "kd_camera_hw.h"
#include "cam_cal.h"
#include "cam_cal_define.h"

#include "kd_camera_typedef.h"


#include <linux/dma-mapping.h>
#ifdef CONFIG_COMPAT
/* 64 bit */
#include <linux/fs.h>
#include <linux/compat.h>
#endif
#define OV_OTP_File_Load // eeprom_slim.txt file load. For SW Eng. 

#ifdef OV_OTP_File_Load
  #include <linux/kernel.h>
  #include <linux/init.h> 
  #include <linux/module.h> 
  #include <linux/syscalls.h> 
  #include <linux/fcntl.h> 
  #include <asm/uaccess.h> 
  #include <linux/types.h> 
  #include <linux/slab.h> 
#endif 

#define PFX "OV8856_EEPROM_FMT"
#define CAM_CAL_DEBUG
#ifdef CAM_CAL_DEBUG
	#define CAM_CALINF(fmt, arg...)    pr_debug("[%s] " fmt, __FUNCTION__, ##arg)
	#define CAM_CALDB(fmt, arg...)     pr_debug("[%s] " fmt, __FUNCTION__, ##arg)
	#define CAM_CALERR(fmt, arg...)    pr_err("[%s] " fmt, __FUNCTION__, ##arg)
#else
	#define CAM_CALINF(x,...)
	#define CAM_CALDB(x,...)
	#define CAM_CALERR(fmt, arg...)    pr_err("[%s] " fmt, __FUNCTION__, ##arg)
#endif

static DEFINE_SPINLOCK(g_CAM_CALLock); // for SMP
#define LOG_INF(format, args...)    pr_debug(PFX "[%s] " format, __FUNCTION__, ##args)
#define USHORT             unsigned short
#define BYTE               unsigned char
#define Sleep(ms) mdelay(ms)
/*******************************************************************************
*
********************************************************************************/
#define CAM_CAL_DRVNAME "OV8856MIPIRAW_OTP_CAL"
/*******************************************************************************
*
********************************************************************************/
static dev_t g_CAM_CALdevno = MKDEV(226,0);
static struct cdev * g_pCAM_CAL_CharDrv = NULL;
static struct class *CAM_CAL_class = NULL;
static atomic_t g_CAM_CALatomic;
#define MAX_LSC_SIZE 1868
#define MAX_OTP_SIZE 2200
typedef struct {

#if 0
	u16	   ChipInfo; //chip id, lot Id, Chip No. Etc
	u8     IdGroupWrittenFlag; //"Bit[7:6]: Flag of WB_Group_0  00: empty  01: valid group 11 or 10: invalid group"
	u8     ModuleInfo; //MID, 0x02 for truly
	u8     Year;
	u8     Month;
	u8     Day;
	u8     LensInfo;
	u8     VcmInfo;
	u8     DriverIcInfo;
	u8     LightTemp;
#endif

    u8     flag;
	u32    CaliVer;//0xff000b01
	u16    SerialNum;
	u8     Version;//0x01
	u8     AwbAfInfo;//0xF
	u8     UnitAwbR;
	u8     UnitAwbGr;
	u8     UnitAwbGb;
	u8     UnitAwbB;
	u8     GoldenAwbR;
	u8     GoldenAwbGr;
	u8     GoldenAwbGb;
	u8     GoldenAwbB;
	u16    AfInfinite;
	u16    AfMacro;
	u16    LscSize;
	u8   Lsc[MAX_LSC_SIZE];
}OTP_MTK_TYPE;


typedef union {
        u8 Data[MAX_OTP_SIZE];
        OTP_MTK_TYPE       MtkOtpData;
} OTP_DATA;
unsigned short ov8856sy_af_infinit;
unsigned short ov8856sy_af_macro;


static OTP_DATA ov8856_eeprom_data = {{0}};
extern int iReadRegI2C(u8 *a_pSendData , u16 a_sizeSendData, u8 * a_pRecvData, u16 a_sizeRecvData, u16 i2cId);
extern int iWriteRegI2C(u8 *a_pSendData , u16 a_sizeSendData, u16 i2cId);
extern void kdSetI2CSpeed(u16 i2cSpeed);
static int read_cmos_sensor(u16 slave_id,u32 addr,u8* data)
{
	char pu_send_cmd[2] = {(char)(addr >> 8) , (char)(addr & 0xFF) };
    	return iReadRegI2C(pu_send_cmd, 2, data, 1, slave_id);
}
static int read_ov8856_eeprom(u8 slv_id, u16 offset, u8* data)
{
	int ret = 0;
	ret = read_cmos_sensor(slv_id,offset,data);
	return ret;
}
static int read_ov8856_eeprom_size(u8 slv_id, u16 offset, u8* data,int size)
{
	int i = 0;
	for(i = 0; i < size; i++){
		if(read_ov8856_eeprom(slv_id, offset+i, data+i) != 0)
			return -1;
	}
	return 0;
}
int read_ov8856_module_id(void)
{
	int module_id =0;
	read_ov8856_eeprom_size(0xA0,0x0003,&module_id,1);
	return module_id;
}

 kal_uint16 read_cmos_sensor_af(kal_uint32 addr)
{
	kal_uint16 get_byte=0;

	char pu_send_cmd[2] = {(char)(addr >> 8), (char)(addr & 0xFF) };
	iReadRegI2C(pu_send_cmd, 2, (u8*)&get_byte, 1, 0x42);

	return get_byte;
}

extern  void write_cmos_sensor(kal_uint32 addr, kal_uint32 para);

static void sensor_init(void)
{
	LOG_INF("E\n");
	/* R1A_AM08a, Ken Cui@2015/9/11
	R1A_AM10, Nick_zhang@2015/10/28
	R1A_AM11, Ken Cui@2015/11/09
	R1A_AM11a, Ken_Cui
	;Xclk 24Mhz
	;pclk 144Mhz
	linelength = 1932(0x78C)
	framelength = 2482(0x9b2)
	grabwindow_width  = 1632
	grabwindow_height = 1224
	max_framerate = 300,	
	mipi_data_lp2hs_settle_dc =23?
	mipi_datarate = 720; Mbps
    */
	write_cmos_sensor(0x0103, 0x01);
	write_cmos_sensor(0x0100, 0x00);
	write_cmos_sensor(0x0302, 0x3c);
	write_cmos_sensor(0x0303, 0x01);
	write_cmos_sensor(0x031e, 0x0c);
	write_cmos_sensor(0x3000, 0x00);
	write_cmos_sensor(0x300e, 0x00);
	write_cmos_sensor(0x3010, 0x00);
	write_cmos_sensor(0x3015, 0x84);
	write_cmos_sensor(0x3018, 0x72);
	write_cmos_sensor(0x3033, 0x24);
	write_cmos_sensor(0x3500, 0x00);
	write_cmos_sensor(0x3501, 0x4c);
	write_cmos_sensor(0x3502, 0xe0);
	write_cmos_sensor(0x3503, 0x08);
	write_cmos_sensor(0x3505, 0x83);
	write_cmos_sensor(0x3508, 0x01);
	write_cmos_sensor(0x3509, 0x80);
	write_cmos_sensor(0x350c, 0x00);
	write_cmos_sensor(0x350d, 0x80);
	write_cmos_sensor(0x350e, 0x04);
	write_cmos_sensor(0x350f, 0x00);
	write_cmos_sensor(0x3510, 0x00);
	write_cmos_sensor(0x3511, 0x02);
	write_cmos_sensor(0x3512, 0x00);
	write_cmos_sensor(0x3600, 0x72);
	write_cmos_sensor(0x3601, 0x40);
	write_cmos_sensor(0x3602, 0x30);
	write_cmos_sensor(0x3610, 0xc5);
	write_cmos_sensor(0x3611, 0x58);
	write_cmos_sensor(0x3612, 0x5c);
	write_cmos_sensor(0x3613, 0x5a);
	write_cmos_sensor(0x3614, 0x60);
	write_cmos_sensor(0x3628, 0xff);
	write_cmos_sensor(0x3629, 0xff);
	write_cmos_sensor(0x362a, 0xff);
	write_cmos_sensor(0x3633, 0x10);
	write_cmos_sensor(0x3634, 0x10);
	write_cmos_sensor(0x3635, 0x10);
	write_cmos_sensor(0x3636, 0x10);
	write_cmos_sensor(0x3663, 0x08);
	write_cmos_sensor(0x3669, 0x34);
	write_cmos_sensor(0x366e, 0x08);
	write_cmos_sensor(0x3706, 0x86);
	write_cmos_sensor(0x370b, 0x7e);
	write_cmos_sensor(0x3714, 0x27);
	write_cmos_sensor(0x3730, 0x12);
	write_cmos_sensor(0x3733, 0x10);
	write_cmos_sensor(0x3764, 0x00);
	write_cmos_sensor(0x3765, 0x00);
	write_cmos_sensor(0x3769, 0x62);
	write_cmos_sensor(0x376a, 0x2a);
	write_cmos_sensor(0x376b, 0x30);
	write_cmos_sensor(0x3780, 0x00);
	write_cmos_sensor(0x3781, 0x24);
	write_cmos_sensor(0x3782, 0x00);
	write_cmos_sensor(0x3783, 0x23);
	write_cmos_sensor(0x3798, 0x2f);
	write_cmos_sensor(0x37a1, 0x60);
	write_cmos_sensor(0x37a8, 0x6a);
	write_cmos_sensor(0x37ab, 0x3f);
	write_cmos_sensor(0x37c2, 0x14);
	write_cmos_sensor(0x37c3, 0xf1);
	write_cmos_sensor(0x37c9, 0x80);
	write_cmos_sensor(0x37cb, 0x03);
	write_cmos_sensor(0x37cc, 0x0a);
	write_cmos_sensor(0x37cd, 0x16);
	write_cmos_sensor(0x37ce, 0x1f);
	write_cmos_sensor(0x3800, 0x00);
	write_cmos_sensor(0x3801, 0x00);
	write_cmos_sensor(0x3802, 0x00);
	write_cmos_sensor(0x3803, 0x0c);
	write_cmos_sensor(0x3804, 0x0c);
	write_cmos_sensor(0x3805, 0xdf);
	write_cmos_sensor(0x3806, 0x09);
	write_cmos_sensor(0x3807, 0xa3);
	write_cmos_sensor(0x3808, 0x06);
	write_cmos_sensor(0x3809, 0x60);
	write_cmos_sensor(0x380a, 0x04);
	write_cmos_sensor(0x380b, 0xc8);
	write_cmos_sensor(0x380c, 0x07);
	write_cmos_sensor(0x380d, 0x8c);
	write_cmos_sensor(0x380e, 0x09);//04
	write_cmos_sensor(0x380f, 0xb2);//de
	write_cmos_sensor(0x3810, 0x00);
	write_cmos_sensor(0x3811, 0x08);
	write_cmos_sensor(0x3812, 0x00);
	write_cmos_sensor(0x3813, 0x02);
	write_cmos_sensor(0x3814, 0x03);
	write_cmos_sensor(0x3815, 0x01);
	write_cmos_sensor(0x3816, 0x00);
	write_cmos_sensor(0x3817, 0x00);
	write_cmos_sensor(0x3818, 0x00);
	write_cmos_sensor(0x3819, 0x00);
	write_cmos_sensor(0x3820, 0x90);
	write_cmos_sensor(0x3821, 0x67);
	write_cmos_sensor(0x382a, 0x03);
	write_cmos_sensor(0x382b, 0x01);
	write_cmos_sensor(0x3830, 0x06);
	write_cmos_sensor(0x3836, 0x02);
	write_cmos_sensor(0x3862, 0x04);
	write_cmos_sensor(0x3863, 0x08);
	write_cmos_sensor(0x3cc0, 0x33);
	write_cmos_sensor(0x3d85, 0x17);
	write_cmos_sensor(0x3d8c, 0x73);
	write_cmos_sensor(0x3d8d, 0xde);
	write_cmos_sensor(0x4001, 0xe0);
	write_cmos_sensor(0x4003, 0x40);
	write_cmos_sensor(0x4008, 0x00);
	write_cmos_sensor(0x4009, 0x05);
	write_cmos_sensor(0x400f, 0x80);
	write_cmos_sensor(0x4010, 0xf0);
	write_cmos_sensor(0x4011, 0xff);
	write_cmos_sensor(0x4012, 0x02);
	write_cmos_sensor(0x4013, 0x01);
	write_cmos_sensor(0x4014, 0x01);
	write_cmos_sensor(0x4015, 0x01);
	write_cmos_sensor(0x4042, 0x00);
	write_cmos_sensor(0x4043, 0x80);
	write_cmos_sensor(0x4044, 0x00);
	write_cmos_sensor(0x4045, 0x80);
	write_cmos_sensor(0x4046, 0x00);
	write_cmos_sensor(0x4047, 0x80);
	write_cmos_sensor(0x4048, 0x00);
	write_cmos_sensor(0x4049, 0x80);
	write_cmos_sensor(0x4041, 0x03);
	write_cmos_sensor(0x404c, 0x20);
	write_cmos_sensor(0x404d, 0x00);
	write_cmos_sensor(0x404e, 0x20);
	write_cmos_sensor(0x4203, 0x80);
	write_cmos_sensor(0x4307, 0x30);
	write_cmos_sensor(0x4317, 0x00);
	write_cmos_sensor(0x4503, 0x08);
	write_cmos_sensor(0x4601, 0x80);
	write_cmos_sensor(0x4816, 0x53);
	write_cmos_sensor(0x481b, 0x58);
	write_cmos_sensor(0x481f, 0x27);
	write_cmos_sensor(0x4837, 0x16);
	write_cmos_sensor(0x5000, 0x77);
	write_cmos_sensor(0x5001, 0x0a);
	write_cmos_sensor(0x5004, 0x04);
	write_cmos_sensor(0x502e, 0x03);
	write_cmos_sensor(0x5030, 0x41);
	write_cmos_sensor(0x5795, 0x00);
	write_cmos_sensor(0x5796, 0x10);
	write_cmos_sensor(0x5797, 0x10);
	write_cmos_sensor(0x5798, 0x73);
	write_cmos_sensor(0x5799, 0x73);
	write_cmos_sensor(0x579a, 0x00);
	write_cmos_sensor(0x579b, 0x28);
	write_cmos_sensor(0x579c, 0x00);
	write_cmos_sensor(0x579d, 0x16);
	write_cmos_sensor(0x579e, 0x06);
	write_cmos_sensor(0x579f, 0x20);
	write_cmos_sensor(0x57a0, 0x04);
	write_cmos_sensor(0x57a1, 0xa0);
	//;DPC setting
	write_cmos_sensor(0x5780, 0x14);
	write_cmos_sensor(0x5781, 0x0f);
	write_cmos_sensor(0x5782, 0x44);
	write_cmos_sensor(0x5783, 0x02);
	write_cmos_sensor(0x5784, 0x01);
	write_cmos_sensor(0x5785, 0x01);
	write_cmos_sensor(0x5786, 0x00);
	write_cmos_sensor(0x5787, 0x04);
	write_cmos_sensor(0x5788, 0x02);
	write_cmos_sensor(0x5789, 0x0f);
	write_cmos_sensor(0x578a, 0xfd);
	write_cmos_sensor(0x578b, 0xf5);
	write_cmos_sensor(0x578c, 0xf5);
	write_cmos_sensor(0x578d, 0x03);
	write_cmos_sensor(0x578e, 0x08);
	write_cmos_sensor(0x578f, 0x0c);
	write_cmos_sensor(0x5790, 0x08);
	write_cmos_sensor(0x5791, 0x04);
	write_cmos_sensor(0x5792, 0x00);
	write_cmos_sensor(0x5793, 0x52);
	write_cmos_sensor(0x5794, 0xa3);
	write_cmos_sensor(0x59f8, 0x3d);//AM11a
	//;
	write_cmos_sensor(0x5a08, 0x02);
	write_cmos_sensor(0x5b00, 0x02);
	write_cmos_sensor(0x5b01, 0x10);
	write_cmos_sensor(0x5b02, 0x03);
	write_cmos_sensor(0x5b03, 0xcf);
	write_cmos_sensor(0x5b05, 0x6c);
	write_cmos_sensor(0x5e00, 0x00);
	write_cmos_sensor(0x0100, 0x01);

}	/*	sensor_init  */
//#ifdef OV_OTP_File_Load
//static int EEPROM_Slim_File_Load(void) 
//{ 

//        CAM_CALDB("HQ_OV_OTP] ==================================\n");
//        ov8856_eeprom_data.Data[0]= 0x01; 
//	ov8856_eeprom_data.Data[1]= 0xFF; 
//	ov8856_eeprom_data.Data[2]= 0x00; 
//	ov8856_eeprom_data.Data[3]= 0x0B; 
//	ov8856_eeprom_data.Data[4]= 0x01; 
//return 1;
//} 
//#endif 

//lujun add af
int read_ov8856_AF_info(void)
{
	sensor_init();
     int otp_flag, addr, temp,temp2,i,af_checksum=0;
       u8 af_data_info[8]= {0};	
       u8 af_data	;   
     	 write_cmos_sensor(0x0100,0x01);
	mdelay(10);
   //set 0x5001[3] to \A1\B00\A1\B1
   int temp1;
   temp1 = read_cmos_sensor_af(0x5001);    
   
   write_cmos_sensor(0x5001, (0x00 & 0x08) | (temp1 & (~0x08)));
   // read OTP into buffer
   write_cmos_sensor(0x3d84, 0xC0);
   write_cmos_sensor(0x3d88, 0x70); // OTP start address
   write_cmos_sensor(0x3d89, 0x10);
   write_cmos_sensor(0x3d8A, 0x72); // OTP end address
   write_cmos_sensor(0x3d8B, 0x0E);
   write_cmos_sensor(0x3d81, 0x01); // load otp into buffer
   mdelay(10);
   
   // OTP VCM Calibration
   otp_flag = read_cmos_sensor_af(0x7021);
   
   LOG_INF("ov8856_otp_read_test  otp_flag_2: 0x%x,\n", otp_flag); 
   addr = 0;
   if((otp_flag & 0xc0) == 0x40) 
   {
     addr = 0x7022; // base address of VCM Calibration group 1
   }
   else if((otp_flag & 0x30) == 0x10) 
   {
     addr = 0x7025; // base address of VCM Calibration group 2
   }
   if(addr != 0) 
   {
   for(i=0;i<3;i++)
   {af_data_info[i]= read_cmos_sensor_af(addr+i);
   af_data=af_data_info[i];
             LOG_INF("lujun_ov8856_otp_read_test  af_data_info: 0x%x,\n",  af_data); 
   }
      for(i=0x7010;i<=0x720e;i++) 
   {
     write_cmos_sensor(i,0); // clear OTP buffer, recommended use continuous write to accelarate
   }
	   temp2 = read_cmos_sensor_af(0x5001);
   write_cmos_sensor(0x5001, (0x08 & 0x08) | (temp2 & (~0x08)));   
  ov8856_eeprom_data.Data[0]= 0x01; 
  ov8856_eeprom_data.Data[1]= 0xFF; 
  ov8856_eeprom_data.Data[2]= 0x00; 
  ov8856_eeprom_data.Data[3]= 0x0B; 
  ov8856_eeprom_data.Data[4]= 0x02; 
   ov8856_eeprom_data.Data[17]=(af_data_info[1]<<2)+((af_data_info[2]>>4)&0x03);
  ov8856_eeprom_data.Data[18]=((((af_data_info[1]<<2)+((af_data_info[2]>>4)&0x03))>>8)&0x03);
  ov8856_eeprom_data.Data[19]=(af_data_info[0]<<2)+((af_data_info[2]>>6)&0x03);
  ov8856_eeprom_data.Data[20]=((((af_data_info[0]<<2)+((af_data_info[2]>>6)&0x03))>>8)&0x03);
       u32    AFinf=0;
       u32    AFmacro=0;
	AFinf=(ov8856_eeprom_data.Data[18]<<8)+ov8856_eeprom_data.Data[17];
CAM_CALDB("read_ov8856_AF_Info 0 AFinf=%d -------> \n",AFinf);
       AFmacro=(ov8856_eeprom_data.Data[20]<<8)+ov8856_eeprom_data.Data[19];
CAM_CALDB("read_ov8856_AF_Info 0 AFmacro=%d -------> \n",AFmacro);
/*    if(260<=AFinf)
{
	AFinf=AFinf-50;
	CAM_CALDB("read_ov8856_AF_Info  260<=AFinf -------> \n");
}
else if (230<=AFinf&&260>AFinf)
{
      AFinf=AFinf-40;
      CAM_CALDB("read_ov8856_AF_Info  240<=AFinf&&260>=AFinf -------> \n");
}
/*else if(150<=AFinf&&230>AFinf)
{
      AFinf=AFinf-15;
	CAM_CALDB("read_Hi843b_AF_Info  230>AFinf -------> \n");
}
else //if(150>AFinf)
{
      AFinf=AFinf-25;
	CAM_CALDB("read_ov8856_AF_Info  230>AFinf -------> \n");
} */
 AFinf=AFinf-50;
 AFmacro=AFmacro+50;
CAM_CALDB("read_ov8856_AF_Info  1 AFinf=%d -------> \n",AFinf);
        ov8856_eeprom_data.Data[17]=AFinf&0xff;
	ov8856_eeprom_data.Data[18]=AFinf>>8&0x0f;
CAM_CALDB("read_ov8856_AF_Info  1 AFmacro=%d -------> \n",AFmacro);
        ov8856_eeprom_data.Data[19]=AFmacro&0xff;
	ov8856_eeprom_data.Data[20]=AFmacro>>8&0x0f;
   LOG_INF("lujun_ov8856_otp_read_test  data[17]: 0x%x,\n",  ov8856_eeprom_data.Data[17]); 
   LOG_INF("lujun_ov8856_otp_read_test  data[18]: 0x%x,\n", ov8856_eeprom_data.Data[18]); 
   LOG_INF("lujun_ov8856_otp_read_test  data[19]: 0x%x,\n", ov8856_eeprom_data.Data[19]); 
   LOG_INF("lujun_ov8856_otp_read_test  data[20]: 0x%x,\n", ov8856_eeprom_data.Data[20]); 
   }	
   return 0;
}
#if 1
int read_ov8856_eeprom_data(void){
	struct timeval ktv1, ktv2;
	unsigned long TimeIntervalUS;
	int i = 0;
	int offset = 0;			
	do_gettimeofday(&ktv1);
//	if(EEPROM_Slim_File_Load())
	
	read_ov8856_AF_info();
do_gettimeofday(&ktv2);
	if(ktv2.tv_sec > ktv1.tv_sec)
                TimeIntervalUS = ktv1.tv_usec + 1000000 - ktv2.tv_usec;
        else
                TimeIntervalUS = ktv2.tv_usec - ktv1.tv_usec;
        printk("lujun>>ov8856>read data take %lu us\n", TimeIntervalUS);
	return 1;
}

#endif
#ifdef CONFIG_COMPAT
static int compat_put_cal_info_struct(
            COMPAT_stCAM_CAL_INFO_STRUCT __user *data32,
            stCAM_CAL_INFO_STRUCT __user *data)
{
    compat_uptr_t p;
    compat_uint_t i;
    int err;

    err = get_user(i, &data->u4Offset);
    err |= put_user(i, &data32->u4Offset);
    err |= get_user(i, &data->u4Length);
    err |= put_user(i, &data32->u4Length);
    /* Assume pointer is not change */
#if 1
    err |= get_user(p, &data->pu1Params);
    err |= put_user(p, &data32->pu1Params);
#endif
    return err;
}
static int compat_get_cal_info_struct(
            COMPAT_stCAM_CAL_INFO_STRUCT __user *data32,
            stCAM_CAL_INFO_STRUCT __user *data)
{
    compat_uptr_t p;
    compat_uint_t i;
    int err;

    err = get_user(i, &data32->u4Offset);
    err |= put_user(i, &data->u4Offset);
    err |= get_user(i, &data32->u4Length);
    err |= put_user(i, &data->u4Length);
    err |= get_user(p, &data32->pu1Params);
    err |= put_user(compat_ptr(p), &data->pu1Params);

    return err;
}

static long ov8856_Ioctl_Compat(struct file *filp, unsigned int cmd, unsigned long arg)
{
    long ret;
    COMPAT_stCAM_CAL_INFO_STRUCT __user *data32;
    stCAM_CAL_INFO_STRUCT __user *data;
    int err;
	CAM_CALDB("[CAMERA SENSOR] ov8856_eeprom_DEVICE_ID,%p %p %x ioc size %d\n",filp->f_op ,filp->f_op->unlocked_ioctl,cmd,_IOC_SIZE(cmd) );
    if (!filp->f_op || !filp->f_op->unlocked_ioctl)
        return -ENOTTY;

    switch (cmd) {

    case COMPAT_CAM_CALIOC_G_READ:
    {
        data32 = compat_ptr(arg);
        data = compat_alloc_user_space(sizeof(*data));
        if (data == NULL)
            return -EFAULT;

        err = compat_get_cal_info_struct(data32, data);
        if (err)
            return err;

        ret = filp->f_op->unlocked_ioctl(filp, CAM_CALIOC_G_READ,(unsigned long)data);
        err = compat_put_cal_info_struct(data32, data);


        if(err != 0)
            CAM_CALERR("[CAMERA SENSOR] compat_put_acdk_sensor_getinfo_struct failed\n");
        return ret;
    }
    default:
        return -ENOIOCTLCMD;
    }
}
#endif
static int selective_read_region(u32 offset, BYTE* data,u32 size)
{
	
	printk("lujun>>ov8856>selective_read_region offset = %d,size = %d\n", offset,size);
         memcpy((void *)data,(void *)&ov8856_eeprom_data.Data[offset],size);
     	 return size;
}

/*******************************************************************************
*
********************************************************************************/
#define NEW_UNLOCK_IOCTL
#ifndef NEW_UNLOCK_IOCTL
	static int CAM_CAL_Ioctl(struct inode * a_pstInode,
	struct file * a_pstFile,
	unsigned int a_u4Command,
	unsigned long a_u4Param)
#else
	static long CAM_CAL_Ioctl(
		struct file *file,
		unsigned int a_u4Command,
		unsigned long a_u4Param
	)
#endif
	{
		int i4RetValue = 0;
		u8 * pBuff = NULL;
		u8 * pu1Params = NULL;
		stCAM_CAL_INFO_STRUCT *ptempbuf;
#ifdef CAM_CALGETDLT_DEBUG
		struct timeval ktv1, ktv2;
		unsigned long TimeIntervalUS;
#endif
	
		CAM_CALDB(" CAM_CAL_Ioctl\n" );
	
		if(_IOC_NONE == _IOC_DIR(a_u4Command))
		{
		}
		else
		{
			pBuff = (u8 *)kmalloc(sizeof(stCAM_CAL_INFO_STRUCT),GFP_KERNEL);
	
			if(NULL == pBuff)
			{
				CAM_CALERR("ioctl allocate mem failed\n");
				return -ENOMEM;
			}
	
			if(_IOC_WRITE & _IOC_DIR(a_u4Command))
			{
				if(copy_from_user((u8 *) pBuff , (u8 *) a_u4Param, sizeof(stCAM_CAL_INFO_STRUCT)))
				{	 //get input structure address
					kfree(pBuff);
					CAM_CALERR("ioctl copy from user failed\n");
					return -EFAULT;
				}
			}
		}
	
		ptempbuf = (stCAM_CAL_INFO_STRUCT *)pBuff;
		pu1Params = (u8*)kmalloc(ptempbuf->u4Length,GFP_KERNEL);
		if(NULL == pu1Params)
		{
			kfree(pBuff);
			CAM_CALERR("ioctl allocate mem failed\n");
			return -ENOMEM;
		}
	
	
		if(copy_from_user((u8*)pu1Params ,	(u8*)ptempbuf->pu1Params, ptempbuf->u4Length))
		{
			kfree(pBuff);
			kfree(pu1Params);
			CAM_CALERR("  ioctl copy from user failed\n");
			return -EFAULT;
		}
	
		switch(a_u4Command)
		{
			case CAM_CALIOC_S_WRITE:
				
#ifdef CAM_CALGETDLT_DEBUG
				do_gettimeofday(&ktv1);
#endif
				i4RetValue = 0;//iWriteData((u16)ptempbuf->u4Offset, ptempbuf->u4Length, pu1Params);
#ifdef CAM_CALGETDLT_DEBUG
				do_gettimeofday(&ktv2);
				if(ktv2.tv_sec > ktv1.tv_sec)
				{
					TimeIntervalUS = ktv1.tv_usec + 1000000 - ktv2.tv_usec;
				}
				else
				{
					TimeIntervalUS = ktv2.tv_usec - ktv1.tv_usec;
				}
#endif
				break;
			case CAM_CALIOC_G_READ:
				CAM_CALDB("[HYNIX_CAM_CAL] Read CMD \n");
#ifdef CAM_CALGETDLT_DEBUG
				do_gettimeofday(&ktv1);
#endif
				i4RetValue = selective_read_region(ptempbuf->u4Offset, pu1Params, ptempbuf->u4Length);
	
#ifdef CAM_CALGETDLT_DEBUG
				do_gettimeofday(&ktv2);
				if(ktv2.tv_sec > ktv1.tv_sec)
				{
					TimeIntervalUS = ktv1.tv_usec + 1000000 - ktv2.tv_usec;
				}
				else
				{
					TimeIntervalUS = ktv2.tv_usec - ktv1.tv_usec;
				}
				CAM_CALDB("[HYNIX_CAM_CAL] Read data %d bytes take %lu us\n",ptempbuf->u4Length, TimeIntervalUS);
#endif
	
				break;
			default :
				 CAM_CALINF("[CAM_CAL] No CMD \n");
				i4RetValue = -EPERM;
			break;
		}
	
		if(_IOC_READ & _IOC_DIR(a_u4Command))
		{
			//copy data to user space buffer, keep other input paremeter unchange.
			if(copy_to_user((u8 __user *) ptempbuf->pu1Params , (u8 *)pu1Params , ptempbuf->u4Length))
			{
				kfree(pBuff);
				kfree(pu1Params);
				CAM_CALERR("[CAM_CAL] ioctl copy to user failed\n");
				return -EFAULT;
			}
		}
	
		kfree(pBuff);
		kfree(pu1Params);
		return i4RetValue;
	}
	
	
	static u32 g_u4Opened = 0;
	//#define
	//Main jobs:
	// 1.check for device-specified errors, device not ready.
	// 2.Initialize the device if it is opened for the first time.
	static int CAM_CAL_Open(struct inode * a_pstInode, struct file * a_pstFile)
	{
		CAM_CALDB(" CAM_CAL_Open\n");
		spin_lock(&g_CAM_CALLock);
		if(g_u4Opened)
		{
			spin_unlock(&g_CAM_CALLock);
			CAM_CALERR(" Opened, return -EBUSY\n");
			return -EBUSY;
		}
		else
		{
			g_u4Opened = 1;
			atomic_set(&g_CAM_CALatomic,0);
		}
		spin_unlock(&g_CAM_CALLock);
		return 0;
	}
	
	//Main jobs:
	// 1.Deallocate anything that "open" allocated in private_data.
	// 2.Shut down the device on last close.
	// 3.Only called once on last time.
	// Q1 : Try release multiple times.
	static int CAM_CAL_Release(struct inode * a_pstInode, struct file * a_pstFile)
	{
		spin_lock(&g_CAM_CALLock);
	
		g_u4Opened = 0;
	
		atomic_set(&g_CAM_CALatomic,0);
	
		spin_unlock(&g_CAM_CALLock);
	
		return 0;
	}
	
	static const struct file_operations g_stCAM_CAL_fops =
	{
		.owner = THIS_MODULE,
		.open = CAM_CAL_Open,
		.release = CAM_CAL_Release,
		//.ioctl = CAM_CAL_Ioctl
#ifdef CONFIG_COMPAT
		.compat_ioctl = ov8856_Ioctl_Compat,
#endif
		.unlocked_ioctl = CAM_CAL_Ioctl
	};
	
#define CAM_CAL_DYNAMIC_ALLOCATE_DEVNO 1
	
	inline static int RegisterCAM_CALCharDrv(void)
	{
		struct device* CAM_CAL_device = NULL;
#if CAM_CAL_DYNAMIC_ALLOCATE_DEVNO
		if( alloc_chrdev_region(&g_CAM_CALdevno, 0, 1,CAM_CAL_DRVNAME) )
		{
			CAM_CALERR(" Allocate device no failed\n");
	
			return -EAGAIN;
		}
#else
		if( register_chrdev_region(  g_CAM_CALdevno , 1 , CAM_CAL_DRVNAME) )
		{
			CAM_CALERR(" HYNIX_CAM_CAL Register device no failed\n");
	
			return -EAGAIN;
		}
#endif
	
		//Allocate driver
		g_pCAM_CAL_CharDrv = cdev_alloc();
	
		if(NULL == g_pCAM_CAL_CharDrv)
		{
			unregister_chrdev_region(g_CAM_CALdevno, 1);
	
			CAM_CALERR("  Allocate mem for kobject failed\n");
	
			return -ENOMEM;
		}
	
		//Attatch file operation.
		cdev_init(g_pCAM_CAL_CharDrv, &g_stCAM_CAL_fops);
	
		g_pCAM_CAL_CharDrv->owner = THIS_MODULE;
	
		//Add to system
		if(cdev_add(g_pCAM_CAL_CharDrv, g_CAM_CALdevno, 1))
		{
			CAM_CALERR("  Attatch file operation failed\n");
	
			unregister_chrdev_region(g_CAM_CALdevno, 1);
	
			return -EAGAIN;
		}
	
    CAM_CAL_class = class_create(THIS_MODULE, CAM_CAL_DRVNAME);
		if (IS_ERR(CAM_CAL_class)) {
			int ret = PTR_ERR(CAM_CAL_class);
			CAM_CALERR(" Unable to create class, err = %d\n", ret);
			return ret;
		}
		CAM_CAL_device = device_create(CAM_CAL_class, NULL, g_CAM_CALdevno, NULL, CAM_CAL_DRVNAME);
		CAM_CALDB("RegisterCAM_CALCharDrv PASSS\n");
	
		return 0;
	}
	
	inline static void UnregisterCAM_CALCharDrv(void)
	{
		//Release char driver
		cdev_del(g_pCAM_CAL_CharDrv);
	
		unregister_chrdev_region(g_CAM_CALdevno, 1);
	
		device_destroy(CAM_CAL_class, g_CAM_CALdevno);
		class_destroy(CAM_CAL_class);
	}
	
	static int CAM_CAL_probe(struct platform_device *pdev)
	{
	
		return 0;//i2c_add_driver(&CAM_CAL_i2c_driver);
	}
	
	static int CAM_CAL_remove(struct platform_device *pdev)
	{
		//i2c_del_driver(&CAM_CAL_i2c_driver);
		return 0;
	}
	
	// platform structure
	static struct platform_driver g_stCAM_CAL_Driver = {
		.probe		= CAM_CAL_probe,
		.remove = CAM_CAL_remove,
		.driver 	= {
			.name	= CAM_CAL_DRVNAME,
			.owner	= THIS_MODULE,
		}
	};
	
	
	static struct platform_device g_stCAM_CAL_Device = {
		.name = CAM_CAL_DRVNAME,
		.id = 0,
		.dev = {
		}
	};
	
	static int __init CAM_CAL_init(void)
	{
		int i4RetValue = 0;
		CAM_CALDB("HYNIX_CAM_CAL_i2C_init\n");
	   //Register char driver
		i4RetValue = RegisterCAM_CALCharDrv();
		if(i4RetValue){
		   CAM_CALDB(" register char device failed!\n");
		   return i4RetValue;
		}
		CAM_CALDB(" HYNIX_CAM_CAL Attached!! \n");
	
	  //  i2c_register_board_info(CAM_CAL_I2C_BUSNUM, &kd_cam_cal_dev, 1);
		if(platform_driver_register(&g_stCAM_CAL_Driver)){
			CAM_CALERR("HYNIX_CAM_CAL failed to register 842otp driver\n");
			return -ENODEV;
		}
	
		if (platform_device_register(&g_stCAM_CAL_Device))
		{
			CAM_CALERR("HYNIX_CAM_CAL failed to register 842otp driver, 2nd time\n");
			return -ENODEV;
		}
		CAM_CALDB("HYNIX_CAM_CAL_i2C_init PASSS\n");
	
		return 0;
	}
	
	static void __exit CAM_CAL_exit(void)
	{
		platform_driver_unregister(&g_stCAM_CAL_Driver);
	}
	
	module_init(CAM_CAL_init);
	module_exit(CAM_CAL_exit);
	
	MODULE_DESCRIPTION("CAM_CAL driver");
	MODULE_AUTHOR("Sean Lin <Sean.Lin@Mediatek.com>");
	MODULE_LICENSE("GPL");
	
	

