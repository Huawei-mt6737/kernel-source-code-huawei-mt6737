#include <linux/videodev2.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <asm/atomic.h>
//#include <linux/xlog.h>
//#include <asm/system.h>

#include <linux/proc_fs.h>
#include "kd_camera_typedef.h"

#include <linux/dma-mapping.h>

#include "cam_cal.h"
#include "cam_cal_define.h"


#include "kd_camera_hw.h"
#include "kd_imgsensor.h"
#include "kd_imgsensor_define.h"
#include "kd_imgsensor_errcode.h"


#include "ov13850mipiraw_Sensor.h"
#include "ov13850_otp.h"

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

#define PFX "OV13850_EEPROM_FMT"

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



#define CAM_CAL_DRVNAME "OV13850MIPIRAW_OTP_CAL"
/*******************************************************************************
 *
 ********************************************************************************/
static dev_t g_CAM_CALdevno = MKDEV(236,0); //to check
static struct cdev * g_pCAM_CAL_CharDrv = NULL;
static struct class *CAM_CAL_class = NULL;
static atomic_t g_CAM_CALatomic;
#define MAX_LSC_SIZE 1868 //to check
#define MAX_OTP_SIZE 2200 //to check
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
OTP_MTK_TYPE vision;


typedef union {
	u8 Data[MAX_OTP_SIZE];
	OTP_MTK_TYPE       MtkOtpData;
} OTP_DATA;

unsigned short ov13850sy_af_infinit;
unsigned short ov13850sy_af_macro;

static OTP_DATA ov13850_eeprom_data = {{0}};




//
extern int iReadReg(u16 a_u2Addr , u8 * a_puBuff , u16 i2cId);
extern int iWriteReg(u16 a_u2Addr , u32 a_u4Data , u32 a_u4Bytes , u16 i2cId);

//#define OV13850_R2A_write_i2c(addr, para) iWriteReg((u16) addr , (u32) para , 1, OV13850MIPI_WRITE_ID)
#define PFX "OV13850_R2A_OTP"
#define LOG_INF(format, args...)	//pr_debug(ANDROID_LOG_INFO   , PFX, "[%s] " format, __FUNCTION__, ##args)


#define Delay(ms)  mdelay(ms)
static unsigned char OV13850MIPI_WRITE_ID = 0x20;


kal_uint16 OV13850_R2A_read_i2c(kal_uint32 addr)
{
	kal_uint16 get_byte=0;
	iReadReg((u16) addr ,(u8*)&get_byte,OV13850MIPI_WRITE_ID);
	return get_byte;
}

kal_uint16 OV13850_R2A_write_i2c(kal_uint16 addr, kal_uint32 para)
{
	iWriteReg(addr, para , 1, OV13850MIPI_WRITE_ID);
	return 1;
}


void otp_cali(unsigned char writeid)
{
	struct otp_struct current_otp;
	OV13850MIPI_WRITE_ID = writeid;
	memset(&current_otp, 0, sizeof(struct otp_struct));
	read_otp(&current_otp);
	apply_otp(&current_otp);
}

int Decode_13850R2A(unsigned char*pInBuf, unsigned char* pOutBuf)
{
	if(pInBuf != NULL)
	{
		LumaDecoder(pInBuf, pOutBuf);
		ColorDecoder((pInBuf+86), (pOutBuf+120));
		ColorDecoder((pInBuf+136), (pOutBuf+240));
		LOG_INF(" OTP OK \n");
		return 1;
	}
	{
		LOG_INF(" OTP FAIL \n");
		return 0;
	}

}
// return value:
// bit[7]: 0 no otp info, 1 valid otp info
// bit[6]: 0 no otp wb, 1 valib otp wb
// bit[5]: 0 no otp vcm, 1 valid otp vcm
// bit[4]: 0 no otp lenc, 1 valid otp lenc
int read_otp(struct otp_struct *otp_ptr)
{

	int otp_flag=0;
	int addr=0;
	int temp=0;
	int i=0;
	int checksumLSC = 0;
	int checksumOTP = 0;
	int checksumTotal = 0;
	unsigned char lenc_out[360];
	//set 0x5002[1] to "0"
	int temp1=0;
	temp1 = OV13850_R2A_read_i2c(0x5002);
	OV13850_R2A_write_i2c(0x5002, (0x00 & 0x02) | (temp1 & (~0x02)));
	// read OTP into buffer
	OV13850_R2A_write_i2c(0x3d84, 0xC0);
	OV13850_R2A_write_i2c(0x3d88, 0x72); // OTP start address
	OV13850_R2A_write_i2c(0x3d89, 0x20);
	OV13850_R2A_write_i2c(0x3d8A, 0x73); // OTP end address
	OV13850_R2A_write_i2c(0x3d8B, 0xBE);

	for(i=0x7220;i<=0x73BE;i++)
	{
		OV13850_R2A_write_i2c(i,0); // clear OTP buffer, recommended use continuous write to accelarate
	}


	OV13850_R2A_write_i2c(0x3d81, 0x01); // load otp into buffer
	Delay(10);
	// OTP base information and WB calibration data
	otp_flag = OV13850_R2A_read_i2c(0x7220);
	printk("travies add for WB flag : 0x%x\n", otp_flag);
	LOG_INF(" WB calibration data : %x \n", otp_flag);
	addr = 0;
	if((otp_flag & 0xc0) == 0x40) {
		addr = 0x7221; // base address of info group 1
	}
	else if((otp_flag & 0x30) == 0x10) {
		addr = 0x7229; // base address of info group 2
	}

	if(addr == 0x7221){
		(*otp_ptr).flag = 0xc0; // valid info and AWB in OTP
		(*otp_ptr).module_integrator_id = OV13850_R2A_read_i2c(addr);
		(*otp_ptr).lens_id = OV13850_R2A_read_i2c( addr + 1);
		(*otp_ptr).production_year = OV13850_R2A_read_i2c(0x7331);
		(*otp_ptr).production_month = OV13850_R2A_read_i2c(0x7332);
		(*otp_ptr).production_day = OV13850_R2A_read_i2c(0x7333);
		//temp = OV13850_R2A_read_i2c(addr + 7);
		//(*otp_ptr).rg_ratio = (OV13850_R2A_read_i2c(addr + 5)<<2) + ((temp>>6) & 0x03);
		//(*otp_ptr).bg_ratio = (OV13850_R2A_read_i2c(addr + 6)<<2) + ((temp>>4) & 0x03);
		(*otp_ptr).rg_ratio = (OV13850_R2A_read_i2c(0x7223) << 2) + ((OV13850_R2A_read_i2c(0x7225) >> 6) & 0x03);
		(*otp_ptr).bg_ratio = (OV13850_R2A_read_i2c(0x7224) << 2) + ((OV13850_R2A_read_i2c(0x7225) >> 4) & 0x03);
	}
	else if(addr == 0x7229){
		(*otp_ptr).flag = 0xc0; // valid info and AWB in OTP
		(*otp_ptr).module_integrator_id = OV13850_R2A_read_i2c(addr);
		(*otp_ptr).lens_id = OV13850_R2A_read_i2c( addr + 1);
		(*otp_ptr).production_year = OV13850_R2A_read_i2c(0x7334);
		(*otp_ptr).production_month = OV13850_R2A_read_i2c(0x7335);
		(*otp_ptr).production_day = OV13850_R2A_read_i2c(0x7336);
		(*otp_ptr).rg_ratio = (OV13850_R2A_read_i2c(0x7228) << 2) + ((OV13850_R2A_read_i2c(0x7225) >> 2) & 0x03);
		(*otp_ptr).bg_ratio = (OV13850_R2A_read_i2c(0x7229) << 2) + ((OV13850_R2A_read_i2c(0x7225) >> 0) & 0x03);
	}
	else {
		(*otp_ptr).flag = 0x00; // not info in OTP
		(*otp_ptr).module_integrator_id = 0;
		(*otp_ptr).lens_id = 0;
		(*otp_ptr).production_year = 0;
		(*otp_ptr).production_month = 0;
		(*otp_ptr).production_day = 0;
		(*otp_ptr).rg_ratio = 0;
		(*otp_ptr).bg_ratio = 0;
	}

	printk("travies add for otp data checking : otp_flag =%d, module_integrator_id =%d,\
			lens_id = %d,production_year =%d, production_month = %d, production_day = %d,rg = %d, bg = %d\n",\
			(*otp_ptr).flag,(*otp_ptr).module_integrator_id,\
			(*otp_ptr).lens_id ,(*otp_ptr).production_year,(*otp_ptr).production_month,(*otp_ptr).production_day,(*otp_ptr).rg_ratio,(*otp_ptr).bg_ratio);

	// OTP VCM Calibration
	//otp_flag = OV13850_R2A_read_i2c(0x73ac);
	otp_flag = OV13850_R2A_read_i2c(0x722a);
	printk("travies add for VCM flag:0x%x\n",otp_flag);
	LOG_INF(" VCM calibration data : %x \n", otp_flag);
	addr = 0;
	if((otp_flag & 0xc0) == 0x40) {
		addr = 0x722b; // base address of VCM Calibration group 1
	}
	else if((otp_flag & 0x30) == 0x10) {
		addr = 0x722e; // base address of VCM Calibration group 2
	}

	if(addr == 0x722b){
		(*otp_ptr).flag |= 0x20;
		(* otp_ptr).VCM_start = (OV13850_R2A_read_i2c(0x722b) << 2) + ((OV13850_R2A_read_i2c(0x722d) >> 6) & 0x03 );
		(* otp_ptr).VCM_end = (OV13850_R2A_read_i2c(0x722c) << 2) + ((OV13850_R2A_read_i2c(0x722d) >> 4) & 0x03 );
		(* otp_ptr).VCM_dir = (OV13850_R2A_read_i2c(0x722d) >> 2) & 0x03;
	}
	else if(addr == 0x722e){
		(*otp_ptr).flag |= 0x20;
		(* otp_ptr).VCM_start = (OV13850_R2A_read_i2c(0x722e) << 2) + ((OV13850_R2A_read_i2c(0x7230) >> 6) & 0x03 );
		(* otp_ptr).VCM_end = (OV13850_R2A_read_i2c(0x722f) << 2) + ((OV13850_R2A_read_i2c(0x7230) >> 4) & 0x03 );
		(* otp_ptr).VCM_dir = (OV13850_R2A_read_i2c(0x7230) >> 2) & 0x03;
	}
	else {
		(* otp_ptr).VCM_start = 0;
		(* otp_ptr).VCM_end = 0;
		(* otp_ptr).VCM_dir = 0;
	}

	printk("travies add for vcm data: VCM_start is %d, VCM_end is %d, VCM_dir is %d\n",\
			(* otp_ptr).VCM_start,(* otp_ptr).VCM_end,(* otp_ptr).VCM_dir);

	// OTP Lenc Calibration
	otp_flag = OV13850_R2A_read_i2c(0x7248);
	printk("travies add for lenc flag:0x%x\n",otp_flag);
	LOG_INF(" Lenc calibration data : %x \n", otp_flag);
	addr = 0;
	//int checksumLSC = 0, checksumOTP = 0, checksumTotal = 0;
	if((otp_flag & 0xc0) == 0x40) {
		addr = 0x7249; // base address of Lenc Calibration group 1
	}
	else if((otp_flag & 0x30) == 0x10) {
		addr = 0x7304; // base address of Lenc Calibration group 2
	}
	//
	LOG_INF(" Lenc calibration addr : %x \n", addr);

	if(addr != 0) {
		for(i=0;i<186;i++) {
			(* otp_ptr).lenc[i]= OV13850_R2A_read_i2c(addr + i);
			checksumLSC += (* otp_ptr).lenc[i];
			LOG_INF(" Lenc (* otp_ptr).lenc[%d] : %x \n", i, (* otp_ptr).lenc[i]);
		}

		for(i=0;i<360;i++)
		{
			lenc_out[i] = 0;
		}

		if(Decode_13850R2A((*otp_ptr).lenc,  lenc_out))
		{
			for(i=0;i<360;i++)
			{
				LOG_INF(" from OTP lenc_out[%d]:%x \n", i, lenc_out[i]);
				checksumOTP = checksumOTP + lenc_out[i];
			}
		}

		checksumLSC = (checksumLSC)%255 +1;
		checksumOTP = (checksumOTP)%255 +1;
		//checksumTotal = (checksumLSC) ^ (checksumOTP);
		//Decode the lenc buffer from OTP , from 186 bytes to 360 bytes
		//int lenc_out[360];
		(* otp_ptr).checksumOTP=OV13850_R2A_read_i2c(addr + 186);

		if((* otp_ptr).checksumOTP == checksumOTP){
			(*otp_ptr).flag |= 0x10;
		}

		printk("travies add for caculated checksumLSC = %d, stored checksumOTP = %d, caculated checksunOTP = %d\n",checksumLSC,(* otp_ptr).checksumOTP,checksumOTP);

	}
	else {
		for(i=0;i<186;i++) {
			(* otp_ptr).lenc[i]=0;
		}
	}

	for(i=0x7249;i<=0x73be;i++)
	{
		OV13850_R2A_write_i2c(i,0); // clear OTP buffer, recommended use continuous write to accelarate
	}

	//set 0x5002[1] to "1"
	temp1 = OV13850_R2A_read_i2c(0x5002);
	OV13850_R2A_write_i2c(0x5002, (0x02 & 0x02) | (temp1 & (~0x02)));

	printk("travies add for final otp_flag = 0x%x\n",(*otp_ptr).flag);
	return (*otp_ptr).flag;
}


// return value:
// bit[7]: 0 no otp info, 1 valid otp info
// bit[6]: 0 no otp wb, 1 valib otp wb
// bit[5]: 0 no otp vcm, 1 valid otp vcm
// bit[4]: 0 no otp lenc, 1 valid otp lenc
int apply_otp(struct otp_struct *otp_ptr)
{
	int rg=0;
	int bg=0;
	int R_gain=0;
	int G_gain=0;
	int B_gain=0;
	int Base_gain=0;
	int temp=0;
	int i=0;
	unsigned char lenc_out[360];
	printk(" travies add for apply_otp (*otp_ptr).flag : %x \n", (*otp_ptr).flag);
	LOG_INF(" apply_otp (*otp_ptr).flag : %x \n", (*otp_ptr).flag);
	// apply OTP WB Calibration
	if ((*otp_ptr).flag & 0x40) {
		printk(" travies add for apply OTP WB Calibration : %x \n", (*otp_ptr).flag);
		LOG_INF(" apply OTP WB Calibration : %x \n", (*otp_ptr).flag);
		rg = (*otp_ptr).rg_ratio;
		bg = (*otp_ptr).bg_ratio;

		//calculate G gain
		R_gain = (RG_Ratio_Typical*1000) / rg;
		B_gain = (BG_Ratio_Typical*1000) / bg;
		G_gain = 1000;

		if (R_gain < 1000 || B_gain < 1000)
		{
			if (R_gain < B_gain)
				Base_gain = R_gain;
			else
				Base_gain = B_gain;
		}
		else
		{
			Base_gain = G_gain;
		}

		R_gain = 0x400 * R_gain / (Base_gain);
		B_gain = 0x400 * B_gain / (Base_gain);
		G_gain = 0x400 * G_gain / (Base_gain);

		printk("travies add for R_gain:%d,B_gain:%d,G_gain:%d\n",R_gain,B_gain,G_gain);

		// update sensor WB gain
		if (R_gain>0x400) {
			OV13850_R2A_write_i2c(0x5056, R_gain>>8);
			OV13850_R2A_write_i2c(0x5057, R_gain & 0x00ff);
		}

		if (G_gain>0x400) {
			OV13850_R2A_write_i2c(0x5058, G_gain>>8);
			OV13850_R2A_write_i2c(0x5059, G_gain & 0x00ff);
		}

		if (B_gain>0x400) {
			OV13850_R2A_write_i2c(0x505A, B_gain>>8);
			OV13850_R2A_write_i2c(0x505B, B_gain & 0x00ff);
		}
	}

	// apply OTP Lenc Calibration
	if ((*otp_ptr).flag & 0x10) {

		printk(" travies add for apply OTP Lenc Calibration : %x \n", (*otp_ptr).flag);
		LOG_INF(" apply OTP Lenc Calibration : %x \n", (*otp_ptr).flag);
		temp = OV13850_R2A_read_i2c(0x5000);
		temp = 0x01 | temp;
		OV13850_R2A_write_i2c(0x5000, temp);

		//Decode the lenc buffer from OTP , from 186 bytes to 360 bytes

		for(i=0;i<360;i++)
		{
			lenc_out[i] = 0;
		}
		/***For function Decode_13850R2A(unsigned char*pInBuf, unsigned char* pOutBuf),please refer to lc42.h***/
		if(Decode_13850R2A((*otp_ptr).lenc,  lenc_out))
		{
			for(i=0;i<360 ;i++) {
				LOG_INF(" apply OTP lenc_out[%d]:%x \n", i, lenc_out[i]);
				OV13850_R2A_write_i2c(0x5200 + i, lenc_out[i]);
			}
		}
	}
}

static int read_ov13850_AF_info(void)
{
	int otp_flag = 0;
	int addr = 0;
	int i = 0;

	//OV13850_R2A_write_i2c(0x0100,0x01); Has done in sensor_init()

	//set 0x5002[1] to "0"
	int temp1 = 0;
	temp1 = OV13850_R2A_read_i2c(0x5002);
	OV13850_R2A_write_i2c(0x5002, (0x00 & 0x02) | (temp1 & (~0x02)));
	// read OTP into buffer
	OV13850_R2A_write_i2c(0x3d84, 0xC0);
	OV13850_R2A_write_i2c(0x3d88, 0x72); // OTP start address
	OV13850_R2A_write_i2c(0x3d89, 0x2A);
	OV13850_R2A_write_i2c(0x3d8A, 0x72); // OTP end address
	OV13850_R2A_write_i2c(0x3d8B, 0x30);

	for(i=0x722a;i<=0x7230;i++)
	{
		OV13850_R2A_write_i2c(i,0); // clear OTP buffer, recommended use continuous write to accelarate
	}

	OV13850_R2A_write_i2c(0x3d81, 0x01); // load otp into buffer
	Delay(10);

	otp_flag = OV13850_R2A_read_i2c(0x722a);
	printk("travies add for VCM flag:0x%x\n",otp_flag);
	LOG_INF(" VCM calibration data : %x \n", otp_flag);

	if((otp_flag & 0xc0) == 0x40) {
		addr = 0x722b; // base address of VCM Calibration group 1
	}
	else if((otp_flag & 0x30) == 0x10) {
		addr = 0x722e; // base address of VCM Calibration group 2
	}
	else addr = 0;


	if((addr == 0x722b) || (addr == 0x722e)){
		ov13850_eeprom_data.Data[17] = ((OV13850_R2A_read_i2c(addr + 2) >> 6) & 0x03) + ((OV13850_R2A_read_i2c(addr) & 0x3F) << 2) ;
    	ov13850_eeprom_data.Data[18] = OV13850_R2A_read_i2c(addr) >> 6;
	vision.AfInfinite = (ov13850_eeprom_data.Data[18] << 8) | ov13850_eeprom_data.Data[17];
	printk("vision666 af infite before>>>%d\n",vision.AfInfinite);
	vision.AfInfinite = vision.AfInfinite - 50;
	ov13850_eeprom_data.Data[17] = vision.AfInfinite & 0xff;
	ov13850_eeprom_data.Data[18] = (vision.AfInfinite >> 8) & 0xff;
	printk("vision666 af infite after>>>%d\n",vision.AfInfinite);
    	ov13850_eeprom_data.Data[19] = ((OV13850_R2A_read_i2c(addr + 2) >> 4) & 0x03) + ((OV13850_R2A_read_i2c(addr + 1) & 0x3F) << 2);
   		ov13850_eeprom_data.Data[20] = OV13850_R2A_read_i2c(addr + 1) >> 6;
	vision.AfMacro = (ov13850_eeprom_data.Data[20] << 8) | ov13850_eeprom_data.Data[19];
	printk("vision666 af macro before>>>%d\n",vision.AfMacro);
	vision.AfMacro = vision.AfMacro + 50;
	ov13850_eeprom_data.Data[19] = vision.AfMacro & 0xff;
	ov13850_eeprom_data.Data[20] = (vision.AfMacro >> 8) & 0xff;
	printk("vision666 af macro after>>>%d\n",vision.AfMacro);
	}
	else {
		ov13850_eeprom_data.Data[17] = 0;
		ov13850_eeprom_data.Data[18] = 0;
		ov13850_eeprom_data.Data[19] = 0;
		ov13850_eeprom_data.Data[20] = 0;
	}


	ov13850_eeprom_data.Data[0]= 0x01; //to fix
	ov13850_eeprom_data.Data[1]= 0xFF;
	ov13850_eeprom_data.Data[2]= 0x00;
	ov13850_eeprom_data.Data[3]= 0x0B;
	ov13850_eeprom_data.Data[4]= 0x02;


	for(i=0x722a;i<=0x7230;i++)
	{
		OV13850_R2A_write_i2c(i,0); // clear OTP buffer, recommended use continuous write to accelarate
	}


	temp1 = OV13850_R2A_read_i2c(0x5002);
	OV13850_R2A_write_i2c(0x5002, (0x02 & 0x02) | (temp1 & (~0x02)));

	return 0;

}



int read_ov13850_eeprom_data(void)
{
	struct timeval ktv1, ktv2;
	unsigned long TimeIntervalUS;
	int i = 0;
	int offset = 0;
	do_gettimeofday(&ktv1);
	//	if(EEPROM_Slim_File_Load())

	read_ov13850_AF_info();

	do_gettimeofday(&ktv2);

	if(ktv2.tv_sec > ktv1.tv_sec)
		TimeIntervalUS = ktv1.tv_usec + 1000000 - ktv2.tv_usec;
	else
		TimeIntervalUS = ktv2.tv_usec - ktv1.tv_usec;

	printk("lizihao add ov13850 otp read data take %lu us\n", TimeIntervalUS);
	return 1;
}



static int selective_read_region(u32 offset, BYTE* data,u32 size)
{

	printk("lizihao add ov13850 selective_read_region offset = %d,size = %d\n", offset,size);
	memcpy((void *)data,(void *)&ov13850_eeprom_data.Data[offset],size);
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

static long ov13850_Ioctl_Compat(struct file *filp, unsigned int cmd, unsigned long arg)
{
	long ret;
	COMPAT_stCAM_CAL_INFO_STRUCT __user *data32;
	stCAM_CAL_INFO_STRUCT __user *data;
	int err;
	CAM_CALDB("[CAMERA SENSOR] ov13850_eeprom_DEVICE_ID,%p %p %x ioc size %d\n",filp->f_op ,filp->f_op->unlocked_ioctl,cmd,_IOC_SIZE(cmd) );
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
	.compat_ioctl = ov13850_Ioctl_Compat,
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





/***********Decode LENC Para Process Start*********************/
void LumaDecoder(uint8_t *pData, uint8_t *pPara)
{

	uint32_t Offset, Bit, Option;
	uint32_t i, k;
	uint8_t pCenter[16], pMiddle[32], pCorner[72];
	Offset = pData[0];
	Bit = pData[1]>>4;
	Option = pData[1]&0xf;
	LOG_INF("Offset:%x, Bit:%x, Option:%x \n",Offset, Bit, Option);
	for (i=0; i<180; i++)
	{
		LOG_INF("data-pData[%d]:%x \n", i, pData[i]);
	}
	if(Bit <= 5)
	{
		for(i=0,k=2; i<120; i+=8,k+=5)
		{
			//LOG_INF("pData[%d]:%x \n", i, pData[i]);
			pPara[i] = pData[k]>>3; // 7~3 (byte0)
			pPara[i+1] = ((pData[k]&0x7)<<2)|(pData[k+1]>>6); // 2~0 (byte0) and 7~6(byte1)
			pPara[i+2] = (pData[k+1]&0x3e)>>1; // 5~1 (byte1)
			pPara[i+3] = ((pData[k+1]&0x1)<<4)|(pData[k+2]>>4); // 0 (byte1) and 7~4(byte2)
			pPara[i+4] = ((pData[k+2]&0xf)<<1)|(pData[k+3]>>7); // 3~0 (byte2) and 7(byte3)
			pPara[i+5] = (pData[k+3]&0x7c)>>2; // 6~2 (byte3)
			pPara[i+6] = ((pData[k+3]&0x3)<<3)|(pData[k+4]>>5); // 1~0 (byte3) and 7~5(byte4)
			pPara[i+7] = pData[k+4]&0x1f; // 4~0 (byte4)

			LOG_INF("bit-pData[%d]:%x \n", k, pData[k]);
			LOG_INF("bit-pData[%d]:%x \n", k+1, pData[k+1]);
			LOG_INF("bit-pData[%d]:%x \n", k+2, pData[k+2]);
			LOG_INF("bit-pData[%d]:%x \n", k+3, pData[k+3]);
			LOG_INF("bit-pData[%d]:%x \n", k+4, pData[k+4]);


			LOG_INF("bit-pData[%d]:%x \n", i, pPara[i]);
			LOG_INF("bit-pData[%d]:%x \n", i+1, pPara[i+1]);
			LOG_INF("bit-pData[%d]:%x \n", i+2, pPara[i+2]);
			LOG_INF("bit-pData[%d]:%x \n", i+3, pPara[i+3]);
			LOG_INF("bit-pData[%d]:%x \n", i+4, pPara[i+4]);
			LOG_INF("bit-pData[%d]:%x \n", i+5, pPara[i+5]);
			LOG_INF("bit-pData[%d]:%x \n", i+6, pPara[i+6]);
			LOG_INF("bit-pData[%d]:%x \n", i+7, pPara[i+7]);
		}
	}
	else
	{
		for(i=0,k=2; i<48; i+=8,k+=5)
		{
			//LOG_INF(" apData[%d]:%x \n", i, pData[i]);
			pPara[i] = pData[k]>>3; // 7~3 (byte0)
			pPara[i+1] = ((pData[k]&0x7)<<2)|(pData[k+1]>>6); // 2~0 (byte0) and 7~6(byte1)
			pPara[i+2] = (pData[k+1]&0x3e)>>1; // 5~1 (byte1)
			pPara[i+3] = ((pData[k+1]&0x1)<<4)|(pData[k+2]>>4); // 0 (byte1) and 7~4(byte2)
			pPara[i+4] = ((pData[k+2]&0xf)<<1)|(pData[k+3]>>7); // 3~0 (byte2) and 7(byte3)
			pPara[i+5] = (pData[k+3]&0x7c)>>2; // 6~2 (byte3)
			pPara[i+6] = ((pData[k+3]&0x3)<<3)|(pData[k+4]>>5); // 1~0 (byte3) and 7~5(byte4)
			pPara[i+7] = pData[k+4]&0x1f; // 4~0 (byte4)

			LOG_INF("else -pData[%d]:%x \n", i, pData[i]);
			LOG_INF("else -pData[%d]:%x \n", i+1, pData[i+1]);
			LOG_INF("else -pData[%d]:%x \n", i+2, pData[i+2]);
			LOG_INF("else -pData[%d]:%x \n", i+3, pData[i+3]);
			LOG_INF("else -pData[%d]:%x \n", i+4, pData[i+4]);
			LOG_INF("else -pData[%d]:%x \n", i+5, pData[i+5]);
			LOG_INF("else -pData[%d]:%x \n", i+6, pData[i+6]);
			LOG_INF("else -pData[%d]:%x \n", i+7, pData[i+7]);
		}
		for(i=48,k=32; i<120; i+=4,k+=3)
		{
			//LOG_INF(" 48--pData[%d]:%x \n", i, pData[i]);
			pPara[i] = pData[k]>>2; // 7~2 (byte0)
			pPara[i+1] = ((pData[k]&0x3)<<4)|(pData[k+1]>>4); //1~0 (byte0) and7~4(byte1)
			pPara[i+2] = ((pData[k+1]&0xf)<<2)|(pData[k+2]>>6); //3~0 (byte1) and7~6(byte2)
			pPara[i+3] = pData[k+2]&0x3f; // 5~0 (byte2)

			LOG_INF("48--pData[%d]:%x \n", i, pData[i]);
			LOG_INF("48--pData[%d]:%x \n", i+1, pData[i+1]);
			LOG_INF("48--pData[%d]:%x \n", i+2, pData[i+2]);
			LOG_INF("48--pData[%d]:%x \n", i+3, pData[i+3]);
		}
		memcpy(pCenter, pPara, 16);
		memcpy(pMiddle, pPara+16, 32);
		memcpy(pCorner, pPara+48, 72);
		for(i=0; i<32; i++)
		{
			pMiddle[i] <<= (Bit-6);
		}
		for(i=0; i<72; i++)
		{
			pCorner[i] <<= (Bit-6);
		}
		if(Option == 0)
		{ // 10x12
			memcpy(pPara, pCorner, 26);
			memcpy(pPara+26, pMiddle, 8);
			memcpy(pPara+34, pCorner+26, 4);
			memcpy(pPara+38, pMiddle+8, 2);
			memcpy(pPara+40, pCenter, 4);
			memcpy(pPara+44, pMiddle+10, 2);
			memcpy(pPara+46, pCorner+30, 4);
			memcpy(pPara+50, pMiddle+12, 2);
			memcpy(pPara+52, pCenter+4, 4);
			memcpy(pPara+56, pMiddle+14, 2);
			memcpy(pPara+58, pCorner+34, 4);
			memcpy(pPara+62, pMiddle+16, 2);
			memcpy(pPara+64, pCenter+8, 4);
			memcpy(pPara+68, pMiddle+18, 2);
			memcpy(pPara+70, pCorner+38, 4);
			memcpy(pPara+74, pMiddle+20, 2);
			memcpy(pPara+76, pCenter+12, 4);
			memcpy(pPara+80, pMiddle+22, 2);
			memcpy(pPara+82, pCorner+42, 4);
			memcpy(pPara+86, pMiddle+24, 8);
			memcpy(pPara+94, pCorner+46, 26);
		}
		else
		{ // 12x10
			memcpy(pPara, pCorner, 22);
			memcpy(pPara+22, pMiddle, 6);
			memcpy(pPara+28, pCorner+22, 4);
			memcpy(pPara+32, pMiddle+6, 6);
			memcpy(pPara+38, pCorner+26, 4);
			memcpy(pPara+42, pMiddle+12, 1);
			memcpy(pPara+43, pCenter, 4);
			memcpy(pPara+47, pMiddle+13, 1);
			memcpy(pPara+48, pCorner+30, 4);
			memcpy(pPara+52, pMiddle+14, 1);
			memcpy(pPara+53, pCenter+4, 4);
			memcpy(pPara+57, pMiddle+15, 1);
			memcpy(pPara+58, pCorner+34, 4);
			memcpy(pPara+62, pMiddle+16, 1);
			memcpy(pPara+63, pCenter+8, 4);
			memcpy(pPara+67, pMiddle+17, 1);
			memcpy(pPara+68, pCorner+38, 4);
			memcpy(pPara+72, pMiddle+18, 1);
			memcpy(pPara+73, pCenter+12, 4);
			memcpy(pPara+77, pMiddle+19, 1);
			memcpy(pPara+78, pCorner+42, 4);
			memcpy(pPara+82, pMiddle+20, 6);
			memcpy(pPara+88, pCorner+46, 4);
			memcpy(pPara+92, pMiddle+26, 6);
			memcpy(pPara+98, pCorner+50, 22);
		}
	}
	for(i=0; i<120; i++)
	{
		LOG_INF(" pPara[%d]:%x \n", i, pPara[i]);
		pPara[i] += Offset;
	}

}
//
void ColorDecoder(uint8_t *pData, uint8_t *pPara)
{

	uint32_t Offset, Bit, Option;
	uint32_t i, k;
	uint8_t pBase[30];
	Offset = pData[0];
	Bit = pData[1]>>7;
	Option = (pData[1]&0x40)>>6;
	pPara[0] = (pData[1]&0x3e)>>1; // 5~1 (byte1)
	pPara[1] = ((pData[1]&0x1)<<4)|(pData[2]>>4); // 0 (byte1) and 7~4 (byte2)
	pPara[2] = ((pData[2]&0xf)<<1)|(pData[3]>>7); // 3~0 (byte2) and 7 (byte3)
	pPara[3] = (pData[3]&0x7c)>>2; // 6~2 (byte3)
	pPara[4] = ((pData[3]&0x3)<<3)|(pData[4]>>5); // 1~0 (byte3) and 7~5 (byte4)
	pPara[5] = pData[4]&0x1f; // 4~0 (byte4)
	for(i=6,k=5; i<30; i+=8,k+=5)
	{
		pPara[i] = pData[k]>>3; // 7~3 (byte0)
		pPara[i+1] = ((pData[k]&0x7)<<2)|(pData[k+1]>>6); // 2~0 (byte0) and 7~6 (byte1)
		pPara[i+2] = (pData[k+1]&0x3e)>>1; // 5~1 (byte1)
		pPara[i+3] = ((pData[k+1]&0x1)<<4)|(pData[k+2]>>4); // 0 (byte1) and 7~4 (byte2)
		pPara[i+4] = ((pData[k+2]&0xf)<<1)|(pData[k+3]>>7); // 3~0 (byte2) and 7 (byte3)
		pPara[i+5] = (pData[k+3]&0x7c)>>2; // 6~2 (byte3)
		pPara[i+6] = ((pData[k+3]&0x3)<<3)|(pData[k+4]>>5); // 1~0 (byte3) and 7~5 (byte4)
		pPara[i+7] = pData[k+4]&0x1f; // 4~0 (byte4)
	}
	memcpy(pBase, pPara, 30);
	for(i=0,k=20; i<120; i+=4,k++)
	{
		pPara[i] = pData[k]>>6;
		pPara[i+1] = (pData[k]&0x30)>>4;
		pPara[i+2] = (pData[k]&0xc)>>2;
		pPara[i+3] = pData[k]&0x3;
	}
	if(Option == 0)
	{ // 10x12
		for(i=0; i<5; i++)
		{
			for(k=0; k<6; k++)
			{
				pPara[i*24+k*2] += pBase[i*6+k];
				pPara[i*24+k*2+1] += pBase[i*6+k];
				pPara[i*24+k*2+12] += pBase[i*6+k];
				pPara[i*24+k*2+13] += pBase[i*6+k];
			}
		}
	}else
	{ // 12x10
		for(i=0; i<6; i++)
		{
			for(k=0; k<5; k++)
			{
				pPara[i*20+k*2] += pBase[i*5+k];
				pPara[i*20+k*2+1] += pBase[i*5+k];
				pPara[i*20+k*2+10] += pBase[i*5+k];
				pPara[i*20+k*2+11] += pBase[i*5+k];
			}
		}
	}
	for(i=0; i<120; i++)
	{
		pPara[i] = (pPara[i]<<Bit) + Offset;
	}

}
//
