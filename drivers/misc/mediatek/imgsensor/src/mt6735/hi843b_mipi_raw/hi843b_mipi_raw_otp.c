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
#include "hi843b_mipi_raw_otp.h"
#include "kd_camera_typedef.h"


#include <linux/dma-mapping.h>
#ifdef CONFIG_COMPAT
/* 64 bit */
#include <linux/fs.h>
#include <linux/compat.h>
#endif

#define PFX "HI843B_OTP_FMT"

//0917 Open Below Line. must open. 
#define Hynix_OTP_File_Load // eeprom_slim.txt file load. For SW Eng. 

#ifdef Hynix_OTP_File_Load
  #include <linux/kernel.h>
  #include <linux/init.h> 
  #include <linux/module.h> 
  #include <linux/syscalls.h> 
  #include <linux/fcntl.h> 
  #include <asm/uaccess.h> 
  #include <linux/types.h> 
  #include <linux/slab.h> 
#endif 

//#define CAM_CALGETDLT_DEBUG
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


#define USHORT             unsigned short
#define BYTE               unsigned char
#define Sleep(ms) mdelay(ms)


/*******************************************************************************
*
********************************************************************************/
#define CAM_CAL_ICS_REVISION 1 //seanlin111208

/*******************************************************************************
*
********************************************************************************/
#define CAM_CAL_DRVNAME "CAM_CAL_DRV"
#define CAM_CAL_I2C_GROUP_ID 0

/*******************************************************************************
*
********************************************************************************/
#define OTP_READ_SKIP 0 
#define OTP_READ 1 

static dev_t g_CAM_CALdevno = MKDEV(CAM_CAL_DEV_MAJOR_NUMBER,0);
static struct cdev * g_pCAM_CAL_CharDrv = NULL;

static struct class *CAM_CAL_class = NULL;
static atomic_t g_CAM_CALatomic;


#define MAX_LSC_SIZE 2048
#define MAX_OTP_SIZE 2200
static int Hi843b_otp_read = 0;

struct Hynix_OTP_Sensor_reg { // OTP Register Type
	u16 addr;
	u16 val;
};


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

OTP_DATA hi843b_otp_data = {{0}}; // Hynix Data Structure 


extern kal_uint16 read_HynixOtp_sensor_8(kal_uint32 addr);
extern kal_uint16 read_HynixOtp_sensor_16(kal_uint32 addr);

extern void write_HynixOtp_sensor_16(kal_uint16 addr,kal_uint16 para);
extern void write_HynixOtp_sensor_8(kal_uint16 addr, kal_uint8 para);

//extern void write_HynixOtp_sensor_16(kal_uint32 addr, kal_uint32 para);

static void Enable_Hi843b_OTP_Read(void)
    {
    CAM_CALERR("OTP.C Enter init\n");
	write_HynixOtp_sensor_16(0x0a00, 0x0000); //stream off
	write_HynixOtp_sensor_16(0x0e00, 0x0002); //tg_pmem_sckpw/sdly
	write_HynixOtp_sensor_16(0x0e02, 0x0002); //tg_dmem_sckpw/sdly
	write_HynixOtp_sensor_16(0x2000, 0x4031);
	write_HynixOtp_sensor_16(0x2002, 0x8400);
	write_HynixOtp_sensor_16(0x2004, 0x40f2);
	write_HynixOtp_sensor_16(0x2006, 0x000a);
	write_HynixOtp_sensor_16(0x2008, 0x0f90);
	write_HynixOtp_sensor_16(0x200a, 0x43c2);
	write_HynixOtp_sensor_16(0x200c, 0x0f82);
	write_HynixOtp_sensor_16(0x200e, 0x40b2);
	write_HynixOtp_sensor_16(0x2010, 0x1807);
	write_HynixOtp_sensor_16(0x2012, 0x82a8);
	write_HynixOtp_sensor_16(0x2014, 0x40b2);
	write_HynixOtp_sensor_16(0x2016, 0x3540);
	write_HynixOtp_sensor_16(0x2018, 0x82aa);
	write_HynixOtp_sensor_16(0x201a, 0x40b2);
	write_HynixOtp_sensor_16(0x201c, 0x3540);
	write_HynixOtp_sensor_16(0x201e, 0x82ac);
	write_HynixOtp_sensor_16(0x2020, 0x4382);
	write_HynixOtp_sensor_16(0x2022, 0x82ae);
	write_HynixOtp_sensor_16(0x2024, 0x4382);
	write_HynixOtp_sensor_16(0x2026, 0x82b0);
	write_HynixOtp_sensor_16(0x2028, 0x4382);
	write_HynixOtp_sensor_16(0x202a, 0x82b2);
	write_HynixOtp_sensor_16(0x202c, 0x40b2);
	write_HynixOtp_sensor_16(0x202e, 0x0600);
	write_HynixOtp_sensor_16(0x2030, 0x82b4);
	write_HynixOtp_sensor_16(0x2032, 0x4382);
	write_HynixOtp_sensor_16(0x2034, 0x82b6);
	write_HynixOtp_sensor_16(0x2036, 0x12b0);
	write_HynixOtp_sensor_16(0x2038, 0xea48);
	write_HynixOtp_sensor_16(0x203a, 0x40b2);
	write_HynixOtp_sensor_16(0x203c, 0x2d0b);
	write_HynixOtp_sensor_16(0x203e, 0x0b96);
	write_HynixOtp_sensor_16(0x2040, 0x0c0a);
	write_HynixOtp_sensor_16(0x2042, 0x40b2);
	write_HynixOtp_sensor_16(0x2044, 0xc009);
	write_HynixOtp_sensor_16(0x2046, 0x0b98);
	write_HynixOtp_sensor_16(0x2048, 0x0c0a);
	write_HynixOtp_sensor_16(0x204a, 0x0900);
	write_HynixOtp_sensor_16(0x204c, 0x7312);
	write_HynixOtp_sensor_16(0x204e, 0x43d2);
	write_HynixOtp_sensor_16(0x2050, 0x0f82);
	write_HynixOtp_sensor_16(0x2052, 0x0cff);
	write_HynixOtp_sensor_16(0x2054, 0x0cff);
	write_HynixOtp_sensor_16(0x2056, 0x0cff);
	write_HynixOtp_sensor_16(0x2058, 0x0cff);
	write_HynixOtp_sensor_16(0x205a, 0x0cff);
	write_HynixOtp_sensor_16(0x205c, 0x0cff);
	write_HynixOtp_sensor_16(0x205e, 0x0cff);
	write_HynixOtp_sensor_16(0x2060, 0x0cff);
	write_HynixOtp_sensor_16(0x2062, 0x0cff);
	write_HynixOtp_sensor_16(0x2064, 0x0cff);
	write_HynixOtp_sensor_16(0x2066, 0x0cff);
	write_HynixOtp_sensor_16(0x2068, 0x0cff);
	write_HynixOtp_sensor_16(0x206a, 0x0cff);
	write_HynixOtp_sensor_16(0x206c, 0x0cff);
	write_HynixOtp_sensor_16(0x206e, 0x0cff);
	write_HynixOtp_sensor_16(0x2070, 0x0cff);
	write_HynixOtp_sensor_16(0x2072, 0x0cff);
	write_HynixOtp_sensor_16(0x2074, 0x0cff);
	write_HynixOtp_sensor_16(0x2076, 0x40f2);
	write_HynixOtp_sensor_16(0x2078, 0x000e);
	write_HynixOtp_sensor_16(0x207a, 0x0f90);
	write_HynixOtp_sensor_16(0x207c, 0x43d2);
	write_HynixOtp_sensor_16(0x207e, 0x0780);
	write_HynixOtp_sensor_16(0x2080, 0x4392);
	write_HynixOtp_sensor_16(0x2082, 0x760e);
	write_HynixOtp_sensor_16(0x2084, 0x9382);
	write_HynixOtp_sensor_16(0x2086, 0x760c);
	write_HynixOtp_sensor_16(0x2088, 0x2002);
	write_HynixOtp_sensor_16(0x208a, 0x0c64);
	write_HynixOtp_sensor_16(0x208c, 0x3ffb);
	write_HynixOtp_sensor_16(0x208e, 0x421f);
	write_HynixOtp_sensor_16(0x2090, 0x760a);
	write_HynixOtp_sensor_16(0x2092, 0x931f);
	write_HynixOtp_sensor_16(0x2094, 0x2013);
	write_HynixOtp_sensor_16(0x2096, 0x421b);
	write_HynixOtp_sensor_16(0x2098, 0x078a);
	write_HynixOtp_sensor_16(0x209a, 0x4b82);
	write_HynixOtp_sensor_16(0x209c, 0x7600);
	write_HynixOtp_sensor_16(0x209e, 0x0260);
	write_HynixOtp_sensor_16(0x20a0, 0x0000);
	write_HynixOtp_sensor_16(0x20a2, 0x0c56);
	write_HynixOtp_sensor_16(0x20a4, 0x0240);
	write_HynixOtp_sensor_16(0x20a6, 0x0000);
	write_HynixOtp_sensor_16(0x20a8, 0x0260);
	write_HynixOtp_sensor_16(0x20aa, 0x0000);
	write_HynixOtp_sensor_16(0x20ac, 0x0c1a);
	write_HynixOtp_sensor_16(0x20ae, 0x4b0f);
	write_HynixOtp_sensor_16(0x20b0, 0x12b0);
	write_HynixOtp_sensor_16(0x20b2, 0xe84c);
	write_HynixOtp_sensor_16(0x20b4, 0x4fc2);
	write_HynixOtp_sensor_16(0x20b6, 0x0788);
	write_HynixOtp_sensor_16(0x20b8, 0x4b0a);
	write_HynixOtp_sensor_16(0x20ba, 0x3fe2);
	write_HynixOtp_sensor_16(0x20bc, 0x903f);
	write_HynixOtp_sensor_16(0x20be, 0x0201);
	write_HynixOtp_sensor_16(0x20c0, 0x23df);
	write_HynixOtp_sensor_16(0x20c2, 0x531b);
	write_HynixOtp_sensor_16(0x20c4, 0x4b0e);
	write_HynixOtp_sensor_16(0x20c6, 0x108e);
	write_HynixOtp_sensor_16(0x20c8, 0xf37e);
	write_HynixOtp_sensor_16(0x20ca, 0xc312);
	write_HynixOtp_sensor_16(0x20cc, 0x100e);
	write_HynixOtp_sensor_16(0x20ce, 0x110e);
	write_HynixOtp_sensor_16(0x20d0, 0x110e);
	write_HynixOtp_sensor_16(0x20d2, 0x110e);
	write_HynixOtp_sensor_16(0x20d4, 0x4a0f);
	write_HynixOtp_sensor_16(0x20d6, 0x108f);
	write_HynixOtp_sensor_16(0x20d8, 0xf37f);
	write_HynixOtp_sensor_16(0x20da, 0xc312);
	write_HynixOtp_sensor_16(0x20dc, 0x100f);
	write_HynixOtp_sensor_16(0x20de, 0x110f);
	write_HynixOtp_sensor_16(0x20e0, 0x110f);
	write_HynixOtp_sensor_16(0x20e2, 0x110f);
	write_HynixOtp_sensor_16(0x20e4, 0x9f0e);
	write_HynixOtp_sensor_16(0x20e6, 0x27e3);
	write_HynixOtp_sensor_16(0x20e8, 0x0261);
	write_HynixOtp_sensor_16(0x20ea, 0x0000);
	write_HynixOtp_sensor_16(0x20ec, 0x4b82);
	write_HynixOtp_sensor_16(0x20ee, 0x7600);
	write_HynixOtp_sensor_16(0x20f0, 0x0260);
	write_HynixOtp_sensor_16(0x20f2, 0x0000);
	write_HynixOtp_sensor_16(0x20f4, 0x0c56);
	write_HynixOtp_sensor_16(0x20f6, 0x0240);
	write_HynixOtp_sensor_16(0x20f8, 0x0000);
	write_HynixOtp_sensor_16(0x20fa, 0x0260);
	write_HynixOtp_sensor_16(0x20fc, 0x0000);
	write_HynixOtp_sensor_16(0x20fe, 0x0c1a);
	write_HynixOtp_sensor_16(0x2100, 0x3fd6);
	write_HynixOtp_sensor_16(0x2102, 0x4030);
	write_HynixOtp_sensor_16(0x2104, 0xf106);
	write_HynixOtp_sensor_16(0x2106, 0xdf02);
	write_HynixOtp_sensor_16(0x2108, 0x3ffe);
	write_HynixOtp_sensor_16(0x210a, 0x4c82);
	write_HynixOtp_sensor_16(0x210c, 0x0130);
	write_HynixOtp_sensor_16(0x210e, 0x4a82);
	write_HynixOtp_sensor_16(0x2110, 0x0138);
	write_HynixOtp_sensor_16(0x2112, 0x4c82);
	write_HynixOtp_sensor_16(0x2114, 0x0134);
	write_HynixOtp_sensor_16(0x2116, 0x421e);
	write_HynixOtp_sensor_16(0x2118, 0x013a);
	write_HynixOtp_sensor_16(0x211a, 0x4292);
	write_HynixOtp_sensor_16(0x211c, 0x013c);
	write_HynixOtp_sensor_16(0x211e, 0x013a);
	write_HynixOtp_sensor_16(0x2120, 0x4b82);
	write_HynixOtp_sensor_16(0x2122, 0x0138);
	write_HynixOtp_sensor_16(0x2124, 0x4d82);
	write_HynixOtp_sensor_16(0x2126, 0x0134);
	write_HynixOtp_sensor_16(0x2128, 0x4a82);
	write_HynixOtp_sensor_16(0x212a, 0x0138);
	write_HynixOtp_sensor_16(0x212c, 0x421f);
	write_HynixOtp_sensor_16(0x212e, 0x013a);
	write_HynixOtp_sensor_16(0x2130, 0x4130);
	write_HynixOtp_sensor_16(0x2132, 0xee0e);
	write_HynixOtp_sensor_16(0x2134, 0x403b);
	write_HynixOtp_sensor_16(0x2136, 0x0011);
	write_HynixOtp_sensor_16(0x2138, 0x3c05);
	write_HynixOtp_sensor_16(0x213a, 0x100d);
	write_HynixOtp_sensor_16(0x213c, 0x6e0e);
	write_HynixOtp_sensor_16(0x213e, 0x9a0e);
	write_HynixOtp_sensor_16(0x2140, 0x2801);
	write_HynixOtp_sensor_16(0x2142, 0x8a0e);
	write_HynixOtp_sensor_16(0x2144, 0x6c0c);
	write_HynixOtp_sensor_16(0x2146, 0x6d0d);
	write_HynixOtp_sensor_16(0x2148, 0x831b);
	write_HynixOtp_sensor_16(0x214a, 0x23f7);
	write_HynixOtp_sensor_16(0x214c, 0x4130);
	write_HynixOtp_sensor_16(0x214e, 0xef0f);
	write_HynixOtp_sensor_16(0x2150, 0xee0e);
	write_HynixOtp_sensor_16(0x2152, 0x4039);
	write_HynixOtp_sensor_16(0x2154, 0x0021);
	write_HynixOtp_sensor_16(0x2156, 0x3c0a);
	write_HynixOtp_sensor_16(0x2158, 0x1008);
	write_HynixOtp_sensor_16(0x215a, 0x6e0e);
	write_HynixOtp_sensor_16(0x215c, 0x6f0f);
	write_HynixOtp_sensor_16(0x215e, 0x9b0f);
	write_HynixOtp_sensor_16(0x2160, 0x2805);
	write_HynixOtp_sensor_16(0x2162, 0x2002);
	write_HynixOtp_sensor_16(0x2164, 0x9a0e);
	write_HynixOtp_sensor_16(0x2166, 0x2802);
	write_HynixOtp_sensor_16(0x2168, 0x8a0e);
	write_HynixOtp_sensor_16(0x216a, 0x7b0f);
	write_HynixOtp_sensor_16(0x216c, 0x6c0c);
	write_HynixOtp_sensor_16(0x216e, 0x6d0d);
	write_HynixOtp_sensor_16(0x2170, 0x6808);
	write_HynixOtp_sensor_16(0x2172, 0x8319);
	write_HynixOtp_sensor_16(0x2174, 0x23f1);
	write_HynixOtp_sensor_16(0x2176, 0x4130);
	write_HynixOtp_sensor_16(0x2ffe, 0xf000);
	write_HynixOtp_sensor_16(0x0d00, 0x0707);
	write_HynixOtp_sensor_16(0x0d04, 0x0100);
	write_HynixOtp_sensor_16(0x004c, 0x0100); //tg enable,hdr off
	write_HynixOtp_sensor_16(0x0a00, 0x0100); //stream on 
	
    write_HynixOtp_sensor_8(0x0A02, 0x01); //fast sleep On
	write_HynixOtp_sensor_8(0x0a00, 0x00); //sleep On
	mdelay(10);
	write_HynixOtp_sensor_8(0x0F02, 0x00); //pll disable
	write_HynixOtp_sensor_8(0x071A, 0x01); //CP TRI_H
	write_HynixOtp_sensor_8(0x071B, 0x09); //IPGM TRIM_H
	write_HynixOtp_sensor_8(0x0D04, 0x01); //Fsync Output enable
	write_HynixOtp_sensor_8(0x0D00, 0x07); //Fsync Output Drivability
	write_HynixOtp_sensor_8(0x003E, 0x10); //OTP R/W
	write_HynixOtp_sensor_8(0x0a00, 0x01); //sleep off
    }


static void Dsiable_Hi843b_OTP_Read(void) 
{
    write_HynixOtp_sensor_8(0x0a00, 0x00); //sleep On
	mdelay(100);
    write_HynixOtp_sensor_8(0x003f, 0x00); //OTP mode off
    write_HynixOtp_sensor_8(0x0a00, 0x01); //sleep Off
}

int read_Hi843b_AF_Info(u8 enable)
{
    int Bank_No = 0 ;     // 1 : Bank 0, 3 : Bank 1, 7 : Bank 2 
    int Start_Add = 0 ;   // OTP Data Read Address
    int Read_Size = 0;    // Read Buffer Size 
    int i = 0;
    int Read_Data = 0;
    u8 temp = 0;
u32    AFinf=0;
u32    AFmacro=0;
    if( OTP_READ_SKIP == enable )
      return 0; // Data Read Skip 

// OTP Flag Read
    write_HynixOtp_sensor_16(0x70a, 0x0CBA); // Set Address 
    write_HynixOtp_sensor_8(0x0702,0x01); // Continous read Mode 
    Bank_No = read_HynixOtp_sensor_8(0x0708);
    CAM_CALDB("read_Hi843b_AF_Info -------> 0x0CBA(8) is 0x%x \n",Bank_No);

    if(Bank_No == 0x77)
        return -1;
    
    Bank_No = Bank_No&0x0f;
// Bank Select 
    switch(Bank_No)
    {
        case 1: Start_Add = 0x0CBC; 
            break ;
        case 3: Start_Add = 0x0CC4; 
            break;
        case 7: Start_Add = 0x0CCC; 
            break;
        default : // Error .. 
            CAM_CALDB("read_Hi843b_LSC_Info -> Bank_No Read ERROR !! \n");
            return -1; 
    }
       
    write_HynixOtp_sensor_16(0x070a, Start_Add); // Set Address 
    write_HynixOtp_sensor_8(0x0702,0x01); // Continous read Mode 
	for(i = 0; i < 4; i++)
    {
        Read_Data = read_HynixOtp_sensor_8(0x0708); // Read Data
        hi843b_otp_data.Data[i+17] = Read_Data;
        CAM_CALDB("read_Hi843b_AF_Info(0x%x) -------> add 0x%x, Read_Data is 0x%x\n",i,Start_Add,Read_Data);
        Start_Add++;
	}  
    temp = hi843b_otp_data.Data[17];
    hi843b_otp_data.Data[17] = hi843b_otp_data.Data[18];
    hi843b_otp_data.Data[18] = temp;
	AFinf=(hi843b_otp_data.Data[18]<<8)+hi843b_otp_data.Data[17];
//CAM_CALDB("read_Hi843b_AF_Info 0 AFinf=%d -------> \n",AFinf);
AFinf=AFinf-50;
//CAM_CALDB("read_Hi843b_AF_Info  1 AFinf=%d -------> \n",AFinf);
        hi843b_otp_data.Data[17]=AFinf&0xff;
	hi843b_otp_data.Data[18]=AFinf>>8&0x0f;
    temp = hi843b_otp_data.Data[19];
    hi843b_otp_data.Data[19] = hi843b_otp_data.Data[20];
    hi843b_otp_data.Data[20] = temp;

	AFmacro=(hi843b_otp_data.Data[20]<<8)+hi843b_otp_data.Data[19];
//CAM_CALDB("read_Hi843b_AF_Info 2 AFmacro=%d -------> \n",AFmacro);
  
AFmacro=AFmacro+50;
//CAM_CALDB("read_Hi843b_AF_Info  3 AFmacro=%d -------> \n",AFmacro);
        hi843b_otp_data.Data[19]=AFmacro&0xff;
	hi843b_otp_data.Data[20]=AFmacro>>8&0x0f;

    return 0;
}

int read_Hi843b_LSC_Info(u8 enable)
{
    int Bank_No = 0 ;     // 1 : Bank 0, 3 : Bank 1, 7 : Bank 2 
    int Start_Add = 0 ;   // OTP Data Read Address
    int Read_Size = 0;  // Read Buffer Size 
    int i = 0;
    int Read_Data = 0; 
    
    if( OTP_READ_SKIP == enable )
      return 0; // Data Read Skip 

// OTP Flag Read
    write_HynixOtp_sensor_16(0x010a, 0x08b5); // Set Address 
    write_HynixOtp_sensor_8(0x0102,0x01); // Continous read Mode 

    Bank_No = read_HynixOtp_sensor_8(0x0108);
    CAM_CALDB("read_Hi843b_LSC_Info -------> 0x08b5(8) is 0x%x \n",Bank_No);

    if(Bank_No == 0x77)
        return -1;
    
    Bank_No = Bank_No&0x0f;

// Bank Select 
    switch(Bank_No)
    {
        case 1: Start_Add = 0x08B6; 
            break ;
        case 3: Start_Add = 0x1005; 
            break;
        case 7: Start_Add = 0x1754; 
            break;
        default : // Error .. 
            CAM_CALDB("read_Hi843b_LSC_Info -> Bank_No Read ERROR !! \n");
            return -1; 
    }
       
    write_HynixOtp_sensor_16(0x010a, Start_Add); // Set Address 
    write_HynixOtp_sensor_8(0x0102,0x01); // Continous read Mode 
	for(i = 0; i < 2; i++)
    {
        Read_Data = read_HynixOtp_sensor_8(0x0108); // Read Data
        hi843b_otp_data.Data[i+21] = Read_Data;

        CAM_CALDB("read_Hi843b_LSC_Info(0x%x) -------> add 0x%x, Read_Data is 0x%x\n",i,Start_Add,Read_Data);
        Start_Add++;
	}

    Read_Size =  (hi843b_otp_data.Data[22]<<8) | (hi843b_otp_data.Data[21]&0xff);

    for(i = 0; i < Read_Size; i++)
    {
        Read_Data = read_HynixOtp_sensor_8(0x0108); // Read Data
        hi843b_otp_data.Data[i+23] = Read_Data;

        CAM_CALDB("read_Hi843b_LSC_Info(0x%x) -------> add 0x%x, Read_Data is 0x%x\n",i,Start_Add,Read_Data);
        Start_Add++;
	}    

    return 0;
}


int read_Hi843b_WB_Info(u8 enable)
{
    int Bank_No = 0 ;     // 1 : Bank 0, 3 : Bank 1, 7 : Bank 2 
    int Start_Add = 0;   // OTP Data Read Address
    int Read_Size = 9;    // Read Buffer Size 
    int i = 0;
    int Read_Data = 0; 
    
    if( OTP_READ_SKIP == enable )
      return 0; // Data Read Skip 

    write_HynixOtp_sensor_16(0x010a, 0x0835); // Set Address 

    write_HynixOtp_sensor_8(0x0102,0x01);     // Continous read Mode 
    //Bank_No = read_HynixOtp_sensor_8(0x0108)&0x0f;

    Bank_No = read_HynixOtp_sensor_8(0x0108);
    CAM_CALDB("read_Hi843b_WB_Info -------> 0x0835(8) is 0x%x \n",Bank_No);

    if(Bank_No == 0x77)
        return -1;
    
    Bank_No = Bank_No&0x0f;

    switch(Bank_No)
    {
        case 1: Start_Add = 0x836;
            break ;
        case 3: Start_Add = 0x854;
            break;
        case 7: Start_Add = 0x872;
            break;
        default : // Error .. 
            CAM_CALDB("read_Hi843b_WB_Info -> Bank_No Read ERROR !! \n");
            return -1; 
    }
    write_HynixOtp_sensor_16(0x010a, Start_Add); // Set Address 
    write_HynixOtp_sensor_8(0x0102,0x01); // Continous read Mode 
	for(i = 0; i < Read_Size; i++)
    {
        Read_Data = read_HynixOtp_sensor_8(0x0108); // Read Data
        hi843b_otp_data.Data[i] = Read_Data;
        CAM_CALDB("read_Hi843b_WB_Info(0x%x) -------> add 0x%x, Read_Data is 0x%x\n",i,Start_Add,Read_Data);
        Start_Add++;
	}


    switch(Bank_No)
    {
        case 1: Start_Add = 0x842;//0x0842; 
            break ;
        case 3: Start_Add = 0x860;//0x0860; 
            break;
        case 7: Start_Add = 0x87E;//0x087E; 
            break;
        default : // Error .. 
            CAM_CALDB("read_Hi843b_WB_Info -> Bank_No Read FAIL !! \n");
            return -1; 
    }
    write_HynixOtp_sensor_16(0x010a, Start_Add); // Set Address 
    write_HynixOtp_sensor_8(0x0102,0x01); // Continous read Mode 
    Read_Size = 8;
	for(i = 0; i < Read_Size; i++)
    {
        Read_Data = read_HynixOtp_sensor_8(0x0108); // Read Data
        hi843b_otp_data.Data[i+9] = Read_Data;
        CAM_CALDB("read_Hi843b_WB_Info(0x%x) -------> add 0x%x, Read_Data is 0x%x\n",i,Start_Add,Read_Data);
        Start_Add++;
	}
    

    return 0;
}

#ifdef Hynix_OTP_File_Load
static int EEPROM_Slim_File_Load(void) 
{ 

        CAM_CALDB("[Hynix_OTP] ==================================\n");

        hi843b_otp_data.Data[0] = 0x01; 
        hi843b_otp_data.Data[1] = 0xFF; 
        hi843b_otp_data.Data[2] = 0x00; 
        hi843b_otp_data.Data[3] = 0x0B; 
        hi843b_otp_data.Data[4] = 0x01; 
        return 1;
} 
#endif 


int read_Hi843b_otp_mtk_fmt(void)
{ 
	int i = 0;
	int offset = 0;
    int rec = 0 ; 
	CAM_CALINF("HYNIX_CAM_CAL OTP readed =%d \n",Hi843b_otp_read);

	if(1 == Hi843b_otp_read ) {
		CAM_CALDB("HYNIX_CAM_CAL OTP readed ! skip\n");
        return 0; 
    }
    
	spin_lock(&g_CAM_CALLock);
	Hi843b_otp_read = 1;
	spin_unlock(&g_CAM_CALLock);

#ifdef Hynix_OTP_File_Load
    if(EEPROM_Slim_File_Load())
#endif 

    Enable_Hi843b_OTP_Read();

    rec |= read_Hi843b_LSC_Info(OTP_READ_SKIP);
    rec |= read_Hi843b_AF_Info(OTP_READ);
    rec |= read_Hi843b_WB_Info(OTP_READ_SKIP);
    
	CAM_CALINF("read_Hi843b_otp_mtk_fmt rec =%d \n",rec);
    if(rec < 0) // If Read Flag Fail.. 
    {
        hi843b_otp_data.Data[0] = 0x00; // No OTP Data 
        hi843b_otp_data.Data[1] = 0x00; // No OTP Data 
        hi843b_otp_data.Data[2] = 0x00; // No OTP Data 
        hi843b_otp_data.Data[3] = 0x00; // No OTP Data 
        hi843b_otp_data.Data[4] = 0x00; // No OTP Data 
    }
    
    Dsiable_Hi843b_OTP_Read();
    
    return 0 ; 

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

static long Hi843botp_Ioctl_Compat(struct file *filp, unsigned int cmd, unsigned long arg)
{
    long ret;
    COMPAT_stCAM_CAL_INFO_STRUCT __user *data32;
    stCAM_CAL_INFO_STRUCT __user *data;
    int err;

	CAM_CALDB("[HYNIX_CAM_CAL] Hi843botp_Ioctl_Compat\n" );

    
	CAM_CALDB("[HYNIX_CAM_CAL] Hi843b_OTP_DEVICE_ID,%p %p %x ioc size %d\n",filp->f_op ,filp->f_op->unlocked_ioctl,cmd,_IOC_SIZE(cmd) );

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
            CAM_CALERR("[HYNIX_CAM_CAL] compat_put_acdk_sensor_getinfo_struct failed\n");
        return ret;
    }
    default:
        return -ENOIOCTLCMD;
    }
}
#endif

static int selective_read_region(u32 offset, BYTE* data,u16 i2c_id,u32 size)
{    
    CAM_CALDB("selective_read_region offset =%x size %d data read = %d\n", offset,size, *data);
    memcpy((void *)data,(void *)&hi843b_otp_data.Data[offset],size);
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

    CAM_CALDB("[HYNIX_CAM_CAL] CAM_CAL_Ioctl\n" );

    if(_IOC_NONE == _IOC_DIR(a_u4Command))
    {
    }
    else
    {
        pBuff = (u8 *)kmalloc(sizeof(stCAM_CAL_INFO_STRUCT),GFP_KERNEL);

        if(NULL == pBuff)
        {
            CAM_CALERR("[HYNIX_CAM_CAL] ioctl allocate mem failed\n");
            return -ENOMEM;
        }

        if(_IOC_WRITE & _IOC_DIR(a_u4Command))
        {
            if(copy_from_user((u8 *) pBuff , (u8 *) a_u4Param, sizeof(stCAM_CAL_INFO_STRUCT)))
            {    //get input structure address
                kfree(pBuff);
                CAM_CALERR("[HYNIX_CAM_CAL]ioctl copy from user failed\n");
                return -EFAULT;
            }
        }
    }

    ptempbuf = (stCAM_CAL_INFO_STRUCT *)pBuff;
    pu1Params = (u8*)kmalloc(ptempbuf->u4Length,GFP_KERNEL);
    if(NULL == pu1Params)
    {
        kfree(pBuff);
        CAM_CALERR("[HYNIX_CAM_CAL]ioctl allocate mem failed\n");
        return -ENOMEM;
    }


    if(copy_from_user((u8*)pu1Params ,  (u8*)ptempbuf->pu1Params, ptempbuf->u4Length))
    {
        kfree(pBuff);
        kfree(pu1Params);
        CAM_CALERR(" [HYNIX_CAM_CAL] ioctl copy from user failed\n");
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
            i4RetValue = selective_read_region(ptempbuf->u4Offset, pu1Params, Hi843B_OTP_DEVICE_ID, ptempbuf->u4Length);

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
      	     CAM_CALINF("[HYNIX_CAM_CAL] No CMD \n");
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
            CAM_CALERR("[HYNIX_CAM_CAL] ioctl copy to user failed\n");
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
    CAM_CALDB("HYNIX_CAM_CAL CAM_CAL_Open\n");
    spin_lock(&g_CAM_CALLock);
    if(g_u4Opened)
    {
        spin_unlock(&g_CAM_CALLock);
		CAM_CALERR("HYNIX_CAM_CAL Opened, return -EBUSY\n");
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
    .compat_ioctl = Hi843botp_Ioctl_Compat,
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
        CAM_CALERR(" HYNIX_CAM_CAL Allocate device no failed\n");

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

        CAM_CALERR(" HYNIX_CAM_CAL Allocate mem for kobject failed\n");

        return -ENOMEM;
    }

    //Attatch file operation.
    cdev_init(g_pCAM_CAL_CharDrv, &g_stCAM_CAL_fops);

    g_pCAM_CAL_CharDrv->owner = THIS_MODULE;

    //Add to system
    if(cdev_add(g_pCAM_CAL_CharDrv, g_CAM_CALdevno, 1))
    {
        CAM_CALERR(" HYNIX_CAM_CAL Attatch file operation failed\n");

        unregister_chrdev_region(g_CAM_CALdevno, 1);

        return -EAGAIN;
    }

    CAM_CAL_class = class_create(THIS_MODULE, "CAM_CALdrv");
    if (IS_ERR(CAM_CAL_class)) {
        int ret = PTR_ERR(CAM_CAL_class);
        CAM_CALERR("HYNIX_CAM_CAL Unable to create class, err = %d\n", ret);
        return ret;
    }
    CAM_CAL_device = device_create(CAM_CAL_class, NULL, g_CAM_CALdevno, NULL, CAM_CAL_DRVNAME);
    CAM_CALDB("HYNIX_CAM_CAL RegisterCAM_CALCharDrv PASSS\n");

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
    .remove	= CAM_CAL_remove,
    .driver		= {
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



