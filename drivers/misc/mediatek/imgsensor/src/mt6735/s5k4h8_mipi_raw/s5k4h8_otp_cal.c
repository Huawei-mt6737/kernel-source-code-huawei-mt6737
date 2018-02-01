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

#include <linux/kernel.h>
#include <linux/init.h> 
#include <linux/module.h> 
#include <linux/syscalls.h> 
#include <linux/fcntl.h> 
#include <asm/uaccess.h> 
#include <linux/types.h> 
#include <linux/slab.h> 

#include "kd_imgsensor_define.h"
//#include "s5k4h8mipiraw_chicony_Sensor.h"
#include "s5k4h8_otp.h"
//
extern int iReadRegI2C(u8 *a_pSendData , u16 a_sizeSendData, u8 *a_pRecvData, u16 a_sizeRecvData, u16 i2cId);
extern int iWriteRegI2C(u8 *a_pSendData, u16 a_sizeSendData, u16 i2cId);

#define PFX "S5K4H8_OTP"
#define LOG_INF(format, args...)	//pr_debug(ANDROID_LOG_INFO   , PFX, "[%s] " format, __FUNCTION__, ##args)

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
#define Delay(ms)  mdelay(ms)

#define CAM_CAL_DRVNAME "S5K4H8_OTP"

static dev_t g_CAM_CALdevno = MKDEV(226,0);
static struct cdev * g_pCAM_CAL_CharDrv = NULL;

static struct class *CAM_CAL_class = NULL;
static atomic_t g_CAM_CALatomic;
#define MAX_LSC_SIZE 1868
#define MAX_OTP_SIZE 2200
typedef struct {
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

OTP_DATA s5k4h8_otp_data = {{0}}; // Hynix Data Structure 

static void write_cmos_sensor_8 (kal_uint16 addr, kal_uint8 para)
{
    char pusendcmd[4] = { (char) (addr >> 8), (char) (addr & 0xFF), (char) (para & 0xFF) };
    iWriteRegI2C (pusendcmd, 3, S5K4H8_DEVICE_ID);
}

static void write_cmos_sensor (kal_uint16 addr, kal_uint16 para)
{
    char pusendcmd[4] = { (char) (addr >> 8), (char) (addr & 0xFF), (char) (para >> 8), (char) (para & 0xFF) };
    iWriteRegI2C (pusendcmd, 4, S5K4H8_DEVICE_ID);
}

static kal_uint16 read_cmos_sensor_8(kal_uint16 addr)
{
    kal_uint16 get_byte = 0;
    char pusendcmd[2] = { (char) (addr >> 8), (char) (addr & 0xFF) };
    //kdSetI2CSpeed (imgsensor_info.i2c_speed);	// Add this func to set i2c speed by each sensor
    iReadRegI2C (pusendcmd, 2, (u8 *) & get_byte, 1, S5K4H8_DEVICE_ID);
    return get_byte;
}

static int read_s5k4h8_AF_info(void)
{
    int i = 0;
    int af_flag = 0;
    int addr;
    u16 AF_otp_data[5];
    u16 infinity_DAC, marco_DAC, start_current_DAC, max_current_DAC,mid_current_DAC;
    //u32 AF_Check_Sum = 0;

    write_cmos_sensor_8 (0x0100, 0x01);	//stream on
    mdelay (10);
    write_cmos_sensor_8 (0x0A02, 0x0F);	//set 15page 
    write_cmos_sensor (0x0A00, 0x0100);	//set start read addr
    af_flag = read_cmos_sensor_8(0x0A35);
    if ((af_flag & 0xC0) == 0x40)
        addr = 0x0A36;		//set base_addr
    else if ((af_flag & 0x30) == 0x10)
        addr = 0x0A3B;

    if (addr != 0) {
        for ( i = 0; i < 5; i++ ) {
            AF_otp_data[i] = read_cmos_sensor_8(addr + i);
        }
    }

    if (AF_otp_data[4] == (AF_otp_data[0] + AF_otp_data[1] + AF_otp_data[2] + AF_otp_data[3]) % 255 + 1) 
        CAM_CALDB("s5k4h8 AF checksum success\n");
    else 
        CAM_CALDB("s5k4h8 AF checksum fail\n");

    infinity_DAC = (AF_otp_data[0] << 8) | AF_otp_data[1];
    marco_DAC = (AF_otp_data[2] << 8) | AF_otp_data[3];
    CAM_CALDB("infinity_DAC=%u, marco_DAC=%u\n", infinity_DAC, marco_DAC);
    s5k4h8_otp_data.Data[17] = AF_otp_data[1];
    s5k4h8_otp_data.Data[18] = AF_otp_data[0];
    s5k4h8_otp_data.Data[19] = AF_otp_data[3];
    s5k4h8_otp_data.Data[20] = AF_otp_data[2];

    return 0;
}


static int read_otp(struct otp_struct *otp_ptr)
{
    struct timeval ktv1, ktv2;
    unsigned long TimeIntervalUS;
   // int i = 0;
   // int offset = 0;			
    do_gettimeofday(&ktv1);

    s5k4h8_otp_data.Data[0] = 0x01;
    s5k4h8_otp_data.Data[1] = 0x63;
    s5k4h8_otp_data.Data[2] = 0x69;
    s5k4h8_otp_data.Data[3] = 0x68;
    s5k4h8_otp_data.Data[4] = 0x63;
    read_s5k4h8_AF_info();

    do_gettimeofday(&ktv2);
    if(ktv2.tv_sec > ktv1.tv_sec)
        TimeIntervalUS = ktv1.tv_usec + 1000000 - ktv2.tv_usec;
    else
        TimeIntervalUS = ktv2.tv_usec - ktv1.tv_usec;
    CAM_CALDB("s5k4h8 otp read data take %lu us\n", TimeIntervalUS);

    return 1;
}


void s5k4h8_otp_cali(void)
{
	struct otp_struct current_otp;
	memset(&current_otp, 0, sizeof(struct otp_struct));
	read_otp(&current_otp);
}




/********************** char driver ***********************/
#ifdef CONFIG_COMPAT
static int compat_put_cal_info_struct(
            COMPAT_stCAM_CAL_INFO_STRUCT __user *data32,
            stCAM_CAL_INFO_STRUCT __user *data)
{
    compat_uint_t p;
    compat_uint_t i;
    int err;

    err = get_user(i, &data->u4Offset);
    err |= put_user(i, &data32->u4Offset);
    err |= get_user(i, &data->u4Length);
    err |= put_user(i, &data32->u4Length);
    /* Assume pointer is not change */
#if 1
    err |= get_user(p, (&(data->u4Offset)+8));//err |= get_user(p, &(data->pu1Params)); by likai change
    err |= put_user(p, &(data32->pu1Params));
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

static long S5k4h8otp_Ioctl_Compat(struct file *filp, unsigned int cmd, unsigned long arg)
{
    long ret;
    COMPAT_stCAM_CAL_INFO_STRUCT __user *data32;
    stCAM_CAL_INFO_STRUCT __user *data;
    int err;

	CAM_CALDB("[S5K4h8_ST_CAM_CAL] S5k4h8otp_Ioctl_Compat\n" );

    
	CAM_CALDB("[S5K4h8_ST_CAM_CAL] S5k4h8_OTP_DEVICE_ID,%p %p %x ioc size %d\n",filp->f_op ,filp->f_op->unlocked_ioctl,cmd,_IOC_SIZE(cmd) );

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
            CAM_CALERR("[S5K4h8_ST_CAM_CAL] compat_put_acdk_sensor_getinfo_struct failed\n");
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
    memcpy((void *)data,(void *)&s5k4h8_otp_data.Data[offset],size);
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

    CAM_CALDB("[S5K4h8_ST_CAM_CAL] CAM_CAL_Ioctl\n" );

    if(_IOC_NONE == _IOC_DIR(a_u4Command))
    {
    }
    else
    {
        pBuff = (u8 *)kmalloc(sizeof(stCAM_CAL_INFO_STRUCT),GFP_KERNEL);

        if(NULL == pBuff)
        {
            CAM_CALERR("[S5K4h8_ST_CAM_CAL] ioctl allocate mem failed\n");
            return -ENOMEM;
        }

        if(_IOC_WRITE & _IOC_DIR(a_u4Command))
        {
            if(copy_from_user((u8 *) pBuff , (u8 *) a_u4Param, sizeof(stCAM_CAL_INFO_STRUCT)))
            {    //get input structure address
                kfree(pBuff);
                CAM_CALERR("[S5K4h8_ST_CAM_CAL]ioctl copy from user failed\n");
                return -EFAULT;
            }
        }
    }

    ptempbuf = (stCAM_CAL_INFO_STRUCT *)pBuff;
    pu1Params = (u8*)kmalloc(ptempbuf->u4Length,GFP_KERNEL);
    if(NULL == pu1Params)
    {
        kfree(pBuff);
        CAM_CALERR("[S5K4h8_ST_CAM_CAL]ioctl allocate mem failed\n");
        return -ENOMEM;
    }


    if(copy_from_user((u8*)pu1Params ,  (u8*)ptempbuf->pu1Params, ptempbuf->u4Length))
    {
        kfree(pBuff);
        kfree(pu1Params);
        CAM_CALERR(" [S5K4h8_ST_CAM_CAL] ioctl copy from user failed\n");
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
            CAM_CALDB("[S5K4h8_ST_CAM_CAL] Read CMD \n");
#ifdef CAM_CALGETDLT_DEBUG
            do_gettimeofday(&ktv1);
#endif
            i4RetValue = selective_read_region(ptempbuf->u4Offset, pu1Params, S5K4H8_DEVICE_ID, ptempbuf->u4Length);

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
            CAM_CALDB("[S5K4h8_ST_CAM_CAL] Read data %d bytes take %lu us\n",ptempbuf->u4Length, TimeIntervalUS);
#endif

            break;
        default :
      	     CAM_CALINF("[S5K4h8_ST_CAM_CAL] No CMD \n");
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
            CAM_CALERR("[S5K4h8_ST_CAM_CAL] ioctl copy to user failed\n");
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
    CAM_CALDB("S5K4h8_ST_CAM_CAL CAM_CAL_Open\n");
    spin_lock(&g_CAM_CALLock);
    if(g_u4Opened)
    {
        spin_unlock(&g_CAM_CALLock);
		CAM_CALERR("S5K4h8_ST_CAM_CAL Opened, return -EBUSY\n");
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
    .compat_ioctl = S5k4h8otp_Ioctl_Compat,
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
        CAM_CALERR(" S5K4h8_ST_CAM_CAL Allocate device no failed\n");

        return -EAGAIN;
    }
#else
    if( register_chrdev_region(  g_CAM_CALdevno , 1 , CAM_CAL_DRVNAME) )
    {
        CAM_CALERR(" S5K4h8_ST_CAM_CAL Register device no failed\n");

        return -EAGAIN;
    }
#endif

    //Allocate driver
    g_pCAM_CAL_CharDrv = cdev_alloc();

    if(NULL == g_pCAM_CAL_CharDrv)
    {
        unregister_chrdev_region(g_CAM_CALdevno, 1);

        CAM_CALERR(" S5K4h8_ST_CAM_CAL Allocate mem for kobject failed\n");

        return -ENOMEM;
    }

    //Attatch file operation.
    cdev_init(g_pCAM_CAL_CharDrv, &g_stCAM_CAL_fops);

    g_pCAM_CAL_CharDrv->owner = THIS_MODULE;

    //Add to system
    if(cdev_add(g_pCAM_CAL_CharDrv, g_CAM_CALdevno, 1))
    {
        CAM_CALERR(" S5K4h8_ST_CAM_CAL Attatch file operation failed\n");

        unregister_chrdev_region(g_CAM_CALdevno, 1);

        return -EAGAIN;
    }

    CAM_CAL_class = class_create(THIS_MODULE, CAM_CAL_DRVNAME);
    if (IS_ERR(CAM_CAL_class)) {
        int ret = PTR_ERR(CAM_CAL_class);
        CAM_CALERR("S5K4h8_ST_CAM_CAL Unable to create class, err = %d\n", ret);
        return ret;
    }
    CAM_CAL_device = device_create(CAM_CAL_class, NULL, g_CAM_CALdevno, NULL, CAM_CAL_DRVNAME);
    CAM_CALDB("S5K4h8_ST_CAM_CAL RegisterCAM_CALCharDrv PASSS\n");

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
    CAM_CALDB("S5K4h8_ST_CAM_CAL_i2C_init\n");
   //Register char driver
	i4RetValue = RegisterCAM_CALCharDrv();
    if(i4RetValue){
 	   CAM_CALDB(" register char device failed!\n");
	   return i4RetValue;
	}
	CAM_CALDB(" S5K4h8_ST_CAM_CAL Attached!! \n");

  //  i2c_register_board_info(CAM_CAL_I2C_BUSNUM, &kd_cam_cal_dev, 1);
    if(platform_driver_register(&g_stCAM_CAL_Driver)){
        CAM_CALERR("S5K4h8_ST_CAM_CAL failed to register s5k4h8 driver\n");
        return -ENODEV;
    }

    if (platform_device_register(&g_stCAM_CAL_Device))
    {
        CAM_CALERR("S5K4h8_ST_CAM_CAL failed to register s5k4h8 driver, 2nd time\n");
        return -ENODEV;
    }
    CAM_CALDB("S5K4h8_ST_CAM_CAL_i2C_init PASS\n");

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
