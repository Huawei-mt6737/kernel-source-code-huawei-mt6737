/*
 *
 * FocalTech TouchScreen driver.
 * 
 * Copyright (c) 2010-2015, Focaltech Ltd. All rights reserved.
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

 /*******************************************************************************
*
* File Name: Focaltech_Gestrue.c
*
* Author: Xu YongFeng
*
* Created: 2015-01-29
*   
* Modify by mshl on 2015-03-20
*
* Abstract:
*
* Reference:
*
*******************************************************************************/

#include "focaltech_core.h"

#if FTS_GESTRUE_EN
#include "ft_gesture_lib.h"
#include <linux/kobject.h>
#include <linux/sysfs.h>
#include<linux/wakelock.h>
#include "tpd.h"
#include "../../../../drivers/base/base.h"


#define TPD_PACKET_LENGTH                   128
#define PROC_READ_DRAWDATA					10

#define GESTURE_LEFT								0x20
#define GESTURE_RIGHT								0x21
#define GESTURE_UP		    						0x22
#define GESTURE_DOWN								0x23
#define GESTURE_DOUBLECLICK						0x24
#define GESTURE_O		    						0x30
#define GESTURE_W		    						0x31
#define GESTURE_M		   	 						0x32
#define GESTURE_E		    						0x33
#define GESTURE_L		    						0x44
#define GESTURE_S		    						0x46
#define GESTURE_V		    						0x54
#define GESTURE_Z		    						0x65
#define GESTURE_C		    					   	0x34
#define FTS_GESTRUE_POINTS 						255
#define FTS_GESTRUE_POINTS_ONETIME  				62
#define FTS_GESTRUE_POINTS_HEADER 				8
#define FTS_GESTURE_OUTPUT_ADRESS 				0xD3
#define FTS_GESTURE_OUTPUT_UNIT_LENGTH 			4
#define PROC_UPGRADE								0
#define TS_WAKE_LOCK_TIMEOUT		(2 * HZ)
unsigned short gesture_id;

unsigned short Drawdata[15] = {0};
extern struct wake_lock gesture_chrg_lock;

//static unsigned char proc_operate_mode = PROC_UPGRADE;

unsigned char buf[FTS_GESTRUE_POINTS * 2] ;
static unsigned char gesture_point_readbuf[TPD_PACKET_LENGTH];
//static struct i2c_client *i2c_client;//client01

short pointnum = 0;

unsigned short coordinate_x[255] = {0};
unsigned short coordinate_y[255] = {0};

static struct proc_dir_entry *proc_ctp_gesture_dir=NULL;
		
static struct gesture_debug_entry
{
	char * name;
	struct file_operations proc_operations;
};

int ft_wakeup_gesture = 0;

static long huawei_gesture = 0;//huawei ui gesture command
static int double_gesture = 0;
static int draw_gesture = 0;

static int gesture_echo[12]={0};

static u32 gesture_start_point_x,gesture_start_point_y;
static u32 gesture_end_point_x,gesture_end_point_y;
static u32 gesture_width,gesture_height;

static ssize_t gesture_switch_read_proc(struct file *file, char __user *page, size_t size, loff_t *ppos)
{
	char *ptr = page;
	if (*ppos)  // CMD call again
		return 0;

	ptr += sprintf(ptr, "%d\n", ft_wakeup_gesture);
	*ppos += ptr - page;

	printk(KERN_ERR "mtk-tpd-gesture: huawei_gesture = %d\n",ft_wakeup_gesture);
	return (ptr - page);
}

static ssize_t gesture_switch_write_proc(struct file *file, char __user *buff, size_t size, loff_t *ppos)
{
	char wtire_data[32] = {0};

	if (size >= 32)
		return -EFAULT;

	if (copy_from_user( &wtire_data, buff, size ))
		return -EFAULT;

	if (wtire_data[0] == '1')
		ft_wakeup_gesture = 1;
	else
		ft_wakeup_gesture = 0;

	printk(KERN_ERR "mtk-tpd-gesture: huawei_gesture = %d\n",ft_wakeup_gesture);

	return size;
}

static ssize_t huawei_gesture_switch_read_proc(struct file *file, char __user *page, size_t size, loff_t *ppos)
{
	char *ptr = page;
	if (*ppos)  // CMD call again
		return 0;

	ptr += sprintf(ptr, "%ld\n", huawei_gesture);
	*ppos += ptr - page;

	printk(KERN_ERR "mtk-tpd-gesture: huawei_gesture = %ld\n",huawei_gesture);
	return (ptr - page);
}

static ssize_t huawei_gesture_switch_write_proc(struct file *file, char __user *buff, size_t size, loff_t *ppos)
{
	char wtire_data[32] = {0};

	if (size >= 32)
		return -EFAULT;

	if (copy_from_user( &wtire_data, buff, size ))
		return -EFAULT;

	huawei_gesture= simple_strtoul(wtire_data,NULL,10);

	if((huawei_gesture & 1) || (huawei_gesture & 0x2000)){
		ft_wakeup_gesture = 1;
	}else{
		ft_wakeup_gesture = 0;
	}
	
	if(huawei_gesture & 1)
		double_gesture = 1;
	else
		double_gesture = 0;

	if(huawei_gesture & 0x2000)
		draw_gesture = 1;
	else
		draw_gesture = 0;

	printk(KERN_ERR "mtk-tpd-gesture: ft_wakeup_gesture = %d huawei_gesture = %ld double_gesture = %d draw_gesture = %d\n", ft_wakeup_gesture,huawei_gesture,double_gesture,draw_gesture);
	
	return size;
}

static int ft3x27_echo_read_proc(struct seq_file *m, void *v)
{
	int i;
	for(i=0;i<=11;i++){
		seq_printf(m, "%04x",gesture_echo[i]);
	}
	seq_printf(m, "\n");
	memset(gesture_echo,0,sizeof(gesture_echo));
	return 0;
}
static int ft3x27_open(struct inode *inode, struct file *file)
{
	return single_open(file, ft3x27_echo_read_proc, NULL);
}

static struct gesture_debug_entry ctp_gesture_proc_entry[]=
{
	{
		.name="gesture_switch",	
		{
			.owner = THIS_MODULE,
			.read  = gesture_switch_read_proc,
			.write = gesture_switch_write_proc,
		}
	},
	{
		.name="g_huawei_control",	
		{
			.owner = THIS_MODULE,
			.read  = huawei_gesture_switch_read_proc,
			.write = huawei_gesture_switch_write_proc,
		}
	},
	{
		.name="gesture_echo",	
		{
			.owner = THIS_MODULE,
			.open  = ft3x27_open,
			.read  = seq_read,
			.write = NULL,
		}
	},
};


static int proc_ctp_gesture_add(struct gesture_debug_entry gesture_debug[],int size)
{
	//struct proc_dir_entry *proc_dir,*proc_entry;
	int i=0;
	proc_ctp_gesture_dir = proc_mkdir("ctp_gesture", NULL);
	if (proc_ctp_gesture_dir == NULL)
	{
	    printk(KERN_ERR "create ctp_gesture/ error !\n");
			return -1;
	}
	for(i=0;i<size;i++) 
	{
		proc_create(gesture_debug[i].name, 0664, proc_ctp_gesture_dir, &gesture_debug[i].proc_operations);
	}
	
	return 0;
}

//-----------------------------------------------------------------------------------------

int fts_Gesture_init(struct input_dev *input_dev)
{
	//ft_wakeup_gesture = 1;

	input_set_capability(tpd->dev, EV_KEY, KEY_POWER);
	input_set_capability(tpd->dev, EV_KEY, KEY_F1);
	input_set_capability(tpd->dev, EV_KEY, KEY_F8);
	input_set_capability(tpd->dev, EV_KEY, KEY_F9);
	input_set_capability(tpd->dev, EV_KEY, KEY_F10);
	input_set_capability(tpd->dev, EV_KEY, KEY_F11);

	__set_bit(KEY_F1, tpd->dev->keybit);
	__set_bit(KEY_F8, tpd->dev->keybit);
	__set_bit(KEY_F9, tpd->dev->keybit);
	__set_bit(KEY_F10, tpd->dev->keybit);
	__set_bit(KEY_F11, tpd->dev->keybit);

	__set_bit(EV_KEY, tpd->dev->evbit);
	__set_bit(EV_SYN, tpd->dev->evbit);

	proc_ctp_gesture_add(ctp_gesture_proc_entry, sizeof(ctp_gesture_proc_entry)/sizeof(struct gesture_debug_entry));
	
	return 0;
}

void fts_Gesture_exit(void)
{
	//ft_gesture_node_uninit();
}

void fts_check_gesture(int gesture_id)
{
   printk("FT3327 gesture_id==0x%x\n ",gesture_id);
   switch(gesture_id)
   {
	   case GESTURE_DOUBLECLICK:
		   if(double_gesture){
		   	 	/*if((gesture_start_point_x > 116) && (gesture_end_point_x > 116)\
					&& (gesture_start_point_x < 604) && (gesture_end_point_x < 604)\
					&& (gesture_start_point_y > 123) && (gesture_end_point_y > 123)\
					&& (gesture_start_point_y < 1157) && (gesture_end_point_y < 1157))
		   	 	{*/
				   input_report_key(tpd->dev, KEY_F1, 1);//KEY_F1
				   //input_sync(tpd->dev);
				   input_report_key(tpd->dev, KEY_F1, 0);
				   input_sync(tpd->dev);
				   //gesture_reval = "KEY_F1";
				   printk("\n gesture double click \n");
		   	 	//}
			}
		   break;
	   case GESTURE_C:
		   if(draw_gesture){
		   		//if((gesture_width > 116) && (gesture_height > 123))
		   	 	//{
				   input_report_key(tpd->dev, KEY_F8, 1);//KEY_F8
				   //input_sync(tpd->dev);
				   input_report_key(tpd->dev, KEY_F8, 0);
				   input_sync(tpd->dev);
				   //gesture_reval = "KEY_F8";
				   printk("\n gesture click c \n");
		   		//}
			}
		   break;
	   case GESTURE_E:
		   if(draw_gesture){
			  // if((gesture_width > 116) && (gesture_height > 123))
			   //{
				   input_report_key(tpd->dev, KEY_F9, 1);
				   //input_sync(tpd->dev);
				   input_report_key(tpd->dev, KEY_F9, 0);
				   input_sync(tpd->dev);
				   //gesture_reval = "KEY_F9";
				   printk("\n gesture click e \n");
			   //	}
			}
		   break;

	   case GESTURE_M:
			if(draw_gesture){
		   		//if((gesture_width > 116) && (gesture_height > 123))
		   	 	//{
				   input_report_key(tpd->dev, KEY_F10, 1);
				   //input_sync(tpd->dev);
				   input_report_key(tpd->dev, KEY_F10, 0);
				   input_sync(tpd->dev);
				   //gesture_reval = "KEY_F10";
				   printk("\n gesture click m \n");
		   		//}
			}
		   break;
		   
	   case GESTURE_W:
			if(draw_gesture){
		   		//if((gesture_width > 116) && (gesture_height > 123))
		   	 	//{
				   input_report_key(tpd->dev, KEY_F11, 1);
				   //input_sync(tpd->dev);
				   input_report_key(tpd->dev, KEY_F11, 0);
				   input_sync(tpd->dev);
				   //gesture_reval = "KEY_F11";
				   printk("\n gesture click w \n");
		   		//}
			}
		   break;
	   default:
		   break;
   }
}

/************************************************************************
* Name: fts_read_Gestruedata
* Brief: read data from TP register
* Input: no
* Output: no
* Return: fail <0
***********************************************************************/
int fts_read_Gestruedata(void)	
{
	unsigned char buf[FTS_GESTRUE_POINTS * 4+2+36] = { 0 };
	//unsigned char buf[FTS_GESTRUE_POINTS * 2] = { 0 }; 
	int ret = -1;
	int i = 0;
	buf[0] = 0xd3;
	int gestrue_id = 0;
	u8 uc_tp_fm_ver;
	short pointnum = 0;
	pointnum = 0;
	short feature_codes = 0x1111;
	/*u32 gesture_start_point_x,gesture_start_point_y;
	u32 gesture_end_point_x,gesture_end_point_y;*/
	u32 gesture_p1_point_x,gesture_p1_point_y;
	u32 gesture_p2_point_x,gesture_p2_point_y;
	u32 gesture_p3_point_x,gesture_p3_point_y;
	u32 gesture_p4_point_x,gesture_p4_point_y;
	ret = fts_i2c_read(i2c_client, buf, 1, buf, FTS_GESTRUE_POINTS_HEADER);
#if 1//FAE debug
	for(i=0;i<8;i++)
	{
		printk("[FTS]buf[%d]=%d\t",i,buf[i]);
	}
	fts_read_reg(i2c_client, FTS_REG_FW_VER, &uc_tp_fm_ver);
	printk("\n[FTS]Reg 0xa6 =%d \n",uc_tp_fm_ver);
#endif
	//ret = i2c_smbus_read_i2c_block_data(i2c_client, 0xd3, FTS_GESTRUE_POINTS_HEADER, buf);
	//i2c_smbus_write_i2c_block_data(i2c_client, 0xd3, 0, &buf);
	//i2c_smbus_read_i2c_block_data(i2c_client, 0xd3, FTS_GESTRUE_POINTS_HEADER, &buf);
	if (ret < 0)
	{
		printk( "%s read touchdata failed.\n", __func__);
		return ret;
	}
	//printk("ft5x0x_read_Touchdata buf[0]=%x \n",buf[0]);
	/* FW */

	if(buf[0]!=0xfe)
	{
		  gestrue_id =	buf[0];
		  //check_gesture(gestrue_id);
	}

	pointnum = (short)(buf[1]) & 0xff;
	buf[0] = 0xd3;
	if((pointnum * 4 + 2 +36)<255) //  + 6
	{
		ret = fts_i2c_read(i2c_client, buf, 1, buf, (pointnum * 4 + 2 + 36)); //  + 6
	} else {
		ret = fts_i2c_read(i2c_client, buf, 1, buf, 255);
		ret = fts_i2c_read(i2c_client, buf, 0, buf+255, (pointnum * 4 + 2 + 36)-255); // + 6
	}
	if (ret < 0)
	{
		printk( "%s read touchdata failed.\n", __func__);
		return ret;
	}
	
	for(i=0;i<34;i++)
	{
		feature_codes = (buf[2*i]<<8)|buf[2*i+1]; //	+ 6
		printk("0x%X ", feature_codes);
	}  
	printk("0x%X  wangyang the pointnum = %d ", feature_codes,pointnum);
	gesture_start_point_x	= (buf[2]<<8)|buf[2+1];
	gesture_start_point_y  = (buf[2+2] << 8)|buf[2+3];
	printk("gesture_start_point_x =0x%X, gesture_start_point_y =0x%X\n", gesture_start_point_x,gesture_start_point_y);
	gesture_end_point_x   = (buf[2+4] << 8)|buf[2+5];
	gesture_end_point_y  = (buf[2+6] << 8)|buf[2+7];
	printk("gesture_end_point_x = 0x%X, gesture_end_point_y = 0x%X\n", gesture_end_point_x,gesture_end_point_y);
	gesture_width   = (buf[2+8] << 8)|buf[2+9];
	gesture_height  = (buf[2+10] << 8)|buf[2+11];
	printk("gesture_width = 0x%X, gesture_height = 0x%X\n", gesture_width,gesture_height);
	gesture_p1_point_x	 = (buf[2+16] << 8)|buf[2+17];
	gesture_p1_point_y	 = (buf[2+18] << 8)|buf[2+19];
	printk("gesture_p1_point_x = 0x%X, gesture_p1_point_y = 0x%X\n", gesture_p1_point_x,gesture_p1_point_y);
	gesture_p2_point_x	 = (buf[2+20] << 8)|buf[2+21];
	gesture_p2_point_y	 = (buf[2+22] << 8)|buf[2+23];
	printk("gesture_p2_point_x = 0x%X, gesture_p2_point_y = 0x%X\n", gesture_p2_point_x,gesture_p2_point_y);
	gesture_p3_point_x	 = (buf[2+24] << 8)|buf[2+25];
	gesture_p3_point_y	 = (buf[2+26] << 8)|buf[2+27];
	printk("gesture_p3_point_x = 0x%X, gesture_p3_point_y = 0x%X\n", gesture_p3_point_x,gesture_p3_point_y);
	gesture_p4_point_x	 = (buf[2+28] << 8)|buf[2+29];
	gesture_p4_point_y	 = (buf[2+30] << 8)|buf[2+31];
	printk("gesture_p4_point_x = 0x%X, gesture_p4_point_y = 0x%X\n", gesture_p4_point_x,gesture_p4_point_y);
	
	gesture_echo[0]  = gesture_start_point_x;
	gesture_echo[1]  = gesture_start_point_y;	
	gesture_echo[2]  = gesture_end_point_x;
	gesture_echo[3]  = gesture_end_point_y; 	
	gesture_echo[4]  = gesture_p1_point_x;
	gesture_echo[5] = gesture_p1_point_y;	
	gesture_echo[6] = gesture_p2_point_x;
	gesture_echo[7] = gesture_p2_point_y;	
	gesture_echo[8] = gesture_p3_point_x;
	gesture_echo[9] = gesture_p3_point_y;	
	gesture_echo[10] = gesture_p4_point_x;
	gesture_echo[11] = gesture_p4_point_y;	
	
	fts_check_gesture(gestrue_id);
	
	return -1;
}

#if 0
int fts_read_Gestruedata(void)        //read data from fw.Drawdata[i] sent to mobile************************************************
{
    //unsigned char buf[FTS_GESTRUE_POINTS * 3] = { 0 };
    int ret = -1;
    //int i = 0;
    int gesture_id = 0;

    buf[0] = 0xd3;
    pointnum = 0;

    ret = fts_i2c_read(fts_i2c_client, buf, 1, buf, FTS_GESTRUE_POINTS_HEADER);
        //printk( "tpd read FTS_GESTRUE_POINTS_HEADER.\n");

    if (ret < 0)
    {
        printk( "%s read touchdata failed.\n", __func__);
        return ret;
    }
#if 0
    /* FW */
     if (fts_updateinfo_curr.CHIP_ID==0x54 || fts_updateinfo_curr.CHIP_ID==0x58 || fts_updateinfo_curr.CHIP_ID==0x86 || fts_updateinfo_curr.CHIP_ID==0x87  || fts_updateinfo_curr.CHIP_ID == 0x64)
     {
	 gesture_id = buf[0];
	 pointnum = (short)(buf[1]) & 0xff;
	 buf[0] = 0xd3;

	 if((pointnum * 4 + 2)<255)
	 {
	    	 ret = fts_i2c_read(fts_i2c_client, buf, 1, buf, (pointnum * 4 + 2));
	 }
	 else
	 {
	        ret = fts_i2c_read(fts_i2c_client, buf, 1, buf, 255);
	        ret = fts_i2c_read(fts_i2c_client, buf, 0, buf+255, (pointnum * 4 + 2) -255);
	 }
	 if (ret < 0)
	 {
	       printk( "%s read touchdata failed.\n", __func__);
	       return ret;
	 }

	 fts_check_gesture(fts_input_dev,gesture_id);
	 for(i = 0;i < pointnum;i++)
	 {
	    	coordinate_x[i] =  (((s16) buf[0 + (4 * i+2)]) & 0x0F) <<
	        	8 | (((s16) buf[1 + (4 * i+2)])& 0xFF);
	    	coordinate_y[i] = (((s16) buf[2 + (4 * i+2)]) & 0x0F) <<
	        	8 | (((s16) buf[3 + (4 * i+2)]) & 0xFF);
	 }
	 return -1;
   }
	 #endif
	// other IC's gestrue in driver
	if (0x24 == buf[0])                                      //check is double click or not*********************************************************
	{
	    	gesture_id = 0x24;
	    	fts_check_gesture(fts_input_dev,gesture_id);
		printk( "%d check_gesture gesture_id.\n", gesture_id);
	    return -1;
	}
	gesture_id = buf[0];
	//pointnum = (short)(buf[1]) & 0xff;
	pointnum = 7;
	buf[0] = 0xd3;
	/*if((pointnum * 4 + 8)<255)
	{
		ret = fts_i2c_read(fts_i2c_client, buf, 1, buf, (pointnum * 4 + 8));
	}
	else
	{
	     ret = fts_i2c_read(fts_i2c_client, buf, 1, buf, 255);
	     ret = fts_i2c_read(fts_i2c_client, buf, 0, buf+255, (pointnum * 4 + 8) -255);
	}
	if (ret < 0)
	{
	    printk( "%s read touchdata failed.\n", __func__);
	    return ret;
	}

	//gesture_id = fetch_object_sample(buf, pointnum);//need gesture lib.a
	gesture_id = 0x24;
	*/
	ret = fts_i2c_read(fts_i2c_client, buf, 1, buf, (pointnum * 4 + 2 +6));
	if (ret < 0)
	{
		printk( "%s read touchdata failed.\n", __func__);
		return ret;
	}

	fts_check_gesture(fts_input_dev,gesture_id);
#if 0
	{
	printk( "%d read gesture_id.\n", gesture_id);
			// add debug info
		

		printk("Debug gestrue_id = %d \n",gesture_id);
		
		for (i = 0; i < 14; i++)
		{
                    Drawdata[i] = (unsigned short)(((buf[1 + i * 2] & 0xFF) << 8) | (buf[1+1 + i * 2] & 0xFF));
		}

		for (i = 0; i < 14; i++)
		{
                    printk("%d ,", Drawdata[i]);
		}

		printk("Debug end \n");

	/*
	for(i = 0;i < pointnum;i++)
	{
	    coordinate_x[i] =  (((s16) buf[0 + (4 * i+8)]) & 0x0F) <<
	        8 | (((s16) buf[1 + (4 * i+8)])& 0xFF);
	    coordinate_y[i] = (((s16) buf[2 + (4 * i+8)]) & 0x0F) <<
	        8 | (((s16) buf[3 + (4 * i+8)]) & 0xFF);
	}
	*/
#endif
	return -1;
}
#endif

#endif
