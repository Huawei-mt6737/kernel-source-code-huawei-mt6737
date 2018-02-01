/* drivers/input/touchscreen/mediatek/gt9xx_mtk/gtp_extents.c
 *
 * Copyright  (C)  2010 - 2016 Goodix., Ltd.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be a reference
 * to you, when you are integrating the GOODiX's CTP IC into your system,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * Version: V2.6
 */

#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/sched.h>
#include <linux/kthread.h>
#include <linux/wait.h>
#include <linux/time.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/miscdevice.h>
#include <linux/input.h>
#include <asm/uaccess.h>
#include <linux/proc_fs.h>
#include <asm/ioctl.h>
#include "tpd.h"
#include "include/tpd_gt9xx_common.h"
#include "../../../../drivers/base/base.h"

#ifdef CONFIG_GTP_GESTURE_WAKEUP

#define GESTURE_NODE "goodix_gesture"
#define GTP_REG_WAKEUP_GESTURE     		0x814B
#define GTP_REG_WAKEUP_GESTURE_DETAIL	0x9420

#define SETBIT(longlong, bit)   (longlong[bit/8] |=  (1 << bit%8))
#define CLEARBIT(longlong, bit) (longlong[bit/8] &=(~(1 << bit%8)))
#define QUERYBIT(longlong, bit) (!!(longlong[bit/8] & (1 << bit%8)))

static u8 gestures_flag[32];
static struct mutex gesture_data_mutex;

//-------------------------------------------------------
extern struct i2c_client *i2c_client_point;
struct gesture_data gesture_data;

static struct proc_dir_entry *proc_ctp_gesture_dir=NULL;

static struct gesture_debug_entry{
	char * name;
	struct file_operations proc_operations;
};

static long huawei_gesture = 0;//huawei ui gesture command
int g_wakeup_gesture = 0;//gloable gesture switch

static int double_gesture = 0;
static int draw_gesture = 0;

int gesture_echo[20]={0};

static u32 gesture_start_point_x,gesture_start_point_y;
static u32 gesture_end_point_x,gesture_end_point_y;
static u32 gesture_width,gesture_height;

static ssize_t gesture_switch_read_proc(struct file *file, char __user *page, size_t size, loff_t *ppos)
{
	char *ptr = page;
	if (*ppos)  // CMD call again
		return 0;

	ptr += sprintf(ptr, "%d\n", g_wakeup_gesture);
	*ppos += ptr - page;

	printk(KERN_ERR "mtk-tpd-gesture: g_wakeup_gesture = %d\n",g_wakeup_gesture);
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
		g_wakeup_gesture = 1;
	else
		g_wakeup_gesture = 0;

	printk(KERN_ERR "mtk-tpd-gesture: g_wakeup_gesture = %d\n",g_wakeup_gesture);
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
		g_wakeup_gesture = 1;
	}else{
		g_wakeup_gesture = 0;
	}
	
	if(huawei_gesture & 1)
		double_gesture = 1;
	else
		double_gesture = 0;

	if(huawei_gesture & 0x2000)
		draw_gesture = 1;
	else
		draw_gesture = 0;

	printk(KERN_ERR "mtk-tpd-gesture: g_wakeup_gesture = %d huawei_gesture = %ld double_gesture = %d draw_gesture = %d\n", g_wakeup_gesture,huawei_gesture,double_gesture,draw_gesture);
	
	return size;
}

static int gt9xx_echo_read_proc(struct seq_file *m, void *v)
{
	int i;
	for(i=0;i<=11;i++){
		seq_printf(m, "%04x",gesture_echo[i]);
	}
	seq_printf(m, "\n");
	memset(gesture_echo,0,sizeof(gesture_echo));
	return 0;
}
static int gt9xx_open(struct inode *inode, struct file *file)
{
	return single_open(file, gt9xx_echo_read_proc, NULL);
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
			.open  = gt9xx_open,
			.read  = seq_read,
			.write = NULL,
		}
	},
};

static int proc_ctp_gesture_add(struct gesture_debug_entry gesture_debug[],int size)
{
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

//proc_hq_dbg_add(ctp_gesture_proc_entry, sizeof(ctp_gesture_proc_entry)/sizeof(struct gesture_debug_entry));

#if 0
static void gtp_check_gesture(struct input_dev *input_dev,int gesture_id)
{
	printk("[GTP]Goodix gesture_id==0x%x\n ",gesture_id);
	
	printk("[GTP]gesture_button= 0x%x\n",gesture_button);

	if ( gesture_button == 1 ) {
		switch(gesture_id) {
			
				case 0xBB:
						printk("[GTP]Gesture is KEY_GESTURE_LEFT");
						key_gesture = SWIPE_X_LEFT;
						input_report_key(input_dev, KEY_GESTURE, 1);
						input_sync(input_dev);
						input_report_key(input_dev, KEY_GESTURE, 0);
						input_sync(input_dev);
						break;
				case 0xAA:
						printk("[GTP]Gesture is KEY_GESTURE_RIGHT");
						key_gesture = SWIPE_X_RIGHT;
						input_report_key(input_dev, KEY_GESTURE, 1);
						input_sync(input_dev);
						input_report_key(input_dev, KEY_GESTURE, 0);
						input_sync(input_dev);
					break;
				case 0xBA:
						printk("[GTP]Gesture is KEY_GESTURE_UP");
						key_gesture = SWIPE_Y_UP;
						input_report_key(input_dev, KEY_GESTURE, 1);
						input_sync(input_dev);
						input_report_key(input_dev, KEY_GESTURE, 0);
						input_sync(input_dev);                                  
						break;
				case 0xAB:
						printk("[GTP]Gesture is KEY_GESTURE_DOWN");
						key_gesture = SWIPE_Y_DOWN;
						input_report_key(input_dev, KEY_GESTURE, 1);
						input_sync(input_dev);
						input_report_key(input_dev, KEY_GESTURE, 0);
						input_sync(input_dev);
						break;
				case 0xCC:
						printk("[GTP]Gesture is KEY_GESTURE_DOUBLE_TAP");
						key_gesture = DOUBLE_TAP;
						input_report_key(input_dev, KEY_GESTURE, 1);
						input_sync(input_dev);
						input_report_key(input_dev, KEY_GESTURE, 0);
						input_sync(input_dev);
						break;
				case 'o':
						printk("[GTP]Gesture is KEY_GESTURE_O");
						key_gesture = UNICODE_O;
						input_report_key(input_dev, KEY_GESTURE, 1);
						input_sync(input_dev);
						input_report_key(input_dev, KEY_GESTURE, 0);
						input_sync(input_dev);
						break;
				case 'w':
						printk("[GTP]Gesture is KEY_GESTURE_W");
						key_gesture = UNICODE_W;
						input_report_key(input_dev, KEY_GESTURE, 1);
						input_sync(input_dev);
						input_report_key(input_dev, KEY_GESTURE, 0);
						input_sync(input_dev);
						break;
				case 'm':
						printk("[GTP]Gesture is KEY_GESTURE_M");
						key_gesture = UNICODE_M;
						input_report_key(input_dev, KEY_GESTURE, 1);
						input_sync(input_dev);
						input_report_key(input_dev, KEY_GESTURE, 0);
						input_sync(input_dev);
						break;
				case 'e':
						printk("[GTP]Gesture is KEY_GESTURE_E");
						key_gesture = UNICODE_E;
						input_report_key(input_dev, KEY_GESTURE, 1);
						input_sync(input_dev);
						input_report_key(input_dev, KEY_GESTURE, 0);
						input_sync(input_dev);
						break;
				case 'c':
						printk("[GTP]Gesture is KEY_GESTURE_C");
						key_gesture = UNICODE_C;
						input_report_key(input_dev, KEY_GESTURE, 1);
						input_sync(input_dev);
						input_report_key(input_dev, KEY_GESTURE, 0);
						input_sync(input_dev);
						break;						
				case 's':
						printk("[GTP]Gesture is KEY_GESTURE_S");
						key_gesture = UNICODE_S;
						input_report_key(input_dev, KEY_GESTURE, 1);
						input_sync(input_dev);
						input_report_key(input_dev, KEY_GESTURE, 0);
						input_sync(input_dev);
						break;
				case 'v':
					       printk("[GTP]Gesture is KEY_GESTURE_V");
						key_gesture = UNICODE_V_DOWN;
						input_report_key(input_dev, KEY_GESTURE, 1);
						input_sync(input_dev);
						input_report_key(input_dev, KEY_GESTURE, 0);
						input_sync(input_dev);
						break;
				case 'z':
						printk("[GTP]Gesture is KEY_GESTURE_Z");
						key_gesture = UNICODE_Z;
						input_report_key(input_dev, KEY_GESTURE, 1);
						input_sync(input_dev);
						input_report_key(input_dev, KEY_GESTURE, 0);
						input_sync(input_dev);
						break;
				case '>':
					       printk("[GTP]Gesture is KEY_GESTURE_>");
					       key_gesture = UNICODE_V_L;
					       input_report_key(input_dev, KEY_GESTURE, 1);
					       input_sync(input_dev);
					       input_report_key(input_dev, KEY_GESTURE, 0);
					       input_sync(input_dev);
   						break;
				case 0x5E:
						printk("[GTP]Gesture is KEY_GESTURE_^");
						key_gesture = UNICODE_V_UP;
						input_report_key(input_dev, KEY_GESTURE, 1);
						input_sync(input_dev);
						input_report_key(input_dev, KEY_GESTURE, 0);
						input_sync(input_dev);
						break;
				default:
						break;
		}
	}
}
#endif
static inline s32 ges_i2c_write_bytes(u16 addr, u8 *buf, s32 len)
{
    return i2c_write_bytes(i2c_client_point, addr, buf, len);
}

static inline s32 ges_i2c_read_bytes(u16 addr, u8 *buf, s32 len)
{
    return i2c_read_bytes(i2c_client_point, addr, buf, len);
}

static ssize_t gtp_gesture_data_read(struct file *file, char __user * page, size_t size, loff_t * ppos)
{
	s32 ret = -1;

	GTP_DEBUG("visit gtp_gesture_data_read. ppos:%d", (int)*ppos);
	if (*ppos) {
		return 0;
	}
	if (size == 4) {
		ret = copy_to_user(((u8 __user *) page), "GT1X", 4);
		return 4;
	}
	ret = simple_read_from_buffer(page, size, ppos, &gesture_data, sizeof(gesture_data));

	GTP_DEBUG("Got the gesture data.");
	return ret;
}

static ssize_t gtp_gesture_data_write(struct file *filp, const char __user * buff, size_t len, loff_t * off)
{
	s32 ret = 0;

	ret = copy_from_user(&gesture_data.enabled, buff, 1);
	if (ret) {
		GTP_ERROR("copy_from_user failed.");
		return -EPERM;
	}

	GTP_DEBUG("gesture enabled:%x, ret:%d", gesture_data.enabled, ret);

	return len;
}

s8 gtp_enter_doze(void)
{
    int ret = -1;
    s8 retry = 0;
    u8 i2c_control_buf[1] = {8};

    GTP_DEBUG("Entering doze mode.");
    while(retry++ < 5) {
        ret = ges_i2c_write_bytes(0x8046, i2c_control_buf, 1);
        if (ret < 0) {
            GTP_DEBUG("failed to set doze flag into 0x8046, %d", retry);
            continue;
        }

        ret = ges_i2c_write_bytes(0x8040, i2c_control_buf, 1);
        if (!ret){
            gesture_data.doze_status = DOZE_ENABLED;
            GTP_INFO("Gesture mode enabled.");
            return ret;
        }
        msleep(10);
    }
    GTP_ERROR("GTP send doze cmd failed.");
    return ret;
}

static int check_maxdata(int *data)
{
	int *a;
		a = data;
	if((a[0]>a[1])&&(a[0]>a[2])&&(a[0]>a[3]))
		return 0;
	else if((a[1]>a[2])&&(a[1]>a[3]))
		return 1;
	else if(a[2]>a[3])	
		return 2;
	else	
		return 3;
}

static int check_mindata(int *data)
{
	int *a;
		a = data;
	
	if((a[0]<a[1])&&(a[0]<a[2])&&(a[0]<a[3]))
		return 0;
	else if((a[1]<a[2])&&(a[1]<a[3]))
		return 1;
	else if(a[2]<a[3])	
		return 2;
	else	
		return 3;
}

//s32 gesture_event_handler(struct input_dev * dev)
void gesture_event_handler(struct input_dev * dev)
{
	s32 ret = 0;
	u8 doze_buf1[4] = { 1,1,1,1 };

	int x_gesture[4]={};
	int y_gesture[4]={};
	s32 x_max,y_max,x_min,y_min,m;

	u8 doze_buf[3] = {0x81, 0x4B};
	u8 gesture_start_point[6] = {0x81,0x4D};
	u8 gesture_end_point[6] = {0x81,0x51};
	u8 gesture_width_height[6] = {0x81, 0x55};
	u8 gesture_mid_point[6] = {0x81, 0x59}; 
	u8 gesture_p1_point[6] = {0x81, 0x5D};
	u8 gesture_p2_point[6] = {0x81, 0x61};
	u8 gesture_p3_point[6] = {0x81, 0x65};
	u8 gesture_p4_point[6] = {0x81, 0x69};
	/*
	u32 gesture_start_point_x,gesture_start_point_y;
	u32 gesture_end_point_x,gesture_end_point_y;
	u32 gesture_width,gesture_height;//*/
	u32 gesture_mid_point_x,gesture_mid_point_y;
	u32 gesture_p1_point_x,gesture_p1_point_y;
	u32 gesture_p2_point_x,gesture_p2_point_y;
	u32 gesture_p3_point_x,gesture_p3_point_y;
	u32 gesture_p4_point_x,gesture_p4_point_y;
	
	if (g_wakeup_gesture) {
		if (DOZE_ENABLED == gesture_data.doze_status) {
			ret = ges_i2c_read_bytes(GTP_REG_WAKEUP_GESTURE, doze_buf1, 4);
			GTP_DEBUG("0x%x = 0x%02X,0x%02X,0x%02X,0x%02X", GTP_REG_WAKEUP_GESTURE, doze_buf1[0], doze_buf1[1], doze_buf1[2], doze_buf1[3]);

			if (ret == 0 && doze_buf1[0] == 0 && doze_buf1[1] == 0 && doze_buf1[2] == 0 && doze_buf1[3] == 0)
			 	//return 0;
			 	return;
			
			ret = gtp_i2c_read(i2c_client_point, gesture_start_point, 6);
			gesture_start_point_x   = gesture_start_point[2] | gesture_start_point[3] << 8;
			gesture_start_point_y  = gesture_start_point[4] | gesture_start_point[5] << 8;
			GTP_DEBUG("gesture_start_point_x =0x%02X, gesture_start_point_y =0x%02X", gesture_start_point_x,gesture_start_point_y);

			ret = gtp_i2c_read(i2c_client_point, gesture_end_point, 6);
			gesture_end_point_x   = gesture_end_point[2] | gesture_end_point[3] << 8;
			gesture_end_point_y  = gesture_end_point[4] | gesture_end_point[5] << 8;
			GTP_DEBUG("gesture_end_point_x = 0x%02X, gesture_end_point_y = 0x%02X", gesture_end_point_x,gesture_end_point_y);

			ret = gtp_i2c_read(i2c_client_point, gesture_width_height, 6);
			gesture_width   = gesture_width_height[2] | gesture_width_height[3] << 8;
			gesture_height  = gesture_width_height[4] | gesture_width_height[5] << 8;
			GTP_DEBUG("gesture_width = 0x%02X, gesture_height = 0x%02X", gesture_width,gesture_height);

			ret = gtp_i2c_read(i2c_client_point, gesture_mid_point, 6);
			gesture_mid_point_x   = gesture_mid_point[2] | gesture_mid_point[3] << 8;
			gesture_mid_point_y   = gesture_mid_point[4] | gesture_mid_point[5] << 8;
			GTP_DEBUG("gesture_mid_point_x = 0x%02X, gesture_mid_point_y = 0x%02X", gesture_mid_point_x,gesture_mid_point_y);

			ret = gtp_i2c_read(i2c_client_point, gesture_p1_point, 6);
			gesture_p1_point_x   = gesture_p1_point[2] | gesture_p1_point[3] << 8;
			gesture_p1_point_y   = gesture_p1_point[4] | gesture_p1_point[5] << 8;
			GTP_DEBUG("gesture_p1_point_x = 0x%02X, gesture_p1_point_y = 0x%02X", gesture_p1_point_x,gesture_p1_point_y);

			ret = gtp_i2c_read(i2c_client_point, gesture_p2_point, 6);
			gesture_p2_point_x   = gesture_p2_point[2] | gesture_p2_point[3] << 8;
			gesture_p2_point_y   = gesture_p2_point[4] | gesture_p2_point[5] << 8;
			GTP_DEBUG("gesture_p2_point_x = 0x%02X, gesture_p2_point_y = 0x%02X", gesture_p2_point_x,gesture_p2_point_y);

			ret = gtp_i2c_read(i2c_client_point, gesture_p3_point, 6);
			gesture_p3_point_x   = gesture_p3_point[2] | gesture_p3_point[3] << 8;
			gesture_p3_point_y   = gesture_p3_point[4] | gesture_p3_point[5] << 8;
			GTP_DEBUG("gesture_p3_point_x = 0x%02X, gesture_p3_point_y = 0x%02X", gesture_p3_point_x,gesture_p3_point_y);

			ret = gtp_i2c_read(i2c_client_point, gesture_p4_point, 6);
			gesture_p4_point_x   = gesture_p4_point[2] | gesture_p4_point[3] << 8;
			gesture_p4_point_y   = gesture_p4_point[4] | gesture_p4_point[5] << 8;
			GTP_DEBUG("gesture_p4_point_x = 0x%02X, gesture_p4_point_y = 0x%02X", gesture_p4_point_x,gesture_p4_point_y);

			gesture_echo[0]  = gesture_start_point_x;
			gesture_echo[1]  = gesture_start_point_y;	
			gesture_echo[2]  = gesture_end_point_x;
			gesture_echo[3]  = gesture_end_point_y;	
		
			x_gesture[0] = gesture_p1_point_x;
			y_gesture[0] = gesture_p1_point_y;	
			x_gesture[1] = gesture_p2_point_x;
			y_gesture[1] = gesture_p2_point_y;	
			x_gesture[2] = gesture_p3_point_x;
			y_gesture[2] = gesture_p3_point_y;	
			x_gesture[3] = gesture_p4_point_x;
			y_gesture[3] = gesture_p4_point_y;	
			x_max = check_maxdata(x_gesture);
			y_max = check_maxdata(y_gesture);
			x_min = check_mindata(x_gesture);
			y_min = check_mindata(y_gesture);
			printk("gesture the x_max = %d y_max = %d x_min = %d y_min = %d \n",x_max,y_max,x_min,y_min);
			gesture_echo[4] = x_gesture[y_min];//up
			gesture_echo[5] = y_gesture[y_min];
			gesture_echo[6] = x_gesture[x_min];//left
			gesture_echo[7] = y_gesture[x_min];
			gesture_echo[8] = x_gesture[y_max];//down
			gesture_echo[9] = y_gesture[y_max];
			gesture_echo[10] = x_gesture[x_max];//right
			gesture_echo[11] = y_gesture[x_max];

			for(m = 0 ;m<12;m++){
				printk("gesture_echo[%d] = %04x \n",m,gesture_echo[m]);
			}

			ret = gtp_i2c_read(i2c_client_point, doze_buf, 3);
		 	GTP_DEBUG("0x814B = 0x%02X", doze_buf[2]);
			if (ret > 0)
			{
				if(double_gesture){
					if ((0xCC == doze_buf[2]) && (gesture_start_point_x > 116) && (gesture_end_point_x > 116)\
					&& (gesture_start_point_x < 604) && (gesture_end_point_x < 604)\
					&& (gesture_start_point_y > 123) && (gesture_end_point_y > 123)\
					&& (gesture_start_point_y < 1157) && (gesture_end_point_y < 1157))
					{
						GTP_INFO("Double click to light up the screen!");
						// doze_status = DOZE_WAKEUP;
						input_report_key(tpd->dev, KEY_F1, 1);//KEY_F1
						//input_sync(tpd->dev);
						input_report_key(tpd->dev, KEY_F1, 0);
						input_sync(tpd->dev);
						doze_buf[2] = 0x00;
						gtp_i2c_write(i2c_client_point, doze_buf, 3);
						return;
					}
				}
				if(draw_gesture){
					if((doze_buf[2]=='c') && (gesture_width > 116) && (gesture_height > 123))// length and width
				   	{
						GTP_INFO("click c!");
						// doze_status = DOZE_WAKEUP;
						input_report_key(tpd->dev, KEY_F8, 1);//KEY_F8
						input_report_key(tpd->dev, KEY_F8, 0);
						input_sync(tpd->dev);		
						// clear 0x814B
						doze_buf[2] = 0x00;
						gtp_i2c_write(i2c_client_point, doze_buf, 3);
						return;
				   	}
					else if((doze_buf[2]=='m') && (gesture_width > 116) && (gesture_height > 123))
					{
						GTP_INFO("click m!");
						// doze_status = DOZE_WAKEUP;
						input_report_key(tpd->dev, KEY_F10, 1);
						// input_sync(tpd->dev);
						input_report_key(tpd->dev, KEY_F10, 0);
						input_sync(tpd->dev);
						// clear 0x814B
						doze_buf[2] = 0x00;
						gtp_i2c_write(i2c_client_point, doze_buf, 3);
						return;
					}
					else if((doze_buf[2]==0x65) && (gesture_width > 116) && (gesture_height > 123))
					{
						GTP_INFO(" click e!");
						// doze_status = DOZE_WAKEUP;
						input_report_key(tpd->dev, KEY_F9, 1);
						//input_sync(tpd->dev);
						input_report_key(tpd->dev, KEY_F9, 0);
						input_sync(tpd->dev);
						// clear 0x814B
						doze_buf[2] = 0x00;
						gtp_i2c_write(i2c_client_point, doze_buf, 3);
						return;
					}  
					else if ((doze_buf[2]==0x77) && (gesture_width > 116) && (gesture_height > 123)) 
					{
						GTP_INFO(" gesture  Draw w!");
						//doze_status = DOZE_WAKEUP;
						input_report_key(tpd->dev, KEY_F11, 1);
						input_report_key(tpd->dev, KEY_F11, 0);
						input_sync(tpd->dev);
						// clear 0x814B
						doze_buf[2] = 0x00;
						gtp_i2c_write(i2c_client_point, doze_buf, 3);
						return;
					} 
				}	  

				// clear 0x814B
				GTP_INFO("***************************");
				doze_buf[2] = 0x00;
				gtp_i2c_write(i2c_client_point, doze_buf, 3);
				gtp_enter_doze();
			}
		}
	}
	else {
		GTP_DEBUG("g_wakeup_gesture = 0,gesture function closed!");
	} 
}

#if 0
s32 gesture_event_handler(struct input_dev * dev)
{
	u8 doze_buf[4] = { 1,1,1,1 };
	unsigned int key_code;
	s32 ret = 0;
	int len, extra_len;

	if (DOZE_ENABLED == gesture_data.doze_status) {
		ret = ges_i2c_read_bytes(GTP_REG_WAKEUP_GESTURE, doze_buf, 4);
		GTP_DEBUG("0x%x = 0x%02X,0x%02X,0x%02X,0x%02X", GTP_REG_WAKEUP_GESTURE, doze_buf[0], doze_buf[1], doze_buf[2], doze_buf[3]);
		/*GTP_DEBUG("0x%x = 0x%02X,0x%02X", GTP_REG_WAKEUP_GESTURE, doze_buf[0], doze_buf[1]);*/
		if (ret == 0 && doze_buf[0] == 0 && doze_buf[1] == 0 && doze_buf[2] == 0 && doze_buf[3] == 0)
		     return 0;
		if (ret == 0 && doze_buf[0] != 0) {
			if (!QUERYBIT(gestures_flag, doze_buf[0])) {
				GTP_INFO("Sorry, this gesture has been disabled.");
				doze_buf[0] = 0x00;
				ges_i2c_write_bytes(GTP_REG_WAKEUP_GESTURE, doze_buf, 1);
				gtp_enter_doze();
				return 0;
			}

			mutex_lock(&gesture_data_mutex);
			len = doze_buf[1] & 0x7F;
			if (len > GESTURE_MAX_POINT_COUNT) {
				GTP_ERROR("Gesture contain too many points!(%d)", len);
				len = GESTURE_MAX_POINT_COUNT;
			}
			if (len > 0) {
				ret = ges_i2c_read_bytes(GTP_REG_WAKEUP_GESTURE_DETAIL, &gesture_data.data[4], len * 4);
				if (ret < 0) {
					GTP_DEBUG("Read gesture data failed.");
					mutex_unlock(&gesture_data_mutex);
					return 0;
				}
			}

			extra_len = doze_buf[1] & 0x80 ? doze_buf[3] : 0;
			if (extra_len > 80) {
				GTP_ERROR("Gesture contain too many extra data!(%d)", extra_len);
				extra_len = 80;
			}
			if (extra_len > 0) {
				ret = ges_i2c_read_bytes(GTP_REG_WAKEUP_GESTURE + 4, &gesture_data.data[4 + len * 4], extra_len);
				if (ret < 0) {
					GTP_DEBUG("Read extra gesture data failed.");
					mutex_unlock(&gesture_data_mutex);
					return 0;
				}
			}

            doze_buf[2] &= ~0x30;
            doze_buf[2] |= extra_len > 0 ? 0x20 : 0x10;
            
			gesture_data.data[0] = doze_buf[0];	/* gesture type*/
			gesture_data.data[1] = len;	/* gesture points number*/
			gesture_data.data[2] = doze_buf[2];
			gesture_data.data[3] = extra_len;
			mutex_unlock(&gesture_data_mutex);

			gtp_check_gesture(dev, gesture_data.data[0]);//wenggaojian@wind-mobi.com 20160510 add 

			key_code = doze_buf[0] < 16 ?  KEY_F3: KEY_F2;
			GTP_DEBUG("Gesture: 0x%02X, points: %d", doze_buf[0], doze_buf[1]);

			doze_buf[0] = 0;
			ges_i2c_write_bytes(GTP_REG_WAKEUP_GESTURE, doze_buf, 1);

			/*input_report_key(dev, key_code, 1);
			input_sync(dev);
			input_report_key(dev, key_code, 0);
			input_sync(dev);*/
			return 2; /* doze enabled and get valid gesture data*/
		}
		return 1; /* doze enabled, but no invalid gesutre data*/
	}
	return 0; /* doze not enabled*/
}
#endif

void gesture_clear_wakeup_data(void)
{
	mutex_lock(&gesture_data_mutex);
	memset(gesture_data.data, 0, 4);
	mutex_unlock(&gesture_data_mutex);
}

#define GOODIX_MAGIC_NUMBER        'G'
#define NEGLECT_SIZE_MASK           (~(_IOC_SIZEMASK << _IOC_SIZESHIFT))

#define GESTURE_ENABLE_TOTALLY      _IO(GOODIX_MAGIC_NUMBER, 1)	// 1
#define GESTURE_DISABLE_TOTALLY     _IO(GOODIX_MAGIC_NUMBER, 2)
#define GESTURE_ENABLE_PARTLY       _IO(GOODIX_MAGIC_NUMBER, 3)
#define GESTURE_DISABLE_PARTLY      _IO(GOODIX_MAGIC_NUMBER, 4)
//#define SET_ENABLED_GESTURE         (_IOW(GOODIX_MAGIC_NUMBER, 5, u8) & NEGLECT_SIZE_MASK)
#define GESTURE_DATA_OBTAIN         (_IOR(GOODIX_MAGIC_NUMBER, 6, u8) & NEGLECT_SIZE_MASK)
#define GESTURE_DATA_ERASE          _IO(GOODIX_MAGIC_NUMBER, 7)

#define IO_IIC_READ                  (_IOR(GOODIX_MAGIC_NUMBER, 100, u8) & NEGLECT_SIZE_MASK)
#define IO_IIC_WRITE                 (_IOW(GOODIX_MAGIC_NUMBER, 101, u8) & NEGLECT_SIZE_MASK)
#define IO_RESET_GUITAR              _IO(GOODIX_MAGIC_NUMBER, 102)
#define IO_DISABLE_IRQ               _IO(GOODIX_MAGIC_NUMBER, 103)
#define IO_ENABLE_IRQ                _IO(GOODIX_MAGIC_NUMBER, 104)
#define IO_GET_VERISON               (_IOR(GOODIX_MAGIC_NUMBER, 110, u8) & NEGLECT_SIZE_MASK)
#define IO_PRINT                     (_IOW(GOODIX_MAGIC_NUMBER, 111, u8) & NEGLECT_SIZE_MASK)
#define IO_VERSION                   "V1.0-20141015"

#define CMD_HEAD_LENGTH             20

static s32 io_iic_read(u8 * data, void __user * arg)
{
	s32 err = -1;
	s32 data_length = 0;
	u16 addr = 0;

	err = copy_from_user(data, arg, CMD_HEAD_LENGTH);
	if (err) {
		GTP_DEBUG("Can't access the memory.");
		return err;
	}

	addr = data[0] << 8 | data[1];
	data_length = data[2] << 8 | data[3];

	err = ges_i2c_read_bytes(addr, &data[CMD_HEAD_LENGTH], data_length);
	if (!err) {
		err = copy_to_user(&((u8 __user *) arg)[CMD_HEAD_LENGTH], &data[CMD_HEAD_LENGTH], data_length);
		if (err) {
			GTP_ERROR("ERROR when copy to user.[addr: %04x], [read length:%d]", addr, data_length);
			return err;
		}
		err = CMD_HEAD_LENGTH + data_length;
	}
	GTP_DEBUG("IIC_READ.addr:0x%4x, length:%d, ret:%d", addr, data_length, err);
	GTP_DEBUG_ARRAY((&data[CMD_HEAD_LENGTH]), data_length);

	return err;
}

static s32 io_iic_write(u8 * data)
{
	s32 err = -1;
	s32 data_length = 0;
	u16 addr = 0;

	addr = data[0] << 8 | data[1];
	data_length = data[2] << 8 | data[3];

	err = ges_i2c_write_bytes(addr, &data[CMD_HEAD_LENGTH], data_length);
	if (!err) {
		err = CMD_HEAD_LENGTH + data_length;
	}

	GTP_DEBUG("IIC_WRITE.addr:0x%4x, length:%d, ret:%d", addr, data_length, err);
	GTP_DEBUG_ARRAY((&data[CMD_HEAD_LENGTH]), data_length);
	return err;
}

/* @return, 0:operate successfully
*         > 0: the length of memory size ioctl has accessed,
*        error otherwise.
*/
static long gtp_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	u32 value = 0;
	s32 ret = 0;		//the initial value must be 0
	u8 *data = NULL;

/*	GTP_DEBUG("IOCTL CMD:%x", cmd); */
/*	GTP_DEBUG("command:%d, length:%d, rw:%s", _IOC_NR(cmd), _IOC_SIZE(cmd),
(_IOC_DIR(cmd) & _IOC_READ) ? "read" : (_IOC_DIR(cmd) & _IOC_WRITE) ? "write" : "-");*/

	if (_IOC_DIR(cmd)) {
		s32 err = -1;
		s32 data_length = _IOC_SIZE(cmd);
		data = (u8 *) kzalloc(data_length, GFP_KERNEL);
		memset(data, 0, data_length);

		if (_IOC_DIR(cmd) & _IOC_WRITE) {
			err = copy_from_user(data, (void __user *)arg, data_length);
			if (err) {
				GTP_DEBUG("Can't access the memory.");
				kfree(data);
				return -1;
			}
		}
	} else {
		value = (u32) arg;
	}

	switch (cmd & NEGLECT_SIZE_MASK) {
	case IO_GET_VERISON:
		if ((u8 __user *) arg) {
			ret = copy_to_user(((u8 __user *) arg), IO_VERSION, sizeof(IO_VERSION));
			if (!ret) {
				ret = sizeof(IO_VERSION);
			}
			GTP_INFO("%s", IO_VERSION);
		}
		break;
	case IO_IIC_READ:
		ret = io_iic_read(data, (void __user *)arg);
		break;

	case IO_IIC_WRITE:
		ret = io_iic_write(data);
		break;

	case IO_RESET_GUITAR: 
		gtp_reset_guitar(i2c_client_point, 10);
		break;

	case IO_DISABLE_IRQ: {
		gtp_irq_disable();
#ifdef CONFIG_GTP_ESD_PROTECT
		gtp_esd_switch(i2c_client_point, SWITCH_OFF);
#endif
		break;
		}
	case IO_ENABLE_IRQ: {
			gtp_irq_enable();
		}
#ifdef CONFIG_GTP_ESD_PROTECT
		gtp_esd_switch(i2c_client_point ,SWITCH_ON);
#endif
		break;

	case IO_PRINT:
		if (data)
			GTP_INFO("%s", (char *)data);
		break;

	case GESTURE_ENABLE_TOTALLY:
		GTP_DEBUG("ENABLE_GESTURE_TOTALLY");
		gesture_data.enabled = 1;
		break;

	case GESTURE_DISABLE_TOTALLY:
		GTP_DEBUG("DISABLE_GESTURE_TOTALLY");
		gesture_data.enabled = 0;
		break;

	case GESTURE_ENABLE_PARTLY:
		SETBIT(gestures_flag, (u8) value);
		gesture_data.enabled = 1;
		GTP_DEBUG("ENABLE_GESTURE_PARTLY, gesture = 0x%02X, gesture_data.enabled = %d", value, gesture_data.enabled);
		break;

	case GESTURE_DISABLE_PARTLY:
		CLEARBIT(gestures_flag, (u8) value);
		GTP_DEBUG("DISABLE_GESTURE_PARTLY, gesture = 0x%02X, gesture_data.enabled = %d", value, gesture_data.enabled);
		break;

	case GESTURE_DATA_OBTAIN:
		GTP_DEBUG("OBTAIN_GESTURE_DATA");

		mutex_lock(&gesture_data_mutex);
		if (gesture_data.data[1] > GESTURE_MAX_POINT_COUNT) {
			gesture_data.data[1] = GESTURE_MAX_POINT_COUNT;
		}
		if (gesture_data.data[3] > 80) {
			gesture_data.data[3] = 80;
		}
		ret = copy_to_user(((u8 __user *) arg), &gesture_data.data, 4 + gesture_data.data[1] * 4 + gesture_data.data[3]);
		mutex_unlock(&gesture_data_mutex);
		if (ret) {
			GTP_ERROR("ERROR when copy gesture data to user.");
		} else {
			ret = 4 + gesture_data.data[1] * 4 + gesture_data.data[3];
		}
		break;

	case GESTURE_DATA_ERASE:
		GTP_DEBUG("ERASE_GESTURE_DATA");
		gesture_clear_wakeup_data();
		break;

	default:
		GTP_INFO("Unknown cmd.");
		ret = -1;
		break;
	}

	if (data != NULL) {
		kfree(data);
	}

	return ret;
}

#ifdef CONFIG_COMPAT
static long gtp_compat_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	void __user *arg32 = compat_ptr(arg);

	if (!file->f_op || !file->f_op->unlocked_ioctl) {
		return -ENOMEM;
	}

	return file->f_op->unlocked_ioctl(file, cmd, (unsigned long)arg32);
}
#endif

static int gtp_gesture_open(struct inode *node, struct file *flip) {
        GTP_DEBUG("gesture node is opened.");
        return 0;
}

static int gtp_gesture_release(struct inode *node, struct file *filp) {
        GTP_DEBUG("gesture node is closed.");
        return 0;  
}

static const struct file_operations gtp_fops = {
	.owner = THIS_MODULE,
	.open = gtp_gesture_open,
	.release = gtp_gesture_release,
	.read = gtp_gesture_data_read,
	.write = gtp_gesture_data_write,
	.unlocked_ioctl = gtp_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl = gtp_compat_ioctl,
#endif
};

s32 gtp_extents_init(void)
{
	struct proc_dir_entry *proc_entry = NULL;

	mutex_init(&gesture_data_mutex);
	memset(gestures_flag, 0xff, sizeof(gestures_flag)); // set 0xff for open all gestures 
	memset((u8 *) & gesture_data, 0, sizeof(struct gesture_data));

	gesture_data.enabled = 1;//yifaguo 2017.0726 add

	//gesture_node_init();//yifaguo 2017.0726 add
	proc_ctp_gesture_add(ctp_gesture_proc_entry, sizeof(ctp_gesture_proc_entry)/sizeof(struct gesture_debug_entry));

	proc_entry = proc_create(GESTURE_NODE, 0664, NULL, &gtp_fops);
	if (proc_entry == NULL) {
		GTP_ERROR("Couldn't create proc entry[GESTURE_NODE]!");
		return -1;
	} else {
		GTP_INFO("Create proc entry[GESTURE_NODE] success!");
	}

	return 0;
}

void gtp_extents_exit(void)
{
	//gesture_node_uninit();//yifaguo 2016.07.26 add 
	remove_proc_entry(GESTURE_NODE, NULL);
}
#endif /* GTP_GESTURE_WAKEUP */
