/*
 *  linux/drivers/mmc/card/mmc_health_check.c
 *
 *  Copyright (c) 2013 Huawei Limited.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or (at
 * your option) any later version.

 */

#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/slab.h>
#include <linux/errno.h>
#include <linux/hdreg.h>
#include <linux/kdev_t.h>
#include <linux/blkdev.h>
#include <linux/mutex.h>
#include <linux/scatterlist.h>
#include <linux/string_helpers.h>
#include <linux/delay.h>
#include <linux/capability.h>
#include <linux/compat.h>
#include <linux/syscalls.h>
#include <linux/proc_fs.h>
#include <linux/device.h>
#include <linux/cdev.h>
#include <linux/mmc/core.h>
#include <linux/pm_runtime.h>
#include <trace/events/mmc.h>
#include <linux/mmc/ioctl.h>
#include <linux/mmc/card.h>
#include <linux/mmc/host.h>
#include <linux/mmc/mmc.h>
#include <linux/mmc/sd.h>
#include "mmc_health_diag.h"


//===========================================================================
struct mmc_blk_err_report_info sd_err_table[MAX_ERR_TYPE] = {
	{MMC_BLK_STUCK_IN_PRG_ERR,	KOBJ_SD_RW_ERROR},	//report rw err message to user
	{MMC_BLK_WR_SPEED_ERR,		KOBJ_SD_SLOW},	//report write speed too lowly message to user
	{MMC_BLK_RW_CHECK_ERR,		KOBJ_SD_RW_ERROR},	//report rw err message to user
	{MMC_RO_CHECK_ERR,		KOBJ_SD_RO_TEST},	//trigger ro check in vold
	{MMC_RO_ERR,			KOBJ_SD_RO},
};
static int g_bad_sd_report_times = 0;
static int g_sd_write_times = 0;
static int g_sd_test_times = 0;
static int g_sd_test2_times = 0;
static unsigned long long mmccid_tag_t1 = 0;//the time stamp the last rw
static unsigned int mmcqd_rq_count1 = 0;//the rw cmd times in one stat cycle
static unsigned long long mmcqd_t_usage_wr1=0,mmcqd_t_usage_rd1=0;//the time which spent in function mmc_blk_issue_rw_rq in one stat cycle
static unsigned int mmcqd_rq_size_wr1=0,mmcqd_rq_size_rd1=0;//the size which read and write totally in one stat cycle
static unsigned int wr_speed_abnor_times = 0;// the times io speed abnormally
//===========================================================================


/*===========================================================================
 * FUNCTION: mmc_clear_report_info
   PARAMETER:NULL
 *
 * Description: clear report times and sd_write_times
 *
 * Returns: NULL

===========================================================================*/
void mmc_clear_report_info(void)
{
   g_bad_sd_report_times = 0;
   g_sd_write_times = 0;
}


/*===========================================================================
 * FUNCTION: mmc_get_err_infor_from_err_table
   PARAMETER:
 *    @num:the err code which will translate to real code
 *
 * Description: according to status code to get the real status code which
                will be sent to user.
 *
 * Returns: index----the num which correspond to err code

===========================================================================*/
int mmc_get_err_infor_from_err_table(int num)
{
   int i;
   int index = -1;
   for(i = 0;i < MAX_ERR_TYPE; i++)
   {
      if(sd_err_table[i].err_num == num)
      {
        index = i;
        break;
      }
   }
   return index;
}

/*===========================================================================
 * FUNCTION: mmc_diag_sd_health_status
   PARAMETER:
 *    @disk:the disk which has been monitor
 *    @status:the err code which has been detected
 *
 * Description: according to status code to get the real status code which
                will be sent to user.and just only report MAX_REPORT_TIMES time
 *
 * Returns: NONE

===========================================================================*/
int g_uevent_test_times=0;
void mmc_diag_sd_health_status(struct mmc_card *card,int status)
{

  int index;
  //printk("[SD_HEALTH_DETECT]mmc_diag_sd_health_status,status=%d\n",status);
 
  if(status >= MMC_BLK_STUCK_IN_PRG_ERR && status <= MMC_RO_ERR)
  {
    if(g_bad_sd_report_times < MAX_REPORT_TIMES)//10 times
    {
	
       index = mmc_get_err_infor_from_err_table(status);
       if(index != -1)
       {

        printk("[SD_HEALTH_DETECT]mmc_diag_sd_health_status,status=%d,index=%d\n",status,index);
           //set_sd_disk_health_status(disk,sd_err_table[index].err_action);
	   kobject_uevent(&card->dev.kobj,sd_err_table[index].err_action);
           g_bad_sd_report_times++;
       }
    }
  }
}


/*===========================================================================
 * FUNCTION: mmc_get_rw_status
   PARAMETER:
 *    @status:the rw return value 
 *
 * Description: according to status to determine to whether rw error
 *
 * Returns: the err code which will be sent to user

===========================================================================*/
int mmc_get_rw_status(int status)
{
    static int err_num = 0;
    int ret = 0;
    //printk("[SD_HEALTH_DETECT]mmc_get_rw_status\n");
    //if err is rw err,must be 10 times continous,so can report!
    if(status >= MMC_BLK_CMD_ERR && status <= MMC_BLK_ECC_ERR)
    {
        err_num++;
        if(err_num >= MAX_ERR_TIMES)//10 times err
        {
            ret = MMC_BLK_RW_CHECK_ERR;
            printk(KERN_ERR "[SD_HEALTH_DETECT]mmc1 rw failed,status = %d,err_num = %d,report!!\n",status,err_num);
            err_num = 0;
        }
        else
        {
            printk(KERN_ERR "[SD_HEALTH_DETECT]mmc1 rw failed,status = %d,err_num = %d\n",status,err_num);
        }
    }
    else
    {
        if(err_num != 0)
        {
            printk(KERN_ERR "[SD_HEALTH_DETECT]mmc1 have %d err,but now rw success,clear err!!status = %d,not report!!\n",err_num,status);
            err_num = 0;
        }
    }
    return ret;
}


/*===========================================================================
 * FUNCTION: mmc_calculate_ioworkload_and_rwspeed
   PARAMETER:
 *    @time: the current time
 *    @rqc: the request which will be sent to host .
 *    @disk:the disk which has been monitor
 *
 * Description: calculate the ioworkload and rwspeed,to determine whether the bad
                card message to be reported
 *
 * Returns: the size which this request will send

===========================================================================*/
unsigned int mmc_calculate_ioworkload_and_rwspeed(struct mmc_card *card,unsigned long long time,struct request *rqc,struct gendisk *disk)
{
   unsigned long long t_period=0,t_usage = 0;
   unsigned int rq_byte=0;
   unsigned int t_percent=0;
   unsigned int t_percent_wr=0;
   unsigned int perf_meter=0;
   pid_t mmcqd_pid=0;

   //printk("[SD_HEALTH_DETECT]mmc_calculate_ioworkload_and_rwspeed\n");
   mmcqd_pid = task_pid_nr(current);
   if(mmccid_tag_t1==0)
   {
      mmccid_tag_t1 = time;
   }
   //get the period between this rw and the last rw
   t_period = time - mmccid_tag_t1;

   if(rqc)
   {
      //get the rw size this rw
      rq_byte = blk_rq_bytes(rqc);
   }

   //if period > 10s ,so stat
   if(t_period >= (unsigned long long )PRT_TIME_PERIOD)
   {
       t_usage = mmcqd_t_usage_wr1 + mmcqd_t_usage_rd1;// total real rw time
       printk("[SD_HEALTH_DETECT]mmcqd/1 rw time:%lld ms,write time:%lld ms,read time:%lld ms\n",t_usage,mmcqd_t_usage_wr1,mmcqd_t_usage_rd1);
       /* worload < 0.01*/
       if(t_period > t_usage*100)// io workload < 1%
       {
          printk("[SD_HEALTH_DETECT]mmmcqd/1 workload < 1%%, duty %lld, period %lld, req_cnt=%d\n",t_usage,t_period,mmcqd_rq_count1);
       }
       else
       {
          do_div(t_period, 100);// divided by 100,just to get %
          t_percent =((unsigned int)t_usage)/((unsigned int)t_period);//   real rw time/stat time
          t_percent_wr = ((unsigned int)mmcqd_t_usage_wr1)/((unsigned int)t_period);//    real write time/stat time-->to get write workload
          printk("[SD_HEALTH_DETECT]mmmcqd/1 rw_workload = %d%%, w_workload = %d%%, duty %lld, period %lld00, req_cnt=%d\n",t_percent, t_percent_wr, t_usage, t_period,mmcqd_rq_count1);
       }

       /* get write speed*/
       if(mmcqd_t_usage_wr1)
       {

          //change ns to ms!
          do_div(mmcqd_t_usage_wr1, 1000000);
          if(mmcqd_t_usage_wr1)  //if >=1ms
          {
              perf_meter = (mmcqd_rq_size_wr1)/((unsigned int)mmcqd_t_usage_wr1);
              if((t_percent_wr >= SD_IO_BUSY) &&(perf_meter < LOW_SPEED_WARTING_VALUE))
              {
                 if(wr_speed_abnor_times == 1)
                 {
                     wr_speed_abnor_times = 0;	
		     //kobject_uevent(&card->dev.kobj,KOBJ_SD_SLOW);
                     mmc_diag_sd_health_status(card,MMC_BLK_WR_SPEED_ERR);
                     printk(KERN_ERR "[SD_HEALTH_DETECT]mmmcqd/1:sd_spec:%d,the card wr_speed is lower than spec\n",mmc_get_sd_speed());
                 }
                 else
                 {
                     wr_speed_abnor_times ++;
                 }
              }
              else
              {
                 wr_speed_abnor_times = 0;
              }
              printk("[SD_HEALTH_DETECT]mmmcqd/1 Write Throughput=%d KB/s, size: %d bytes, time:%lld ms\n",perf_meter,mmcqd_rq_size_wr1,mmcqd_t_usage_wr1);
          }
       }

       /* get read speed*/
       if(mmcqd_t_usage_rd1)
       {
          do_div(mmcqd_t_usage_rd1, 1000000);
          if(mmcqd_t_usage_rd1)
          {
             perf_meter = (mmcqd_rq_size_rd1)/((unsigned int)mmcqd_t_usage_rd1);
             printk("[SD_HEALTH_DETECT]mmmcqd/1 Read Throughput=%d kB/s, size: %d bytes, time:%lld ms\n",perf_meter,mmcqd_rq_size_rd1,mmcqd_t_usage_rd1);
          }
       }

       mmccid_tag_t1 = time;

       mmcqd_t_usage_wr1 = 0;
       mmcqd_t_usage_rd1 = 0;
       mmcqd_rq_count1 = 0;
       mmcqd_rq_size_wr1 = 0;
       mmcqd_rq_size_rd1 = 0;


   }
   return rq_byte;

}



/*===========================================================================
 * FUNCTION: mmc_calculate_rw_size
   PARAMETER:
 *    @time: the time when the function mmc_blk_issue_rw_rq was entered
 *    @rq_byte: the size which this request will send .
 *    @rqc:the request which has been sent to host
 *
 * Description: at the end of mmc_blk_issue_rw_rq,to record the total rw size
                and rw time for
 *
 * Returns: 0-----SUCCESS

===========================================================================*/
int mmc_calculate_rw_size(unsigned long long time,unsigned int rq_byte,struct request *rqc)
{
   unsigned long long time2 = 0;
	//printk("[SD_HEALTH_DETECT]mmc_calculate_rw_size\n");
   //get current time from the phone boot ,ns!
   time2 = sched_clock();
   if(rqc)
   {
      if(rq_data_dir(rqc) == WRITE)
      {
         mmcqd_t_usage_wr1 += time2-time;
         mmcqd_rq_size_wr1 += rq_byte;
      }
      else
      {
         mmcqd_t_usage_rd1 += time2-time;
         mmcqd_rq_size_rd1 += rq_byte;
      }
      mmcqd_rq_count1 ++;
   }
   return 0;
}
/*===========================================================================
 * FUNCTION: mmc_trigger_ro_check
   PARAMETER:
 *    @rqc:the request which has been sent to host
 *    @disk:the disk which has been monitor
 *    @read_only:which indicated whether mmc is ro in hardware(according csd reg)
 *
 * Description: if mmc is ro (csd reg),so report to user;
                else if the first write time,trigger ro test
 *
 * Returns: 0-----SUCCESS

===========================================================================*/
int mmc_trigger_ro_check(struct mmc_card *card,struct request *rqc,struct gendisk *disk,unsigned int read_only)
{
    //printk("[SD_HEALTH_DETECT]mmc_trigger_ro_check,readonly=%d\n",read_only);
   if(read_only)
   {
      mmc_diag_sd_health_status(card,MMC_RO_ERR);
      printk("[SD_HEALTH_DETECT]mmc_trigger_ro_check,readonly=%d\n",read_only);
      return 0;
   }
   if(rqc && (rq_data_dir(rqc) == WRITE) && (g_sd_write_times == 0))
   {
      mmc_diag_sd_health_status(card,MMC_RO_CHECK_ERR);
      printk("[SD_HEALTH_DETECT]mmc_trigger_ro_check_err\n");
      g_sd_write_times++;
   }
   return 0;
}
