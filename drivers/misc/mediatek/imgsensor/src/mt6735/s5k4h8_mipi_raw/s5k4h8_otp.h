#ifndef _S5K4H8_OTP_H
#define _S5K4H8_OTP_H

/*file discription : otp*/
#define OTP_DRV_LSC_SIZE 186
#define S5K4H8_DEVICE_ID 0x5A//slave id of chicony s5k3l2

struct otp_struct {
int flag;//bit[7]:info,bit[6]:wb,bit[5]:vcm,bit[4]:lenc
int module_integrator_id;
int lens_id;
int production_year;
int production_month;
int production_day;
int rg_ratio;
int bg_ratio;
//int light_rg;
//int light_bg;
//int typical_rg_ratio;
//int typical_bg_ratio;
unsigned char lenc[OTP_DRV_LSC_SIZE];
int checksumLSC;
int checksumOTP;
int checksumTotal;
int VCM_start;
int VCM_end;
int VCM_dir;
};

extern void s5k4h8_otp_cali(void);

#endif

