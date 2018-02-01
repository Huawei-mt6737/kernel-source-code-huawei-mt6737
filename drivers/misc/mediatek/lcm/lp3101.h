#ifndef LP3101_H
#define LP3101_H
#include <linux/ioctl.h>

#define LP_TAG "[lp3101]"
#define LP_LOG(fmt, args...)   pr_debug(LP_TAG fmt, ##args)

struct lcd_bais_pinctrl{
	struct pinctrl_state *enn_low;
	struct pinctrl_state *enn_high;
	struct pinctrl_state *enp_low;
	struct pinctrl_state *enp_high;
};

struct lcd_bais{
	struct pinctrl *pinctrl;
	struct lcd_bais_pinctrl *pin_cfg;
};

#endif
