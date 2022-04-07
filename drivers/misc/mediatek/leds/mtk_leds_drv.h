// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (c) 2019 MediaTek Inc.
 *
 */


#include <linux/leds.h>
#include <mtk_leds_hal.h>

/****************************************************************************
 * LED DRV functions
 ***************************************************************************/

#ifdef CONTROL_BL_TEMPERATURE
int setMaxbrightness(int max_level, int enable);
#endif

extern int mt65xx_leds_brightness_set(enum mt65xx_led_type type,
		enum led_brightness level);
#ifdef CONFIG_MTK_LEDS
extern int backlight_brightness_set(int level);
#else
#define backlight_brightness_set(level) do { } while (0)
#endif
extern int disp_bls_set_max_backlight(unsigned int level);

bool __weak disp_aal_is_support(void) { return false; };
int __weak disp_bls_set_max_backlight(unsigned int level_1024) { return 0; };
int __weak disp_bls_set_backlight(int level_1024) { return 0; }
int __weak mtkfb_set_backlight_level(unsigned int level) { return 0; };
void __weak disp_pq_notify_backlight_changed(int bl_1024) {};
