// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2020 MediaTek Inc.
 */

#include "flashlight-core.h"

#if defined(mt6580)
	#if defined(tb8321p2_bsp) || defined(tb8321p2_bsp_1g) || \
	defined(tb8321p2_bsp_3gds) || defined(tb8321p2_bsp_512m) || \
	defined(tb8321p2_bsp_trusty) || defined(tb8321p2_bspab) || \
	defined(tb8321p2_bsp_2g) || defined(tb8321p2_bsp_ab) || \
	defined(tb8321p2_bspota)
	const struct flashlight_device_id flashlight_id[] = {
		/* {TYPE, CT, PART, "NAME", CHANNEL, DECOUPLE} */
		{0, 0, 0, "flashlights_cp2155", 0, 0},
	};
	#elif defined(CONFIG_MTK_FLASHLIGHT_OCP81373)
	const struct flashlight_device_id flashlight_id[] = {
		/* {TYPE, CT, PART, "NAME", CHANNEL, DECOUPLE} */
		{0, 0, 0, "flashlights-ocp81373", 1, 1},
		{1, 0, 0, "flashlights-ocp81373", 0, 1},
	};
	#else
	const struct flashlight_device_id flashlight_id[] = {
		/* {TYPE, CT, PART, "NAME", CHANNEL, DECOUPLE} */
		{0, 0, 0, "flashlights-main-gpio", 0, 0},
#if defined(CONFIG_MTK_FLASHLIGHT_SUB_GPIO)
		{1, 0, 0, "flashlights-sub-gpio", 1, 0},
#endif		
	};
	#endif
#elif defined(mt6735)
	const struct flashlight_device_id flashlight_id[] = {
		/* {TYPE, CT, PART, "NAME", CHANNEL, DECOUPLE} */
		{0, 0, 0, "flashlights-main-gpio", 0, 0},
#if defined(CONFIG_MTK_FLASHLIGHT_SUB_GPIO)		
		{1, 0, 0, "flashlights-sub-gpio", 1, 0},
#endif		
	};
#elif defined(mt6739)
	#if defined(tb8765ap1_64_bsp)
	const struct flashlight_device_id flashlight_id[] = {
		/* {TYPE, CT, PART, "NAME", CHANNEL, DECOUPLE} */
		{0, 0, 0, "flashlights_led191", 0, 0},
		{1, 0, 0, "flashlights_led191", 1, 0},
	};
	#else
	const struct flashlight_device_id flashlight_id[] = {
		/* {TYPE, CT, PART, "NAME", CHANNEL, DECOUPLE} */
		{0, 0, 0, "flashlights-rt4505", 0, 0},
	};
	#endif
#elif defined(mt6757)
	#if defined(evb6757p_dm_64) || defined(k57pv1_dm_64) || \
	defined(k57pv1_64_baymo) || defined(k57pv1_dm_64_bif) || \
	defined(k57pv1_dm_64_baymo) || defined(k57pv1_dm_teei_2g) || \
	defined(k57pv1_dm_64_zoom)
	const struct flashlight_device_id flashlight_id[] = {
		/* {TYPE, CT, PART, "NAME", CHANNEL, DECOUPLE} */
		{0, 0, 0, "flashlights-rt5081", 0, 0},
		{0, 1, 0, "flashlights-rt5081", 1, 0},
	};
	#elif defined(CONFIG_MTK_FLASHLIGHT_RT5081)
	const struct flashlight_device_id flashlight_id[] = {
		/* {TYPE, CT, PART, "NAME", CHANNEL, DECOUPLE} */
		{0, 0, 0, "flashlights-rt5081", 0, 0},
		{0, 1, 0, "flashlights-rt5081", 1, 0},
	};
	#else
	const struct flashlight_device_id flashlight_id[] = {
		/* {TYPE, CT, PART, "NAME", CHANNEL, DECOUPLE} */
		{0, 0, 0, "flashlights-lm3643", 0, 0},
		{0, 1, 0, "flashlights-lm3643", 1, 0},
	};
	#endif
#elif defined(mt6758)
	const struct flashlight_device_id flashlight_id[] = {
		/* {TYPE, CT, PART, "NAME", CHANNEL, DECOUPLE} */
		{0, 0, 0, "flashlights-mt6370", 0, 0},
		{0, 1, 0, "flashlights-mt6370", 1, 0},
	};
#elif defined(mt6759)
	const struct flashlight_device_id flashlight_id[] = {
		/* {TYPE, CT, PART, "NAME", CHANNEL, DECOUPLE} */
		{0, 0, 0, "flashlights-rt5081", 0, 0},
		{0, 1, 0, "flashlights-rt5081", 1, 0},
	};
#elif defined(mt6763)
	const struct flashlight_device_id flashlight_id[] = {
		/* {TYPE, CT, PART, "NAME", CHANNEL, DECOUPLE} */
		{0, 0, 0, "flashlights-mt6370", 0, 0},
		{0, 1, 0, "flashlights-mt6370", 1, 0},
	};
#elif defined(mt6799)
	const struct flashlight_device_id flashlight_id[] = {
		/* {TYPE, CT, PART, "NAME", CHANNEL, DECOUPLE} */
		{0, 0, 0, "flashlights-mt6336", 0, 0},
		{0, 1, 0, "flashlights-mt6336", 1, 0},
	};
#elif defined(mt8167)
	const struct flashlight_device_id flashlight_id[] = {
		/* {TYPE, CT, PART, "NAME", CHANNEL, DECOUPLE} */
		{0, 0, 0, "flashlights-lm3642", 0, 0},
	};
#else
	const struct flashlight_device_id flashlight_id[] = {
		/* {TYPE, CT, PART, "NAME", CHANNEL, DECOUPLE} */
		{0, 0, 0, "flashlights-none", -1, 0},
		{0, 1, 0, "flashlights-none", -1, 0},
		{1, 0, 0, "flashlights-none", -1, 0},
		{1, 1, 0, "flashlights-none", -1, 0},
		{0, 0, 1, "flashlights-none", -1, 0},
		{0, 1, 1, "flashlights-none", -1, 0},
		{1, 0, 1, "flashlights-none", -1, 0},
		{1, 1, 1, "flashlights-none", -1, 0},
	};
#endif

const int flashlight_device_num =
	sizeof(flashlight_id) / sizeof(struct flashlight_device_id);

