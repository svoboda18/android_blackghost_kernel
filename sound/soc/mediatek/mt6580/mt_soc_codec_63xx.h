// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2020 MediaTek Inc.
 * Author: Shane Chien <shane.chien@mediatek.com>
 */

/*******************************************************************************
 *
 * Filename:
 * ---------
 *  mt_sco_codec_63xx.h
 *
 * Project:
 * --------
 *    Audio codec header file
 *
 * Description:
 * ------------
 *   Audio codec function
 *
 * Author:
 * -------
 * Chipeng Chang
 *
 *------------------------------------------------------------------------------
 *
 *******************************************************************************/

#ifndef _AUDIO_CODEC_63xx_H
#define _AUDIO_CODEC_63xx_H

void audckbufEnable(bool enable);
void ClsqEnable(bool enable);
void Topck_Enable(bool enable);
void NvregEnable(bool enable);
void OpenClassAB(void);
void OpenAnalogHeadphone(bool bEnable);
void OpenAnalogTrimHardware(bool bEnable);
void SetSdmLevel(unsigned int level);
void setOffsetTrimMux(unsigned int Mux);
void setOffsetTrimBufferGain(unsigned int gain);
void EnableTrimbuffer(bool benable);
void SetHplTrimOffset(int Offset);
void SetHprTrimOffset(int Offset);
void setHpGainZero(void);
bool OpenHeadPhoneImpedanceSetting(bool bEnable);
void SetAnalogSuspend(bool bEnable);
void OpenTrimBufferHardware(bool bEnable);
void setHpDcCalibration(unsigned int type, int dc_cali_value);
void setHpDcCalibrationGain(unsigned int type, int gain_value);


#endif
