// SPDX-License-Identifier: GPL-2.0
/*
* Copyright (C) 2019  MediaTek Inc.
*
*/
/*******************************************************************************
 *
 * Filename:
 * ---------
 *  mt_sco_codec_speaker_63xx.hh
 *
 * Project:
 * --------
 *    Audio speaker
 *
 * Description:
 * ------------
 *   Audio speaker function
 *
 * Author:
 * -------
 * Chipeng Chang
 *
 *------------------------------------------------------------------------------
 *
 *
 *******************************************************************************/

#ifndef _AUDIO_CODEC_SPEAKER_H
#define _AUDIO_CODEC_SPEAKER_H

void Speaker_ClassD_Open(void);
void Speaker_ClassD_close(void);
void Speaker_ClassAB_Open(void);
void Speaker_ClassAB_close(void);
void Speaker_ReveiverMode_Open(void);
void Speaker_ReveiverMode_close(void);
bool GetSpeakerOcFlag(void);

#endif
