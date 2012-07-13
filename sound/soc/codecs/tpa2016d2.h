/*
 * ALSA SoC TPA6130A2 amplifier driver
 *
 * Copyright (C) Nokia Corporation
 *
 * Author: Peter Ujfalusi <peter.ujfalusi@nokia.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA
 * 02110-1301 USA
 *
 */

#ifndef __TPA2016D2_H__
#define __TPA2016D2_H__

/* Register addresses */
#define TPA2016D2_REG_CONTROL		0x01
#define TPA2016D2_REG_AGC_ATTACH	0x02
#define TPA2016D2_REG_AGC_RELEASE	0x03
#define TPA2016D2_REG_AGC_HOLD		0x04
#define TPA2016D2_REG_AGC_FIXED_GAIN	0x05
#define TPA2016D2_REG_AGC_CONTROL	0x06
#define TPA2016D2_REG_AGC_CONTROL_1	0x07

#define TPA2016D2_CACHEREGNUM	(TPA2016D2_REG_AGC_CONTROL_1 + 1)

/* Register bits */

/* TPA2016D2_REG_CONTROL (0x01) */
#define TPA2016D2_NG_EN			(0x01 << 0)
#define TPA2016D2_THERMAL		(0x01 << 2)
#define TPA2016D2_FAULT_L		(0x01 << 3)
#define TPA2016D2_FAULT_R		(0x01 << 4)
#define TPA2016D2_SWS			(0x01 << 5)
#define TPA2016D2_SPK_EN_L		(0x01 << 6)
#define TPA2016D2_SPK_EN_R		(0x01 << 7)


/* #define TPA2016D2_REG_AGC_CONTROL	0x06 */
#define TPA2016D2_OUT_LIM_EN		(0x01 << 7)
#define TPA2016D2_NG_THRES		(0x03 << 5)
#define TPA2016D2_OUT_LIM_LEV		(0x01f << 0)


/* #define TPA2016D2_REG_AGC_CONTROL_1	0x07 */
#define TPA2016D2_MAX_GAIN		(0xf << 4)
#define TPA2016D2_COMPR_RATIO		(0x3 << 0)

#define TPA2016D2_REG_AGC_FIXED_GAIN_24DB 0x18

extern int tpa2016d2_add_controls(struct snd_soc_codec *codec);

#endif /* __TPA6130A2_H__ */