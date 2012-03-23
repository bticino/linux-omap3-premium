/*
 * drivers/media/video/tvp51x.h
 *
 * Copyright (C) 2012 Bticino S.p.A.
 * Author: Raffaele Recalcati <raffaele.recalcati@bticino.it>
 *
 *
 *
 * This package is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 *
 */

#ifndef _TVP514X_REGS_H
#define _TVP514X_REGS_H

#define VIDEO_STD_MASK                  (0x07)
#define VIDEO_STD_AUTO_SWITCH_BIT       (0x00)
#define VIDEO_STD_NTSC_MJ_BIT           (0x01)
#define VIDEO_STD_PAL_BDGHIN_BIT        (0x02)
#define VIDEO_STD_PAL_M_BIT             (0x03)
#define VIDEO_STD_PAL_COMBINATION_N_BIT (0x04)
#define VIDEO_STD_NTSC_4_43_BIT         (0x05)
#define VIDEO_STD_SECAM_BIT             (0x06)
#define VIDEO_STD_PAL_60_BIT            (0x07)

