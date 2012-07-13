/*
 * TPA6130A2 driver platform header
 *
 * Copyright (C) Bticino S.p.A.
 *
 * Written by Raffaele Recalcati <raffaele.recalcati@bticino.it>
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
 */

#ifndef TPA2016D2_PLAT_H
#define TPA2016D2_PLAT_H

struct tpa2016d2_platform_data {
	int shutdown_gpio;
};

#endif
