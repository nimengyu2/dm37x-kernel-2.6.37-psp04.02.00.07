/*
 * Helper module for board specific I2C bus registration
 *
 * Copyright (C) 2009 Nokia Corporation.
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
// 经过实际测试 在beagle的配置中，这个文件需要被编译进去
#include <plat/cpu.h>
#include <plat/i2c.h>

#include "mux.h"

void __init omap2_i2c_mux_pins(int bus_id)
{
	char mux_name[sizeof("i2c2_scl.i2c2_scl")];

	/* First I2C bus is not muxable */
	if (bus_id == 1)
		return;

	// 初始化i2c的引脚
	sprintf(mux_name, "i2c%i_scl.i2c%i_scl", bus_id, bus_id);
	omap_mux_init_signal(mux_name, OMAP_PIN_INPUT);
	sprintf(mux_name, "i2c%i_sda.i2c%i_sda", bus_id, bus_id);
	omap_mux_init_signal(mux_name, OMAP_PIN_INPUT);
}
