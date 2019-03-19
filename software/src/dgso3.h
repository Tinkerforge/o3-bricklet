/* o3-bricklet
 * Copyright (C) 2019 Olaf Lüke <olaf@tinkerforge.com>
 *
 * dgs03.h: Driver for DGS-O3 sensor
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the
 * Free Software Foundation, Inc., 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 */

#ifndef DGSO3_H
#define DGSO3_H

#include <stdint.h>

#include "bricklib2/utility/ringbuffer.h"

#define DGSO3_BUFFER_SIZE 1024
#define DGS03_VALUES_SIZE 11

typedef struct {
	uint8_t buffer_rx[DGSO3_BUFFER_SIZE];
	Ringbuffer ringbuffer_rx;

    char buffer[DGSO3_BUFFER_SIZE+1];
    uint16_t buffer_index;

    int64_t values[DGS03_VALUES_SIZE];

    int64_t sn;
    uint16_t o3;
    int16_t temperature;
    uint16_t humidity;

} DGSO3;

extern DGSO3 dgso3;

void dgso3_tick(void);
void dgso3_init(void);
uint16_t dgso3_get_o3(void);

#endif