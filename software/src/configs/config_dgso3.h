/* o3-bricklet
 * Copyright (C) 2019 Olaf Lüke <olaf@tinkerforge.com>
 *
 * config_dgso3.h: Config for DGS-O3 sensor
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

#ifndef CONFIG_DGSO3_H
#define CONFIG_DGS03_H

#include "xmc_common.h"
#include "xmc_gpio.h"

#define DGSO3_USIC_CHANNEL        USIC1_CH1
#define DGSO3_USIC                XMC_UART1_CH1

#define DGSO3_RX_PIN              P2_13
#define DGSO3_RX_INPUT            XMC_USIC_CH_INPUT_DX0
#define DGSO3_RX_SOURCE           0b011 // DX0D

#define DGSO3_TX_PIN              P2_12
#define DGSO3_TX_PIN_AF           (XMC_GPIO_MODE_OUTPUT_PUSH_PULL_ALT7 | P2_12_AF_U1C1_DOUT0)

#define DGSO3_SERVICE_REQUEST_RX  2  // receive
#define DGSO3_SERVICE_REQUEST_TX  3  // transfer

#define DGSO3_IRQ_RX              11
#define DGSO3_IRQ_RX_PRIORITY     0
#define DGSO3_IRQCTRL_RX          XMC_SCU_IRQCTRL_USIC1_SR2_IRQ11

#endif