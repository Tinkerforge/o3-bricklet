/* o3-bricklet
 * Copyright (C) 2019 Olaf Lüke <olaf@tinkerforge.com>
 *
 * dgs03.c: Driver for DGS-O3 sensor
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

#include "dgso3.h"

#include "configs/config_dgso3.h"

#include "bricklib2/hal/system_timer/system_timer.h"
#include "bricklib2/utility/ringbuffer.h"
#include "bricklib2/utility/util_definitions.h"
#include "bricklib2/logging/logging.h"

#include "xmc_uart.h"
#include "xmc_scu.h"

#include <string.h>
#include <stdlib.h>

DGSO3 dgso3;

// Set const pointer to rx ringbuffer variables.
// With this the compiler can properly optimize the access!
uint8_t  *const dgso3_ringbuffer_rx_buffer = &(dgso3.buffer_rx[0]);
uint16_t *const dgso3_ringbuffer_rx_end    = &(dgso3.ringbuffer_rx.end);
uint16_t *const dgso3_ringbuffer_rx_start  = &(dgso3.ringbuffer_rx.start);
uint16_t *const dgso3_ringbuffer_rx_size   = &(dgso3.ringbuffer_rx.size);

#define dgso3_rx_irq_handler  IRQ_Hdlr_11
void __attribute__((optimize("-O3"))) __attribute__ ((section (".ram_code"))) dgso3_rx_irq_handler(void) {
	while(!XMC_USIC_CH_RXFIFO_IsEmpty(DGSO3_USIC)) {
		// Instead of ringbuffer_add() we add the byte to the buffer
		// by hand.
		//
		// We need to save the low watermark calculation overhead.

		uint16_t new_end = *dgso3_ringbuffer_rx_end + 1;

		if(new_end >= *dgso3_ringbuffer_rx_size) {
			new_end = 0;
		}

		if(new_end == *dgso3_ringbuffer_rx_start) {
			// In the case of an overrun we read the byte and throw it away.
			volatile uint8_t __attribute__((unused)) _  = DGSO3_USIC->OUTR;
		} else {
			dgso3_ringbuffer_rx_buffer[*dgso3_ringbuffer_rx_end] = DGSO3_USIC->OUTR;
			*dgso3_ringbuffer_rx_end = new_end;
		}
	}
}

void dgso3_print_frame(void) {
	uartbb_printf("Values:\n\r");
	uartbb_printf(" * SN: %d %d\n\r",   (int32_t)((int64_t)(dgso3.values[0] / 1000000000LL)), (int32_t)((int64_t)(dgso3.values[0] % 1000000000LL)));
	uartbb_printf(" * PPB: %d\n\r",     (int32_t)dgso3.values[1]);
	uartbb_printf(" * T (°C): %d\n\r",  (int32_t)dgso3.values[2]);
	uartbb_printf(" * RH (\%): %d\n\r", (int32_t)dgso3.values[3]);
	uartbb_printf(" * ADC Raw: %d\n\r", (int32_t)dgso3.values[4]);
	uartbb_printf(" * T Raw: %d\n\r",   (int32_t)dgso3.values[5]);
	uartbb_printf(" * RH Raw: %d\n\r",  (int32_t)dgso3.values[6]);
	uartbb_printf(" * Day: %d\n\r",     (int32_t)dgso3.values[7]);
	uartbb_printf(" * Hour: %d\n\r",    (int32_t)dgso3.values[8]);
	uartbb_printf(" * Minute: %d\n\r",  (int32_t)dgso3.values[9]);
	uartbb_printf(" * Second: %d\n\r",  (int32_t)dgso3.values[10]);
	uartbb_printf("\n\r");
}

void dgso3_write(const char command) {
	DGSO3_USIC->IN[0] = command;
}

void dgso3_parse_values(void) {
	if(dgso3.sn != 0) {
		if(dgso3.sn != dgso3.values[0]) {
			loge("Unexpected serial number: %d %d vs %d %d\n\r",
			     (int32_t)((int64_t)(dgso3.values[0] / 1000000000LL)), (int32_t)((int64_t)(dgso3.values[0] % 1000000000LL)),
			     (int32_t)((int64_t)(dgso3.sn / 1000000000LL)), (int32_t)((int64_t)(dgso3.sn % 1000000000LL)));
			return;
		}
	} else {
		dgso3.sn = dgso3.values[0];	
	}

	dgso3.o3          = MAX(0, dgso3.values[1]);
	dgso3.temperature = dgso3.values[2];
	dgso3.humidity    = MAX(0, dgso3.values[3]);
}

bool dgso3_parse_buffer(void) {
	if((dgso3.buffer[dgso3.buffer_index-1] == '\n') &&
	   (dgso3.buffer[dgso3.buffer_index-2] == '\r')) {
		dgso3.buffer[dgso3.buffer_index-1] = '\0';
		dgso3.buffer[dgso3.buffer_index-2] = '\0';
	} else {
		loge("Malformed buffer (no \\r\\n): %d, %d\n\r", dgso3.buffer[dgso3.buffer_index-2], dgso3.buffer[dgso3.buffer_index-1]);
		return false;
	}

	uint8_t i = 0;
	char *cur = dgso3.buffer;
	while(true) {
		char *end = strchr(cur, ',');
		if(end != NULL) {
			*end = '\0';
		}

		dgso3.values[i] = atoll(cur);
		i++;

		if(end == NULL) {
			if(i != DGS03_VALUES_SIZE) {
				loge("Malformed buffer (too few values): %d\n\r", i);
				return false;
			}
			break;
		}
		
		if(i >= DGS03_VALUES_SIZE) {
			loge("Malformed buffer (too many values)\n\r");
			return false;
		}

		cur = end+1;

	}

	dgso3_print_frame();

	return true;
}

void dgso3_handle_data(uint8_t data) {
	dgso3.buffer[dgso3.buffer_index] = data;
	dgso3.buffer_index++;

	if(data == '\n') {
		dgso3.buffer[dgso3.buffer_index] = '\0';
		if(dgso3_parse_buffer()) {
			dgso3_parse_values();
		}

		memset(dgso3.buffer, 0, DGSO3_BUFFER_SIZE);
		dgso3.buffer_index = 0;
	} else if(dgso3.buffer_index >= DGSO3_BUFFER_SIZE) {
		memset(dgso3.buffer, 0, DGSO3_BUFFER_SIZE);
		dgso3.buffer_index = 0;

		loge("Buffer Overflow\n\r");
	}
}

void dgso3_init_hardware(void) {
	// TX pin configuration
	const XMC_GPIO_CONFIG_t tx_pin_config = {
		.mode             = DGSO3_TX_PIN_AF,
		.output_level     = XMC_GPIO_OUTPUT_LEVEL_HIGH
	};

	// RX pin configuration
	const XMC_GPIO_CONFIG_t rx_pin_config = {
		.mode             = XMC_GPIO_MODE_INPUT_PULL_UP,
		.input_hysteresis = XMC_GPIO_INPUT_HYSTERESIS_STANDARD
	};

	// Configure pins
	XMC_GPIO_Init(DGSO3_TX_PIN, &tx_pin_config);
	XMC_GPIO_Init(DGSO3_RX_PIN, &rx_pin_config);

	// Initialize USIC channel in UART master mode
	// USIC channel configuration
	XMC_UART_CH_CONFIG_t config;
	config.oversampling = 16;
	config.frame_length = 8;
	config.baudrate     = 9600;
	config.stop_bits    = 1;
	config.data_bits    = 8;
	config.parity_mode  = XMC_USIC_CH_PARITY_MODE_NONE;
	XMC_UART_CH_Init(DGSO3_USIC, &config);

	// Set input source path
	XMC_UART_CH_SetInputSource(DGSO3_USIC, DGSO3_RX_INPUT, DGSO3_RX_SOURCE);

	// Configure transmit FIFO
	XMC_USIC_CH_TXFIFO_Configure(DGSO3_USIC, 32, XMC_USIC_CH_FIFO_SIZE_32WORDS, 16);

	// Configure receive FIFO
	XMC_USIC_CH_RXFIFO_Configure(DGSO3_USIC, 0, XMC_USIC_CH_FIFO_SIZE_32WORDS, 0);

	// Set service request for tx FIFO transmit interrupt
	XMC_USIC_CH_TXFIFO_SetInterruptNodePointer(DGSO3_USIC, XMC_USIC_CH_TXFIFO_INTERRUPT_NODE_POINTER_STANDARD, DGSO3_SERVICE_REQUEST_TX);

	// Set service request for rx FIFO receive interrupt
	XMC_USIC_CH_RXFIFO_SetInterruptNodePointer(DGSO3_USIC, XMC_USIC_CH_RXFIFO_INTERRUPT_NODE_POINTER_STANDARD, DGSO3_SERVICE_REQUEST_RX);
	XMC_USIC_CH_RXFIFO_SetInterruptNodePointer(DGSO3_USIC, XMC_USIC_CH_RXFIFO_INTERRUPT_NODE_POINTER_ALTERNATE, DGSO3_SERVICE_REQUEST_RX);

	// Set priority and enable NVIC node for receive interrupt
	NVIC_SetPriority((IRQn_Type)DGSO3_IRQ_RX, DGSO3_IRQ_RX_PRIORITY);
	XMC_SCU_SetInterruptControl(DGSO3_IRQ_RX, DGSO3_IRQCTRL_RX);
	NVIC_EnableIRQ((IRQn_Type)DGSO3_IRQ_RX);

	// Start UART
	XMC_UART_CH_Start(DGSO3_USIC);

	XMC_USIC_CH_EnableEvent(DGSO3_USIC, XMC_USIC_CH_EVENT_ALTERNATIVE_RECEIVE);
	XMC_USIC_CH_RXFIFO_EnableEvent(DGSO3_USIC, XMC_USIC_CH_RXFIFO_EVENT_CONF_STANDARD | XMC_USIC_CH_RXFIFO_EVENT_CONF_ALTERNATE);
}

void dgso3_init_buffer(void) {
	// Disable interrupts so we can't accidentally
	// receive ringbuffer_adds in between a re-init
	NVIC_DisableIRQ((IRQn_Type)DGSO3_IRQ_RX);
	__DSB();
	__ISB();

	// Initialize dgso3 buffer
	memset(dgso3.buffer_rx, 0, DGSO3_BUFFER_SIZE);
	ringbuffer_init(&dgso3.ringbuffer_rx, DGSO3_BUFFER_SIZE, dgso3.buffer_rx);

	NVIC_EnableIRQ((IRQn_Type)DGSO3_IRQ_RX);
}

void dgso3_tick(void) {
	while(ringbuffer_get_used(&dgso3.ringbuffer_rx) > 0) {
		uint8_t data = 0;
		ringbuffer_get(&dgso3.ringbuffer_rx, &data);
		dgso3_handle_data(data);
	}
}

void dgso3_init(void) {
	memset(&dgso3, 0, sizeof(DGSO3));

	dgso3_init_buffer();
	dgso3_init_hardware();

	dgso3_write('a'); // any key (wakeup)
	dgso3_write('c'); // continuous measurement
}

uint16_t dgso3_get_o3(void) {
	return 0;
}