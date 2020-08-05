/*
 * MIT License
 *
 * Copyright (c) 2017 David Antliff
 * Copyright (c) 2017 Chris Morgan <chmorgan@gmail.com>
 *
*/

#include "uart.h"
#include "driver/uart.h"


// static const char * TAG = "uart";

	//setup UART
	const int uart_buffer_size = (1024 * 2);

void uart_init()
{	uart_config_t uart_config = {
		.baud_rate = 115200,
		//.baud_rate = 921600,
		.data_bits = UART_DATA_8_BITS,
		.parity = UART_PARITY_DISABLE,
		.stop_bits = UART_STOP_BITS_1,
		.flow_ctrl = UART_HW_FLOWCTRL_DISABLE};
	uart_param_config(UART_NUM_0, &uart_config);
	uart_set_pin(UART_NUM_0, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
	uart_driver_install(UART_NUM_0, uart_buffer_size, 0, 0, NULL, 0);
}
