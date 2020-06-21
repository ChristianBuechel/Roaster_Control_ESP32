/*
 * MIT License
 *
 * Copyright (c) 2017 David Antliff
 * Copyright (c) 2017 Chris Morgan <chmorgan@gmail.com>
 *
 * SOFTWARE.
 */

/**
 * @file
 * @brief Interface definitions for the 1-Wire bus component.
 *
 * This component provides structures and functions that are useful for communicating
 * with devices connected to a Maxim Integrated 1-Wire® bus via a single GPIO.
 *
 * Currently only externally powered devices are supported. Parasitic power is not supported.
 */

#ifndef _UART_H_
#define _UART_H_

#ifdef __cplusplus
extern "C" 
{
#endif

void uart_init();

#ifdef __cplusplus
}
#endif
#endif