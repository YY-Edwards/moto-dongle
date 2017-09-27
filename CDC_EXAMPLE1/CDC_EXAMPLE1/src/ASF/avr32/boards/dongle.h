/*******************************************************************************
*
*                         C  H E A D E R  F I L E
*
*            COPYRIGHT 2010 MOTOROLA, INC. ALL RIGHTS RESERVED.
*                    MOTOROLA CONFIDENTIAL RESTRICTED
*
********************************************************************************/
/*
 * dongle.h
 *
 *  Created on: Sep 10, 2009
 *      Author: cmr003
 */

#ifndef DONGLE_H_
#define DONGLE_H_
#include <avr32/io.h>
#include "compiler.h"

#define FOSC0           12000000                              //!< Osc0 frequency: Hz.
#define OSC0_STARTUP    AVR32_PM_OSCCTRL0_STARTUP_2048_RCOSC  //!< Osc0 startup time: RCOsc periods.

#define USB_ID                      AVR32_USBB_USB_ID_0_1

//	LED_YELLOW				[   PortA Pin  7  00000080]
//  LED_GREEN1              [   PortA Pin  8  00000100]
//  LED_RED                 [   PortA Pin  9  00000200]
//  LED_GREEN2              [   PortA Pin  10 00000400]
//  AMBE_RQST               [   PortA Pin  11 00000800]
//  AMBE_RESET              [   PortA Pin  20 00100000]
//  AMBE_RDY                [   PortA Pin  12 00001000] Input
//  AMBE_RUNNING            [   PortA Pin  13 00002000] Input
//  AMBE_IDLE               [   PortA Pin  14 00004000] Input
//  AMBE_STANDBY            [   PortA Pin  15 00008000] Input

#define PDCA_CHANNEL_USARTTX_EXAMPLE 0
#define PDCA_CHANNEL_USARTRX_EXAMPLE 1
// USART1
//  UART_TXtoAMBE           [   PortB Pin  2  00000004] Function C
//  UART_RXfoAMBE           [   PortB Pin  3  00000008] Function C
//  UART_ImCTS              [   PortB Pin  4  00000010] Function A
//  STANDBY_ENABLE          [   PortB Pin  5  00000020] GPIO

#endif /* DONGLE_H_ */
