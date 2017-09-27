/**************************************************************************
 *
 * \file
 *
 * \brief Management of the USB device CDC task.
 *
 * This file manages the USB device CDC task.
 *
 * Copyright (c) 2009-2014 Atmel Corporation. All rights reserved.
 *
 * \asf_license_start
 *
 * \page License
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. The name of Atmel may not be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * 4. This software may only be redistributed and used in connection with an
 *    Atmel microcontroller product.
 *
 * THIS SOFTWARE IS PROVIDED BY ATMEL "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT ARE
 * EXPRESSLY AND SPECIFICALLY DISCLAIMED. IN NO EVENT SHALL ATMEL BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * \asf_license_stop
 *
 ***************************************************************************/
 /**
 * Support and FAQ: visit <a href="http://www.atmel.com/design-support/">Atmel Support</a>
 */


//_____  I N C L U D E S ___________________________________________________

#include <stdio.h>
#include "usart.h"     // Shall be included before FreeRTOS header files, since 'inline' is defined to ''; leading to
// link errors
#include "conf_usb.h"
//#include "power_clocks_lib.h"
//#include "print_funcs.h"
//#include "cdc_example.h"


#if USB_DEVICE_FEATURE == true

#include "board.h"
#include "usb_drv.h"
#include "gpio.h"
//#include "intc.h"
#include "usb_descriptors.h"
#include "usb_standard_request.h"
#include "device_cdc_task.h"
#include "uart_usb_lib.h"


//_____ M A C R O S ________________________________________________________


//_____ D E F I N I T I O N S ______________________________________________

//_____ D E C L A R A T I O N S ____________________________________________

static volatile uint16_t  sof_cnt;

#define USART_IRQ           DBG_USART_IRQ
#define USART_RX_BUFFER     64 // Unit is in characters.

//!
//! @brief This function initializes the hardware/software resources
//! required for device CDC task.
//!
void device_cdc_task_init(void)
{
	sof_cnt   =0 ;
	uart_usb_init();

	// Register the USART interrupt handler to the interrupt controller.
	// Highest priority is required for the USART, since we do not want to loose
	// any characters.

#ifndef FREERTOS_USED
#if USB_HOST_FEATURE == true
	// If both device and host features are enabled, check if device mode is engaged
	// (accessing the USB registers of a non-engaged mode, even with load operations,
	// may corrupt USB FIFO data).
	if (Is_usb_device())
#endif  // USB_HOST_FEATURE == true
		//Usb_enable_sof_interrupt();
#endif  // FREERTOS_USED


		Usb_enable_sof_interrupt();

}


//!
//! @brief Entry point of the device CDC task management
//!

void device_cdc_task(void)
{
	uint8_t c;
	int status;
	static bool b_startup=true;


		// First, check the device enumeration state
		if (!Is_device_enumerated()) return;

		if (b_startup) {
			printf("\r\nUSB DEVICE Communications Device Class demo.\r\n");
			b_startup=false;
		}

		if (sof_cnt >= NB_MS_BEFORE_FLUSH) { //Flush buffer in Timeout
			sof_cnt=0;
			uart_usb_flush();
		}


		if (uart_usb_test_hit()) { // Something received from the USB ?
			//if (usart_tx_ready(DBG_USART)) { // USART free ?
				c = uart_usb_getchar();
				//usart_write_char(DBG_USART, c);   // loop back USB to USART
			}
		//}

	//}
}


//!
//! @brief usb_sof_action
//!
//! This function increments the sof_cnt counter each time
//! the USB Start-of-Frame interrupt subroutine is executed (1 ms).
//! Useful to manage time delays
//!
void usb_sof_action(void)
{
	sof_cnt++;
}

#endif  // USB_DEVICE_FEATURE == true
