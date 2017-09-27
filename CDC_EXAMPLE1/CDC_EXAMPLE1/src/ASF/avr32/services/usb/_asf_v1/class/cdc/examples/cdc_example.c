/**
 * \file
 *
 * \mainpage
 * \section title Main file of the USB CDC example.
 *
 * \section file File(s)
 * - \ref cdc_example.c
 * - \ref device_cdc_task.c
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
 ******************************************************************************/
 /**
 * Support and FAQ: visit <a href="http://www.atmel.com/design-support/">Atmel Support</a>
 */


//_____  I N C L U D E S ___________________________________________________

#ifndef FREERTOS_USED
#if (defined __GNUC__)
#include "nlao_cpu.h"
#include "nlao_usart.h"
#endif
#else
#include <stdio.h>
#endif
#include "compiler.h"
#include "board.h"
//#include "print_funcs.h"
#include "intc.h"
#include "power_clocks_lib.h"
#include "cdc_example.h"
#ifdef FREERTOS_USED
#include "FreeRTOS.h"
#include "task.h"
#endif
#include "conf_usb.h"
#include "usb_task.h"
#if USB_DEVICE_FEATURE == true
#include "device_cdc_task.h"
#endif
#if USB_HOST_FEATURE == true
#include "host_cdc_task.h"
#endif
#include <stdio.h>


#include "usb_drv.h"
#include "usb_descriptors.h"
#include "usb_standard_request.h"
#include "uart_usb_lib.h"
#include "usart.h"
#include "dongle.h"



//_____ M A C R O S ________________________________________________________

static const U16 VOICEOVERHEAD[4] = {
	0x6101, 0x4402, 0x00A0, 0x2F00  //This includes parity in bytecount.
};

static const U8 AMBEP25HALFRATE[8] = {	//不对，之所以都可以，是因为之前的PC上软件的会先对dongle进行配置。（经测试dongle驱动里设置不带FEC功能，和设置为带FEC功能都可以正常解码...）
										//也即是这里必须配置成索引码：33（带FEC，科立讯的语音才能正常解码）
										//MOTO 例子中设定的是按索引方式（0x09）,索引码为：0x22.现在科立讯中我们需要设置索引码为：0x21(index:33){Total Rate(bps):3600=Speech Rate(bps):2450+FEC Rate(bps):1150.}
	
	0x61, 0x00, 0x04, 0x00, 0x09, 0x22, 0x2F, 0x00
	//0x61, 0x00, 0x04, 0x00, 0x09, 0x21, 0x2F, 0x03//最后一字节的值为校验值，是(不包括0x61)异或校验值。
};
	
static const U8 AMBEP25HALFRATERESPONSE[8] = {
	0x61, 0x00, 0x04, 0x00, 0x09, 0x00, 0x2F, 0x22//最后一字节的值为校验值，是(不包括0x61)异或校验值。
};

typedef enum {
	WAITINGFORENUM,
	WAKINGDVSICHIP,
	INITDVSICHIP,
	RUNNING
}_Dongle_states;

typedef enum {
	WAITINGFORSTART,
	WAITINGFORLENGTHMSB,
	WAITINGFORLENGTHLSB,
	WAITINGFORTYPE,
	READINGIN,
	DISCARDING
} _USB_Rx_states;

typedef enum {
	WAITINGFORUART,
	WRITING
} _USB_Tx_states;

#define BYTESINPCM 328
#define DVSISTARTBYTE 0x61
typedef union {
	U16  RAW_hword[BYTESINPCM/2];
	U8   RAW_Bytes[BYTESINPCM];
	U32  RAW_Words[BYTESINPCM/4];
}_PCMBlock;

#define MAXBUFFERCOUNT  4
#define BUFFERCOUNTMASK 0x0003
_PCMBlock  Buffer1[MAXBUFFERCOUNT];
_PCMBlock  Buffer2[MAXBUFFERCOUNT];

U32 Dongle_state;
U32 USB_Rx_state;
U32 USB_Tx_state;
S16 bytesexpected1;
S16 bytesarrived1[MAXBUFFERCOUNT];
S16 HeadIndex1;
S16 TailIndex1;
S16 bytestosend2;
S16 bytessent2;
S16 HeadIndex2;
S16 TailIndex2;



S32  MonitorIndex;
volatile U16  sof_cnt;

#define NUMSTAMPS 64
U32 TimeStamps[NUMSTAMPS];
U32 mParam1[NUMSTAMPS];
U32 mParam2[NUMSTAMPS];
S16 SofCounter[NUMSTAMPS];
S16 EventStamps[NUMSTAMPS];



//_____ D E F I N I T I O N S ______________________________________________

pm_freq_param_t   pm_freq_param=
{
	   .cpu_f  =       APPLI_CPU_SPEED
	,  .pba_f    =     APPLI_PBA_SPEED
	,  .osc0_f     =   FOSC0
	,  .osc0_startup = OSC0_STARTUP
};

void FlagEvent(S16 theEvent, U32 Param1, U32 Param2){

	if (NUMSTAMPS != MonitorIndex){
		TimeStamps[MonitorIndex] = Get_system_register(AVR32_COUNT);
		SofCounter[MonitorIndex] = sof_cnt;
		EventStamps[MonitorIndex] = theEvent;
		mParam1[MonitorIndex] = Param1;
		mParam2[MonitorIndex] = Param2;
		MonitorIndex++;
	}

}


/*! \name System Clock Frequencies
 */
//! @{
pcl_freq_param_t pcl_freq_param =
{
  .cpu_f        = APPLI_CPU_SPEED,
  .pba_f        = APPLI_PBA_SPEED,
  .osc0_f       = FOSC0,
  .osc0_startup = OSC0_STARTUP
};
//! @}


#ifndef FREERTOS_USED

  #if (defined __GNUC__)

/*! \brief Low-level initialization routine called during startup, before the
 *         main function.
 *
 * This version comes in replacement to the default one provided by the Newlib
 * add-ons library.
 * Newlib add-ons' _init_startup only calls init_exceptions, but Newlib add-ons'
 * exception and interrupt vectors are defined in the same section and Newlib
 * add-ons' interrupt vectors are not compatible with the interrupt management
 * of the INTC module.
 * More low-level initializations are besides added here.
 */
int _init_startup(void)
{
  // Import the Exception Vector Base Address.
  extern void _evba;

  // Load the Exception Vector Base Address in the corresponding system register.
  Set_system_register(AVR32_EVBA, (int)&_evba);

  // Enable exceptions.
  Enable_global_exception();

  // Initialize interrupt handling.
  INTC_init_interrupts();

  // Initialize the USART used for the debug trace with the configured parameters.
  //set_usart_base( ( void * ) DBG_USART );

  // Don't-care value for GCC.
  return 1;
}

  #elif __ICCAVR32__

/*! \brief Low-level initialization routine called during startup, before the
 *         main function.
 */

  #endif  // Compiler

#endif  // FREERTOS_USED

S16 my_usb_test_hit(void)
{
	S32 num_chars_in;

	num_chars_in = 0;
	if( Is_usb_out_received(RX_EP) ) //reads RXOUTI bit
	{
		num_chars_in = Usb_byte_count(RX_EP);
		Usb_reset_endpoint_fifo_access(RX_EP);
		if( num_chars_in == 0 )
		{
			Usb_ack_out_received_free(RX_EP);
		}
	}
	return num_chars_in;
}

void depleteTxBuffer(void)
{
	TailIndex2 = (TailIndex2 + 1) & BUFFERCOUNTMASK;
	USB_Tx_state = WAITINGFORUART;
}


void my_device_cdc_TXtask(void)
{
	S32 i;

	if (HeadIndex2 != TailIndex2){
		switch (USB_Tx_state){
			case   WAITINGFORUART:
			i = Buffer2[TailIndex2].RAW_Words[0];
			if (0x61000000 != (i & 0xFF000000)){//判断数据包的起始字节是否正确，即是否为有效数据
				depleteTxBuffer();
				break;
			}


			i = ((i & 0x00FFFF00)>>8) + 4;
			if (i > BYTESINPCM){//判断数据包的长度是否大于预设值
				depleteTxBuffer();
				break;
			}
			bytestosend2 = i;
			bytessent2   = 0;
			USB_Tx_state = WRITING;
			//Fall through. Do not break.
			case WRITING:
			if( uart_usb_tx_ready() ){                    //This *should* be a whole 64 byte buffer,
				Usb_reset_endpoint_fifo_access(TX_EP);    //So reset the access.
				for (i = 0; i < 64; i++){
					Usb_write_endpoint_data(TX_EP, 8, Buffer2[TailIndex2].RAW_Bytes[bytessent2++]);
					if (bytessent2 == bytestosend2)break; //break out of loop
				}
				Usb_ack_in_ready_send(TX_EP);
				if (bytessent2 == bytestosend2){
					depleteTxBuffer();
				}
			}
			break;
		}
	}
}

void my_device_cdc_RXtask(void)
{
	//U32 debug;
	static S32 num_chars_in;
	U8 c;


	while (0 != (num_chars_in = my_usb_test_hit())){ //Keep going back for entire millisecond USB frame
		while (0 != num_chars_in--){
			c = Usb_read_endpoint_data(RX_EP, 8);

			switch (USB_Rx_state){
				case   WAITINGFORSTART:
				if (DVSISTARTBYTE == c){//判断数据包起始为是否=0x61
					bytesarrived1[HeadIndex1] = 0;
					Buffer1[HeadIndex1].RAW_Bytes[bytesarrived1[HeadIndex1]++] = c;
					USB_Rx_state = WAITINGFORLENGTHMSB;
				}
				break;
				case  WAITINGFORLENGTHMSB:
				bytesexpected1 = c;
				Buffer1[HeadIndex1].RAW_Bytes[bytesarrived1[HeadIndex1]++] = c;
				USB_Rx_state = WAITINGFORLENGTHLSB;
				break;
				case  WAITINGFORLENGTHLSB:
				bytesexpected1 = (bytesexpected1<<8) + c;
				Buffer1[HeadIndex1].RAW_Bytes[bytesarrived1[HeadIndex1]++] = c;
				if (bytesexpected1 > (BYTESINPCM - 4)){//USB接收的数据长度判断
					USB_Rx_state = WAITINGFORSTART;
					}else{
					USB_Rx_state = WAITINGFORTYPE;
				}
				break;
				case WAITINGFORTYPE:
				Buffer1[HeadIndex1].RAW_Bytes[bytesarrived1[HeadIndex1]++] = c;
				USB_Rx_state = READINGIN;
				break;
				case  READINGIN:
				Buffer1[HeadIndex1].RAW_Bytes[bytesarrived1[HeadIndex1]++] = c;
				if (0 == (--bytesexpected1)){
					FlagEvent(1, HeadIndex1, Buffer1[HeadIndex1].RAW_Words[0]);
					AVR32_GPIO.port[0].ovrs  =  0x00000400;
					HeadIndex1 = (HeadIndex1 + 1) & BUFFERCOUNTMASK;
					if (HeadIndex1 == TailIndex1){ //Collision
						HeadIndex1 = (HeadIndex1 - 1) & BUFFERCOUNTMASK;
					}
					USB_Rx_state = WAITINGFORSTART;
				}
				break;
				case DISCARDING:
				break;
			}
		}
		Usb_ack_out_received_free(RX_EP);
	}

	if (TailIndex1 != HeadIndex1){ //Something new, or still in progress.
		if (0 != (((&AVR32_PDCA.channel[PDCA_CHANNEL_USARTTX_EXAMPLE])->isr) & 0x00000002)){//传输是否完成
			//Was something in progress?
			if (0 == bytesarrived1[TailIndex1]){
				//Release Buffer
				TailIndex1 = (TailIndex1 - 1) & BUFFERCOUNTMASK;
			}
			if (TailIndex1 != HeadIndex1){//Anything new?
				(&AVR32_PDCA.channel[PDCA_CHANNEL_USARTTX_EXAMPLE])->mar = (U32)&Buffer1[TailIndex1].RAW_Bytes[0];
				(&AVR32_PDCA.channel[PDCA_CHANNEL_USARTTX_EXAMPLE])->tcr = bytesarrived1[TailIndex1];   
				bytesarrived1[TailIndex1] = 0; //Mark DMA has started.
			}
		}
	}
}

void init_usart(void)
{
	//  UART_TXtoAMBE           [   PortB Pin  2  00000004] Function C
	//  UART_RXfoAMBE           [   PortB Pin  3  00000008] Function C
	//  UART_myCTS              [   PortB Pin  4  00000010] Function A
	//  STANDBY_ENABLE          [   PortB Pin  5  00000020] GPIO
	AVR32_GPIO.port[1].pmr0c  = 0x0000001C;
	AVR32_GPIO.port[1].pmr1s  = 0x0000000C;
	AVR32_GPIO.port[1].pmr1c  = 0x00000010;
	AVR32_GPIO.port[1].gperc  = 0x0000001C;
	AVR32_GPIO.port[1].oders =  0x00000020;
	AVR32_GPIO.port[1].ovrs =   0x00000020;
	AVR32_GPIO.port[1].gpers =  0x00000020;

	// pba_hz = 24000000
	// baudrate = 230400
	//pba_hz >= 16 * baudrate so OVER = 16
	//cd = pba_hz / (over * baudrate) = 6  (.51)

	(&AVR32_USART1)->idr = 0xFFFFFFFF; //Some of this shouldn't be needed...
	(&AVR32_USART1)->csr;
	(&AVR32_USART1)->rtor = 0;
	(&AVR32_USART1)->ttgr = 0;
	(&AVR32_USART1)->cr = AVR32_USART_CR_RSTRX_MASK   |
	AVR32_USART_CR_RSTTX_MASK   |
	AVR32_USART_CR_RSTSTA_MASK  |
	AVR32_USART_CR_RSTIT_MASK   |
	AVR32_USART_CR_RSTNACK_MASK |
	AVR32_USART_CR_DTRDIS_MASK  |
	AVR32_USART_CR_RTSDIS_MASK;
	(&AVR32_USART1)->mr = AVR32_USART_MR_MODE_HARDWARE << AVR32_USART_MR_MODE_OFFSET
	|AVR32_USART_MR_USCLKS_MCK << AVR32_USART_MR_USCLKS_OFFSET
	| (8 - 5) << AVR32_USART_MR_CHRL_OFFSET
	| USART_NO_PARITY << AVR32_USART_MR_PAR_OFFSET
	| USART_1_STOPBIT << AVR32_USART_MR_NBSTOP_OFFSET
	| AVR32_USART_MR_MODE_NORMAL << AVR32_USART_MR_MODE_OFFSET
	| AVR32_USART_MR_OVER_X16 << AVR32_USART_MR_OVER_OFFSET;

	(&AVR32_USART1)->brgr = 6 << AVR32_USART_BRGR_CD_OFFSET
	| 4 << AVR32_USART_BRGR_FP_OFFSET;


}

void armNextRxBuffer(void){
	(&AVR32_PDCA.channel[PDCA_CHANNEL_USARTRX_EXAMPLE])->cr = 0x00000002; //Stop Rx DMA.
	(&AVR32_USART1)->cr = AVR32_USART_CR_RXDIS_MASK
	| AVR32_USART_CR_TXEN_MASK; //Disable USART Rx.
	(&AVR32_USART1)->rtor = 5; //SetRx timeout to 5 bits.
	(&AVR32_USART1)->cr = AVR32_USART_CR_RSTSTA_MASK
	| AVR32_USART_CR_STTTO_MASK
	| AVR32_USART_CR_TXEN_MASK;
	HeadIndex2 = (HeadIndex2 + 1) & BUFFERCOUNTMASK;
	if (HeadIndex2 == TailIndex2){//Collision
		HeadIndex2 = (HeadIndex2 - 1) & BUFFERCOUNTMASK;
	}
	Buffer2[HeadIndex2].RAW_Words[0] = 0x00000000; //Null header
	(&AVR32_PDCA.channel[PDCA_CHANNEL_USARTRX_EXAMPLE])->mar = (U32)&Buffer2[HeadIndex2].RAW_Bytes[0];
	(&AVR32_PDCA.channel[PDCA_CHANNEL_USARTRX_EXAMPLE])->tcr = BYTESINPCM;//328字节
	(&AVR32_PDCA.channel[PDCA_CHANNEL_USARTRX_EXAMPLE])->cr = 0x00000100; //Clear any error condition.
	(&AVR32_PDCA.channel[PDCA_CHANNEL_USARTRX_EXAMPLE])->isr;
	(&AVR32_USART1)->csr;
	(&AVR32_USART1)->cr = AVR32_USART_CR_RXEN_MASK
	| AVR32_USART_CR_TXEN_MASK;  //Enable USART Rx.
	(&AVR32_PDCA.channel[PDCA_CHANNEL_USARTRX_EXAMPLE])->cr = 0x00000001; //Start Rx DMA.
}

void serviceUSART_Rx(void)
{
	if (0 != ( ((&AVR32_USART1)->csr) & AVR32_USART_CSR_TIMEOUT_MASK)){//
		FlagEvent(2, HeadIndex2, Buffer2[HeadIndex2].RAW_Words[0]);
		armNextRxBuffer();
		AVR32_GPIO.port[0].ovrs  =  0x00000200;
	}
}
















/*! \brief Main function. Execution starts here.
 *
 * \retval 42 Fatal error.
 */
int main(void)
{
  
  U32 i = 0;
  U32 j = 0;
  S32 wakeuptimer;
  S32 LEDblinker;
  
	//Configure system clocks.
	//if (pcl_configure_clocks(&pcl_freq_param) != PASS)
    //return 42;
	// Set CPU and PBA clock
	 if( PM_FREQ_STATUS_FAIL==pm_configure_clocks(&pm_freq_param) )
	 return 43;
	 //return 42;

  // Initialize usart comm
  //init_dbg_rs232(pcl_freq_param.pba_f);

  // Give the used CPU clock frequency to Newlib, so it can work properly.
  //set_cpu_hz(pcl_freq_param.pba_f);
   set_cpu_hz(pm_freq_param.cpu_f);
 
	Disable_global_interrupt();

	AVR32_GPIO.port[0].ovrc  =  0x00100F80;  //Values will be low.
	AVR32_GPIO.port[0].oders =  0x00100F80;  //Output Drivers will be Enabled.
	AVR32_GPIO.port[0].oderc =  0x0000F000;  //Output Drivers will be Disabled.
	AVR32_GPIO.port[0].gpers =  0x0010FF80;  //Enable as GPIO.
	LEDblinker = 0;
	
	//	LED_YELLOW				[   PortA Pin  7  00000080]
	//  LED_GREEN1              [   PortA Pin  8  00000100]
	//  LED_RED                 [   PortA Pin  9  00000200]
	//  LED_GREEN2              [   PortA Pin  10 00000400]
	//  AMBE_RESET              [   PortA Pin  20 00100000]

  // Initialize USB clock.
  //pcl_configure_usb_clock();
  pm_configure_usb_clock();
  
     //Debug monitor.
     for (i=0; i<NUMSTAMPS; i++){
	     TimeStamps[i] = 0;
	     SofCounter[i]= 0;
	     EventStamps[i] = 0;
	     mParam1[MonitorIndex] = 0;
	     mParam2[MonitorIndex] = 0;
     }
     MonitorIndex = 0;


     Dongle_state = WAITINGFORENUM;
     USB_Rx_state = DISCARDING;
     sof_cnt = 0;
     bytesexpected1 = 0;
     HeadIndex1 = 0;
     TailIndex1 = 0;
     bytestosend2 = 0;
     bytessent2 = 0;
     HeadIndex2 = 0;
     TailIndex2 = 0;

     for (i=0; i<MAXBUFFERCOUNT; i++){
	     bytesarrived1[i] = 0;
     }
  
  Set_system_register(AVR32_COUNT, 0);
  
  
   //Enable USART. DVSI chip still heldin reset.
   init_usart();
  
  // Initialize USB task
  usb_task_init();


#if USB_DEVICE_FEATURE == true
  // Initialize device CDC USB task
  device_cdc_task_init();
#endif
#if USB_HOST_FEATURE == true
  // Initialize host CDC USB task
  host_cdc_task_init();
#endif


  // No OS here. Need to call each task in round-robin mode.
  Enable_global_interrupt();
  while (true)
  {
    usb_task();
	
	  switch (Dongle_state){
		  case WAITINGFORENUM:
		  if (Is_device_enumerated()){
			  //Perform any local DVSI chip initialization.
			  AVR32_GPIO.port[0].ovrs  =  0x00100800; //Enable DVSI chip.[ForceRQST High]

			  for (j=0; j<MAXBUFFERCOUNT; j++){
				  for (i=0; i<BYTESINPCM; i++){
					  Buffer1[j].RAW_Bytes[i] = 0;
					  Buffer2[j].RAW_Bytes[i] = 0;
				  }
			  }
			  AVR32_GPIO.port[0].ovrs  =  0x00000780; //ON All LED.
			  LEDblinker = 0;
			  wakeuptimer = Get_system_register(AVR32_COUNT);
			  Dongle_state = WAKINGDVSICHIP;
			  }else{
			  if (6000 < LEDblinker++){
				  LEDblinker = 0;
				  AVR32_GPIO.port[0].ovrt  =  0x00000080; //Blink LED.
			  }
		  }
		  break;
		  case WAKINGDVSICHIP:
		  if ((Get_system_register(AVR32_COUNT) - wakeuptimer) > 24000000){
			  for (i=0; i<8; i++){
				  Buffer1[0].RAW_Bytes[i] = AMBEP25HALFRATE[i];
			  }
			  //There may have been a transient with turning on the DVSI chip.
			  //Clear out anything stuck in USART.
			  (&AVR32_USART1)->csr;
			  (&AVR32_USART1)->rhr;
			  (&AVR32_USART1)->cr = AVR32_USART_CR_RSTSTA_MASK;
			  (&AVR32_USART1)->cr = AVR32_USART_CR_RXEN_MASK
			  | AVR32_USART_CR_TXEN_MASK;

			  (&AVR32_PDCA.channel[PDCA_CHANNEL_USARTTX_EXAMPLE])->psr = AVR32_PDCA_PID_USART1_TX;
			  (&AVR32_PDCA.channel[PDCA_CHANNEL_USARTTX_EXAMPLE])->mr = AVR32_PDCA_BYTE;
			  (&AVR32_PDCA.channel[PDCA_CHANNEL_USARTRX_EXAMPLE])->psr = AVR32_PDCA_PID_USART1_RX;
			  (&AVR32_PDCA.channel[PDCA_CHANNEL_USARTRX_EXAMPLE])->mr = AVR32_PDCA_BYTE;

			  (&AVR32_PDCA.channel[PDCA_CHANNEL_USARTRX_EXAMPLE])->mar = (U32)&Buffer2[0].RAW_Bytes[0];
			  (&AVR32_PDCA.channel[PDCA_CHANNEL_USARTRX_EXAMPLE])->tcr = 8;
			  (&AVR32_PDCA.channel[PDCA_CHANNEL_USARTRX_EXAMPLE])->isr;
			  (&AVR32_PDCA.channel[PDCA_CHANNEL_USARTRX_EXAMPLE])->cr = 0x00000001;//启动接收数据

			  (&AVR32_PDCA.channel[PDCA_CHANNEL_USARTTX_EXAMPLE])->mar = (U32)&Buffer1[0].RAW_Bytes[0];
			  (&AVR32_PDCA.channel[PDCA_CHANNEL_USARTTX_EXAMPLE])->tcr = 8;
			  (&AVR32_PDCA.channel[PDCA_CHANNEL_USARTTX_EXAMPLE])->isr;
			  (&AVR32_PDCA.channel[PDCA_CHANNEL_USARTTX_EXAMPLE])->cr = 0x00000001;//启动发送数据
			  Dongle_state = INITDVSICHIP;
		  }
		  break;
		  case INITDVSICHIP:
		  if ( 0 != (((&AVR32_PDCA.channel[PDCA_CHANNEL_USARTRX_EXAMPLE])->isr) & 0x00000002)){//接收中断是否触发
			  AVR32_GPIO.port[0].ovrc  =  0x00000600; //Two LED's ON.
			  armNextRxBuffer();
			  TailIndex2 = HeadIndex2; //Force this first time.
			  USB_Rx_state = WAITINGFORSTART;
			  USB_Tx_state = WAITINGFORUART;
			  Dongle_state = RUNNING;
		  }
		  break;
		  case RUNNING:
		  serviceUSART_Rx();
		  my_device_cdc_TXtask();
		  if (6000 < LEDblinker++){
			  LEDblinker = 0;
			  AVR32_GPIO.port[0].ovrc  =  0x00000600; //Turn OFF, (activity will turn ON).
		  }
		  break;
	  }

	
  #if USB_DEVICE_FEATURE == true
    //device_cdc_task();
	 my_device_cdc_RXtask();
	
  #endif
  #if USB_HOST_FEATURE == true
    host_cdc_task();
  #endif
  }
 return 0;
}
