/* Copyright 2017, XXXX
 *
 *  * All rights reserved.
 *
 * This file is part of CIAA Firmware.
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
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 */

/** \brief Blinking Bare Metal example source file
 **
 ** This is a mini example of the CIAA Firmware.
 **
 **/

/** \addtogroup CIAA_Firmware CIAA Firmware
 ** @{ */

/** \addtogroup Examples CIAA Firmware Examples
 ** @{ */
/** \addtogroup Baremetal Bare Metal example source file
 ** @{ */

/*
 * Initials     Name
 * ---------------------------
 *
 */

/*
 * modification history (new versions first)
 * -----------------------------------------------------------
 * yyyymmdd v0.0.1 initials initial version
 */

/*==================[inclusions]=============================================*/
#include "tpf_app.h"       /* <= own header */
#include "task.h"
#include "event_groups.h"
#include "stdint.h"
#include "led.h"
#include "switch.h"
#include "uart.h"
#include <string.h>
#include "timers.h"
#include "adc.h"

/*==================[macros and definitions]=================================*/

#define EVENT_START     	( 1 << 0)
#define EVENT_TRANSMIT    	( 1 << 1)
#define EVENT_RECIEVED    	( 1 << 2)

typedef struct {
//   char * comando;
	uint8_t command;
   uint8_t action;
} command_t;

/** time interval of the RTI timer interrupt in [us] */
#define TIME_INTERVAL_US 1000
#define TIME_INTERVAL_MS 10


/*==================[internal data declaration]==============================*/

EventGroupHandle_t eventos;

/*==================[internal functions declaration]=========================*/


/*==================[internal data definition]===============================*/
uint16_t adc_ch1_value = 0;
uint16_t adc_ch2_value = 0;
uint16_t adc_ch3_value = 0;
/*==================[external data definition]===============================*/

/*==================[internal functions definition]==========================*/

/*---------------------------------------------------------------------------*/
void Keyboard(void * parameters) {
	int8_t inputs = 0;
	static uint8_t previousState = 0;

   while(1)
   	{


   		/** Inputs read */
   		inputs = Read_Switches();

   		/* -------- EDU-CIAA board keyboard behavior  -------------- */
   		/*
   		 * Rising edge detector
   		 *
   		 * */
   		if((((inputs ^ previousState) & (1<<TEC1))  != 0)&& ((inputs & (1<<TEC1)) == 0))
   		{
   			Led_Toggle(RGB_B_LED);
   		}

   		if((((inputs ^ previousState) & (1<<TEC2))  != 0)&& ((inputs & (1<<TEC2)) == 0))
   		{

   		}

   		if((((inputs ^ previousState) & (1<<TEC3))  != 0)&& ((inputs & (1<<TEC3)) == 0))
   		{

   		}

   		if((((inputs ^ previousState) & (1<<TEC4))  != 0)&& ((inputs & (1<<TEC4)) == 0))
   		{

   		}


   		/* -------- \EDU-CIAA board keyboard behavior  -------------- */

   		previousState = inputs;
   		Led_Toggle(GREEN_LED);

   		adc_ch1_value = Read_ADC_pooling(CH1);
			xEventGroupSetBits(eventos, EVENT_START);
   		vTaskDelay(10);

   	}
}

void Transmit(void * parameters) {
//   static const char cadena[] = "a\r\n";
//   uint16_t ch0Value = 0;
//   uint8_t ch1Value = 0;
//   uint8_t ch2Value = 0;
   
   while(1)
   {

      while(xEventGroupWaitBits(eventos, EVENT_START,
         TRUE, FALSE, portMAX_DELAY) == 0);

      Led_On(YELLOW_LED);
//      if (EnviarTexto(cadena)) {
//         while(xEventGroupWaitBits(eventos, EVENT_TRANSMIT,
//            TRUE, FALSE, portMAX_DELAY) == 0);
//      }

//      ch0Value++;
//      if(ch0Value > 1000) ch0Value = 0;
//      SendByteUartFtdi(ch0Value);
//      SendByteUartFtdi(ch0Value>>8);

      SendByteUartFtdi(adc_ch1_value);
		SendByteUartFtdi(adc_ch1_value>>8);

		SendByteUartFtdi(adc_ch2_value);
		SendByteUartFtdi(adc_ch2_value>>8);

		SendByteUartFtdi(adc_ch3_value);
		SendByteUartFtdi(adc_ch3_value>>8);


      Led_Off(YELLOW_LED);

   }
}

void Receive(void * parameters) {
   static const command_t serialCommands[] = {
      { .command = 0x01, .action = RGB_R_LED },
      { .command = 0x01, .action = RGB_B_LED },
      { .command = 0x01, .action = RGB_G_LED },
      { .command = 0x01, .action = YELLOW_LED },

   };

   char cadena[16];
   uint8_t indice;
   
   while(1) {
      if (RecibirTexto(cadena, sizeof(cadena))) {
         xEventGroupWaitBits(eventos, EVENT_RECIEVED, TRUE, FALSE, portMAX_DELAY);
      }
//      Led_On(YELLOW_LED);
      for(indice = 0; indice < sizeof(serialCommands) / sizeof(command_t); indice++) {
//         if (strcmp(cadena, serialCommands[indice].command) == 0)
//         {
//            Led_Toggle(serialCommands[indice].action);
//         }
      }
//      Led_Off(YELLOW_LED);
   }
}
/*==================[external functions definition]==========================*/

void EventoSerial(void) {
   if (EnviarCaracter()) {
      xEventGroupSetBits(eventos, EVENT_TRANSMIT);
   };
   if (RecibirCaracter()) {
      xEventGroupSetBits(eventos, EVENT_RECIEVED);
   };
}

void RIT_IRQHandler(void)
{
//	adc_ch1_value = read_ADC_pooling(CH1);
//	xEventGroupSetBits(eventos, EVENT_START);
}

/** \brief Main function
 *
 * This is the main entry point of the software.
 *
 * \returns 0
 *
 * \remarks This function never returns. Return value is only to avoid compiler
 *          warnings or errors.
 */

int main(void) {

	statusFlagADC flag = ENABLE_ADC;

   /* inicializaciones */
   Init_Leds();
   Init_Switches();
   Init_Uart_Ftdi();


   enableInterruptUartFtdi();
//	RITSetTimeInterval_us(TIME_INTERVAL_US);
//	RITSetTimeInterval_ms(TIME_INTERVAL_MS);
//	RITInitInterrupt();

   Init_Adc();
	Enable_ADC_ch1(flag);
	Enable_ADC_ch2(flag);
	Enable_ADC_ch3(flag);

//	read_ADC_value_pooling();

   eventos = xEventGroupCreate();
   if (eventos != NULL) {
      xTaskCreate(Keyboard, "Keyboard", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY + 3, NULL);
      xTaskCreate(Transmit, "Transmit", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY + 1, NULL);
      xTaskCreate(Receive, "Receive", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY + 3, NULL);
      vTaskStartScheduler();
   }


   for(;;);
	return 0;
}

/** @} doxygen end group definition */
/** @} doxygen end group definition */
/** @} doxygen end group definition */
/*==================[end of file]============================================*/

