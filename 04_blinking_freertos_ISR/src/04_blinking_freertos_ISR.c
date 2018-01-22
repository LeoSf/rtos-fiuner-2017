/* Copyright 2017, Leandro D. Medus
 * lmedus@bioingenieria.edu.ar
 * Faculty of Engineering
 * National University of Entre Ríos
 * Argentina
 *
 * All rights reserved.
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

/** \brief Interrupts in FreeRTOS
 **
 ** Proposed Exercise:
 **
 **
 ** Dependencies: baremetal drivers library (v1.0) provided by
 ** MSc. Filomena, MSc. Reta, and Eng. Volentini.
 **
 ** IMPORTANT: some changes have been made to leds.c, switch.c and uart.c in the
 ** driver_bm (v1.0) directory in order to support LEDs physical location in the
 ** first version of the EDU-CIAA board, to get just one word with the
 ** information of keys pressed and to support sending and receiving strings.
 **
 **/

/** \addtogroup CIAA_Firmware CIAA Firmware
 ** @{ */
/** \addtogroup Sources_LDM Leandro D. Medus Source Files
 ** @{ */
/** \addtogroup RTOS-FIUNER Postgraduate Course Source Files
 ** @{ */

/*
 * Initials     Name
 * ---------------------------
 *	LM				Leandro Medus
 */

/*
 * modification history (new versions first)
 * -----------------------------------------------------------
 * 20171020 v1.0 Exercise complete.
 */

/*==================[inclusions]=============================================*/
#include <04_blinking_freertos_ISR.h>       /* <= own header */

/*==================[macros and definitions]=================================*/
/** Period of time to make the polling action over the keyboard */
#define KEYBOARD_POLLING_PERIOD 100

/** */
#define EVENT_TRANSMISSION_COMPLETE 0x01

/** */
#define EVENT_FROM_KEYBOARD 0x02




/*==================[internal data declaration]==============================*/
EventGroupHandle_t eventGroup;

BaseType_t xHigherPriorityTaskWoken;
/*==================[internal functions declaration]=========================*/

/*==================[internal data definition]===============================*/

/*==================[external data definition]===============================*/

/*==================[internal functions definition]==========================*/
/** \brief  KeyboardTask Task
 *
 *		Key 1: Enable/Disable Blue LED.
 *		Key 2: Enable/Disable Yellow LED.
 *		Key 3: Enable/Disable Red LED.
 *		Key 4: Enable/Disable Green LED.
 */
void KeyboardTask(void * parameters)
{

	while(1)
	{
		int8_t inputs = 0;
		static uint8_t previousState = 0;

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
			xEventGroupSetBits(eventGroup, EVENT_FROM_KEYBOARD);
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


		vTaskDelay(KEYBOARD_POLLING_PERIOD);

	}
}


void SendTask(void * parameters)
{
	uint16_t i;

   while(1)
   {
   	while(!xEventGroupWaitBits(eventGroup, EVENT_FROM_KEYBOARD,
   					TRUE, FALSE, 20000));

		static const char * const cadenas[]={
				"Hola\n",
				"Mundooo\n"};

		for(i=0; i<2;i++)
		{
			Led_On(RGB_B_LED);
			if (EnviarTexto(cadenas[i]))
			{
				while(!xEventGroupWaitBits(eventGroup, EVENT_TRANSMISSION_COMPLETE,
				   					TRUE, FALSE, 20000));
			}
			Led_Off(RGB_B_LED);
		}
   }
}

void SerialEvent(void)
{
	if (EnviarCaracter())
	{
		xEventGroupSetBits(eventGroup, EVENT_TRANSMISSION_COMPLETE);
//		xEventGroupSetBitsFromISR(eventGroup, EVENT_TRANSMISSION_COMPLETE, &xHigherPriorityTaskWoken);
	}
}
/*==================[external functions definition]==========================*/
/** \brief Main function
 *
 * This is the main entry point of the software.
 *
 * \returns 0
 *
 * \remarks This function never returns. Return value is only to avoid compiler
 *          warnings or errors.
 */
int main(void)
{
   /* inicializaciones */

   SystemCoreClockUpdate();
	Init_Leds();
	Init_Switches();
	Init_Uart_Ftdi();

	NVIC_EnableIRQ(26);

	/** */
	eventGroup = xEventGroupCreate();
	xHigherPriorityTaskWoken = pdFALSE;

   xTaskCreate(KeyboardTask, "KeyboardTask", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY + 1, NULL);
   xTaskCreate(SendTask, "SendTask", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY + 2, NULL);
   vTaskStartScheduler();
	return 0;
}

/** @} doxygen end group definition */
/** @} doxygen end group definition */
/** @} doxygen end group definition */
/*==================[end of file]============================================*/

