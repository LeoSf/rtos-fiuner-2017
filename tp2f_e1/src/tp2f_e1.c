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

/** \brief TP2.1 FreeRTOS exercise source file
 **
 ** This is a entry example to implement in another program a RTOS.
 **
 ** Proposed Exercise:
 ** Design and implement an FreeRTOS-based firmware for EDU-CIAA to turn on the 6 board LEDs,
 ** one at a time and sequentially. The power-on time of each LED should be 250 ms.
 ** A single task and an alarm should be used. The task should be launched at 500 ms
 ** after the OS starts.
 **
 ** Dependencies: baremetal drivers library (v1.0) provided by
 ** MSc. Filomena, MSc. Reta, and Eng. Volentini.
 **
 ** IMPORTANT: some changes have been made to leds.c and switch.c in the driver_bm (v1.0)
 ** directory in order to support LEDs physical location in the first version of the
 ** EDU-CIAA board and to get just one word with the information of keys pressed.
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
 * 20171006 v1.0 Exercise TP2 n° 1 complete.
 */

/*==================[inclusions]=============================================*/
#include <tp2f_e1.h>       /* <= own header */

/*==================[macros and definitions]=================================*/
/** Total number of LEDs to control*/
#define ELEMENTS_IN_SEQUENCE 6

/*==================[internal data declaration]==============================*/
/** Array of leds with a custom order */
uint8_t sequence_leds[ELEMENTS_IN_SEQUENCE]={
		RGB_R_LED, RGB_B_LED, RGB_G_LED, YELLOW_LED, RED_LED, GREEN_LED
};

/** Index that enable a specific LED */
static uint8_t ledIndex = 0;
/*==================[internal functions declaration]=========================*/

/*==================[internal data definition]===============================*/

/*==================[external data definition]===============================*/

/*==================[internal functions definition]==========================*/
/** \brief Fade Leds Task
 *
 *
 *
 */
void FadeLedsTask(void * parametros)
{
	while(1)
	{
		for(uint8_t i=0; i < ELEMENTS_IN_SEQUENCE; i++)
		{
			Led_Off(sequence_leds[i]);
		}

		ledIndex++;
		if(ledIndex>= ELEMENTS_IN_SEQUENCE)
			ledIndex = 0;
		Led_On(sequence_leds[ledIndex]);

		vTaskDelay(250);
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

   xTaskCreate(FadeLedsTask, "Fade Led Task", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY + 1, NULL);
   vTaskStartScheduler();
	return 0;
}

/** @} doxygen end group definition */
/** @} doxygen end group definition */
/** @} doxygen end group definition */
/*==================[end of file]============================================*/

