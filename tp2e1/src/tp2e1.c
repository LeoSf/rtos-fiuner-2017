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
 ** Design and implement an OSEK-based firmware for EDU-CIAA to turn on the 6 board LEDs,
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
#include <tp2e1.h>         /* <= own header */
#include "led.h"
#include "os.h"            /* <= operating system header */

/*==================[macros and definitions]=================================*/
/** Total number of LEDs to control*/
#define ELEMENTS_IN_SEQUENCE 6

/*==================[internal data declaration]==============================*/
/** Array of leds with a custom order */
uint8_t sequence_leds[ELEMENTS_IN_SEQUENCE]={
		RGB_R_LED, RGB_B_LED, RGB_G_LED, YELLOW_LED, RED_LED, GREEN_LED
};

/** Half Period for the  LED in [ms] */
static uint32_t alarmFadePeriod = 250;
/** Index that enable a specific LED */
static uint8_t ledIndex = 0;
/*==================[internal functions declaration]=========================*/

/*==================[internal data definition]===============================*/

/*==================[external data definition]===============================*/

/*==================[internal functions definition]==========================*/

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
   /* Starts the operating system in the Application Mode 1 */
   /* This example has only one Application Mode */
   StartOS(Normal);

   /* StartOs shall never returns, but to avoid compiler warnings or errors
    * 0 is returned */
   return 0;
}

/** \brief Error Hook function
 *
 * This fucntion is called from the os if an os interface (API) returns an
 * error. Is for debugging proposes. If called this function triggers a
 * ShutdownOs which ends in a while(1).
 *
 * The values:
 *    OSErrorGetServiceId
 *    OSErrorGetParam1
 *    OSErrorGetParam2
 *    OSErrorGetParam3
 *    OSErrorGetRet
 *
 * will provide you the interface, the input parameters and the returned value.
 * For more details see the OSEK specification:
 * http://portal.osek-vdx.org/files/pdf/specs/os223.pdf
 *
 */
void ErrorHook(void)
{
    Led_On(RED_LED);
   // ciaaPOSIX_printf("ErrorHook was called\n");
   // ciaaPOSIX_printf("Service: %d, P1: %d, P2: %d, P3: %d, RET: %d\n", OSErrorGetServiceId(), OSErrorGetParam1(), OSErrorGetParam2(), OSErrorGetParam3(), OSErrorGetRet());
   ShutdownOS(0);
}

/** \brief Initial task
 *
 * This task is started automatically in the application mode 1.
 */
TASK(ConfigTask)
{

   /** Initializations */
   Init_Leds();


   /* activate periodic task:
    *  - for the first time after 500 ticks ( 500 ms)
    *  - and then every alarmFadePeriod ticks ( alarmFadePeriod ms)
    */
   SetRelAlarm(ActivateFadeLedsTask, 500, alarmFadePeriod);
//   Led_On(GREEN_LED);

   /* terminate task */
   TerminateTask();
}

/** \brief Fade Leds Task
 *
 *
 *
 */
TASK(FadeLedsTask)
{

	for(uint8_t i=0; i < ELEMENTS_IN_SEQUENCE; i++)
	{
		Led_Off(sequence_leds[i]);
	}

	ledIndex++;
	if(ledIndex>= ELEMENTS_IN_SEQUENCE)
		ledIndex = 0;
	Led_On(sequence_leds[ledIndex]);


   /* terminate task */
   TerminateTask();
}

/** @} doxygen end group definition */
/** @} doxygen end group definition */
/** @} doxygen end group definition */
/*==================[end of file]============================================*/
