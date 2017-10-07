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

/** \brief TP2.3 FreeOSEK exercise source file
 **
 ** Proposed Exercise:
 ** Add to the program TP2.2 the key operation to enable and disable the blinking
 ** of each of the corresponding LEDs. The key-reading task must be performed
 ** with a separate lower-priority task that must be started automatically along
 ** with the operating system.
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
  * 20171006 v1.0 Exercise TP2 n° 3 complete.
 */

/*==================[inclusions]=============================================*/
#include <tp2e3.h>         /* <= own header */

#include "led.h"
#include "switch.h"

#include "os.h"               /* <= operating system header */

/*==================[macros and definitions]=================================*/

/** Period of time to make the polling action over the keyboard */
#define KEYBOARD_POLLING_PERIOD 100

/*==================[internal data declaration]==============================*/

/** Half Period for the red led */
static uint32_t alarmPeriodLedRed = 100;
/** Half Period for the green led */
static uint32_t alarmPeriodLedYellow = 250;
/** Half Period for the blue led */
static uint32_t alarmPeriodLedGreen = 400;
/** Half Period for the blue led */
static uint32_t alarmPeriodLedBlue = 555;

/** Enable flag of the Red LED */
static uint8_t enableRedLed = 0;
/** Enable flag of the Yellow LED */
static uint8_t enableYellowLed = 0;
/** Enable flag of the Green LED */
static uint8_t enableGreenLed = 0;
/** Enable flag of the Blue LED */
static uint8_t enableBlueLed = 0;

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
   Init_Switches();

   SetRelAlarm(ActivateRedLedTask, 400, alarmPeriodLedRed);
   SetRelAlarm(ActivateYellowLedTask, 500, alarmPeriodLedYellow);
   SetRelAlarm(ActivateGreenLedTask, 600, alarmPeriodLedGreen);
   SetRelAlarm(ActivateBlueLedTask, 700, alarmPeriodLedBlue);

   SetRelAlarm(ActivateKeyboardTask, 300, KEYBOARD_POLLING_PERIOD);

   /* terminate task */
   TerminateTask();
}

/** \brief  Red Led Task
 *
 *
 *
 */
TASK(RedLedTask)
{
	if(enableRedLed)
		Led_Toggle(RED_LED);
	else
		Led_Off(RED_LED);
   /* terminate task */
   TerminateTask();
}

/** \brief  Yellow Led Task
 *
 *
 *
 */
TASK(YellowLedTask)
{
	if(enableYellowLed)
		Led_Toggle(YELLOW_LED);
	else
		Led_Off(YELLOW_LED);

   /* terminate task */
   TerminateTask();
}

/** \brief  Green Led Task
 *
 *
 *
 */
TASK(GreenLedTask)
{
	if(enableGreenLed)
		Led_Toggle(GREEN_LED);
	else
		Led_Off(GREEN_LED);

   /* terminate task */
   TerminateTask();
}

/** \brief  Blue Led Task
 *
 *
 *
 */
TASK(BlueLedTask)
{

	if(enableBlueLed)
		Led_Toggle(RGB_B_LED);
	else
		Led_Off(RGB_B_LED);
   /* terminate task */
   TerminateTask();
}

/** \brief  KeyboardTask Task
 *
 *		Key 1: Enable/Disable Blue LED.
 *		Key 2: Enable/Disable Yellow LED.
 *		Key 3: Enable/Disable Red LED.
 *		Key 4: Enable/Disable Green LED.
 */
TASK(KeyboardTask)
{

	int8_t inputs = 0;
	static uint8_t previousState = 0;

	/** Inputs read */
	inputs = Read_Switches();

	/* -------- edu-ciaa board keyboard behavior  -------------- */
	/*
	 * Rising edge detector
	 *
	 * */
	if((((inputs ^ previousState) & (1<<TEC1))  != 0)&& ((inputs & (1<<TEC1)) == 0))
	{
		enableBlueLed = !enableBlueLed;
	}

	if((((inputs ^ previousState) & (1<<TEC2))  != 0)&& ((inputs & (1<<TEC2)) == 0))
	{
		enableYellowLed = !enableYellowLed;
	}

	if((((inputs ^ previousState) & (1<<TEC3))  != 0)&& ((inputs & (1<<TEC3)) == 0))
	{
		enableRedLed = !enableRedLed;
	}

	if((((inputs ^ previousState) & (1<<TEC4))  != 0)&& ((inputs & (1<<TEC4)) == 0))
	{
		enableGreenLed = !enableGreenLed;
	}
	/* -------- \edu-ciaa board keyboard behavior  -------------- */

	previousState = inputs;

   /* terminate task */
   TerminateTask();
}
/** @} doxygen end group definition */
/** @} doxygen end group definition */
/** @} doxygen end group definition */
/*==================[end of file]============================================*/
