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

/** \brief TP2.4 FreeOSEK exercise source file
 **
 ** Proposed Exercise:
 ** Design and implement a firmware on the EDU-CIAA that blinks the LEDs with
 ** configurable period. Initially all LEDs should flash according to the detail
 ** of the previous exercise. The system must be able to select one of 4 LEDs
 ** and modify its count using the keys according to the following detail:
 ** 	Key 1: Select the LED to the left of the current one.
 ** 	Key 2: Select LED to the right of the current one.
 ** 	Key 3: Decreases the flashing period.
 ** 	Key 4: Increases the flashing period.
 **
 ** Dependencies: baremetal drivers library (v1.0) provided by
 ** MSc. Filomena, MSc. Reta, and Eng. Volentini.
 **
 ** NOTES:
 ** -----
 ** 	* the user CAN select LEDs from left to right and right to left, it is not a
 ** 	round robin scheme.
 ** 	* the user CAN know which LED is active by holding key 3 or 4.
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
 * 20171006 v1.0 Exercise TP2 n° 4 complete.
 */

/*==================[inclusions]=============================================*/
#include <tp2e4.h>         /* <= own header */
#include "led.h"
#include "switch.h"
#include "os.h"          	/* <= operating system header */

/*==================[macros and definitions]=================================*/
/** Disable value of a LED */
#define OFF	0
/** Enable value of a LED */
#define ON	!OFF

/** Period of time to make the polling action over the keyboard */
#define KEYBOARD_POLLING_PERIOD 100

/** Half Period for the red led */
#define ALARM_PERIOD_LED_RED_DEFAULT 100
/** Half Period for the yellow led */
#define ALARM_PERIOD_LED_YELLOW_DEFAULT 250
/** Half Period for the green led */
#define ALARM_PERIOD_LED_GREEN_DEFAULT 400
/** Half Period for the blue led */
#define ALARM_PERIOD_LED_BLUE_DEFAULT 555

/** Total number of LEDs to control*/
#define ELEMENTS_IN_SEQUENCE 4

/*==================[internal data declaration]==============================*/

/** Array of leds with a custom order */
uint32_t sequence_leds[ELEMENTS_IN_SEQUENCE]={
		RGB_B_LED, YELLOW_LED, RED_LED, GREEN_LED
};

/** Array of half periods related with each led in sequence_leds */
uint32_t sequence_halfPeriod[ELEMENTS_IN_SEQUENCE]={
		ALARM_PERIOD_LED_BLUE_DEFAULT,		/** Half Period for the blue led */
		ALARM_PERIOD_LED_YELLOW_DEFAULT, 	/** Half Period for the yellow led */
		ALARM_PERIOD_LED_RED_DEFAULT, 		/** Half Period for the red led */
		ALARM_PERIOD_LED_GREEN_DEFAULT 		/** Half Period for the green led */

};

/** Array of the Activation Tasks IDs */
uint8_t sequence_activations[ELEMENTS_IN_SEQUENCE] = {
		ActivateBlueLedTask,
		ActivateYellowLedTask,
		ActivateRedLedTask,
		ActivateGreenLedTask

};

/** Array of enables of each LED */
uint8_t sequence_enables[ELEMENTS_IN_SEQUENCE] = {ON, ON, ON, ON};

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
   Init_Switches();

   SetRelAlarm(	ActivateBlueLedTask, 	400, sequence_halfPeriod[0]);
   SetRelAlarm(	ActivateYellowLedTask, 	500, sequence_halfPeriod[1]);
   SetRelAlarm(	ActivateRedLedTask, 		600, sequence_halfPeriod[2]);
   SetRelAlarm(	ActivateGreenLedTask, 	700, sequence_halfPeriod[3]);


   SetRelAlarm(ActivateKeyboardTask, 300, KEYBOARD_POLLING_PERIOD);

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
	if(sequence_enables[0])
		Led_Toggle(RGB_B_LED);
	else
		Led_Off(RGB_B_LED);
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
	if(sequence_enables[1])
		Led_Toggle(YELLOW_LED);
	else
		Led_Off(YELLOW_LED);

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
	if(sequence_enables[2])
		Led_Toggle(RED_LED);
	else
		Led_Off(RED_LED);
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
	if(sequence_enables[3])
		Led_Toggle(GREEN_LED);
	else
		Led_Off(GREEN_LED);

   /* terminate task */
   TerminateTask();
}


/** \brief  KeyboardTask Task
 *
 *		Key 1: Select the LED to the left of the current one.
 *		Key 2: Select LED to the right of the current one.
 *		Key 3: Decreases the flashing period.
 *		Key 4: Increases the flashing period.
 *
 *		Note: holding key 2 or 3, only blinks the LED that it is selected at that momment
 *
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
		ledIndex--;
		if(ledIndex <= 0)
			ledIndex=0;
	}

	if((((inputs ^ previousState) & (1<<TEC2))  != 0)&& ((inputs & (1<<TEC2)) == 0))
	{
		ledIndex++;
		if(ledIndex >= ELEMENTS_IN_SEQUENCE )
			ledIndex = ELEMENTS_IN_SEQUENCE-1;
	}

	if((((inputs ^ previousState) & (1<<TEC3))  != 0)&& ((inputs & (1<<TEC3)) == 0))
	{
		ActivateTask(IncreaseTimerPeriodTask);
	}

	if((((inputs ^ previousState) & (1<<TEC4))  != 0)&& ((inputs & (1<<TEC4)) == 0))
	{
		ActivateTask(DecreaseTimerPeriodTask);
	}


	if(((inputs & (1<<TEC3)) != 0) || ((inputs & (1<<TEC4)) != 0))
	{
		/** Cleaning all the enables flags */
		memset(sequence_enables, OFF , sizeof(sequence_enables));
		/** Enable just the only led that it is active to be modified */
		sequence_enables[ledIndex] = ON;
	}
	else
	{
		memset(sequence_enables, 0xFF, sizeof(sequence_enables));
	}

	/* -------- \edu-ciaa board keyboard behavior -------------- */
	previousState = inputs;

   /* terminate task */
   TerminateTask();
}


/** \brief Increase Timer Period Task
 *
 *	\details Task to blablablabla
 *
 */
TASK(IncreaseTimerPeriodTask)
{
	TickRefType currentTick;
	GetAlarm(sequence_activations[ledIndex], &currentTick);

	CancelAlarm(sequence_activations[ledIndex]);
	if (sequence_halfPeriod[ledIndex] < 2000)
	{
		sequence_halfPeriod[ledIndex] += 50;
	}
	else
	{
		sequence_halfPeriod[ledIndex] = 2000;
	}

	SetRelAlarm((AlarmType)sequence_activations[ledIndex], (TickType)currentTick, sequence_halfPeriod[ledIndex]);

	/* terminate task */
	TerminateTask();
}

/** \brief Decrease Timer Period Task
 *
 *	\details Task to blablablabla of the green led
 *
 */
TASK(DecreaseTimerPeriodTask)
{
	TickRefType currentTick;
	GetAlarm((AlarmType)sequence_activations[ledIndex], &currentTick);

	CancelAlarm(sequence_activations[ledIndex]);
	if (sequence_halfPeriod[ledIndex] > 51)
	{
		sequence_halfPeriod[ledIndex] -= 50;
	}
	else
	{
		sequence_halfPeriod[ledIndex] = 50;
	}

	SetRelAlarm((AlarmType)sequence_activations[ledIndex], (TickType)currentTick, sequence_halfPeriod[ledIndex]);

	/* terminate task */
	TerminateTask();
}

/** @} doxygen end group definition */
/** @} doxygen end group definition */
/** @} doxygen end group definition */
/*==================[end of file]============================================*/
