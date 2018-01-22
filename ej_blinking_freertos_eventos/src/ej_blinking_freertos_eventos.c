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

/** \brief Process Synchronization: Events whit FreeRTOS
 **
 ** Proposed Exercise:
 ** Design an OSEK task-based firmware that turns on the blue led by pressing key 1 and
 ** turns it off when it is pressed again. Also, if the user does not press the key,
 ** after 3 seconds, the LED must be turned off automatically.
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
 * 20171016 v1.0 Exercise complete.
 */

/*==================[inclusions]=============================================*/
#include <ej_blinking_freertos_eventos.h>       /* <= own header */

/*==================[macros and definitions]=================================*/
/** Name descriptor for Blue Blinking Task */
#define BLUE_DESCRIPTOR 	"Blue Blinking Task"

/** Name descriptor for Keyboard Task */
#define KEYBOARD_DESCRIPTOR 	"Pooling Keyboard Task"

/** Half Period for the blue led */
#define BLUE_LED_PERIOD 450

/** Period of time to make the polling action over the keyboard */
#define KEYBOARD_POLLING_PERIOD 100

/** Timeout period of the led task to control when a user doesn't press key 1 */
#define TASK_TIME_TIMEOUT 3000

/** */
#define EVENT_FROM_KEYBOARD 0x01


/*==================[internal data declaration]==============================*/
/** Struct to model the LED's main characteristics */
typedef struct {
	uint8_t led;
	uint16_t delay;
	EventGroupHandle_t eventGroupHandler;
}blinking_t;

/*==================[internal functions declaration]=========================*/

/*==================[internal data definition]===============================*/

/*==================[external data definition]===============================*/

/*==================[internal functions definition]==========================*/
/** \brief Blue Led Task
 *
 *
 *
 */
void BlinkingBlueTask(void * parameters)
{
	blinking_t * ledHandler = (blinking_t *)parameters;
	EventBits_t result;

	while(1)
	{
		while(!xEventGroupWaitBits(ledHandler->eventGroupHandler, EVENT_FROM_KEYBOARD,
				TRUE, FALSE, ledHandler->delay));
		Led_On(ledHandler->led);
		result = xEventGroupWaitBits(ledHandler->eventGroupHandler, EVENT_FROM_KEYBOARD,
				TRUE, FALSE, TASK_TIME_TIMEOUT);
		Led_Off(ledHandler->led);
	}
}

/** \brief  KeyboardTask Task
 *
 *		Key 1: turn on blue LED for 2 seconds / turn off blue led.
 *		Key 2: -
 *		Key 3: -
 *		Key 4: -
 */
void KeyboardTask(void * parameters)
{

	blinking_t * ledHandler = (blinking_t *)parameters;
	EventGroupHandle_t event = ledHandler->eventGroupHandler;

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
			xEventGroupSetBits(event, EVENT_FROM_KEYBOARD);
		}

		/* -------- \EDU-CIAA board keyboard behavior  -------------- */

		previousState = inputs;

		vTaskDelay(KEYBOARD_POLLING_PERIOD);

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
	/** Variable to store task IDs */
	TaskHandle_t tarea;

	/** Variable in Flash to save LEDs configurations */
	static  blinking_t led = { .led = RGB_B_LED, .delay = BLUE_LED_PERIOD};

	/** */
	EventGroupHandle_t eventGroup = xEventGroupCreate();

	led.eventGroupHandler = eventGroup;

	/* Initializations  */
	Init_Leds();
	Init_Switches();


	/* Tasks creation */
	xTaskCreate(BlinkingBlueTask, BLUE_DESCRIPTOR, configMINIMAL_STACK_SIZE, (void*)  &led, tskIDLE_PRIORITY + 1, &tarea);
	xTaskCreate(KeyboardTask, KEYBOARD_DESCRIPTOR, configMINIMAL_STACK_SIZE, (void*)  &led, tskIDLE_PRIORITY + 3, &tarea);
	vTaskStartScheduler();


	/** This point should never be reached */
	for(;;);

	return 0;
}

/** @} doxygen end group definition */
/** @} doxygen end group definition */
/** @} doxygen end group definition */
/*==================[end of file]============================================*/

