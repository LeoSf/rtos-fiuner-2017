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
 * 20171014 v1.0 Exercise TP2 n° 4 complete.
 */

/*==================[inclusions]=============================================*/
#include <tp2f_e4.h>       /* <= own header */

/*==================[macros and definitions]=================================*/
/** Struct to model the main characteristics of a LED */
typedef struct {
	uint8_t led;
	uint8_t enable;
	uint16_t delay;
}blinking_t;

/** Disable value of a LED */
#define OFF	0
/** Enable value of a LED */
#define ON	!OFF

/** Half Period for the blue led */
#define BLUE_LED_PERIOD 555
/** Half Period for the yellow led */
#define YELLOW_LED_PERIOD 250
/** Half Period for the red led */
#define RED_LED_PERIOD 100
/** Half Period for the green led */
#define GREEN_LED_PERIOD 400

/** Period of time to make the polling action over the keyboard */
#define KEYBOARD_POLLING_PERIOD 100

/** Name descriptor for Blue Blinking Task */
#define BLUE_DESCRIPTOR 	"Blue Blinking Task"
/** Name descriptor for Yellow Blinking Task */
#define YELLOW_DESCRIPTOR 	"Yellow Blinking Task"
/** Name descriptor for Red Blinking Task */
#define RED_DESCRIPTOR 		"Red Blinking Task"
/** Name descriptor for Green Blinking Task */
#define GREEN_DESCRIPTOR 	"Green Blinking Task"
/** Name descriptor for Keyboard Task */
#define KEYBOARD_DESCRIPTOR 	"Pooling Keyboard Task"

/** Total number of LEDs to control*/
#define ELEMENTS_IN_SEQUENCE 4


/*==================[internal data declaration]==============================*/
//uint16_t led_delays [] = {BLUE_LED_PERIOD, YELLOW_LED_PERIOD, RED_LED_PERIOD, GREEN_LED_PERIOD};

/** Array of leds with a custom order */
static blinking_t sequence_leds[] = {
		{.delay = BLUE_LED_PERIOD, .led = RGB_B_LED, .enable = ON },
		{.delay = YELLOW_LED_PERIOD, .led = YELLOW_LED, .enable = OFF },
		{.delay = RED_LED_PERIOD, .led = RED_LED, .enable = OFF },
		{.delay = GREEN_LED_PERIOD, .led = GREEN_LED, .enable = OFF }
};

/** Index that enable a specific LED */
static uint8_t ledIndex = 0;
/*==================[internal functions declaration]=========================*/

/*==================[internal data definition]===============================*/

/*==================[external data definition]===============================*/

/*==================[internal functions definition]==========================*/
/** \brief Blinking LEDs Task
 *
 *
 *
 */
void BlinkingTaks(void * parameters)
{

	blinking_t * ledInSequence = (blinking_t *) parameters;

	while(1)
	{
		if(ledInSequence->enable == TRUE)
		{
			Led_Toggle(ledInSequence->led);
			vTaskDelay(ledInSequence->delay);
		}
		else
			Led_Off(ledInSequence->led);
	}
}

/** \brief  KeyboardTask Task
 *
 *		Key 1: Enable/Disable Blue LED.
 *		Key 2: Enable/Disable Yellow LED.
 *		Key 3: Enable/Disable Red LED.
 *		Key 4: Enable/Disable Green LED.
 */
void KeyboardTask(void * parameters)
{
	blinking_t (*leds)[] = parameters;
	uint8_t i = 0;

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
			if ((*leds)[ledIndex].delay < 2000)
			{
				(*leds)[ledIndex].delay += 50;
			}
			else
			{
				(*leds)[ledIndex].delay = 2000;
			}
		}

		if((((inputs ^ previousState) & (1<<TEC4))  != 0)&& ((inputs & (1<<TEC4)) == 0))
		{
			if ((*leds)[ledIndex].delay > 51)
			{
				(*leds)[ledIndex].delay -= 50;
			}
			else
			{
				(*leds)[ledIndex].delay = 50;
			}
		}


		if(((inputs & (1<<TEC3)) != 0) || ((inputs & (1<<TEC4)) != 0))
		{
			/** Cleaning all the enables flags */
			for(i =0; i< ELEMENTS_IN_SEQUENCE; i++)
				if(i != ledIndex)
					(*leds)[i].enable = OFF;

			/** Enable just the only led that it is active to be modified */
			(*leds)[ledIndex].enable = ON;
		}
		else
		{
			for(i =0; i< ELEMENTS_IN_SEQUENCE; i++)
				(*leds)[i].enable = ON;
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

	/** Initializations  */
	Init_Leds();
	Init_Switches();


	/* Task creations */
	xTaskCreate(BlinkingTaks, BLUE_DESCRIPTOR, configMINIMAL_STACK_SIZE, (void*)  &sequence_leds[0], tskIDLE_PRIORITY + 1, &tarea);
	xTaskCreate(BlinkingTaks, YELLOW_DESCRIPTOR, configMINIMAL_STACK_SIZE, (void*) &sequence_leds[1], tskIDLE_PRIORITY + 1, &tarea);
	xTaskCreate(BlinkingTaks, RED_DESCRIPTOR, configMINIMAL_STACK_SIZE, (void*) &sequence_leds[2], tskIDLE_PRIORITY + 1, &tarea);
	xTaskCreate(BlinkingTaks, GREEN_DESCRIPTOR, configMINIMAL_STACK_SIZE, (void*)  &sequence_leds[3], tskIDLE_PRIORITY + 1, &tarea);


	xTaskCreate(KeyboardTask, KEYBOARD_DESCRIPTOR, configMINIMAL_STACK_SIZE, (void*)  &sequence_leds, tskIDLE_PRIORITY + 3, &tarea);
	vTaskStartScheduler();


	/** This point should never be reached */
	for(;;);

	return 0;
}

/** @} doxygen end group definition */
/** @} doxygen end group definition */
/** @} doxygen end group definition */
/*==================[end of file]============================================*/

