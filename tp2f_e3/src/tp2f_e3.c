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

/** \brief TP2.3 FreeRTOS exercise source file
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
 * 20171014 v1.0 Exercise TP2 n° 3 complete.
 * 20171013 v0.1 initial version..
 */

/*==================[inclusions]=============================================*/
#include <tp2f_e3.h>       /* <= own header */

/*==================[macros and definitions]=================================*/
/** Struct to model the main characteristics of a LED */
typedef struct {
	uint8_t led;
	uint8_t enable;
	uint16_t delay;
}blinking_t;

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


/*==================[internal data declaration]==============================*/
//uint16_t led_delays [] = {BLUE_LED_PERIOD, YELLOW_LED_PERIOD, RED_LED_PERIOD, GREEN_LED_PERIOD};

/** Variable  to save LEDs configurations */
static blinking_t valores[] = {
		{.delay = BLUE_LED_PERIOD, .led = RGB_B_LED, .enable = TRUE },
		{.delay = YELLOW_LED_PERIOD, .led = YELLOW_LED, .enable = FALSE },
		{.delay = RED_LED_PERIOD, .led = RED_LED, .enable = FALSE },
		{.delay = GREEN_LED_PERIOD, .led = GREEN_LED, .enable = FALSE }
};
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

	blinking_t * valores = (blinking_t *) parameters;

	while(1)
	{
		if(valores->enable == TRUE)
		{
			Led_Toggle(valores->led);
			vTaskDelay(valores->delay);
		}
		else
			Led_Off(valores->led);
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
	blinking_t (*valores)[] = parameters;

	while(1)
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
			(*valores)[0].enable = !(*valores)[0].enable;
		}

		if((((inputs ^ previousState) & (1<<TEC2))  != 0)&& ((inputs & (1<<TEC2)) == 0))
		{
			(*valores)[1].enable = !(*valores)[1].enable;
		}

		if((((inputs ^ previousState) & (1<<TEC3))  != 0)&& ((inputs & (1<<TEC3)) == 0))
		{
			(*valores)[2].enable = !(*valores)[2].enable;
		}

		if((((inputs ^ previousState) & (1<<TEC4))  != 0)&& ((inputs & (1<<TEC4)) == 0))
		{
			(*valores)[3].enable = !(*valores)[3].enable;
		}
		/* -------- \edu-ciaa board keyboard behavior  -------------- */

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
	xTaskCreate(BlinkingTaks, BLUE_DESCRIPTOR, configMINIMAL_STACK_SIZE, (void*)  &valores[0], tskIDLE_PRIORITY + 1, &tarea);
	xTaskCreate(BlinkingTaks, YELLOW_DESCRIPTOR, configMINIMAL_STACK_SIZE, (void*) &valores[1], tskIDLE_PRIORITY + 1, &tarea);
	xTaskCreate(BlinkingTaks, RED_DESCRIPTOR, configMINIMAL_STACK_SIZE, (void*) &valores[2], tskIDLE_PRIORITY + 1, &tarea);
	xTaskCreate(BlinkingTaks, GREEN_DESCRIPTOR, configMINIMAL_STACK_SIZE, (void*)  &valores[3], tskIDLE_PRIORITY + 1, &tarea);


	xTaskCreate(KeyboardTask, KEYBOARD_DESCRIPTOR, configMINIMAL_STACK_SIZE, (void*)  &valores, tskIDLE_PRIORITY + 3, &tarea);
	vTaskStartScheduler();


	/** This point should never be reached */
	for(;;);

	return 0;
}

/** @} doxygen end group definition */
/** @} doxygen end group definition */
/** @} doxygen end group definition */
/*==================[end of file]============================================*/

