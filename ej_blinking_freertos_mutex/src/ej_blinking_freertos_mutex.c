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

/** \brief Resource FreeRTOS Lab example
 **
 ** Proposed Exercise:
 ** This is an example to test different conditions to use Resources in FreeOSEK
 ** with three tasks.
 **
 ** Control two LEDs using mutex. It's mandatory to use the Blue and Red RGB LED.
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
 * 20171013 v1.0 Exercise complete.
 */

/*==================[inclusions]=============================================*/
#include <ej_blinking_freertos_mutex.h>       /* <= own header */

/*==================[macros and definitions]=================================*/
/** Name descriptor for Blue Blinking Task */
#define BLUE_DESCRIPTOR 	"Blue Blinking Task"
/** Name descriptor for Red Blinking Task */
#define RED_DESCRIPTOR 		"Red Blinking Task"


/** Half Period for the blue led */
#define BLUE_LED_PERIOD 450
/** Half Period for the red led */
#define RED_LED_PERIOD 350

/*==================[internal data declaration]==============================*/
/** Struct to model the main characteristics of a LED */
typedef struct {
	uint16_t delay;
	uint8_t led;
}blinking_t;


SemaphoreHandle_t xSemaphoreLED;
/*==================[internal functions declaration]=========================*/

/*==================[internal data definition]===============================*/

/*==================[external data definition]===============================*/

/*==================[internal functions definition]==========================*/
/** \brief Fade Leds Task
 *
 *
 *
 */
void BlinkingTask(void * parameters)
{
	blinking_t * valores = (blinking_t *) parameters;

	while(1)
	{
//		BaseType_t resultOperation = xSemaphoreTake( xSemaphoreLED, 20 );
		if(xSemaphoreTake( xSemaphoreLED, 100 ) == pdTRUE)
		{
			Led_On(valores->led);
			vTaskDelay(valores->delay);
			Led_Off(valores->led);
			xSemaphoreGive(xSemaphoreLED);
		}
		vTaskDelay(valores->delay);

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
	/** Variable in Flash to save LEDs configurations */
	static const blinking_t leds[] = {
			{.delay = RED_LED_PERIOD, .led = RGB_R_LED },
			{.delay = BLUE_LED_PERIOD, .led = RGB_B_LED }
	};

	/** Variable to store task IDs */
	TaskHandle_t tarea;

	/** Initializations  */
	Init_Leds();

	/* Attempt to create a mutex type semaphore. */
	xSemaphoreLED = xSemaphoreCreateMutex();

	/* Task creations */
	xTaskCreate(BlinkingTask, RED_DESCRIPTOR, configMINIMAL_STACK_SIZE, (void*) &leds[0], tskIDLE_PRIORITY + 1, &tarea);
	xTaskCreate(BlinkingTask, BLUE_DESCRIPTOR, configMINIMAL_STACK_SIZE, (void*)  &leds[1], tskIDLE_PRIORITY + 1, &tarea);
	vTaskStartScheduler();
	return 0;
}

/** @} doxygen end group definition */
/** @} doxygen end group definition */
/** @} doxygen end group definition */
/*==================[end of file]============================================*/

