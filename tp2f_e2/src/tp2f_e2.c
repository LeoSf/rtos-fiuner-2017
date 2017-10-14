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

/** \brief TP2.2 FreeRTOS exercise source file
 **
 ** Proposed Exercise:
 ** Design and implement a firmware on the EDU-CIAA that, using four tasks
 ** of the same priority, flashes the red led with a frequency of 5 Hz, yellow at
 ** 2 Hz, green at 1.25 Hz and blue at 0.9 Hz.
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
 * 20171006 v1.0 Exercise TP2 n° 2 complete.
 */

/*==================[inclusions]=============================================*/
#include <tp2f_e2.h>       /* <= own header */

/*==================[macros and definitions]=================================*/
/** Struct to model the main characteristics of a LED */
typedef struct {
	uint16_t delay;
	uint8_t led;
}blinking_t;

/** Half Period for the red led */
#define RED_LED_PERIOD 100
/** Half Period for the yellow led */
#define YELLOW_LED_PERIOD 250
/** Half Period for the green led */
#define GREEN_LED_PERIOD 400
/** Half Period for the blue led */
#define BLUE_LED_PERIOD 555

/** Name descriptor for Yellow Blinking Task */
#define YELLOW_DESCRIPTOR 	"Yellow Blinking Task"
/** Name descriptor for Red Blinking Task */
#define RED_DESCRIPTOR 		"Red Blinking Task"
/** Name descriptor for Green Blinking Task */
#define GREEN_DESCRIPTOR 	"Green Blinking Task"
/** Name descriptor for Blue Blinking Task */
#define BLUE_DESCRIPTOR 	"Blue Blinking Task"

/*==================[internal data declaration]==============================*/

/*==================[internal functions declaration]=========================*/


/*==================[internal data definition]===============================*/

/*==================[external data definition]===============================*/

/*==================[internal functions definition]==========================*/

/*==================[external functions definition]==========================*/
/** \brief Blinking LEDs Task
 *
 *
 *
 */
void TareaBlinking(void * parameters){
//void TareaBlinking(void){	// Also valid
//	(void) parameters;			// if I don't use parameters

	blinking_t * valores = (blinking_t *) parameters;

	while(1)
	{
		Led_Toggle(valores->led);
		vTaskDelay(valores->delay);

	}
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
int main(void)
{
	/** Variable in Flash to save LEDs configurations */
	static const blinking_t valores[] = {
			{.delay = YELLOW_LED_PERIOD, .led = YELLOW_LED },
			{.delay = RED_LED_PERIOD, .led = RED_LED },
			{.delay = GREEN_LED_PERIOD, .led = GREEN_LED },
			{.delay = BLUE_LED_PERIOD, .led = RGB_B_LED }
	};

	/** Variable to store task IDs */
	TaskHandle_t tarea;

	/** Initializations  */
	Init_Leds();

	/* Task creations */
	xTaskCreate(TareaBlinking, YELLOW_DESCRIPTOR, configMINIMAL_STACK_SIZE, (void*) &valores[0], tskIDLE_PRIORITY + 1, &tarea);
	xTaskCreate(TareaBlinking, RED_DESCRIPTOR, configMINIMAL_STACK_SIZE, (void*) &valores[1], tskIDLE_PRIORITY + 1, &tarea);
	xTaskCreate(TareaBlinking, GREEN_DESCRIPTOR, configMINIMAL_STACK_SIZE, (void*)  &valores[2], tskIDLE_PRIORITY + 1, &tarea);
	xTaskCreate(TareaBlinking, BLUE_DESCRIPTOR, configMINIMAL_STACK_SIZE, (void*)  &valores[3], tskIDLE_PRIORITY + 1, &tarea);
	vTaskStartScheduler();


	/** This point should never be reached*/
	for(;;);

	return 0;
}

/** @} doxygen end group definition */
/** @} doxygen end group definition */
/** @} doxygen end group definition */
/*==================[end of file]============================================*/

