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

/** \brief Interrupts in FreeOSEK
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
#include <04_blinking_OSEK_ISR.h>         /* <= own header */
#include "led.h"
#include "switch.h"
#include "uart.h"
#include "os.h"               				/* <= operating system header */

/*==================[macros and definitions]=================================*/
/** Period of time to make the polling action over the keyboard */
#define KEYBOARD_POLLING_PERIOD 100

/*==================[internal data declaration]==============================*/


TaskType tarea;

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
    Led_Toggle(RED_LED);
   // ciaaPOSIX_printf("ErrorHook was called\n");
   // ciaaPOSIX_printf("Service: %d, P1: %d, P2: %d, P3: %d, RET: %d\n", OSErrorGetServiceId(), OSErrorGetParam1(), OSErrorGetParam2(), OSErrorGetParam3(), OSErrorGetRet());
   ShutdownOS(0);
}

/** \brief Initial task
 *
 * This task is started automatically in the application mode 1.
 */
TASK(Configuracion)
{

   /** Initializations */
   Init_Leds();
   Init_Switches();
   Init_Uart_Ftdi();

   NVIC_EnableIRQ(26);


   /* activate periodic task:
    *  - for the first time after 350 ticks (350 ms)
    *  - and then every 250 ticks (250 ms)
    */
   SetRelAlarm(RevisarTeclado, 350, KEYBOARD_POLLING_PERIOD);

   /* terminate task */
   TerminateTask();
}

/** \brief Periodic Task
 *
 * This task is started automatically every time that the alarm
 * ActivatePeriodicTask expires.
 *
 */
TASK(TareaPeriodicaBlinking)
{

   Led_Toggle(GREEN_LED);

   /* terminate task */
   TerminateTask();
}


/** \brief  KeyboardTask Task
 *
 *		Key 1:
 *		Key 2:
 *		Key 3:
 *		Key 4:
 *
 */
TASK(Teclado)
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
		ActivateTask(Enviar);
	}

	if((((inputs ^ previousState) & (1<<TEC2))  != 0)&& ((inputs & (1<<TEC2)) == 0))
	{

	}

	if((((inputs ^ previousState) & (1<<TEC3))  != 0)&& ((inputs & (1<<TEC3)) == 0))
	{

	}

	if((((inputs ^ previousState) & (1<<TEC4))  != 0)&& ((inputs & (1<<TEC4)) == 0))
	{
		SetEvent(tarea,Completo);
	}


	/* -------- \edu-ciaa board keyboard behavior -------------- */
	previousState = inputs;

	Led_Toggle(GREEN_LED);

   /* terminate task */
   TerminateTask();
}

/** \brief Periodic Task
 *
 *
 */
TASK(Enviar)
{

	Led_Toggle(RGB_B_LED);

	static const char cadena1[] = "Hola\n";
	static const char cadena2[] = "Mundo\n";

	Led_On(RGB_B_LED);
	if (EnviarTexto(cadena1))
	{
		GetTaskID(&tarea);
		ClearEvent(Completo);
		WaitEvent(Completo);
	}
	Led_Off(RGB_B_LED);

	Led_On(RGB_G_LED);
	if (EnviarTexto(cadena2))
	{
		GetTaskID(&tarea);
		ClearEvent(Completo);
		WaitEvent(Completo);
	}
	Led_Off(RGB_G_LED);

   /* terminate task */
   TerminateTask();
}

/** \brief Interrupt Service Routine: Serial
 *		irq26 recep
 *
 */
ISR(EventoSerial)
{
	if (EnviarCaracter())
	{
		SetEvent(tarea, Completo);
	}

}

/** @} doxygen end group definition */
/** @} doxygen end group definition */
/** @} doxygen end group definition */
/*==================[end of file]============================================*/
