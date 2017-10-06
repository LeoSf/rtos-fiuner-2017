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

/** \brief TP1.3 Bare Metal exercise source file (Context change of a operative system example)
 **
 ** This exercise is based on the CIAA Firmware and applied basic concepts
 ** of context change to understand a real time operative system. In this
 ** case, we implement a cooperative OS.
 **
 ** Dependencies: baremetal drivers library (v1.0) provided by
 ** MSc. Filomena and MSc. Reta.
 **
 ** Proposed Exercise:
 ** Combine the programs of TP1.1 and TP1.2 and write a subroutine that allows the
 ** concurrent execution in cooperative form of the two programs. To do this you
 ** must define global variables to store the context of each of the tasks and a
 ** main program that completes the context of the tasks before executing them.
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
 * 20171006 v1.1 improvements over variable's names and methods.
 * 20171005 v1.0 example with accurate documentation.
 * 20170922 v0.1 Base version of a context change in a OS.
 */

/*==================[inclusions]=============================================*/
#include "blinking.h"       /* <= own header */

/*==================[macros and definitions]=================================*/
/** Space allocated for each task: */
#define STACK_SIZE	256
/** Number of task present in our Operative System */
#define TASK_COUNT	2

/** Max counter value to make a basic delay with a for loop */
#define COUNT_DELAY 300000

/** Stack size for each task */
typedef uint8_t stack_t[STACK_SIZE];

/*==================[internal data declaration]==============================*/
/** total stack of tasks */
static stack_t stack[TASK_COUNT];

/** addresses of the pointers for the different tasks */
static uint32_t context[TASK_COUNT+1];


/*==================[internal functions declaration]=========================*/
/** \brief Basic delay function by software. */
void Delay(void)
{
	/** \details Basic delay function to obtain a delay between flashing lights */
	uint32_t i;

	for(i=COUNT_DELAY; i!=0; i--)
	{
		asm  ("nop");
		CambioDeContexto();
	}
}

/** \brief Delay function of ~3 seconds by software. */
void Delay3s(void)
{
	/** \details Basic delay function to obtain a delay between flashing lights */
	uint32_t i;

	for(i=10*COUNT_DELAY; i!=0; i--)
	{
		asm  ("nop");
		CambioDeContexto();
	}
}

/*==================[internal data definition]===============================*/

/*==================[external data definition]===============================*/

/*==================[internal functions definition]==========================*/
/**
* Task A: Toggle the red led at 1 Hz.
*/
void TareaA(void)
{
	/**
	 * \details This function toggle the red led using a delay by software.
	 */
	while(1)
	{
		// Led_Off(YELLOW_LED);
		Led_Toggle(RED_LED);
		Delay();
		/** Llamada al cambio de contexto */
		// CambioDeContexto();
	}
}

/**
* Task B: Turn on the yellow led when a key is pressed.
*/
void TaskB(void)
{
	/**
	 * \details This method turn on the yellow LED when TEC2 is pressed and
	* it holds its state for approximately 3 seconds.
	*/
	while(1)
	{
		// Led_Off(RED_LED);
		if(Read_Switches()==TEC2)
		{
			Led_Off(RED_LED);
			Led_Off(GREEN_LED);
			Led_On(YELLOW_LED);
			// Delay3s();
			Delay();
		}
		else
		{
			Led_Off(YELLOW_LED);
			CambioDeContexto();
		}
		/** Llamada al cambio de contexto */
		//  CambioDeContexto();
	}
}

void CambioDeContexto(void)
{
	static int activa = TASK_COUNT;

	/** guardo contexto de registros y dirección de la pila */
	asm ("push {r0-r6,r8-r12}");
	asm ("str r13, %0": "=m"(context[activa]));
	asm ("ldr r13, %0": : "m"(context[TASK_COUNT]));

	/** Pongo activa la tarea */
	activa = (activa + 1) % TASK_COUNT;

	/** El SO cambia el estado del Led Verde */
	Led_Toggle(GREEN_LED);

	/** Carga el contexto de la tarea activa */
	asm ("str r13, %0": "=m"(context[TASK_COUNT]));
	asm ("ldr r13, %0": : "m"(context[activa]));
	asm ("pop {r0-r6,r8-r12}");

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
	struct {
		struct {
			uint32_t r0;
			uint32_t r1;
			uint32_t r2;
			uint32_t r3;
			uint32_t r4;
			uint32_t r5;
			uint32_t r6;
			uint32_t r8;
			uint32_t r9;
			uint32_t r10;
			uint32_t r11;
			uint32_t ip;
		} context;
		struct {
			uint32_t r7;
			uint32_t lr;
		} subrutine;
	} frame_call;

	/** Puntero a la pila */
	void * pointer = stack;

	memset(stack, 0, sizeof(stack));
	memset(&frame_call, 0 , sizeof(frame_call));

	pointer += STACK_SIZE;
	frame_call.subrutine.r7 = (uint32_t) (pointer);
	frame_call.subrutine.lr = (uint32_t) (TareaA);
	memcpy(pointer - sizeof(frame_call), &frame_call, sizeof(frame_call));
	context[0] = (uint32_t) (pointer - sizeof(frame_call));

	pointer += STACK_SIZE;
	frame_call.subrutine.r7 = (uint32_t) (pointer);
	frame_call.subrutine.lr = (uint32_t) (TaskB);
	memcpy(pointer - sizeof(frame_call), &frame_call, sizeof(frame_call));
	context[1] = (uint32_t) (pointer - sizeof(frame_call));


	/* Inicialización de cosas */

	/** Initializations of the LEDs in the EDU-CIAA Board */
	Init_Leds();
	/** Initializations of the switched in the EDU-CIAA Board */
	Init_Switches();

	 /* Main program */
	// TareaA();
	// TareaB();
	CambioDeContexto();


	return 0;
}

/** @} doxygen end group definition */
/** @} doxygen end group definition */
/** @} doxygen end group definition */
/*==================[end of file]============================================*/
