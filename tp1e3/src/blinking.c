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

/** Stack size for each task. Ex. 256x1 (w:1byte)  */
typedef uint8_t stack_t[STACK_SIZE];

/*==================[internal data declaration]==============================*/
/** total stack of tasks [STACK_SIZE x TASK_COUNT] Ex. 256x2 (w:1byte)*/
static stack_t stack[TASK_COUNT];

/** addresses of the pointers for the different tasks. Ex. 1x3  (w:4byte)*/
static uint32_t context[TASK_COUNT+1];


/*==================[internal functions declaration]=========================*/
/**
 * \brief ContextChange:
 */
void ContextChange(void)
{
	/**
	 * \details
	 * R13: Stack Pointer (SP) alias of banked registers
	 */
	static int active = TASK_COUNT;

	/* Save current context: registers and stack address */
	/** store the desired registers into the stack */
	asm ("push {r0-r6,r8-r12}");
	/** save the registers value of the processor into a variable */
	asm ("str r13, %0": "=m"(context[active]));
	/** load the register of the processor with the value of a variable */
	asm ("ldr r13, %0": : "m"(context[TASK_COUNT]));

	/** Set the task as active */
	active = (active + 1) % TASK_COUNT;

	/** The Operative Systems toggle the green LED to show a context change */
	Led_Toggle(GREEN_LED);

	/* Load the context of the active task  */
	asm ("str r13, %0": "=m"(context[TASK_COUNT]));
	asm ("ldr r13, %0": : "m"(context[active]));
	asm ("pop {r0-r6,r8-r12}");
}

/** \brief Basic delay function by software. */
void Delay(void)
{
	/** \details Basic delay function to obtain a delay between flashing lights */
	uint32_t i;

	for(i=COUNT_DELAY; i!=0; i--)
	{
		asm  ("nop");
		ContextChange();
	}
}

/** \brief Delay function of ~3 seconds by software. */
void Delay3s(void)
{
	/** \details Basic delay function to obtain a delay between flashing lights */
	uint32_t i;

	for(i=5*COUNT_DELAY; i!=0; i--)
	{
		asm  ("nop");
		ContextChange();
	}
}

/*==================[internal data definition]===============================*/

/*==================[external data definition]===============================*/

/*==================[internal functions definition]==========================*/
/**
* Task A: Toggle the red led at 1 Hz.
*/
void TaskA(void)
{
	/**
	 * \details This function toggle the red led using a delay by software.
	 */
	while(1)
	{
		// Led_Off(YELLOW_LED);
		Led_Toggle(RED_LED);

		/** Llamada al cambio de contexto */
		 ContextChange();
		 Delay();
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
		if(Read_Switches()==TEC2)
		{
			Led_On(YELLOW_LED);
			Delay3s();
		}
		else
		{
			Led_Off(YELLOW_LED);
		}
		ContextChange();
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
	/** \brief Frame_call: struct to represent the current state of the processor registers  */
	struct {
		/** r0-r12: general purpose registers */
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
			uint32_t ip;		/** r12 */
		} context;
		/** registers used automatically by the compiler arm-none-eabi-gcc at the beginning of a subroutine */
		struct {
			uint32_t r7;		/** r7: macro pointer used by C compilers */
			uint32_t lr;		/** r14: Link Register. it stores the returning address  */
		} subrutine;
	} frame_call;

	/** initialization of the pointer to the stack */
	void * pointer = stack;

	/* The C library function void *memset(void *ptr, int c, size_t n) copies the
	 * value c to the first n characters of the data pointed to, by the argument ptr.
	 * 	ptr − This is a pointer to the block of memory to fill.
	 * 	c − This is the value to be set. The value is passed as an int, but the
	 * 	function fills the block of memory using the unsigned char conversion of
	 * 	this value.
	 * 	n − This is the number of bytes to be set to the value.
	 */

	/** Cleaning the stack */
	memset(stack, 0, sizeof(stack));
	/** Cleaning the frame_call */
	memset(&frame_call, 0 , sizeof(frame_call));

	/** from the lowest position of the memory, the pointer is located STACK_SIZE (256)
	 * words later because the stack works from */
	pointer += STACK_SIZE;
	/** Save the address of taskA frame call */
	frame_call.subrutine.r7 = (uint32_t) (pointer);
	/** Save the returning address to the task A */
	frame_call.subrutine.lr = (uint32_t) (TaskA);
	/** Copy the frame_call to the first part of the stack (highest address positions)*/
	memcpy(pointer - sizeof(frame_call), &frame_call, sizeof(frame_call));
	/** Save the position where the available stack is located after the frame call */
	context[0] = (uint32_t) (pointer - sizeof(frame_call));

	pointer += STACK_SIZE;
	frame_call.subrutine.r7 = (uint32_t) (pointer);
	frame_call.subrutine.lr = (uint32_t) (TaskB);
	memcpy(pointer - sizeof(frame_call), &frame_call, sizeof(frame_call));
	context[1] = (uint32_t) (pointer - sizeof(frame_call));


	/* Initializations of peripherals */

	/** Initializations of the LEDs in the EDU-CIAA Board */
	Init_Leds();
	/** Initializations of the switched in the EDU-CIAA Board */
	Init_Switches();

	/* Main program */
	/** We don't call anymore TaskA or TaskB, we just change the context and the pointer
	 * set which task will be active at that moment */
	ContextChange();


	return 0;
}

/** @} doxygen end group definition */
/** @} doxygen end group definition */
/** @} doxygen end group definition */
/*==================[end of file]============================================*/
