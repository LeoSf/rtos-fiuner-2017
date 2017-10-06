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

/** \brief TP1.2 Bare Metal exercise source file
 **
 ** This is a entry example to implement in another program a RTOS.
 ** Dependencies: baremetal drivers library (v1.0) provided by
 ** MSc. Filomena and MSc. Reta.
 **
 ** Proposed Exercise:
 ** Write a program for the EDU-CIAA-NXP similar to TP1.1 that turns
 ** on for about 3 seconds the yellow led by pressing the button 2.
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
 * 20171005 v1.0 example with accurate documentation.
 * 20170922 v0.1 Base version of a blinking example with multiple functions calls.
 */

/*==================[inclusions]=============================================*/
#include <blinking.h>       /* <= own header */

/*==================[macros and definitions]=================================*/
/** Max counter value to make a basic delay with a for loop */
#define COUNT_DELAY 3000000


/*==================[internal data declaration]==============================*/

/*==================[internal functions declaration]=========================*/
/** \brief Basic delay function by software. */
void Delay(void)
{
	/** \details Basic delay function to obtain a delay between flashing lights */
	uint32_t i;

	for(i=COUNT_DELAY; i!=0; i--)
	{
		   asm  ("nop");
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
	}
}

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
		Led_Toggle(RED_LED);
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

	}

}

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
   /** Initializations of the LEDs in the EDU-CIAA Board */
	Init_Leds();
	/** Initializations of the switched in the EDU-CIAA Board */
	Init_Switches();

   /* Main program */
//	TaskA();
	TaskB();

	return 0;
}

/** @} doxygen end group definition */
/** @} doxygen end group definition */
/** @} doxygen end group definition */
/*==================[end of file]============================================*/
