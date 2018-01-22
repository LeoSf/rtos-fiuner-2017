/* Copyright 2016,
 * Leandro D. Medus
 * lmedus@ingenieria.uner.edu.ar
 * Eduardo Filomena
 * efilomena@ingenieria.uner.edu.ar
 * Juan Manuel Reta
 * jmrera@ingenieria.uner.edu.ar
 * Facultad de Ingeniería
 * Universidad Nacional de Entre Ríos
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

/** \brief AD Converter Bare Metal driver for the peripheral in the EDU-CIAA Board.
 **
 ** This is a driver to control the peripheral Analog to Digital Converter.
 **
 **/

/** \addtogroup EDU-CIAA_Course
 ** @{ */
/** \addtogroup Sources_LDM Leandro D. Medus Sources
 ** @{ */
/** \addtogroup Baremetal_App Bare Metal application source file
 ** @{ */

/*
 * Initials     Name
 * ---------------------------
 *	LM			Leandro Medus
 * EF			Eduardo Filomena
 * JMR		JuanManuel Reta
 */

/*
 * modification history (new versions first)
 * -----------------------------------------------------------
 * 20160422 v0.1 initials initial version by Leandro Medus
 * 20160807 v0.2 modifications and improvements made by Eduardo Filomena
 * 20160808 v0.3 modifications and improvements made by Juan Manuel Reta
 * 20171227 v0.2 modifications and new configuration methods by Leandro Medus
 */

/*==================[inclusions]=============================================*/
#include "adc.h"

/*==================[macros and definitions]=================================*/

/*==================[internal data declaration]==============================*/
void (*pIsrADC0)();
/*==================[internal functions declaration]=========================*/

/*==================[internal data definition]===============================*/

/*==================[external data definition]===============================*/

/*==================[internal functions definition]==========================*/
 void ISR_ADC0(){

	pIsrADC0();
}
/*==================[external functions definition]==========================*/
/** \brief ADC Initialization method  */
uint8_t Init_Adc(void)
{

	/** \details
	 * This function initialize the ADC peripheral in the EDU-CIAA board,
	 * with the correct parameters with LPCOpen library. It uses CH1
	 *
	 * \param none
	 *
	 * \return uint8_t: TBD (to support errors in the init function)
	 * */
	static ADC_CLOCK_SETUP_T configADC;

	configADC.adcRate=4000;		/** max 409 KHz*/
	configADC.burstMode=DISABLE;
	configADC.bitsAccuracy=ADC_10BITS;

	Chip_ADC_Init(LPC_ADC0,&configADC);
//	Chip_ADC_EnableChannel(LPC_ADC0,ADC_CH1,ENABLE);
	Chip_ADC_SetSampleRate(LPC_ADC0, &configADC,ADC_MAX_SAMPLE_RATE);

	return TRUE;
}

/** \brief ADC Ch1 Acquisition method by pooling */
uint16_t Read_Adc_Value_Pooling(void)
{
	/** \details
	 * This function initialize the DAC peripheral in the EDU-CIAA board,
	 * with the correct parameters with LPCOpen methods.
	 *
	 * \param none
	 *
	 * \return uint8_t: TBD (to support errors in the init function)
	 * */
	uint16_t valueRead = 0 ;

	/** Start Acquisition */
	Chip_ADC_SetStartMode(LPC_ADC0, ADC_START_NOW, ADC_TRIGGERMODE_RISING);
	/** The pooling magic! */
	while (Chip_ADC_ReadStatus(LPC_ADC0, ADC_CH1, ADC_DR_DONE_STAT) != SET)
	{
		/** pooooliiinnggg maaagggicccc plif plif pluf pluf */
	}
	/** Conversion complete, and value reading */
	Chip_ADC_ReadValue(LPC_ADC0,ADC_CH1, &valueRead);

	return valueRead;
}

/** Start Acquisition */
void Start_Adc(void){
  Chip_ADC_SetStartMode(LPC_ADC0, ADC_START_NOW, ADC_TRIGGERMODE_RISING);
	
}
uint16_t Read_Adc_Value(void){
  uint16_t data;
  Chip_ADC_ReadValue(LPC_ADC0,ADC_CH1, &data);
  return data;

  }
  
void Enable_Adc_Irq(void *pfunc){
	pIsrADC0=pfunc;
	/*Enable interrupt for ADC channel */

	Chip_ADC_Int_SetChannelCmd(LPC_ADC0,ADC_CH1,ENABLE);
	NVIC_EnableIRQ(ADC0_IRQn);
  }

uint8_t Enable_ADC_ch1(uint8_t configFlag)
{
	if(configFlag == ENABLE_ADC)
		Chip_ADC_EnableChannel(LPC_ADC0,ADC_CH1,ENABLE);
	else
		Chip_ADC_EnableChannel(LPC_ADC0,ADC_CH1,DISABLE);

	return TRUE;
}

uint8_t Enable_ADC_ch2(uint8_t configFlag)
{
	if(configFlag == ENABLE_ADC)
		Chip_ADC_EnableChannel(LPC_ADC0,ADC_CH2,ENABLE);
	else
			Chip_ADC_EnableChannel(LPC_ADC0,ADC_CH2,DISABLE);

	return TRUE;
}

uint8_t Enable_ADC_ch3(uint8_t configFlag)
{
	if(configFlag == ENABLE_ADC)
		Chip_ADC_EnableChannel(LPC_ADC0,ADC_CH3,ENABLE);
	else
			Chip_ADC_EnableChannel(LPC_ADC0,ADC_CH3,DISABLE);

	return TRUE;
}

/** \brief ADC Chx Acquisition method by pooling */
uint16_t Read_ADC_pooling(uint8_t channel)
{
	/** \details
	 * This function initialize the DAC peripheral in the EDU-CIAA board,
	 * with the correct parameters with LPCOpen methods.
	 *
	 * \param none
	 *
	 * \return uint8_t: TBD (to support errors in the init function)
	 * */
	uint16_t valueRead = 0 ;

	/** Start Acquisition */
	Chip_ADC_SetStartMode(LPC_ADC0, ADC_START_NOW, ADC_TRIGGERMODE_RISING);
	/** The pooling magic! */
	while (Chip_ADC_ReadStatus(LPC_ADC0, channel, ADC_DR_DONE_STAT) != SET)
	{
		/** pooooliiinnggg maaagggicccc plif plif pluf pluf */
	}
	/** Conversion complete, and value reading */
	Chip_ADC_ReadValue(LPC_ADC0,channel, &valueRead);

	return valueRead;
}


/** @} doxygen end group definition */
/** @} doxygen end group definition */
/** @} doxygen end group definition */
/*==================[end of file]============================================*/
