

/** \brief Vector Interrupt tables
 **
 ** This file contains the vector interrupt tables
 **
 **/

/** \addtogroup CIAA_Firmware CIAA Firmware
 ** @{ */
/** \addtogroup Vector Vector interrupt tables
 ** @{ */

/*
 * Initials     Name
 * ---------------------------
 *
 */

/*
 * modification history (new versions first)
 * -----------------------------------------------------------
 * yyyymmdd v0.0.1 initials initial version
 */

/*==================[inclusions]=============================================*/
#include <tpf_app.h>

/*==================[macros and definitions]=================================*/

/*==================[internal data declaration]==============================*/

/*==================[internal functions declaration]=========================*/
/** \brief Handler for NMI */
static void NMI_Handler(void);

/** \brief Handler for HardFault */
static void HardFault_Handler(void);

/** \brief Handler for Memory Manager */
static void MemManage_Handler(void);

/** \brief Handler for Bus Fault */
static void BusFault_Handler(void);

/** \brief Handler for Usage Fault */
static void UsageFault_Handler(void);

/** \brief Handler for SVC */
extern void SVC_Handler(void);

/** \brief Handler for Debug Monitor */
static void DebugMon_Handler(void);

/** \brief System Tick Handler */
extern void SysTick_Handler(void);

/** \brief Pending Service Handler */
extern void PendSV_Handler(void);

/** \brief Dummy empty ISR Handler */
static void ISR_NoHandler(void);

/*==================[internal data definition]===============================*/

/*==================[external data definition]===============================*/

/** \brief LPC4337 Interrupt vector */
__attribute__ ((section(".isr_vector")))
void (* const g_pfnVectors[])(void) = {
   /* System ISRs */
   &_vStackTop,                    /* The initial stack pointer  */
   ResetISR,                       /* The reset handler          */
   NMI_Handler,                    /* The NMI handler            */
   HardFault_Handler,              /* The hard fault handler     */
   MemManage_Handler,              /* The MPU fault handler      */
   BusFault_Handler,               /* The bus fault handler      */
   UsageFault_Handler,             /* The usage fault handler    */
   0,                              /* Reserved                   */
   0,                              /* Reserved                   */
   0,                              /* Reserved                   */
   0,                              /* Reserved                   */
   SVC_Handler,                    /* SVCall handler             */
   DebugMon_Handler,               /* Debug monitor handler      */
   0,                              /* Reserved                   */
   PendSV_Handler,                 /* The PendSV handler         */
   SysTick_Handler,                /* The SysTick handler        */
   /*** User Interruptions ***/
   ISR_NoHandler,      /* 0x10 0x00000040 - No Handler set for ISR DAC (IRQ 0) */
   ISR_NoHandler,      /* 0x11 0x00000044 - No Handler set for ISR M0APP (IRQ 1) */
   ISR_NoHandler,      /* 0x12 0x00000048 - No Handler set for ISR DMA (IRQ 2) */
   ISR_NoHandler,      /* 0x13 0x0000004C - No Handler set for ISR RES1 (IRQ 3) */
   ISR_NoHandler,      /* 0x14 0x00000050 - No Handler set for ISR FLASH_EEPROM (IRQ 4) */
   ISR_NoHandler,      /* 0x15 0x00000054 - No Handler set for ISR ETH (IRQ 5) */
   ISR_NoHandler,      /* 0x16 0x00000058 - No Handler set for ISR SDIO (IRQ 6) */
   ISR_NoHandler,      /* 0x17 0x0000005C - No Handler set for ISR LCD (IRQ 7) */
   ISR_NoHandler,      /* 0x18 0x00000060 - No Handler set for ISR USB0 (IRQ 8) */
   ISR_NoHandler,      /* 0x19 0x00000064 - No Handler set for ISR USB1 (IRQ 9) */
   ISR_NoHandler,      /* 0x1a 0x00000068 - No Handler set for ISR SCT (IRQ 10) */
	RIT_IRQHandler,     /* --- 0x1b 0x0000006C - No Handler set for ISR RIT (IRQ 11) */
   ISR_NoHandler,      /* 0x1c 0x00000070 - No Handler set for ISR TIMER0 (IRQ 12) */
   ISR_NoHandler,      /* 0x1d 0x00000074 - No Handler set for ISR TIMER1 (IRQ 13) */
   ISR_NoHandler,      /* 0x1e 0x00000078 - No Handler set for ISR TIMER2 (IRQ 14) */
   ISR_NoHandler,      /* 0x1f 0x0000007C - No Handler set for ISR TIMER3 (IRQ 15) */
   ISR_NoHandler,      /* 0x20 0x00000080 - No Handler set for ISR MCPWM (IRQ 16) */
   ISR_NoHandler,      /* --- 0x21 0x00000084 - No Handler set for ISR ADC0 (IRQ 17) */
   ISR_NoHandler,      /* 0x22 0x00000088 - No Handler set for ISR I2C0 (IRQ 18) */
   ISR_NoHandler,      /* 0x23 0x0000008C - No Handler set for ISR I2C1 (IRQ 19) */
   ISR_NoHandler,      /* 0x24 0x00000090 - No Handler set for ISR SPI (IRQ 20) */
   ISR_NoHandler,      /* 0x25 0x00000094 - No Handler set for ISR ADC1 (IRQ 21) */
   ISR_NoHandler,      /* 0x26 0x00000098 - No Handler set for ISR SSP0 (IRQ 22) */
   ISR_NoHandler,      /* 0x27 0x0000009C - No Handler set for ISR SSP1 (IRQ 23) */
   ISR_NoHandler,      /* 0x28 0x000000A0 - No Handler set for ISR UART0 (IRQ 24) */
   ISR_NoHandler,      /* 0x29 0x000000A4 - No Handler set for ISR UART1 (IRQ 25) */
   EventoSerial,       /* --- 0x2a 0x000000A8 - No Handler set for ISR UART2 (IRQ 26) */
   ISR_NoHandler,      /* 0x2b 0x000000AC - No Handler set for ISR UART3 (IRQ 27) */
   ISR_NoHandler,      /* 0x2c 0x000000B0 - No Handler set for ISR I2S0 (IRQ 28) */
   ISR_NoHandler,      /* 0x2d 0x000000B4 - No Handler set for ISR I2S1 (IRQ 29) */
   ISR_NoHandler,      /* 0x2e 0x000000B8 - No Handler set for ISR SPIFI (IRQ 30) */
   ISR_NoHandler,      /* 0x2f 0x000000BC - No Handler set for ISR SGPIO (IRQ 31) */
   ISR_NoHandler,      /* 0x30 0x000000C0 - No Handler set for ISR GPIO0 (IRQ 32) */
   ISR_NoHandler,      /* 0x31 0x000000C4 - No Handler set for ISR GPIO1 (IRQ 33) */
   ISR_NoHandler,      /* 0x32 0x000000C8 - No Handler set for ISR GPIO2 (IRQ 34) */
   ISR_NoHandler,      /* 0x33 0x000000CC - No Handler set for ISR GPIO3 (IRQ 35) */
   ISR_NoHandler,      /* 0x34 0x000000D0 - No Handler set for ISR GPIO4 (IRQ 36) */
   ISR_NoHandler,      /* 0x35 0x000000D4 - No Handler set for ISR GPIO5 (IRQ 37) */
   ISR_NoHandler,      /* 0x36 0x000000D8 - No Handler set for ISR GPIO6 (IRQ 38) */
   ISR_NoHandler,      /* 0x37 0x000000DC - No Handler set for ISR GPIO7 (IRQ 39) */
   ISR_NoHandler,      /* 0x38 0x000000E0 - No Handler set for ISR GINT0 (IRQ 40) */
   ISR_NoHandler,      /* 0x39 0x000000E4 - No Handler set for ISR GINT1 (IRQ 41) */
   ISR_NoHandler,      /* 0x3a 0x000000E8 - No Handler set for ISR EVRT (IRQ 42) */
   ISR_NoHandler,      /* 0x3b 0x000000EC - No Handler set for ISR CAN1 (IRQ 43) */
   ISR_NoHandler,      /* 0x3c 0x000000F0 - No Handler set for ISR RES6 (IRQ 44) */
   ISR_NoHandler,      /* 0x3d 0x000000F4 - No Handler set for ISR ADCHS (IRQ 45) */
   ISR_NoHandler,      /* 0x3e 0x000000F8 - No Handler set for ISR ATIMER (IRQ 46) */
   ISR_NoHandler,      /* 0x3f 0x000000FC - No Handler set for ISR RTC (IRQ 47) */
   ISR_NoHandler,      /* 0x40 0x00000100 - No Handler set for ISR RES8 (IRQ 48) */
   ISR_NoHandler,      /* 0x41 0x00000104 - No Handler set for ISR WDT (IRQ 49) */
   ISR_NoHandler,      /* 0x42 0x00000108 - No Handler set for ISR M0SUB (IRQ 50) */
   ISR_NoHandler,      /* 0x43 0x0000010C - No Handler set for ISR CAN0 (IRQ 51) */
   ISR_NoHandler,      /* 0x44 0x00000110 - No Handler set for ISR QEI (IRQ 52) */
};

/*==================[internal functions definition]==========================*/
/* Default exception handlers. */
__attribute__ ((section(".after_vectors")))
static void NMI_Handler(void) {
    while (1) {
    }
}

__attribute__ ((section(".after_vectors")))
static void HardFault_Handler(void) {
    while (1) {
    }
}

__attribute__ ((section(".after_vectors")))
static void MemManage_Handler(void) {
    while (1) {
    }
}

__attribute__ ((section(".after_vectors")))
static void BusFault_Handler(void) {
    while (1) {
    }
}

__attribute__ ((section(".after_vectors")))
static void UsageFault_Handler(void) {
    while (1) {
    }
}

__attribute__ ((section(".after_vectors")))
static void DebugMon_Handler(void) {
    while (1) {
    }
}

__attribute__ ((section(".after_vectors")))
static void ISR_NoHandler(void) {
    while (1) {
    }
}

/*==================[external functions definition]==========================*/

/** @} doxygen end group definition */
/** @} doxygen end group definition */
/*==================[end of file]============================================*/

