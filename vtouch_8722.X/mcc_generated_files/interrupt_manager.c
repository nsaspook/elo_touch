/**
  Generated Interrupt Manager Header File

  @Company:
    Microchip Technology Inc.

  @File Name:
    interrupt_manager.h

  @Summary:
    This is the Interrupt Manager file generated using PIC10 / PIC12 / PIC16 / PIC18 MCUs

  @Description:
    This header file provides implementations for global interrupt handling.
    For individual peripheral handlers please see the peripheral driver for
    all modules selected in the GUI.
    Generation Information :
	Product Revision  :  PIC10 / PIC12 / PIC16 / PIC18 MCUs - 1.81.7
	Device            :  PIC18F47Q43
	Driver Version    :  2.12
    The generated drivers are tested against the following:
	Compiler          :  XC8 2.31 and above or later
	MPLAB 	          :  MPLAB X 5.45
 */

/*
    (c) 2018 Microchip Technology Inc. and its subsidiaries. 
    
    Subject to your compliance with these terms, you may use Microchip software and any 
    derivatives exclusively with Microchip products. It is your responsibility to comply with third party 
    license terms applicable to your use of third party software (including open source software) that 
    may accompany Microchip software.
    
    THIS SOFTWARE IS SUPPLIED BY MICROCHIP "AS IS". NO WARRANTIES, WHETHER 
    EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS SOFTWARE, INCLUDING ANY 
    IMPLIED WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY, AND FITNESS 
    FOR A PARTICULAR PURPOSE.
    
    IN NO EVENT WILL MICROCHIP BE LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE, 
    INCIDENTAL OR CONSEQUENTIAL LOSS, DAMAGE, COST OR EXPENSE OF ANY KIND 
    WHATSOEVER RELATED TO THE SOFTWARE, HOWEVER CAUSED, EVEN IF MICROCHIP 
    HAS BEEN ADVISED OF THE POSSIBILITY OR THE DAMAGES ARE FORESEEABLE. TO 
    THE FULLEST EXTENT ALLOWED BY LAW, MICROCHIP'S TOTAL LIABILITY ON ALL 
    CLAIMS IN ANY WAY RELATED TO THIS SOFTWARE WILL NOT EXCEED THE AMOUNT 
    OF FEES, IF ANY, THAT YOU HAVE PAID DIRECTLY TO MICROCHIP FOR THIS 
    SOFTWARE.
 */

#include "interrupt_manager.h"
#include "mcc.h"

void INTERRUPT_Initialize(void)
{
	// Enable Interrupt Priority Vectors
	INTCON0bits.IPEN = 1;

	// Assign peripheral interrupt priority vectors

	// UTXI - high priority
	IPR8bits.U2TXIP = 1;

	// URXI - high priority
	IPR8bits.U2RXIP = 1;

	// UTXI - high priority
	IPR4bits.U1TXIP = 1;

	// URXI - high priority
	IPR4bits.U1RXIP = 1;

	// TMRI - high priority
	IPR3bits.TMR0IP = 1;

	// TMRI - high priority
	IPR15bits.TMR6IP = 1;

	// TMRI - high priority
	IPR8bits.TMR5IP = 1;


}

void rxtx_handler(void);

void __interrupt() INTERRUPT_InterruptManagerHigh(void)
{
	goto done;

	// interrupt handler
	if (PIE8bits.U2TXIE == 1 && PIR8bits.U2TXIF == 1) {
		UART2_TxInterruptHandler();
	} else if (PIE8bits.U2RXIE == 1 && PIR8bits.U2RXIF == 1) {
		UART2_RxInterruptHandler();
	} else if (PIE4bits.U1TXIE == 1 && PIR4bits.U1TXIF == 1) {
		UART1_TxInterruptHandler();
	} else if (PIE4bits.U1RXIE == 1 && PIR4bits.U1RXIF == 1) {
		UART1_RxInterruptHandler();
	} else if (PIE3bits.TMR0IE == 1 && PIR3bits.TMR0IF == 1) {
		TMR0_ISR();
	} else if (PIE15bits.TMR6IE == 1 && PIR15bits.TMR6IF == 1) {
		TMR6_ISR();
	} else if (PIE8bits.TMR5IE == 1 && PIR8bits.TMR5IF == 1) {
		TMR5_ISR();
	} else {
		//Unhandled Interrupt
	}
done:
	rxtx_handler();
}

/**
 End of File
 */
