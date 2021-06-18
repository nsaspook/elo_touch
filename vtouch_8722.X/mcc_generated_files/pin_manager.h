/**
  @Generated Pin Manager Header File

  @Company:
    Microchip Technology Inc.

  @File Name:
    pin_manager.h

  @Summary:
    This is the Pin Manager file generated using PIC10 / PIC12 / PIC16 / PIC18 MCUs

  @Description
    This header file provides APIs for driver for .
    Generation Information :
        Product Revision  :  PIC10 / PIC12 / PIC16 / PIC18 MCUs - 1.81.7
        Device            :  PIC18F47Q43
        Driver Version    :  2.11
    The generated drivers are tested against the following:
        Compiler          :  XC8 2.31 and above
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

#ifndef PIN_MANAGER_H
#define PIN_MANAGER_H

/**
  Section: Included Files
*/

#include <xc.h>

#define INPUT   1
#define OUTPUT  0

#define HIGH    1
#define LOW     0

#define ANALOG      1
#define DIGITAL     0

#define PULL_UP_ENABLED      1
#define PULL_UP_DISABLED     0

// get/set JMP1 aliases
#define JMP1_TRIS                 TRISAbits.TRISA0
#define JMP1_LAT                  LATAbits.LATA0
#define JMP1_PORT                 PORTAbits.RA0
#define JMP1_WPU                  WPUAbits.WPUA0
#define JMP1_OD                   ODCONAbits.ODCA0
#define JMP1_ANS                  ANSELAbits.ANSELA0
#define JMP1_SetHigh()            do { LATAbits.LATA0 = 1; } while(0)
#define JMP1_SetLow()             do { LATAbits.LATA0 = 0; } while(0)
#define JMP1_Toggle()             do { LATAbits.LATA0 = ~LATAbits.LATA0; } while(0)
#define JMP1_GetValue()           PORTAbits.RA0
#define JMP1_SetDigitalInput()    do { TRISAbits.TRISA0 = 1; } while(0)
#define JMP1_SetDigitalOutput()   do { TRISAbits.TRISA0 = 0; } while(0)
#define JMP1_SetPullup()          do { WPUAbits.WPUA0 = 1; } while(0)
#define JMP1_ResetPullup()        do { WPUAbits.WPUA0 = 0; } while(0)
#define JMP1_SetPushPull()        do { ODCONAbits.ODCA0 = 0; } while(0)
#define JMP1_SetOpenDrain()       do { ODCONAbits.ODCA0 = 1; } while(0)
#define JMP1_SetAnalogMode()      do { ANSELAbits.ANSELA0 = 1; } while(0)
#define JMP1_SetDigitalMode()     do { ANSELAbits.ANSELA0 = 0; } while(0)

// get/set JMP1SET aliases
#define JMP1SET_TRIS                 TRISAbits.TRISA1
#define JMP1SET_LAT                  LATAbits.LATA1
#define JMP1SET_PORT                 PORTAbits.RA1
#define JMP1SET_WPU                  WPUAbits.WPUA1
#define JMP1SET_OD                   ODCONAbits.ODCA1
#define JMP1SET_ANS                  ANSELAbits.ANSELA1
#define JMP1SET_SetHigh()            do { LATAbits.LATA1 = 1; } while(0)
#define JMP1SET_SetLow()             do { LATAbits.LATA1 = 0; } while(0)
#define JMP1SET_Toggle()             do { LATAbits.LATA1 = ~LATAbits.LATA1; } while(0)
#define JMP1SET_GetValue()           PORTAbits.RA1
#define JMP1SET_SetDigitalInput()    do { TRISAbits.TRISA1 = 1; } while(0)
#define JMP1SET_SetDigitalOutput()   do { TRISAbits.TRISA1 = 0; } while(0)
#define JMP1SET_SetPullup()          do { WPUAbits.WPUA1 = 1; } while(0)
#define JMP1SET_ResetPullup()        do { WPUAbits.WPUA1 = 0; } while(0)
#define JMP1SET_SetPushPull()        do { ODCONAbits.ODCA1 = 0; } while(0)
#define JMP1SET_SetOpenDrain()       do { ODCONAbits.ODCA1 = 1; } while(0)
#define JMP1SET_SetAnalogMode()      do { ANSELAbits.ANSELA1 = 1; } while(0)
#define JMP1SET_SetDigitalMode()     do { ANSELAbits.ANSELA1 = 0; } while(0)

// get/set JMP2 aliases
#define JMP2_TRIS                 TRISAbits.TRISA2
#define JMP2_LAT                  LATAbits.LATA2
#define JMP2_PORT                 PORTAbits.RA2
#define JMP2_WPU                  WPUAbits.WPUA2
#define JMP2_OD                   ODCONAbits.ODCA2
#define JMP2_ANS                  ANSELAbits.ANSELA2
#define JMP2_SetHigh()            do { LATAbits.LATA2 = 1; } while(0)
#define JMP2_SetLow()             do { LATAbits.LATA2 = 0; } while(0)
#define JMP2_Toggle()             do { LATAbits.LATA2 = ~LATAbits.LATA2; } while(0)
#define JMP2_GetValue()           PORTAbits.RA2
#define JMP2_SetDigitalInput()    do { TRISAbits.TRISA2 = 1; } while(0)
#define JMP2_SetDigitalOutput()   do { TRISAbits.TRISA2 = 0; } while(0)
#define JMP2_SetPullup()          do { WPUAbits.WPUA2 = 1; } while(0)
#define JMP2_ResetPullup()        do { WPUAbits.WPUA2 = 0; } while(0)
#define JMP2_SetPushPull()        do { ODCONAbits.ODCA2 = 0; } while(0)
#define JMP2_SetOpenDrain()       do { ODCONAbits.ODCA2 = 1; } while(0)
#define JMP2_SetAnalogMode()      do { ANSELAbits.ANSELA2 = 1; } while(0)
#define JMP2_SetDigitalMode()     do { ANSELAbits.ANSELA2 = 0; } while(0)

// get/set JMP2SET aliases
#define JMP2SET_TRIS                 TRISAbits.TRISA3
#define JMP2SET_LAT                  LATAbits.LATA3
#define JMP2SET_PORT                 PORTAbits.RA3
#define JMP2SET_WPU                  WPUAbits.WPUA3
#define JMP2SET_OD                   ODCONAbits.ODCA3
#define JMP2SET_ANS                  ANSELAbits.ANSELA3
#define JMP2SET_SetHigh()            do { LATAbits.LATA3 = 1; } while(0)
#define JMP2SET_SetLow()             do { LATAbits.LATA3 = 0; } while(0)
#define JMP2SET_Toggle()             do { LATAbits.LATA3 = ~LATAbits.LATA3; } while(0)
#define JMP2SET_GetValue()           PORTAbits.RA3
#define JMP2SET_SetDigitalInput()    do { TRISAbits.TRISA3 = 1; } while(0)
#define JMP2SET_SetDigitalOutput()   do { TRISAbits.TRISA3 = 0; } while(0)
#define JMP2SET_SetPullup()          do { WPUAbits.WPUA3 = 1; } while(0)
#define JMP2SET_ResetPullup()        do { WPUAbits.WPUA3 = 0; } while(0)
#define JMP2SET_SetPushPull()        do { ODCONAbits.ODCA3 = 0; } while(0)
#define JMP2SET_SetOpenDrain()       do { ODCONAbits.ODCA3 = 1; } while(0)
#define JMP2SET_SetAnalogMode()      do { ANSELAbits.ANSELA3 = 1; } while(0)
#define JMP2SET_SetDigitalMode()     do { ANSELAbits.ANSELA3 = 0; } while(0)

// get/set CAM_R aliases
#define CAM_R_TRIS                 TRISBbits.TRISB0
#define CAM_R_LAT                  LATBbits.LATB0
#define CAM_R_PORT                 PORTBbits.RB0
#define CAM_R_WPU                  WPUBbits.WPUB0
#define CAM_R_OD                   ODCONBbits.ODCB0
#define CAM_R_ANS                  ANSELBbits.ANSELB0
#define CAM_R_SetHigh()            do { LATBbits.LATB0 = 1; } while(0)
#define CAM_R_SetLow()             do { LATBbits.LATB0 = 0; } while(0)
#define CAM_R_Toggle()             do { LATBbits.LATB0 = ~LATBbits.LATB0; } while(0)
#define CAM_R_GetValue()           PORTBbits.RB0
#define CAM_R_SetDigitalInput()    do { TRISBbits.TRISB0 = 1; } while(0)
#define CAM_R_SetDigitalOutput()   do { TRISBbits.TRISB0 = 0; } while(0)
#define CAM_R_SetPullup()          do { WPUBbits.WPUB0 = 1; } while(0)
#define CAM_R_ResetPullup()        do { WPUBbits.WPUB0 = 0; } while(0)
#define CAM_R_SetPushPull()        do { ODCONBbits.ODCB0 = 0; } while(0)
#define CAM_R_SetOpenDrain()       do { ODCONBbits.ODCB0 = 1; } while(0)
#define CAM_R_SetAnalogMode()      do { ANSELBbits.ANSELB0 = 1; } while(0)
#define CAM_R_SetDigitalMode()     do { ANSELBbits.ANSELB0 = 0; } while(0)

// get/set CAM_R1 aliases
#define CAM_R1_TRIS                 TRISBbits.TRISB1
#define CAM_R1_LAT                  LATBbits.LATB1
#define CAM_R1_PORT                 PORTBbits.RB1
#define CAM_R1_WPU                  WPUBbits.WPUB1
#define CAM_R1_OD                   ODCONBbits.ODCB1
#define CAM_R1_ANS                  ANSELBbits.ANSELB1
#define CAM_R1_SetHigh()            do { LATBbits.LATB1 = 1; } while(0)
#define CAM_R1_SetLow()             do { LATBbits.LATB1 = 0; } while(0)
#define CAM_R1_Toggle()             do { LATBbits.LATB1 = ~LATBbits.LATB1; } while(0)
#define CAM_R1_GetValue()           PORTBbits.RB1
#define CAM_R1_SetDigitalInput()    do { TRISBbits.TRISB1 = 1; } while(0)
#define CAM_R1_SetDigitalOutput()   do { TRISBbits.TRISB1 = 0; } while(0)
#define CAM_R1_SetPullup()          do { WPUBbits.WPUB1 = 1; } while(0)
#define CAM_R1_ResetPullup()        do { WPUBbits.WPUB1 = 0; } while(0)
#define CAM_R1_SetPushPull()        do { ODCONBbits.ODCB1 = 0; } while(0)
#define CAM_R1_SetOpenDrain()       do { ODCONBbits.ODCB1 = 1; } while(0)
#define CAM_R1_SetAnalogMode()      do { ANSELBbits.ANSELB1 = 1; } while(0)
#define CAM_R1_SetDigitalMode()     do { ANSELBbits.ANSELB1 = 0; } while(0)

// get/set RB6 procedures
#define RB6_SetHigh()            do { LATBbits.LATB6 = 1; } while(0)
#define RB6_SetLow()             do { LATBbits.LATB6 = 0; } while(0)
#define RB6_Toggle()             do { LATBbits.LATB6 = ~LATBbits.LATB6; } while(0)
#define RB6_GetValue()              PORTBbits.RB6
#define RB6_SetDigitalInput()    do { TRISBbits.TRISB6 = 1; } while(0)
#define RB6_SetDigitalOutput()   do { TRISBbits.TRISB6 = 0; } while(0)
#define RB6_SetPullup()             do { WPUBbits.WPUB6 = 1; } while(0)
#define RB6_ResetPullup()           do { WPUBbits.WPUB6 = 0; } while(0)
#define RB6_SetAnalogMode()         do { ANSELBbits.ANSELB6 = 1; } while(0)
#define RB6_SetDigitalMode()        do { ANSELBbits.ANSELB6 = 0; } while(0)

// get/set RB7 procedures
#define RB7_SetHigh()            do { LATBbits.LATB7 = 1; } while(0)
#define RB7_SetLow()             do { LATBbits.LATB7 = 0; } while(0)
#define RB7_Toggle()             do { LATBbits.LATB7 = ~LATBbits.LATB7; } while(0)
#define RB7_GetValue()              PORTBbits.RB7
#define RB7_SetDigitalInput()    do { TRISBbits.TRISB7 = 1; } while(0)
#define RB7_SetDigitalOutput()   do { TRISBbits.TRISB7 = 0; } while(0)
#define RB7_SetPullup()             do { WPUBbits.WPUB7 = 1; } while(0)
#define RB7_ResetPullup()           do { WPUBbits.WPUB7 = 0; } while(0)
#define RB7_SetAnalogMode()         do { ANSELBbits.ANSELB7 = 1; } while(0)
#define RB7_SetDigitalMode()        do { ANSELBbits.ANSELB7 = 0; } while(0)

// get/set RS aliases
#define RS_TRIS                 TRISCbits.TRISC1
#define RS_LAT                  LATCbits.LATC1
#define RS_PORT                 PORTCbits.RC1
#define RS_WPU                  WPUCbits.WPUC1
#define RS_OD                   ODCONCbits.ODCC1
#define RS_ANS                  ANSELCbits.ANSELC1
#define RS_SetHigh()            do { LATCbits.LATC1 = 1; } while(0)
#define RS_SetLow()             do { LATCbits.LATC1 = 0; } while(0)
#define RS_Toggle()             do { LATCbits.LATC1 = ~LATCbits.LATC1; } while(0)
#define RS_GetValue()           PORTCbits.RC1
#define RS_SetDigitalInput()    do { TRISCbits.TRISC1 = 1; } while(0)
#define RS_SetDigitalOutput()   do { TRISCbits.TRISC1 = 0; } while(0)
#define RS_SetPullup()          do { WPUCbits.WPUC1 = 1; } while(0)
#define RS_ResetPullup()        do { WPUCbits.WPUC1 = 0; } while(0)
#define RS_SetPushPull()        do { ODCONCbits.ODCC1 = 0; } while(0)
#define RS_SetOpenDrain()       do { ODCONCbits.ODCC1 = 1; } while(0)
#define RS_SetAnalogMode()      do { ANSELCbits.ANSELC1 = 1; } while(0)
#define RS_SetDigitalMode()     do { ANSELCbits.ANSELC1 = 0; } while(0)

// get/set CSB aliases
#define CSB_TRIS                 TRISCbits.TRISC2
#define CSB_LAT                  LATCbits.LATC2
#define CSB_PORT                 PORTCbits.RC2
#define CSB_WPU                  WPUCbits.WPUC2
#define CSB_OD                   ODCONCbits.ODCC2
#define CSB_ANS                  ANSELCbits.ANSELC2
#define CSB_SetHigh()            do { LATCbits.LATC2 = 1; } while(0)
#define CSB_SetLow()             do { LATCbits.LATC2 = 0; } while(0)
#define CSB_Toggle()             do { LATCbits.LATC2 = ~LATCbits.LATC2; } while(0)
#define CSB_GetValue()           PORTCbits.RC2
#define CSB_SetDigitalInput()    do { TRISCbits.TRISC2 = 1; } while(0)
#define CSB_SetDigitalOutput()   do { TRISCbits.TRISC2 = 0; } while(0)
#define CSB_SetPullup()          do { WPUCbits.WPUC2 = 1; } while(0)
#define CSB_ResetPullup()        do { WPUCbits.WPUC2 = 0; } while(0)
#define CSB_SetPushPull()        do { ODCONCbits.ODCC2 = 0; } while(0)
#define CSB_SetOpenDrain()       do { ODCONCbits.ODCC2 = 1; } while(0)
#define CSB_SetAnalogMode()      do { ANSELCbits.ANSELC2 = 1; } while(0)
#define CSB_SetDigitalMode()     do { ANSELCbits.ANSELC2 = 0; } while(0)

// get/set RC3 procedures
#define RC3_SetHigh()            do { LATCbits.LATC3 = 1; } while(0)
#define RC3_SetLow()             do { LATCbits.LATC3 = 0; } while(0)
#define RC3_Toggle()             do { LATCbits.LATC3 = ~LATCbits.LATC3; } while(0)
#define RC3_GetValue()              PORTCbits.RC3
#define RC3_SetDigitalInput()    do { TRISCbits.TRISC3 = 1; } while(0)
#define RC3_SetDigitalOutput()   do { TRISCbits.TRISC3 = 0; } while(0)
#define RC3_SetPullup()             do { WPUCbits.WPUC3 = 1; } while(0)
#define RC3_ResetPullup()           do { WPUCbits.WPUC3 = 0; } while(0)
#define RC3_SetAnalogMode()         do { ANSELCbits.ANSELC3 = 1; } while(0)
#define RC3_SetDigitalMode()        do { ANSELCbits.ANSELC3 = 0; } while(0)

// get/set RC4 procedures
#define RC4_SetHigh()            do { LATCbits.LATC4 = 1; } while(0)
#define RC4_SetLow()             do { LATCbits.LATC4 = 0; } while(0)
#define RC4_Toggle()             do { LATCbits.LATC4 = ~LATCbits.LATC4; } while(0)
#define RC4_GetValue()              PORTCbits.RC4
#define RC4_SetDigitalInput()    do { TRISCbits.TRISC4 = 1; } while(0)
#define RC4_SetDigitalOutput()   do { TRISCbits.TRISC4 = 0; } while(0)
#define RC4_SetPullup()             do { WPUCbits.WPUC4 = 1; } while(0)
#define RC4_ResetPullup()           do { WPUCbits.WPUC4 = 0; } while(0)
#define RC4_SetAnalogMode()         do { ANSELCbits.ANSELC4 = 1; } while(0)
#define RC4_SetDigitalMode()        do { ANSELCbits.ANSELC4 = 0; } while(0)

// get/set RC5 procedures
#define RC5_SetHigh()            do { LATCbits.LATC5 = 1; } while(0)
#define RC5_SetLow()             do { LATCbits.LATC5 = 0; } while(0)
#define RC5_Toggle()             do { LATCbits.LATC5 = ~LATCbits.LATC5; } while(0)
#define RC5_GetValue()              PORTCbits.RC5
#define RC5_SetDigitalInput()    do { TRISCbits.TRISC5 = 1; } while(0)
#define RC5_SetDigitalOutput()   do { TRISCbits.TRISC5 = 0; } while(0)
#define RC5_SetPullup()             do { WPUCbits.WPUC5 = 1; } while(0)
#define RC5_ResetPullup()           do { WPUCbits.WPUC5 = 0; } while(0)
#define RC5_SetAnalogMode()         do { ANSELCbits.ANSELC5 = 1; } while(0)
#define RC5_SetDigitalMode()        do { ANSELCbits.ANSELC5 = 0; } while(0)

// get/set RC6 procedures
#define RC6_SetHigh()            do { LATCbits.LATC6 = 1; } while(0)
#define RC6_SetLow()             do { LATCbits.LATC6 = 0; } while(0)
#define RC6_Toggle()             do { LATCbits.LATC6 = ~LATCbits.LATC6; } while(0)
#define RC6_GetValue()              PORTCbits.RC6
#define RC6_SetDigitalInput()    do { TRISCbits.TRISC6 = 1; } while(0)
#define RC6_SetDigitalOutput()   do { TRISCbits.TRISC6 = 0; } while(0)
#define RC6_SetPullup()             do { WPUCbits.WPUC6 = 1; } while(0)
#define RC6_ResetPullup()           do { WPUCbits.WPUC6 = 0; } while(0)
#define RC6_SetAnalogMode()         do { ANSELCbits.ANSELC6 = 1; } while(0)
#define RC6_SetDigitalMode()        do { ANSELCbits.ANSELC6 = 0; } while(0)

// get/set RC7 procedures
#define RC7_SetHigh()            do { LATCbits.LATC7 = 1; } while(0)
#define RC7_SetLow()             do { LATCbits.LATC7 = 0; } while(0)
#define RC7_Toggle()             do { LATCbits.LATC7 = ~LATCbits.LATC7; } while(0)
#define RC7_GetValue()              PORTCbits.RC7
#define RC7_SetDigitalInput()    do { TRISCbits.TRISC7 = 1; } while(0)
#define RC7_SetDigitalOutput()   do { TRISCbits.TRISC7 = 0; } while(0)
#define RC7_SetPullup()             do { WPUCbits.WPUC7 = 1; } while(0)
#define RC7_ResetPullup()           do { WPUCbits.WPUC7 = 0; } while(0)
#define RC7_SetAnalogMode()         do { ANSELCbits.ANSELC7 = 1; } while(0)
#define RC7_SetDigitalMode()        do { ANSELCbits.ANSELC7 = 0; } while(0)

// get/set K1 aliases
#define K1_TRIS                 TRISDbits.TRISD0
#define K1_LAT                  LATDbits.LATD0
#define K1_PORT                 PORTDbits.RD0
#define K1_WPU                  WPUDbits.WPUD0
#define K1_OD                   ODCONDbits.ODCD0
#define K1_ANS                  ANSELDbits.ANSELD0
#define K1_SetHigh()            do { LATDbits.LATD0 = 1; } while(0)
#define K1_SetLow()             do { LATDbits.LATD0 = 0; } while(0)
#define K1_Toggle()             do { LATDbits.LATD0 = ~LATDbits.LATD0; } while(0)
#define K1_GetValue()           PORTDbits.RD0
#define K1_SetDigitalInput()    do { TRISDbits.TRISD0 = 1; } while(0)
#define K1_SetDigitalOutput()   do { TRISDbits.TRISD0 = 0; } while(0)
#define K1_SetPullup()          do { WPUDbits.WPUD0 = 1; } while(0)
#define K1_ResetPullup()        do { WPUDbits.WPUD0 = 0; } while(0)
#define K1_SetPushPull()        do { ODCONDbits.ODCD0 = 0; } while(0)
#define K1_SetOpenDrain()       do { ODCONDbits.ODCD0 = 1; } while(0)
#define K1_SetAnalogMode()      do { ANSELDbits.ANSELD0 = 1; } while(0)
#define K1_SetDigitalMode()     do { ANSELDbits.ANSELD0 = 0; } while(0)

// get/set LED2 aliases
#define LED2_TRIS                 TRISDbits.TRISD1
#define LED2_LAT                  LATDbits.LATD1
#define LED2_PORT                 PORTDbits.RD1
#define LED2_WPU                  WPUDbits.WPUD1
#define LED2_OD                   ODCONDbits.ODCD1
#define LED2_ANS                  ANSELDbits.ANSELD1
#define LED2_SetHigh()            do { LATDbits.LATD1 = 1; } while(0)
#define LED2_SetLow()             do { LATDbits.LATD1 = 0; } while(0)
#define LED2_Toggle()             do { LATDbits.LATD1 = ~LATDbits.LATD1; } while(0)
#define LED2_GetValue()           PORTDbits.RD1
#define LED2_SetDigitalInput()    do { TRISDbits.TRISD1 = 1; } while(0)
#define LED2_SetDigitalOutput()   do { TRISDbits.TRISD1 = 0; } while(0)
#define LED2_SetPullup()          do { WPUDbits.WPUD1 = 1; } while(0)
#define LED2_ResetPullup()        do { WPUDbits.WPUD1 = 0; } while(0)
#define LED2_SetPushPull()        do { ODCONDbits.ODCD1 = 0; } while(0)
#define LED2_SetOpenDrain()       do { ODCONDbits.ODCD1 = 1; } while(0)
#define LED2_SetAnalogMode()      do { ANSELDbits.ANSELD1 = 1; } while(0)
#define LED2_SetDigitalMode()     do { ANSELDbits.ANSELD1 = 0; } while(0)

// get/set CAM_R1_O aliases
#define CAM_R1_O_TRIS                 TRISDbits.TRISD6
#define CAM_R1_O_LAT                  LATDbits.LATD6
#define CAM_R1_O_PORT                 PORTDbits.RD6
#define CAM_R1_O_WPU                  WPUDbits.WPUD6
#define CAM_R1_O_OD                   ODCONDbits.ODCD6
#define CAM_R1_O_ANS                  ANSELDbits.ANSELD6
#define CAM_R1_O_SetHigh()            do { LATDbits.LATD6 = 1; } while(0)
#define CAM_R1_O_SetLow()             do { LATDbits.LATD6 = 0; } while(0)
#define CAM_R1_O_Toggle()             do { LATDbits.LATD6 = ~LATDbits.LATD6; } while(0)
#define CAM_R1_O_GetValue()           PORTDbits.RD6
#define CAM_R1_O_SetDigitalInput()    do { TRISDbits.TRISD6 = 1; } while(0)
#define CAM_R1_O_SetDigitalOutput()   do { TRISDbits.TRISD6 = 0; } while(0)
#define CAM_R1_O_SetPullup()          do { WPUDbits.WPUD6 = 1; } while(0)
#define CAM_R1_O_ResetPullup()        do { WPUDbits.WPUD6 = 0; } while(0)
#define CAM_R1_O_SetPushPull()        do { ODCONDbits.ODCD6 = 0; } while(0)
#define CAM_R1_O_SetOpenDrain()       do { ODCONDbits.ODCD6 = 1; } while(0)
#define CAM_R1_O_SetAnalogMode()      do { ANSELDbits.ANSELD6 = 1; } while(0)
#define CAM_R1_O_SetDigitalMode()     do { ANSELDbits.ANSELD6 = 0; } while(0)

// get/set CAM_R_O aliases
#define CAM_R_O_TRIS                 TRISDbits.TRISD7
#define CAM_R_O_LAT                  LATDbits.LATD7
#define CAM_R_O_PORT                 PORTDbits.RD7
#define CAM_R_O_WPU                  WPUDbits.WPUD7
#define CAM_R_O_OD                   ODCONDbits.ODCD7
#define CAM_R_O_ANS                  ANSELDbits.ANSELD7
#define CAM_R_O_SetHigh()            do { LATDbits.LATD7 = 1; } while(0)
#define CAM_R_O_SetLow()             do { LATDbits.LATD7 = 0; } while(0)
#define CAM_R_O_Toggle()             do { LATDbits.LATD7 = ~LATDbits.LATD7; } while(0)
#define CAM_R_O_GetValue()           PORTDbits.RD7
#define CAM_R_O_SetDigitalInput()    do { TRISDbits.TRISD7 = 1; } while(0)
#define CAM_R_O_SetDigitalOutput()   do { TRISDbits.TRISD7 = 0; } while(0)
#define CAM_R_O_SetPullup()          do { WPUDbits.WPUD7 = 1; } while(0)
#define CAM_R_O_ResetPullup()        do { WPUDbits.WPUD7 = 0; } while(0)
#define CAM_R_O_SetPushPull()        do { ODCONDbits.ODCD7 = 0; } while(0)
#define CAM_R_O_SetOpenDrain()       do { ODCONDbits.ODCD7 = 1; } while(0)
#define CAM_R_O_SetAnalogMode()      do { ANSELDbits.ANSELD7 = 1; } while(0)
#define CAM_R_O_SetDigitalMode()     do { ANSELDbits.ANSELD7 = 0; } while(0)

// get/set BLED aliases
#define BLED_TRIS                 TRISEbits.TRISE0
#define BLED_LAT                  LATEbits.LATE0
#define BLED_PORT                 PORTEbits.RE0
#define BLED_WPU                  WPUEbits.WPUE0
#define BLED_OD                   ODCONEbits.ODCE0
#define BLED_ANS                  ANSELEbits.ANSELE0
#define BLED_SetHigh()            do { LATEbits.LATE0 = 1; } while(0)
#define BLED_SetLow()             do { LATEbits.LATE0 = 0; } while(0)
#define BLED_Toggle()             do { LATEbits.LATE0 = ~LATEbits.LATE0; } while(0)
#define BLED_GetValue()           PORTEbits.RE0
#define BLED_SetDigitalInput()    do { TRISEbits.TRISE0 = 1; } while(0)
#define BLED_SetDigitalOutput()   do { TRISEbits.TRISE0 = 0; } while(0)
#define BLED_SetPullup()          do { WPUEbits.WPUE0 = 1; } while(0)
#define BLED_ResetPullup()        do { WPUEbits.WPUE0 = 0; } while(0)
#define BLED_SetPushPull()        do { ODCONEbits.ODCE0 = 0; } while(0)
#define BLED_SetOpenDrain()       do { ODCONEbits.ODCE0 = 1; } while(0)
#define BLED_SetAnalogMode()      do { ANSELEbits.ANSELE0 = 1; } while(0)
#define BLED_SetDigitalMode()     do { ANSELEbits.ANSELE0 = 0; } while(0)

// get/set DEBUG1 aliases
#define DEBUG1_TRIS                 TRISEbits.TRISE1
#define DEBUG1_LAT                  LATEbits.LATE1
#define DEBUG1_PORT                 PORTEbits.RE1
#define DEBUG1_WPU                  WPUEbits.WPUE1
#define DEBUG1_OD                   ODCONEbits.ODCE1
#define DEBUG1_ANS                  ANSELEbits.ANSELE1
#define DEBUG1_SetHigh()            do { LATEbits.LATE1 = 1; } while(0)
#define DEBUG1_SetLow()             do { LATEbits.LATE1 = 0; } while(0)
#define DEBUG1_Toggle()             do { LATEbits.LATE1 = ~LATEbits.LATE1; } while(0)
#define DEBUG1_GetValue()           PORTEbits.RE1
#define DEBUG1_SetDigitalInput()    do { TRISEbits.TRISE1 = 1; } while(0)
#define DEBUG1_SetDigitalOutput()   do { TRISEbits.TRISE1 = 0; } while(0)
#define DEBUG1_SetPullup()          do { WPUEbits.WPUE1 = 1; } while(0)
#define DEBUG1_ResetPullup()        do { WPUEbits.WPUE1 = 0; } while(0)
#define DEBUG1_SetPushPull()        do { ODCONEbits.ODCE1 = 0; } while(0)
#define DEBUG1_SetOpenDrain()       do { ODCONEbits.ODCE1 = 1; } while(0)
#define DEBUG1_SetAnalogMode()      do { ANSELEbits.ANSELE1 = 1; } while(0)
#define DEBUG1_SetDigitalMode()     do { ANSELEbits.ANSELE1 = 0; } while(0)

// get/set DEBUG2 aliases
#define DEBUG2_TRIS                 TRISEbits.TRISE2
#define DEBUG2_LAT                  LATEbits.LATE2
#define DEBUG2_PORT                 PORTEbits.RE2
#define DEBUG2_WPU                  WPUEbits.WPUE2
#define DEBUG2_OD                   ODCONEbits.ODCE2
#define DEBUG2_ANS                  ANSELEbits.ANSELE2
#define DEBUG2_SetHigh()            do { LATEbits.LATE2 = 1; } while(0)
#define DEBUG2_SetLow()             do { LATEbits.LATE2 = 0; } while(0)
#define DEBUG2_Toggle()             do { LATEbits.LATE2 = ~LATEbits.LATE2; } while(0)
#define DEBUG2_GetValue()           PORTEbits.RE2
#define DEBUG2_SetDigitalInput()    do { TRISEbits.TRISE2 = 1; } while(0)
#define DEBUG2_SetDigitalOutput()   do { TRISEbits.TRISE2 = 0; } while(0)
#define DEBUG2_SetPullup()          do { WPUEbits.WPUE2 = 1; } while(0)
#define DEBUG2_ResetPullup()        do { WPUEbits.WPUE2 = 0; } while(0)
#define DEBUG2_SetPushPull()        do { ODCONEbits.ODCE2 = 0; } while(0)
#define DEBUG2_SetOpenDrain()       do { ODCONEbits.ODCE2 = 1; } while(0)
#define DEBUG2_SetAnalogMode()      do { ANSELEbits.ANSELE2 = 1; } while(0)
#define DEBUG2_SetDigitalMode()     do { ANSELEbits.ANSELE2 = 0; } while(0)

/**
   @Param
    none
   @Returns
    none
   @Description
    GPIO and peripheral I/O initialization
   @Example
    PIN_MANAGER_Initialize();
 */
void PIN_MANAGER_Initialize (void);

/**
 * @Param
    none
 * @Returns
    none
 * @Description
    Interrupt on Change Handling routine
 * @Example
    PIN_MANAGER_IOC();
 */
void PIN_MANAGER_IOC(void);



#endif // PIN_MANAGER_H
/**
 End of File
*/