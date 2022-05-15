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
        Device            :  PIC18F14Q41
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

// get/set DB0 aliases
#define DB0_TRIS                 TRISAbits.TRISA0
#define DB0_LAT                  LATAbits.LATA0
#define DB0_PORT                 PORTAbits.RA0
#define DB0_WPU                  WPUAbits.WPUA0
#define DB0_OD                   ODCONAbits.ODCA0
#define DB0_ANS                  ANSELAbits.ANSELA0
#define DB0_SetHigh()            do { LATAbits.LATA0 = 1; } while(0)
#define DB0_SetLow()             do { LATAbits.LATA0 = 0; } while(0)
#define DB0_Toggle()             do { LATAbits.LATA0 = ~LATAbits.LATA0; } while(0)
#define DB0_GetValue()           PORTAbits.RA0
#define DB0_SetDigitalInput()    do { TRISAbits.TRISA0 = 1; } while(0)
#define DB0_SetDigitalOutput()   do { TRISAbits.TRISA0 = 0; } while(0)
#define DB0_SetPullup()          do { WPUAbits.WPUA0 = 1; } while(0)
#define DB0_ResetPullup()        do { WPUAbits.WPUA0 = 0; } while(0)
#define DB0_SetPushPull()        do { ODCONAbits.ODCA0 = 0; } while(0)
#define DB0_SetOpenDrain()       do { ODCONAbits.ODCA0 = 1; } while(0)
#define DB0_SetAnalogMode()      do { ANSELAbits.ANSELA0 = 1; } while(0)
#define DB0_SetDigitalMode()     do { ANSELAbits.ANSELA0 = 0; } while(0)

// get/set DB1 aliases
#define DB1_TRIS                 TRISAbits.TRISA1
#define DB1_LAT                  LATAbits.LATA1
#define DB1_PORT                 PORTAbits.RA1
#define DB1_WPU                  WPUAbits.WPUA1
#define DB1_OD                   ODCONAbits.ODCA1
#define DB1_ANS                  ANSELAbits.ANSELA1
#define DB1_SetHigh()            do { LATAbits.LATA1 = 1; } while(0)
#define DB1_SetLow()             do { LATAbits.LATA1 = 0; } while(0)
#define DB1_Toggle()             do { LATAbits.LATA1 = ~LATAbits.LATA1; } while(0)
#define DB1_GetValue()           PORTAbits.RA1
#define DB1_SetDigitalInput()    do { TRISAbits.TRISA1 = 1; } while(0)
#define DB1_SetDigitalOutput()   do { TRISAbits.TRISA1 = 0; } while(0)
#define DB1_SetPullup()          do { WPUAbits.WPUA1 = 1; } while(0)
#define DB1_ResetPullup()        do { WPUAbits.WPUA1 = 0; } while(0)
#define DB1_SetPushPull()        do { ODCONAbits.ODCA1 = 0; } while(0)
#define DB1_SetOpenDrain()       do { ODCONAbits.ODCA1 = 1; } while(0)
#define DB1_SetAnalogMode()      do { ANSELAbits.ANSELA1 = 1; } while(0)
#define DB1_SetDigitalMode()     do { ANSELAbits.ANSELA1 = 0; } while(0)

// get/set RA2 procedures
#define RA2_SetHigh()            do { LATAbits.LATA2 = 1; } while(0)
#define RA2_SetLow()             do { LATAbits.LATA2 = 0; } while(0)
#define RA2_Toggle()             do { LATAbits.LATA2 = ~LATAbits.LATA2; } while(0)
#define RA2_GetValue()              PORTAbits.RA2
#define RA2_SetDigitalInput()    do { TRISAbits.TRISA2 = 1; } while(0)
#define RA2_SetDigitalOutput()   do { TRISAbits.TRISA2 = 0; } while(0)
#define RA2_SetPullup()             do { WPUAbits.WPUA2 = 1; } while(0)
#define RA2_ResetPullup()           do { WPUAbits.WPUA2 = 0; } while(0)
#define RA2_SetAnalogMode()         do { ANSELAbits.ANSELA2 = 1; } while(0)
#define RA2_SetDigitalMode()        do { ANSELAbits.ANSELA2 = 0; } while(0)

// get/set RA4 procedures
#define RA4_SetHigh()            do { LATAbits.LATA4 = 1; } while(0)
#define RA4_SetLow()             do { LATAbits.LATA4 = 0; } while(0)
#define RA4_Toggle()             do { LATAbits.LATA4 = ~LATAbits.LATA4; } while(0)
#define RA4_GetValue()              PORTAbits.RA4
#define RA4_SetDigitalInput()    do { TRISAbits.TRISA4 = 1; } while(0)
#define RA4_SetDigitalOutput()   do { TRISAbits.TRISA4 = 0; } while(0)
#define RA4_SetPullup()             do { WPUAbits.WPUA4 = 1; } while(0)
#define RA4_ResetPullup()           do { WPUAbits.WPUA4 = 0; } while(0)
#define RA4_SetAnalogMode()         do { ANSELAbits.ANSELA4 = 1; } while(0)
#define RA4_SetDigitalMode()        do { ANSELAbits.ANSELA4 = 0; } while(0)

// get/set RB4 procedures
#define RB4_SetHigh()            do { LATBbits.LATB4 = 1; } while(0)
#define RB4_SetLow()             do { LATBbits.LATB4 = 0; } while(0)
#define RB4_Toggle()             do { LATBbits.LATB4 = ~LATBbits.LATB4; } while(0)
#define RB4_GetValue()              PORTBbits.RB4
#define RB4_SetDigitalInput()    do { TRISBbits.TRISB4 = 1; } while(0)
#define RB4_SetDigitalOutput()   do { TRISBbits.TRISB4 = 0; } while(0)
#define RB4_SetPullup()             do { WPUBbits.WPUB4 = 1; } while(0)
#define RB4_ResetPullup()           do { WPUBbits.WPUB4 = 0; } while(0)
#define RB4_SetAnalogMode()         do { ANSELBbits.ANSELB4 = 1; } while(0)
#define RB4_SetDigitalMode()        do { ANSELBbits.ANSELB4 = 0; } while(0)

// get/set RB5 procedures
#define RB5_SetHigh()            do { LATBbits.LATB5 = 1; } while(0)
#define RB5_SetLow()             do { LATBbits.LATB5 = 0; } while(0)
#define RB5_Toggle()             do { LATBbits.LATB5 = ~LATBbits.LATB5; } while(0)
#define RB5_GetValue()              PORTBbits.RB5
#define RB5_SetDigitalInput()    do { TRISBbits.TRISB5 = 1; } while(0)
#define RB5_SetDigitalOutput()   do { TRISBbits.TRISB5 = 0; } while(0)
#define RB5_SetPullup()             do { WPUBbits.WPUB5 = 1; } while(0)
#define RB5_ResetPullup()           do { WPUBbits.WPUB5 = 0; } while(0)
#define RB5_SetAnalogMode()         do { ANSELBbits.ANSELB5 = 1; } while(0)
#define RB5_SetDigitalMode()        do { ANSELBbits.ANSELB5 = 0; } while(0)

// get/set CS aliases
#define CS_TRIS                 TRISBbits.TRISB6
#define CS_LAT                  LATBbits.LATB6
#define CS_PORT                 PORTBbits.RB6
#define CS_WPU                  WPUBbits.WPUB6
#define CS_OD                   ODCONBbits.ODCB6
#define CS_ANS                  ANSELBbits.ANSELB6
#define CS_SetHigh()            do { LATBbits.LATB6 = 1; } while(0)
#define CS_SetLow()             do { LATBbits.LATB6 = 0; } while(0)
#define CS_Toggle()             do { LATBbits.LATB6 = ~LATBbits.LATB6; } while(0)
#define CS_GetValue()           PORTBbits.RB6
#define CS_SetDigitalInput()    do { TRISBbits.TRISB6 = 1; } while(0)
#define CS_SetDigitalOutput()   do { TRISBbits.TRISB6 = 0; } while(0)
#define CS_SetPullup()          do { WPUBbits.WPUB6 = 1; } while(0)
#define CS_ResetPullup()        do { WPUBbits.WPUB6 = 0; } while(0)
#define CS_SetPushPull()        do { ODCONBbits.ODCB6 = 0; } while(0)
#define CS_SetOpenDrain()       do { ODCONBbits.ODCB6 = 1; } while(0)
#define CS_SetAnalogMode()      do { ANSELBbits.ANSELB6 = 1; } while(0)
#define CS_SetDigitalMode()     do { ANSELBbits.ANSELB6 = 0; } while(0)

// get/set IO_RB7 aliases
#define IO_RB7_TRIS                 TRISBbits.TRISB7
#define IO_RB7_LAT                  LATBbits.LATB7
#define IO_RB7_PORT                 PORTBbits.RB7
#define IO_RB7_WPU                  WPUBbits.WPUB7
#define IO_RB7_OD                   ODCONBbits.ODCB7
#define IO_RB7_ANS                  ANSELBbits.ANSELB7
#define IO_RB7_SetHigh()            do { LATBbits.LATB7 = 1; } while(0)
#define IO_RB7_SetLow()             do { LATBbits.LATB7 = 0; } while(0)
#define IO_RB7_Toggle()             do { LATBbits.LATB7 = ~LATBbits.LATB7; } while(0)
#define IO_RB7_GetValue()           PORTBbits.RB7
#define IO_RB7_SetDigitalInput()    do { TRISBbits.TRISB7 = 1; } while(0)
#define IO_RB7_SetDigitalOutput()   do { TRISBbits.TRISB7 = 0; } while(0)
#define IO_RB7_SetPullup()          do { WPUBbits.WPUB7 = 1; } while(0)
#define IO_RB7_ResetPullup()        do { WPUBbits.WPUB7 = 0; } while(0)
#define IO_RB7_SetPushPull()        do { ODCONBbits.ODCB7 = 0; } while(0)
#define IO_RB7_SetOpenDrain()       do { ODCONBbits.ODCB7 = 1; } while(0)
#define IO_RB7_SetAnalogMode()      do { ANSELBbits.ANSELB7 = 1; } while(0)
#define IO_RB7_SetDigitalMode()     do { ANSELBbits.ANSELB7 = 0; } while(0)

// get/set MIN0 aliases
#define MIN0_TRIS                 TRISCbits.TRISC0
#define MIN0_LAT                  LATCbits.LATC0
#define MIN0_PORT                 PORTCbits.RC0
#define MIN0_WPU                  WPUCbits.WPUC0
#define MIN0_OD                   ODCONCbits.ODCC0
#define MIN0_ANS                  ANSELCbits.ANSELC0
#define MIN0_SetHigh()            do { LATCbits.LATC0 = 1; } while(0)
#define MIN0_SetLow()             do { LATCbits.LATC0 = 0; } while(0)
#define MIN0_Toggle()             do { LATCbits.LATC0 = ~LATCbits.LATC0; } while(0)
#define MIN0_GetValue()           PORTCbits.RC0
#define MIN0_SetDigitalInput()    do { TRISCbits.TRISC0 = 1; } while(0)
#define MIN0_SetDigitalOutput()   do { TRISCbits.TRISC0 = 0; } while(0)
#define MIN0_SetPullup()          do { WPUCbits.WPUC0 = 1; } while(0)
#define MIN0_ResetPullup()        do { WPUCbits.WPUC0 = 0; } while(0)
#define MIN0_SetPushPull()        do { ODCONCbits.ODCC0 = 0; } while(0)
#define MIN0_SetOpenDrain()       do { ODCONCbits.ODCC0 = 1; } while(0)
#define MIN0_SetAnalogMode()      do { ANSELCbits.ANSELC0 = 1; } while(0)
#define MIN0_SetDigitalMode()     do { ANSELCbits.ANSELC0 = 0; } while(0)

// get/set RC1 procedures
#define RC1_SetHigh()            do { LATCbits.LATC1 = 1; } while(0)
#define RC1_SetLow()             do { LATCbits.LATC1 = 0; } while(0)
#define RC1_Toggle()             do { LATCbits.LATC1 = ~LATCbits.LATC1; } while(0)
#define RC1_GetValue()              PORTCbits.RC1
#define RC1_SetDigitalInput()    do { TRISCbits.TRISC1 = 1; } while(0)
#define RC1_SetDigitalOutput()   do { TRISCbits.TRISC1 = 0; } while(0)
#define RC1_SetPullup()             do { WPUCbits.WPUC1 = 1; } while(0)
#define RC1_ResetPullup()           do { WPUCbits.WPUC1 = 0; } while(0)
#define RC1_SetAnalogMode()         do { ANSELCbits.ANSELC1 = 1; } while(0)
#define RC1_SetDigitalMode()        do { ANSELCbits.ANSELC1 = 0; } while(0)

// get/set RC2 procedures
#define RC2_SetHigh()            do { LATCbits.LATC2 = 1; } while(0)
#define RC2_SetLow()             do { LATCbits.LATC2 = 0; } while(0)
#define RC2_Toggle()             do { LATCbits.LATC2 = ~LATCbits.LATC2; } while(0)
#define RC2_GetValue()              PORTCbits.RC2
#define RC2_SetDigitalInput()    do { TRISCbits.TRISC2 = 1; } while(0)
#define RC2_SetDigitalOutput()   do { TRISCbits.TRISC2 = 0; } while(0)
#define RC2_SetPullup()             do { WPUCbits.WPUC2 = 1; } while(0)
#define RC2_ResetPullup()           do { WPUCbits.WPUC2 = 0; } while(0)
#define RC2_SetAnalogMode()         do { ANSELCbits.ANSELC2 = 1; } while(0)
#define RC2_SetDigitalMode()        do { ANSELCbits.ANSELC2 = 0; } while(0)

// get/set MIN1 aliases
#define MIN1_TRIS                 TRISCbits.TRISC3
#define MIN1_LAT                  LATCbits.LATC3
#define MIN1_PORT                 PORTCbits.RC3
#define MIN1_WPU                  WPUCbits.WPUC3
#define MIN1_OD                   ODCONCbits.ODCC3
#define MIN1_ANS                  ANSELCbits.ANSELC3
#define MIN1_SetHigh()            do { LATCbits.LATC3 = 1; } while(0)
#define MIN1_SetLow()             do { LATCbits.LATC3 = 0; } while(0)
#define MIN1_Toggle()             do { LATCbits.LATC3 = ~LATCbits.LATC3; } while(0)
#define MIN1_GetValue()           PORTCbits.RC3
#define MIN1_SetDigitalInput()    do { TRISCbits.TRISC3 = 1; } while(0)
#define MIN1_SetDigitalOutput()   do { TRISCbits.TRISC3 = 0; } while(0)
#define MIN1_SetPullup()          do { WPUCbits.WPUC3 = 1; } while(0)
#define MIN1_ResetPullup()        do { WPUCbits.WPUC3 = 0; } while(0)
#define MIN1_SetPushPull()        do { ODCONCbits.ODCC3 = 0; } while(0)
#define MIN1_SetOpenDrain()       do { ODCONCbits.ODCC3 = 1; } while(0)
#define MIN1_SetAnalogMode()      do { ANSELCbits.ANSELC3 = 1; } while(0)
#define MIN1_SetDigitalMode()     do { ANSELCbits.ANSELC3 = 0; } while(0)

// get/set MLED aliases
#define MLED_TRIS                 TRISCbits.TRISC4
#define MLED_LAT                  LATCbits.LATC4
#define MLED_PORT                 PORTCbits.RC4
#define MLED_WPU                  WPUCbits.WPUC4
#define MLED_OD                   ODCONCbits.ODCC4
#define MLED_ANS                  ANSELCbits.ANSELC4
#define MLED_SetHigh()            do { LATCbits.LATC4 = 1; } while(0)
#define MLED_SetLow()             do { LATCbits.LATC4 = 0; } while(0)
#define MLED_Toggle()             do { LATCbits.LATC4 = ~LATCbits.LATC4; } while(0)
#define MLED_GetValue()           PORTCbits.RC4
#define MLED_SetDigitalInput()    do { TRISCbits.TRISC4 = 1; } while(0)
#define MLED_SetDigitalOutput()   do { TRISCbits.TRISC4 = 0; } while(0)
#define MLED_SetPullup()          do { WPUCbits.WPUC4 = 1; } while(0)
#define MLED_ResetPullup()        do { WPUCbits.WPUC4 = 0; } while(0)
#define MLED_SetPushPull()        do { ODCONCbits.ODCC4 = 0; } while(0)
#define MLED_SetOpenDrain()       do { ODCONCbits.ODCC4 = 1; } while(0)
#define MLED_SetAnalogMode()      do { ANSELCbits.ANSELC4 = 1; } while(0)
#define MLED_SetDigitalMode()     do { ANSELCbits.ANSELC4 = 0; } while(0)

// get/set RLED aliases
#define RLED_TRIS                 TRISCbits.TRISC5
#define RLED_LAT                  LATCbits.LATC5
#define RLED_PORT                 PORTCbits.RC5
#define RLED_WPU                  WPUCbits.WPUC5
#define RLED_OD                   ODCONCbits.ODCC5
#define RLED_ANS                  ANSELCbits.ANSELC5
#define RLED_SetHigh()            do { LATCbits.LATC5 = 1; } while(0)
#define RLED_SetLow()             do { LATCbits.LATC5 = 0; } while(0)
#define RLED_Toggle()             do { LATCbits.LATC5 = ~LATCbits.LATC5; } while(0)
#define RLED_GetValue()           PORTCbits.RC5
#define RLED_SetDigitalInput()    do { TRISCbits.TRISC5 = 1; } while(0)
#define RLED_SetDigitalOutput()   do { TRISCbits.TRISC5 = 0; } while(0)
#define RLED_SetPullup()          do { WPUCbits.WPUC5 = 1; } while(0)
#define RLED_ResetPullup()        do { WPUCbits.WPUC5 = 0; } while(0)
#define RLED_SetPushPull()        do { ODCONCbits.ODCC5 = 0; } while(0)
#define RLED_SetOpenDrain()       do { ODCONCbits.ODCC5 = 1; } while(0)
#define RLED_SetAnalogMode()      do { ANSELCbits.ANSELC5 = 1; } while(0)
#define RLED_SetDigitalMode()     do { ANSELCbits.ANSELC5 = 0; } while(0)

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

// get/set RELAY aliases
#define RELAY_TRIS                 TRISCbits.TRISC7
#define RELAY_LAT                  LATCbits.LATC7
#define RELAY_PORT                 PORTCbits.RC7
#define RELAY_WPU                  WPUCbits.WPUC7
#define RELAY_OD                   ODCONCbits.ODCC7
#define RELAY_ANS                  ANSELCbits.ANSELC7
#define RELAY_SetHigh()            do { LATCbits.LATC7 = 1; } while(0)
#define RELAY_SetLow()             do { LATCbits.LATC7 = 0; } while(0)
#define RELAY_Toggle()             do { LATCbits.LATC7 = ~LATCbits.LATC7; } while(0)
#define RELAY_GetValue()           PORTCbits.RC7
#define RELAY_SetDigitalInput()    do { TRISCbits.TRISC7 = 1; } while(0)
#define RELAY_SetDigitalOutput()   do { TRISCbits.TRISC7 = 0; } while(0)
#define RELAY_SetPullup()          do { WPUCbits.WPUC7 = 1; } while(0)
#define RELAY_ResetPullup()        do { WPUCbits.WPUC7 = 0; } while(0)
#define RELAY_SetPushPull()        do { ODCONCbits.ODCC7 = 0; } while(0)
#define RELAY_SetOpenDrain()       do { ODCONCbits.ODCC7 = 1; } while(0)
#define RELAY_SetAnalogMode()      do { ANSELCbits.ANSELC7 = 1; } while(0)
#define RELAY_SetDigitalMode()     do { ANSELCbits.ANSELC7 = 0; } while(0)

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




#endif // PIN_MANAGER_H
/**
 End of File
*/