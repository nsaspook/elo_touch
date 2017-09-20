
// PIC18F8722 Configuration Bit Settings

#include <p18f8722.h>

// CONFIG1H
#pragma config OSC = HSPLL      // Oscillator Selection bits (HS oscillator, PLL enabled (Clock Frequency = 4 x FOSC1))
#pragma config FCMEN = OFF      // Fail-Safe Clock Monitor Enable bit (Fail-Safe Clock Monitor disabled)
#pragma config IESO = OFF       // Internal/External Oscillator Switchover bit (Two-Speed Start-up disabled)

// CONFIG2L
#pragma config PWRT = ON       // Power-up Timer Enable bit (PWRT disabled)
#pragma config BOREN = SBORDIS  // Brown-out Reset Enable bits (Brown-out Reset enabled in hardware only (SBOREN is disabled))
#pragma config BORV = 3         // Brown-out Voltage bits (Minimum setting)

// CONFIG2H
#pragma config WDT = ON         // Watchdog Timer (WDT enabled)
#pragma config WDTPS = 128      // Watchdog Timer Postscale Select bits (1:128)

// CONFIG3L
#pragma config MODE = MC        // Processor Data Memory Mode Select bits (Microcontroller mode)
#pragma config ADDRBW = ADDR20BIT// Address Bus Width Select bits (20-bit Address Bus)
#pragma config DATABW = DATA16BIT// Data Bus Width Select bit (16-bit External Bus mode)
#pragma config WAIT = OFF       // External Bus Data Wait Enable bit (Wait selections are unavailable for table reads and table writes)

// CONFIG3H
#pragma config CCP2MX = PORTC   // CCP2 MUX bit (ECCP2 input/output is multiplexed with RC1)
#pragma config ECCPMX = PORTE   // ECCP MUX bit (ECCP1/3 (P1B/P1C/P3B/P3C) are multiplexed onto RE6, RE5, RE4 and RE3 respectively)
#pragma config LPT1OSC = OFF    // Low-Power Timer1 Oscillator Enable bit (Timer1 configured for higher power operation)
#pragma config MCLRE = ON       // MCLR Pin Enable bit (MCLR pin enabled; RG5 input pin disabled)

// CONFIG4L
#pragma config STVREN = ON      // Stack Full/Underflow Reset Enable bit (Stack full/underflow will cause Reset)
#pragma config LVP = OFF         // Single-Supply ICSP Enable bit (Single-Supply ICSP enabled)
#pragma config BBSIZ = BB2K     // Boot Block Size Select bits (1K word (2 Kbytes) Boot Block size)
#pragma config XINST = ON   // Extended Instruction Set Enable bit (Instruction set extension and Indexed Addressing mode disabled (Legacy mode))

// CONFIG5L
#pragma config CP0 = OFF        // Code Protection bit Block 0 (Block 0 (000800, 001000 or 002000-003FFFh) not code-protected)
#pragma config CP1 = OFF        // Code Protection bit Block 1 (Block 1 (004000-007FFFh) not code-protected)
#pragma config CP2 = OFF        // Code Protection bit Block 2 (Block 2 (008000-00BFFFh) not code-protected)
#pragma config CP3 = OFF        // Code Protection bit Block 3 (Block 3 (00C000-00FFFFh) not code-protected)
#pragma config CP4 = OFF        // Code Protection bit Block 4 (Block 4 (010000-013FFFh) not code-protected)
#pragma config CP5 = OFF        // Code Protection bit Block 5 (Block 5 (014000-017FFFh) not code-protected)
#pragma config CP6 = OFF        // Code Protection bit Block 6 (Block 6 (01BFFF-018000h) not code-protected)
#pragma config CP7 = OFF        // Code Protection bit Block 7 (Block 7 (01C000-01FFFFh) not code-protected)

// CONFIG5H
#pragma config CPB = OFF        // Boot Block Code Protection bit (Boot Block (000000-0007FFh) not code-protected)
#pragma config CPD = OFF        // Data EEPROM Code Protection bit (Data EEPROM not code-protected)

// CONFIG6L
#pragma config WRT0 = OFF       // Write Protection bit Block 0 (Block 0 (000800, 001000 or 002000-003FFFh) not write-protected)
#pragma config WRT1 = OFF       // Write Protection bit Block 1 (Block 1 (004000-007FFFh) not write-protected)
#pragma config WRT2 = OFF       // Write Protection bit Block 2 (Block 2 (008000-00BFFFh) not write-protected)
#pragma config WRT3 = OFF       // Write Protection bit Block 3 (Block 3 (00C000-00FFFFh) not write-protected)
#pragma config WRT4 = OFF       // Write Protection bit Block 4 (Block 4 (010000-013FFFh) not write-protected)
#pragma config WRT5 = OFF       // Write Protection bit Block 5 (Block 5 (014000-017FFFh) not write-protected)
#pragma config WRT6 = OFF       // Write Protection bit Block 6 (Block 6 (01BFFF-018000h) not write-protected)
#pragma config WRT7 = OFF       // Write Protection bit Block 7 (Block 7 (01C000-01FFFFh) not write-protected)

// CONFIG6H
#pragma config WRTC = OFF       // Configuration Register Write Protection bit (Configuration registers (300000-3000FFh) not write-protected)
#pragma config WRTB = OFF       // Boot Block Write Protection bit (Boot Block (000000-007FFF, 000FFF or 001FFFh) not write-protected)
#pragma config WRTD = OFF       // Data EEPROM Write Protection bit (Data EEPROM not write-protected)

// CONFIG7L
#pragma config EBTR0 = OFF      // Table Read Protection bit Block 0 (Block 0 (000800, 001000 or 002000-003FFFh) not protected from table reads executed in other blocks)
#pragma config EBTR1 = OFF      // Table Read Protection bit Block 1 (Block 1 (004000-007FFFh) not protected from table reads executed in other blocks)
#pragma config EBTR2 = OFF      // Table Read Protection bit Block 2 (Block 2 (008000-00BFFFh) not protected from table reads executed in other blocks)
#pragma config EBTR3 = OFF      // Table Read Protection bit Block 3 (Block 3 (00C000-00FFFFh) not protected from table reads executed in other blocks)
#pragma config EBTR4 = OFF      // Table Read Protection bit Block 4 (Block 4 (010000-013FFFh) not protected from table reads executed in other blocks)
#pragma config EBTR5 = OFF      // Table Read Protection bit Block 5 (Block 5 (014000-017FFFh) not protected from table reads executed in other blocks)
#pragma config EBTR6 = OFF      // Table Read Protection bit Block 6 (Block 6 (018000-01BFFFh) not protected from table reads executed in other blocks)
#pragma config EBTR7 = OFF      // Table Read Protection bit Block 7 (Block 7 (01C000-01FFFFh) not protected from table reads executed in other blocks)

// CONFIG7H
#pragma config EBTRB = OFF      // Boot Block Table Read Protection bit (Boot Block (000000-007FFF, 000FFF or 001FFFh) not protected from table reads executed in other blocks)


/*
 Viision terminal code */
/*
 * This program converts the rs-232 output from a ELO controller type LCD monitor 
 * to a format that can be used with the Varian Viision 80 Implanter with ADYIN CRT monitor
 * The LCD touchscreen will be  programmed to the correct touch response and configuration at program start
 *
 * USART1 is the host comm port
 * USART2 is the touch-screen comm port
 *
 * Fred Brooks, Microchip Inc , Oct 2016
 * Gresham, Oregon
 *
 *
 * This application is designed for use with the
 * ET-BASE PIC8722 board and  device
 *
 * usart1	connected to implant host computer at 9600
 * usart2       connected to touchscreen  serial post at 9600
 *
 *
 * V1.00	First release
 * V1.01	pre compute X/Y scale values
 * V1.02	Clean up port setup code and WDT (remove clrwdt from isr)
 * V1.03        New touchscreen support.
 * V1.04	xmit rs232 uses int mode in ISR
 * V1.05	Configuration for touch code repeats sending to host
 * V1.06	Set touch repeats to zero
 * V1.07        Set Z touch value to max 0x0F
 * V1.08	Set Z to 1 and remove multi-touch testing
 * V2.00        Code rewrite
 * V3.00	add code for smartset again
 * V3.01	mode checks and lamps for proper smartset to emulation operation
 * V3.10	working Intellitouch version
 * V3.20	code cleanup
 * V3.30	unified Viision/E220/E500 driver, see vtouch_build.h
 *
 *
 *
 * HOST RS-232  5-1     uC port1
 * Female       2-2-tx
 *              3-3-rx
 * LCD  RS-232  5-1     uC port2
 * Male         2-3-rx
 *              3-2-tx
 */

/* E220/E500 terminal code
 /*
 * This program converts the rs-232 output from a ELO touch-screen controller
 * to a format that can be used with the Varian E220/E500 Implanter
 * The touch controller must be first programmed
 * USART1 is the host comm port
 * USART2 is the touch-screen comm port
 *
 * PRORTA, PORTE Camera, aux switching with touchs in target box
 * PORTJ		LED bar display
 * PORTH0		run flasher led onboard.
 * 8 led status lights.
 *
 * Fred Brooks, Microchip Inc , Aug 2009,2016
 * Gresham, Oregon
 *
 *
 * This application is designed for use with the
 * ET-BASE PIC8722 board and  device.
 * HOST RS-232  5-1     uC port1
 * Female       2-2-tx
 *              3-3-rx
 * LCD  RS-232  5-1     uC port2
 * Male         2-3-rx
 *              3-2-tx
 * 
 * VGA converter box relay
 * Omron 
 * G6k-2P bottom view
 * Pin		8 - gnd, wire tag 0, to RELAY output	pin 2 on connector for RA1, RE1 PORT OUTPUT
 * Pin		1 + 5vdc,		Power PIN	pin 9 connector for RA or RE PORT VCC		 
 */


#include <usart.h>
#include <delays.h>
#include <timers.h>
#include <stdlib.h>
#include <EEP.h>
#include <GenericTypeDefs.h>
#include "vtouch.h"
#include "vtouch_build.h"

void rxtx_handler(void);

typedef struct reporttype {
	uint8_t headder, status;
	uint16_t x_cord, y_cord, z_cord;
	uint8_t checksum;
	uint8_t tohost;
} volatile reporttype;

typedef struct statustype {
	int32_t alive_led, touch_count, resync_count, rawint_count, status_count;
	uint8_t host_write : 1;
	uint8_t scrn_write : 1;
	uint8_t do_cap : 1;
	uint8_t comm_check, init_check, touch_good, cam_time;
	uint16_t restart_delay;
} volatile statustype;

volatile uint16_t c_idx = 0, speedup = 0;

typedef struct flag_var_t {
	uint8_t CATCH : 1, TOUCH : 1;
} volatile F;

volatile uint8_t CATCH = FALSE, TOUCH = FALSE, UNTOUCH = FALSE, LCD_OK = FALSE,
	SCREEN_INIT = FALSE,
	CATCH46 = FALSE, CATCH37 = FALSE, TSTATUS = FALSE,
	DATA1 = FALSE, DATA2 = FALSE, CAM = FALSE;

/*
 * Old monitors
 * E757389	CarrollTouch use DELL_E224864 setting
 * E224864	CarrollTouch use DELL_E224864 setting
 * E779866	SecureTouch  use DELL_E215546 setting
 * E215546	IntelliTouch use DELL_E215546 setting
 * NEW REPLACEMENT MONITORS 1990L 1991L
 * E328700	IntelliTouch use DELL_E215546 setting
 * E328497	IntelliTouch use DELL_E215546 setting
 * E483757	new remote OSD
 * E005277	power brick
 */
enum screen_type_t {
	DELL_E224864, DELL_E215546, OTHER_SCREEN
};

enum emulat_type_t {
	VIISION, E220, OTHER_MECH
};

volatile enum screen_type_t screen_type;
volatile enum emulat_type_t emulat_type;

volatile int32_t j = 0;
volatile float xs = X_SCALE, ys = Y_SCALE, xs_ss = X_SCALE_SS, ys_ss = Y_SCALE_SS; // defaults
volatile uint16_t timer0_off = TIMEROFFSET;

#pragma idata bigdata
volatile uint8_t elobuf[BUF_SIZE], elobuf_out[BUF_SIZE_V80], elobuf_in[BUF_SIZE_V80], xl = X_LOGICAL, yl = Y_LOGICAL;
volatile uint8_t ssbuf[BUF_SIZE];

volatile struct reporttype ssreport;
volatile struct statustype status;

const rom uint8_t elocodes_s_v[] = {
	0x0d, 0x0d, 0x0d, 0x0d, 0x0d, 0x3c, 0x2b, 0x44, 0x25, 0x29, 0x44, 0x3d, 0x2a, 0x37
}; // initial carrol-touch config codes, tracking, add end point modifier, get frame size report

const rom uint8_t elocodes[ELO_SEQ_V80][ELO_SIZE_I_V80] = {// elo 2210/2216 program codes
	'U', 'M', 0x00, 0x87, 0x40, '0', '0', '0', '0', '0', // initial touch,stream Point,untouch,Z-axis,no scaling, tracking
	'U', 'S', 'X', 0x00, 0x0ff, 0x00, 0x01, '0', '0', '0', // scale x: X,Y,Z scaling Not Used
	'U', 'S', 'Y', 0x00, 0x0ff, 0x00, 0x01, '0', '0', '0', // scale y
	'U', 'S', 'Z', 0x00, 0x01, 0x00, 0x0f, '0', '0', '0', // scale z
	'U', 'B', 5, 20, 0x00, 0x00, 0x0f, '0', '0', '0', // packet delays to match old terminal
	'U', 'E', '1', '6', '0', '0', '0', '0', '0', '0', // emulation E281A-4002 Binary (Z=1-255 on touch, Z=0 on untouch)
	'U', 'N', '1', '7', '0', '0', '0', '0', '0', '0', // nvram save
	'U', 'R', '2', '0', '0', '0', '0', '0', '0', '0', // nvram reset
}; // initial intelli-touch codes

uint8_t elocodes_m_e[] = {// 5 char, soft-reset,touch scanning off, report transfer on, (0x26) tracking mode, report transfer on, clear touch buffer, touch scanning on
	0x0d, 0x0d, 0x0d, 0x0d, 0x0d, 0x3c, 0x2b, 0x44, 0x26, 0x44, 0x3d, 0x2a
}; // initial touch config codes, tracking
uint8_t elocodes_s_e[] = {// same as above ex (0x25) enter point mode
	0x0d, 0x0d, 0x0d, 0x0d, 0x0d, 0x3c, 0x2b, 0x44, 0x25, 0x44, 0x3d, 0x2a
}; // initial touch config codes, single


// SmartSet codes 0 command, 1 status, 2 low byte, 3 high byte, etc ...
uint8_t elocodes_e0[] = {
	'U', 'B', 0x01, 0x4, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 // smartset timing and spacing setup
};
uint8_t elocodes_e1[] = {
	'U', 'E', '1', '4', 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};
uint8_t elocodes_e2[] = {
	'U', 'N', '1', '7', 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};
uint8_t elocodes_e3[] = {
	'U', 'R', '2', 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};
uint8_t elocodes_e4[] = {
	'U', 'S', 'Y', 0x01, 0x00, 0xff, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};
uint8_t elocodes_e5[] = {
	'U', 'i', 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};
uint8_t elocodes_e6[] = {
	'U', 'g', 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};
uint8_t elocodes_e7[] = {// dummy packet
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

uint16_t touch_corner1 = 0, touch_corner_timed = 0;
#pragma idata

#pragma idata sddata
volatile uint8_t host_rec[CAP_SIZE] = "H";
volatile uint8_t scrn_rec[CAP_SIZE] = "S";
#pragma idata 

#pragma code touch_int = 0x8

void touch_int(void)
{
	_asm goto rxtx_handler _endasm
}
#pragma code

#pragma interrupt rxtx_handler

void rxtx_handler(void) // all timer & serial data transform functions are handled here
{
	static union Timers timer0;
	static uint8_t junk = 0, c = 0, *data_ptr,
		i = 0, data_pos, data_len, tchar, uchar;
	uint16_t x_tmp, y_tmp, uvalx, lvalx, uvaly, lvaly;
	static uint16_t scrn_ptr = 0, host_ptr = 0;
	static uint8_t sum = 0xAA + 'U', idx = 0;

	status.rawint_count++;

	if (INTCONbits.RBIF) {
		junk = PORTB;
		INTCONbits.RBIF = 0;
	}

	/* start with data_ptr pointed to address of data, data_len to length of data in bytes, data_pos to 0 to start at the beginning of data block */
	/* then enable the interrupt and wait for the interrupt enable flag to clear
	/* send buffer and count xmit data bytes for host link */
	if (PIE1bits.TX1IE && PIR1bits.TX1IF) { // send data to host USART
		if (data_pos >= data_len) { // buffer has been sent
			if (TXSTA1bits.TRMT) { // last bit has been shifted out
				PIE1bits.TX1IE = 0; // stop data xmit
			}
		} else {
			LATHbits.LATH0 = !LATHbits.LATH0; // flash onboard led
			LATEbits.LATE0 = !LATEbits.LATE0; // flash external led
			LATEbits.LATE7 = LATEbits.LATE0; // flash external led
			TXREG1 = *data_ptr; // send data and clear PIR1bits.TX1IF
			data_pos++; // move the data pointer
			data_ptr++; // move the buffer pointer position
		}
	}

	if (PIR1bits.RCIF) { // is data from host COMM1, only from E220/E500 machines
		if (RCSTA1bits.OERR) {
			RCSTA1bits.CREN = 0; //	clear overrun
			RCSTA1bits.CREN = 1; // re-enable
		}
		if (status.do_cap) {
			if (host_ptr < CAP_SIZE) {
				host_rec[host_ptr] = RCREG1; // read data from touchscreen
				while (!TXSTA2bits.TRMT) {
				}; // wait until the usart is clear
				TXREG2 = host_rec[host_ptr];
				host_ptr++;
				LATFbits.LATF1 = !LATFbits.LATF1; // flash  led
			} else {
				tchar = RCREG1; // read from host
				status.host_write = TRUE;
				LATFbits.LATF3 = !LATFbits.LATF3; // flash  led
			}
		} else {
			tchar = RCREG1; // read from host
			DATA1 = TRUE; // usart is connected to data
			if ((tchar == (uint8_t) 0x46)) { // send one report to host
				CATCH46 = TRUE;
				status.touch_good = 0;
			}
			if ((tchar == (uint8_t) 0x37)) { // start of touch scan read
				CATCH37 = TRUE;
				status.touch_good = 0;
			}
			if ((tchar == (uint8_t) 0x3C)) { // touch reset from host
				// a possible setup command
			}
		};
	}

	if (INTCONbits.TMR0IF) { // check timer0 irq 1 second timer
		//check for TMR0 overflow
		idx = 0; // reset packet char index counter
		ssreport.tohost = FALSE; // when packets stop allow for next updates
		INTCONbits.TMR0IF = 0; //clear interrupt flag
		WriteTimer0(timer0_off);

		if (LCD_OK)
			LATF = 0xff;

		if (!LCD_OK && (status.init_check++ >LCD_CHK_TIME)) {
			status.init_check = 0; // reset screen init code counter
			SCREEN_INIT = TRUE; // set init code flag so it can be sent in main loop
			LATEbits.LATE3 = 0; // init  led ON
		}

		if ((status.comm_check++ >COMM_CHK_TIME) && !CATCH) { // check for LCD screen connection
			status.comm_check = 0; // reset connect heartbeat counter
			LCD_OK = FALSE; // reset the connect flag while waiting for response from controller.
			while (!TXSTA2bits.TRMT) {
			}; // wait until the usart is clear
			if (screen_type == DELL_E224864) {
				TXREG2 = 0x37; // send frame size request to LCD touch
				LATF = 0xff;
			}
			if (screen_type == DELL_E215546) {
				LATF = 0xff;
			}

		}
	}

	if (emulat_type == E220) {
		if (PIR3bits.RC2IF) { // is data from touchscreen
			timer0.lt = TIMERPACKET; // set timer to charge rate time
			TMR0H = timer0.bt[1]; // Write high byte to Timer0
			TMR0L = timer0.bt[0]; // Write low byte to Timer0
			// clear  TMR0  flag
			INTCONbits.TMR0IF = 0; //clear interrupt flag
			if (CAM && (status.cam_time > MAX_CAM_TIME)) {
				CAM_RELAY_TIME = 0;
				CAM_RELAY_AUX = 0; // clear video switch
				CAM_RELAY = 0; // clear video switch
				CAM = FALSE;
			}

			c = RCREG2; // read data from touchscreen
			if (status.do_cap) {
				if (scrn_ptr < CAP_SIZE) {
					scrn_rec[scrn_ptr] = RCREG2; // read data from touchscreen
					while (!TXSTA1bits.TRMT) {
					}; // wait until the usart is clear
					TXREG1 = scrn_rec[scrn_ptr];
					scrn_ptr++;
					LATFbits.LATF2 = !LATFbits.LATF2; // flash  led
				} else {
					status.scrn_write = TRUE;
					LATFbits.LATF4 = !LATFbits.LATF4; // flash  led
				}
			} else {
				if (screen_type == DELL_E215546) { // IntelliTouch
					LATFbits.LATF0 = 0;
					ssbuf[idx] = c;
					switch (idx++) {
					case 0: // start of touch controller packet, save data and compute checksum
						sum = 0xaa;
						if (c != 'U') {
							idx = 0;
							LATFbits.LATF1 = 0;
						}
						break;
					case 9: // end of touch controller packet
						LATFbits.LATF0 = 1;
						idx = 0;
						if (c != sum) { // bad checksum
							LATF = 0xff;
							break;
						}
						if (ssbuf[1] == 'T') {
							LATFbits.LATF6 = 0;
							status.restart_delay = 0;
							CATCH = TRUE;
							if (!ssreport.tohost) {
								ssreport.x_cord = (ELO_REV_H - (((uint16_t) ssbuf[3])+(((uint16_t) ssbuf[4]) << 8))) >> 4;
								ssreport.y_cord = (((uint16_t) ssbuf[5])+(((uint16_t) ssbuf[6]) << 8)) >> 4;
							}
						} else if (ssbuf[1] == 'A') {
							status.restart_delay = 0;
							LATEbits.LATE2 = 0; // connect  led ON
							speedup = -10000;
						}
						break;
					}
					sum += c;
					DATA2 = TRUE; // usart is connected to data

				}
				if (screen_type == DELL_E224864) { // CarrollTouch
					LATFbits.LATF0 = 0;
					status.touch_good++; // chars received before a status report
					DATA2 = TRUE; // USART is connected to data
					if (TOUCH) {
						elobuf[c_idx++] = c;
						if (c == 0xFF && TOUCH) { // end of report
							LATFbits.LATF1 = 0;
							CATCH = TRUE;
							status.restart_delay = 0;
							TOUCH = FALSE; // stop buffering touchscreen data.
						};
					};
					if (c == 0xFE && (!CATCH)) { // looks like a touch report
						LATFbits.LATF6 = 0;
						TOUCH = TRUE;
						TSTATUS = TRUE;
						status.restart_delay = 0;
						CATCH = FALSE;
						c_idx = 0;
					};
					if (c == 0xF5) { // looks like a status report
						LATFbits.LATF3 = 0;
						TSTATUS = TRUE;
						status.restart_delay = 0;
						LATFbits.LATF6 = 0; // led 6 touch-screen connected
						speedup = -10000;
					};
					if (c_idx > (BUF_SIZE - 2)) {
						c_idx = 0; // stop buffer-overflow
						TOUCH = FALSE;
						CATCH = FALSE;
					};
					if (status.touch_good > GOOD_MAX) { // check for max count and no host to get touch data
						while (TRUE) { // lockup for reboot
							status.touch_good++;
						};
					}
				}
			}
		}
	}

	if (emulat_type == VIISION) {
		if (screen_type == DELL_E215546) { // This is for the newer SMARTSET intellitouch screens
			if (PIR3bits.RC2IF) { // is data from screen COMM2
				if (RCSTA2bits.OERR) {
					RCSTA2bits.CREN = 0; //	clear overrun
					RCSTA2bits.CREN = 1; // re-enable
				}

				/* Get the character received from the USART */
				c = RCREG2;

				if (((c & 0xc0) == 0xc0) || CATCH) { // start of touch sequence
					LATFbits.LATF0 = 0;
					CATCH = TRUE; // found elo touch command start of sequence
					j = 0; // reset led timer
					elobuf[i++] = c; // start stuffing the command buffer
				}
				if (i == CMD_SIZE_SS_V80) { // see if we should send it
					LATFbits.LATF5 = !LATFbits.LATF5;
					i = FALSE; // reset i to start of cmd
					uchar = 0; /* check for proper touch format */
					if ((elobuf[0]& 0xc0) == 0xc0) /* binary start code? */
						uchar = TRUE;

					LATFbits.LATF1 = 0;
					CATCH = FALSE; // reset buffering now

					/* munge the data for proper Varian format */
					if (elobuf[5]) { // TOUCH
						TOUCH = TRUE; // first touch sequence has been sent
						uvalx = elobuf[0]&0x3f; // prune the data to 6-bits
						lvalx = elobuf[1]&0x3f;
						uvaly = elobuf[2]&0x3f;
						lvaly = elobuf[3]&0x3f;
						x_tmp = lvalx | (uvalx << 6); // 12-bit X value
						y_tmp = lvaly | (uvaly << 6); // 12-bit Y value
						x_tmp = 4095 - x_tmp; // FLIP X
						y_tmp = 4095 - y_tmp; // FLIP Y	
						x_tmp = (uint16_t) ((float) x_tmp * (float) xs_ss); // X rescale range
						y_tmp = (uint16_t) ((float) y_tmp * (float) ys_ss); // Y rescale
						x_tmp = (x_tmp >> (uint16_t) 4); // rescale x to 8-bit value
						y_tmp = (y_tmp >> (uint16_t) 4); // rescale y				
						elobuf_in[1] = x_tmp; // X to 8-bit var
						elobuf_in[2] = y_tmp; // Y 
						elobuf_out[0] = 0xc0 + ((elobuf_in[1]&0xc0) >> 6); // stuff into binary 4002 format
						elobuf_out[1] = 0x80 + (elobuf_in[1]&0x3f);
						elobuf_out[2] = 0x40 + ((elobuf_in[2]&0xc0) >> 6);
						elobuf_out[3] = 0x00 + (elobuf_in[2]&0x3f);
						elobuf_out[4] = 0x00;
						elobuf_out[5] = 0x0f;
						LATFbits.LATF6 = 0;
					}

					if (!elobuf[5]) { //UNTOUCH
						UNTOUCH = TRUE; // untouch seqence found
						elobuf_out[0] = 0xc0; // restuff the buffer with needed varian untouch sequence
						elobuf_out[1] = 0x80;
						elobuf_out[2] = 0x40;
						elobuf_out[3] = 0x00;
						elobuf_out[4] = 0x00;
						elobuf_out[5] = 0x00;
						LATFbits.LATF7 = 0;
					}

					if (TOUCH || UNTOUCH) { // send both
						LATFbits.LATF2 = 0;
						if (uchar) { /* only send valid data */
							LATFbits.LATF3 = 0;
							data_ptr = elobuf_out;
							data_pos = 0;
							data_len = HOST_CMD_SIZE_V80;
							PIE1bits.TX1IE = 1; // start sending data
						}
						LCD_OK = TRUE; // looks like a screen controller is connected
						SCREEN_INIT = FALSE; // command code has been received by lcd controller
						LATEbits.LATE3 = 1; // init  led OFF
						status.init_check = 0; // reset init code timer
						LATEbits.LATE2 = 0; // connect  led ON

						if (UNTOUCH) { // After untouch is sent dump buffer and clear all.
							TOUCH = FALSE;
							UNTOUCH = FALSE;
							CATCH = FALSE;
							LATFbits.LATF4 = 0;
							WriteTimer0(timer0_off);
						}
					}
				}

				/* Clear the interrupt flag */
				if (i > CMD_OVERFLOW_V80) {
					i = 0; // just incase i is greater than CMD_SIZE*2 somehow
					CATCH = FALSE;
					TOUCH = FALSE;
					UNTOUCH = FALSE;
					LATF = 0xff;
				}

				LATEbits.LATE0 = !LATEbits.LATE0; // flash external led
				LATEbits.LATE6 = !LATEbits.LATE6; // flash external led
				LATEbits.LATE7 = LATEbits.LATE0; // flash external led
			}
		}

		if (screen_type == DELL_E224864) { // This is for the DELL ELO Carroltouch screen.
			if (PIR3bits.RC2IF) { // is data from touchscreen COMM2
				if (RCSTA2bits.OERR) {
					RCSTA2bits.CREN = 0; //	clear overrun
					RCSTA2bits.CREN = 1; // re-enable
				}
				/* Get the character received from the USART */
				c = RCREG2;

				LATEbits.LATE0 = !LATEbits.LATE0; // flash external led
				LATEbits.LATE5 = !LATEbits.LATE5; // flash external led
				LATEbits.LATE7 = LATEbits.LATE0; // flash external led

				// touch 'FE X Y FF',    untouch 'FD X Y FF' from screen,    'F4 X Y FF' frame size report

				if (CATCH || (c == 0xFE) || (c == 0xFD) || (c == 0xF4)) { // in frame or start of touch or untouch sequence or frame size report
					LATFbits.LATF0 = 0;
					CATCH = TRUE; // found elo CT touch command start of sequence, we hope
					elobuf_in[i++] = c; // start stuffing the command buffer
					j = 0; // reset led timer
				}

				if ((i == CMD_SIZE_V80) && (elobuf_in[3] == 0xFF)) { // see if we should send it, right size and end char
					LATFbits.LATF5 = !LATFbits.LATF5;
					i = 0; // reset i to start of cmd frame
					CATCH = FALSE; // reset buffering now
					uchar = elobuf_in[i]; //  load into uchar

					if (uchar == 0xFE) { // touch sequence found restuff the buffer with varian touch sequence
						TOUCH = TRUE; // set TOUCH flag after first touch
						elobuf_in[2] = yl - elobuf_in[2]; // FLIP Y
						elobuf_in[1] = (uint8_t) ((float) elobuf_in[1]* (float) xs); // X scale
						elobuf_in[2] = (uint8_t) ((float) elobuf_in[2]* (float) ys); // Y scale
						elobuf_out[i ] = 0xc0 + ((elobuf_in[1]&0xc0) >> 6); // stuff into binary 4002 format
						elobuf_out[i + 1] = 0x80 + (elobuf_in[1]&0x3f);
						elobuf_out[i + 2] = 0x40 + ((elobuf_in[2]&0xc0) >> 6);
						elobuf_out[i + 3] = 0x00 + (elobuf_in[2]&0x3f);
						elobuf_out[i + 4] = 0x00;
						elobuf_out[i + 5] = 0x15; // Z value = 15 "hard touch"
						LATFbits.LATF6 = 0;
					}

					if (uchar == 0xFD) { // this is a untouch command
						UNTOUCH = TRUE; // untouch sequence found
						elobuf_out[i] = 0xc0; // restuff the buffer with varian binary 4002 untouch sequence
						elobuf_out[i + 1] = 0x80;
						elobuf_out[i + 2] = 0x40;
						elobuf_out[i + 3] = 0x00;
						elobuf_out[i + 4] = 0x00;
						elobuf_out[i + 5] = 0x00;
						LATFbits.LATF7 = 0;
					}

					if (!TOUCH && !UNTOUCH) { // check for proper touch frames
						if (uchar == 0xF4) { // check for frame size report
							LCD_OK = TRUE; // looks like a screen controller is connected
							SCREEN_INIT = FALSE; // command code has been received by lcd controller
							LATEbits.LATE3 = 1; // init  led OFF
							status.init_check = 0; // reset init code timer
							LATEbits.LATE2 = 0; // connect  led ON
						}
					}

					if (TOUCH || UNTOUCH) { // send both
						LATFbits.LATF2 = 0;
						data_ptr = elobuf_out;
						data_pos = 0;
						data_len = HOST_CMD_SIZE_V80;
						PIE1bits.TX1IE = 1; // start sending data
					}

					if (UNTOUCH) { // cleanup and reset for next touch
						TOUCH = FALSE;
						UNTOUCH = FALSE;
						CATCH = FALSE;
						LATFbits.LATF4 = 0;
						WriteTimer0(timer0_off);
					}
				}

				if (i > CMD_OVERFLOW_V80) {
					i = 0; // just incase i is greater than overflow somehow
					CATCH = FALSE;
					TOUCH = FALSE;
					UNTOUCH = FALSE;
					LATF = 0xff;
				}

				LATEbits.LATE0 = !LATEbits.LATE0;
				LATEbits.LATE6 = !LATEbits.LATE6;
				LATEbits.LATE7 = LATEbits.LATE0;
			}
		}
	}

	if (emulat_type == OTHER_SCREEN) {
		if (PIR3bits.RC2IF) { // is data from touchscreen COMM2
			if (RCSTA2bits.OERR) {
				RCSTA2bits.CREN = 0; //	clear overrun
				RCSTA2bits.CREN = 1; // re-enable
			}
			/* Get the character received from the USART */
			c = RCREG2;
			LATEbits.LATE0 = !LATEbits.LATE0;
			LATEbits.LATE5 = !LATEbits.LATE5;
			LATEbits.LATE7 = LATEbits.LATE0;
		}
	}
}

void touch_cam(void)
{
	//	check for corner presses
	if (CATCH) {
		if ((elobuf[0] <= (uint8_t) 0x06) && (elobuf[1] >= (uint8_t) 0x5a)) { // check for left bottom corner
			touch_corner1++;
			touch_corner_timed = TRUE;
		};

		if ((elobuf[0] >= (uint8_t) 0x72) && (elobuf[1] >= (uint8_t) 0x5a)) { // check for right bottom corner
			touch_corner1++;
		};
	};

	if (touch_corner1 >= MAX_CAM_TOUCH) { // we have several corner presses 
		CAM = TRUE;
		status.cam_time = 0;
		CAM_RELAY_TIME = 1;
		touch_corner1 = 0;
		CAM_RELAY_AUX = 1; // set secondary VGA/CAM switch
		CAM_RELAY = 1; // set primary VGA/CAM switch
		elobuf[0] = 0;
		elobuf[1] = 0;
	};
}

void wdtdelay(uint32_t delay)
{
	uint32_t dcount;
	for (dcount = 0; dcount <= delay; dcount++) { // delay a bit
		ClrWdt(); // reset the WDT timer
	};
}

void elocmdout(uint8_t * elostr)
{
	LATFbits.LATF5 = !LATFbits.LATF5; // touch screen commands led
	while (Busy2USART()) {
	}; // wait until the usart is clear
	putc2USART(elostr[0]);
	while (Busy2USART()) {
	}; // wait until the usart is clear
	LATHbits.LATH0 = !LATHbits.LATH0; // flash onboard led
	LATEbits.LATE0 = !LATEbits.LATE0; // flash external led
	LATEbits.LATE7 = LATEbits.LATE0; // flash external led
	wdtdelay(30000);
}

void eloSScmdout(uint8_t elostr)
{
	LATFbits.LATF5 = !LATFbits.LATF5; // touch screen commands led
	while (Busy2USART()) {
	}; // wait until the usart is clear
	putc2USART(elostr);
	while (Busy2USART()) {
	}; // wait until the usart is clear
	LATHbits.LATH0 = !LATHbits.LATH0; // flash onboard led
	LATEbits.LATE0 = !LATEbits.LATE0; // flash external led
	LATEbits.LATE7 = LATEbits.LATE0; // flash external led
	wdtdelay(10000); // inter char delay
}

void elopacketout(uint8_t *strptr, uint8_t strcount, uint8_t slow)
{
	uint8_t i, c, sum = 0;

	for (i = 0; i < strcount; i++) {
		switch (i) {
		case 0:
			c = 'U';
			sum = 0xAA + 'U';
			break;
		case 9:
			c = sum;
			break;
		default:
			sum += (c = strptr[i]);
		}
		eloSScmdout(c);
	};
	if (slow)
		wdtdelay(30000);
}

void elocmdout_v80(const rom uint8_t * elostr)
{
	int16_t e;
	int8_t elo_char;
	for (e = 0; e < ELO_SIZE_V80; e++) { // send buffered data
		while (Busy2USART()) {
		}; // wait until the usart is clear
		elo_char = elostr[e];
		putc2USART(elo_char); // send to LCD touch
		LATHbits.LATH0 = !LATHbits.LATH0; // flash onboard led
		LATEbits.LATE0 = !LATEbits.LATE0; // flash external led
		LATEbits.LATE7 = LATEbits.LATE0; // flash external led
		wdtdelay(10000); // inter char delay
	}
	wdtdelay(50000); // wait for LCD controller reset
}

void setup_lcd(void)
{
	uint16_t code_count;
	uint8_t single_t = SINGLE_TOUCH;

	if (screen_type == DELL_E215546) {
		elopacketout(elocodes_e3, ELO_SEQ, 0); // reset to default smartset
		wdtdelay(700000); // wait for LCD touch controller reset
		elopacketout(elocodes_e0, ELO_SEQ, 0); // set touch packet spacing and timing
		elopacketout(elocodes_e2, ELO_SEQ, 0); // nvram save
	} else {
		if (TS_TYPE == 1)
			single_t = FALSE;
		for (code_count = 0; code_count < ELO_SIZE_V80; code_count++) {
			if (single_t) {
				elocmdout(&elocodes_s_e[code_count]);
			} else {
				elocmdout(&elocodes_m_e[code_count]);
			}
		};
	}
}

void putc1(uint16_t c)
{
	while (Busy1USART()) {
	}; // wait until the usart is clear
	putc1USART(c);
}

void putc2(uint16_t c)
{
	while (Busy2USART()) {
	}; // wait until the usart is clear
	putc2USART(c);
}

void start_delay(void)
{
	wdtdelay(100000);
}

uint8_t Test_Screen(void)
{
	while (Busy2USART()) {
	}; // wait until the USART is clear
	if (screen_type == DELL_E224864)
		return TRUE;
	putc2(0x46);
	wdtdelay(30000);
	setup_lcd(); // send lcd touch controller setup codes
	return DATA2;
}

void Cylon_Eye(uint8_t invert)
{
	static uint8_t cylon = 0xfe, LED_UP = TRUE;
	static int32_t alive_led = 0xfe;

	if (invert) { // screen status feedback
		LATD = ~cylon; // roll LEDs cylon style
	} else {
		LATD = cylon; // roll leds cylon style (inverted)
	}

	if (LED_UP && (alive_led != 0)) {
		alive_led = alive_led * 2;
		cylon = cylon << 1;
	} else {
		if (alive_led != 0) alive_led = alive_led / 2;
		cylon = cylon >> 1;
	}
	if (alive_led < 2) {
		alive_led = 2;
		LED_UP = TRUE;
	} else {
		if (alive_led > 128) {
			alive_led = 128;
			LED_UP = FALSE;
		}
	}
}

void main(void)
{
	uint8_t z, check_byte;
	uint16_t eep_ptr;
	uint8_t scaled_char;
	float rez_scale_h = 1.0, rez_parm_h, rez_scale_v = 1.0, rez_parm_v;
	float rez_scale_h_ss = ELO_SS_H_SCALE, rez_scale_v_ss = ELO_SS_V_SCALE;

	status.do_cap = DO_CAP;

	INTCON = 0;
	INTCON3bits.INT1IE = 0;
	INTCON3bits.INT2IE = 0;
	INTCON3bits.INT3IE = 0;
	// default interface
	screen_type = DELL_E215546;
	emulat_type = VIISION;
	/* Configure  PORT pins for output */
	TRISA = 0;
	LATA = 0;
	TRISG = 0;
	LATG = 0;
	LATGbits.LATG3 = 1;
	LATGbits.LATG4 = 1;
	/* check for touchscreen configuration data and setup switch on port B */
	INTCON2bits.RBPU = 0;
	TRISB = 0xff; // inputs
	LATB = 0xff;
	z = PORTB; // read the config switches
	wdtdelay(7000);
	if (z != PORTB) // glitch check
		z = 0xff;
	TRISB = 0; // outputs
	Busy_eep();
	check_byte = Read_b_eep(0);

	if (z != 0xff) { // some config switches read logic zero
		Busy_eep();
		Write_b_eep(0, 0x57);
		Busy_eep();
		Write_b_eep(1, z);

	}

	Busy_eep();
	check_byte = Read_b_eep(0);
	if (check_byte == 0x57) { // change config from default settings if needed
		Busy_eep();
		z = Read_b_eep(1);
		if (z == 0b11111110 || (!LATGbits.LATG3)) {
			screen_type = DELL_E215546;
			emulat_type = VIISION;
			z = 0b11111110;
		}
		if (z == 0b11111010) {
			screen_type = DELL_E224864;
			emulat_type = VIISION;
		}
		if (z == 0b11111101 || (!LATGbits.LATG4)) {
			screen_type = DELL_E215546;
			emulat_type = E220;
			z = 0b11111101;
		}
		if (z == 0b11111001) {
			screen_type = DELL_E224864;
			emulat_type = E220;
		}
		if (z == 0b11111100) {
			screen_type = DELL_E215546;
			emulat_type = OTHER_MECH;
		}
		if (z == 0b11111000) {
			screen_type = DELL_E224864;
			emulat_type = OTHER_MECH;
		}
	}

	TRISB = 0; // turn config inputs to outputs
	TRISC = 0;
	LATC = 0;
	TRISD = 0;
	LATD = z; // show EEPROM configuration data here
	TRISE = 0;
	LATE = 0xFF;
	TRISF = 0;
	LATF = 0xFF;
	TRISH = 0;
	LATH = 0;
	TRISJ = 0;

	CAM_RELAY_TIME = 0;
	CAM_RELAY = 0;
	status.touch_count = 0;
	CAM = 0;
	ssreport.tohost = TRUE;

	LATEbits.LATE3 = 0; // init  led ON
	wdtdelay(700000); // wait for LCD controller reset on power up
	LATEbits.LATE3 = 1; // init  led OFF

	if (emulat_type == OTHER_MECH) {
		/*
		 * Open the USART configured as
		 * 8N1, 9600 baud, in /transmit/receive INT mode
		 */
		/* Host */
		Open1USART(USART_TX_INT_ON &
			USART_RX_INT_ON &
			USART_ASYNCH_MODE &
			USART_EIGHT_BIT &
			USART_CONT_RX &
			USART_BRGH_LOW, 64); // 40mhz osc HS		9600 baud

		/* TouchScreen */
		Open2USART(USART_TX_INT_OFF &
			USART_RX_INT_ON &
			USART_ASYNCH_MODE &
			USART_EIGHT_BIT &
			USART_CONT_RX &
			USART_BRGH_LOW, 64); // 40mhz osc HS		9600 baud

		OpenTimer0(TIMER_INT_ON & T0_16BIT & T0_SOURCE_INT & T0_PS_1_256);
		WriteTimer0(timer0_off); //	start timer0 at 1 second ticks
	}

	if (emulat_type == VIISION) {
		/*
		 * Open the USART configured as
		 * 8N1, 9600 baud, in /transmit/receive INT mode
		 */
		/* Host */
		Open1USART(USART_TX_INT_ON &
			USART_RX_INT_ON &
			USART_ASYNCH_MODE &
			USART_EIGHT_BIT &
			USART_CONT_RX &
			USART_BRGH_LOW, 64); // 40mhz osc HS		9600 baud

		/* TouchScreen */
		Open2USART(USART_TX_INT_OFF &
			USART_RX_INT_ON &
			USART_ASYNCH_MODE &
			USART_EIGHT_BIT &
			USART_CONT_RX &
			USART_BRGH_LOW, 64); // 40mhz osc HS		9600 baud

		OpenTimer0(TIMER_INT_ON & T0_16BIT & T0_SOURCE_INT & T0_PS_1_256);
		WriteTimer0(timer0_off); //	start timer0 at 1 second ticks

		if (screen_type == DELL_E224864) {
			elocmdout_v80(elocodes_s_v); // send touchscreen setup data, causes a frame size report to be send from screen
		}
		if (screen_type == DELL_E215546) {
			elocmdout_v80(&elocodes[7][0]); // reset;
			wdtdelay(700000); // wait for LCD touch controller reset
			/* program the display */
			elocmdout_v80(&elocodes[0][0]);
			elocmdout_v80(&elocodes[4][0]);
			elocmdout_v80(&elocodes[5][0]);
			elocmdout_v80(&elocodes[6][0]);
		}
	}

	if (emulat_type == E220) {
		/*
		 * Open the USART configured as0
		 * 8N1, 9600 baud, in receive INT mode
		 */
		Open1USART(USART_TX_INT_OFF &
			USART_RX_INT_ON &
			USART_ASYNCH_MODE &
			USART_EIGHT_BIT &
			USART_CONT_RX &
			USART_BRGH_LOW, 64);
		// 40mhz osc HS: 9600 baud, USART_BRGH_LOW and 64, for 9600 baud @ 40MHz   (0.16% ERROR IN RATE)

		Open2USART(USART_TX_INT_OFF &
			USART_RX_INT_ON &
			USART_ASYNCH_MODE &
			USART_EIGHT_BIT &
			USART_CONT_RX &
			USART_BRGH_LOW, 64);

		OpenTimer0(TIMER_INT_ON & T0_16BIT & T0_SOURCE_INT & T0_PS_1_1);
		WriteTimer0(TIMERPACKET); //	start timer0 

		setup_lcd(); // send lcd touch controller setup codes
	}

	/* Display a prompt to the USART */
	putrs1USART(build_version);

	while (DataRdy1USART()) { // dump rx data
		z = Read1USART();
	};
	while (DataRdy2USART()) { // dump rx data
		z = Read2USART();
	};

	/* Enable interrupt priority */
	RCONbits.IPEN = 1;
	PIR1bits.RCIF = 0;
	PIR3bits.RC2IF = 0;
	PIR1bits.TX1IF = 0;
	PIE1bits.TX1IE = 0;
	PIR3bits.TX2IF = 0;
	INTCONbits.GIEL = 0; // disable low ints
	INTCONbits.GIEH = 1; // enable high ints
	PORTD = 0x00;

	if (emulat_type == E220) {
		DATA1 = FALSE; // reset COMM flags.
		DATA2 = FALSE; // reset touch COMM flag
		// leds from outputs to ground via resistor.
		while (status.do_cap) {
			ClrWdt();
			LATFbits.LATF5 = !LATFbits.LATF5; // flash  led
			if (status.host_write || status.scrn_write) {
				eep_ptr = 0;
				while (eep_ptr <= 255) {
					INTCONbits.GIE = 0; // global int enable
					INTCONbits.PEIE = 0; // enable all unmasked int
					Busy_eep();
					Write_b_eep(eep_ptr, host_rec[eep_ptr]); //  data
					ClrWdt(); // reset the WDT timer
					Busy_eep();
					Write_b_eep(eep_ptr + CAP_SIZE, scrn_rec[eep_ptr]); //  data
					Busy_eep();
					INTCONbits.GIE = 1; // global int enable
					INTCONbits.PEIE = 1; // enable all unmasked int
					ClrWdt(); // reset the WDT timer
					eep_ptr++;
					ClrWdt(); // reset the WDT timer
				}
			}
		}

		Test_Screen(); // send touch init commands
		/* Loop forever */
		while (TRUE) {
			if (j++ >= (BLINK_RATE_E220 + speedup)) { // delay a bit ok
				LATHbits.LATH0 = !LATHbits.LATH0; // flash onboard led
				LATEbits.LATE0 = !LATEbits.LATE0; // flash external led
				LATEbits.LATE7 = LATEbits.LATE0; // flash external led
				Cylon_Eye(LCD_OK);
				if (status.cam_time > MAX_CAM_TIMEOUT) {
					CAM_RELAY_TIME = 0;
					if (touch_corner_timed) {
						touch_corner_timed = FALSE;
						CAM_RELAY_TIME = 0;
						CAM_RELAY_AUX = 0; // clear video switch
						CAM_RELAY = 0; // clear video switch
						CAM = FALSE;
					}
				}
				status.cam_time++;

				/*		For the auto-restart switch						*/
				//FIXME
				if (AUTO_RESTART) { // enable auto-restarts
					if ((status.restart_delay++ >= (uint16_t) 60) && (!TSTATUS)) { // try and reinit lcd after delay
						start_delay();
						setup_lcd(); // send lcd touch controller setup codes
						start_delay();
						while (TRUE) {
						}; // lockup WDT counter to restart
					} else {
						if ((status.restart_delay >= (uint16_t) 150) && (TSTATUS)) { // after delay restart TS status.
							TSTATUS = FALSE; // lost comms while connected
							status.restart_delay = 0;
						};
					};
				}
				j = 0;
			}

			LATFbits.LATF4 = !LATFbits.LATF4; // toggle bits program run led
			touch_cam(); // always check the cam touch

			if (CATCH46) { // flag to send report to host
				LATFbits.LATF0 = 1; // flash status led
				if (CATCH) { // send the buffered touch report
					Delay10KTCYx(75); // 75 ms
					putc1(0xFE); // send position report header to host
					if (screen_type == DELL_E215546) {
						ssreport.tohost = TRUE;
						rez_parm_h = ((float) (ssreport.x_cord)) * rez_scale_h_ss;
						rez_parm_v = ((float) (ssreport.y_cord)) * rez_scale_v_ss;
						ssreport.tohost = FALSE;
						scaled_char = ((uint16_t) (rez_parm_h));
						elobuf[0] = scaled_char;
						putc1(scaled_char); // send h scaled touch coord
						scaled_char = ((uint16_t) (rez_parm_v));
						elobuf[1] = scaled_char;
						putc1(scaled_char); // send v scaled touch coord
					} else {
						rez_parm_h = ((float) (elobuf[0])) * rez_scale_h;
						scaled_char = ((uint16_t) (rez_parm_h));
						putc1(scaled_char); // send h scaled touch coord
						rez_parm_v = ((float) (elobuf[1])) * rez_scale_v;
						scaled_char = ((uint16_t) (rez_parm_v));
						putc1(scaled_char); // send v scaled touch coord
						c_idx = 0;
					}
					putc1(0xFF); // send end of report to host
					status.touch_count++;
					CATCH = FALSE;
					CATCH46 = FALSE;
				} else { // just send status
					Delay10KTCYx(65); // 65 ms
					putc1(0xF5); // send status report
					putc1(0xFF); // end of report
					status.status_count++;
					CATCH46 = FALSE;
				};
			};

			if (CATCH37) { // send screen size codes
				LATFbits.LATF7 = 0; // off blink for rez codes sent
				Delay10KTCYx(75); // 75 ms
				rez_scale_h = 1.0; // LCD touch screen real H/V rez
				rez_scale_v = 1.0;
				if (!(screen_type == DELL_E215546))
					putc2(0x3D); // send clear buffer to touch

				putc1(0xF4); // send status report
				if (TS_TYPE == 0) { // CRT type screens
					putc1(0x77); // touch parm
					putc1(0x5f); // touch parm
				}
				if (TS_TYPE == 1) { // new LCD type screens
					putc1(0x71); // touch parm 113
					putc1(0x59); // touch parm 89
				}
				putc1(0xFF); // end of report
				status.resync_count++;
				CATCH37 = FALSE;
			};

			if (TOUCH) {
				// do nothing now.
			};

			/*	check for port errors and clear if needed	*/
			if (RCSTA1bits.OERR) {
				LATF = 0xFF; // all leds off with error
				RCSTA1bits.CREN = 0;
				RCSTA1bits.CREN = 1;
			}
			if (RCSTA2bits.OERR) {
				LATF = 0xFF;
				RCSTA2bits.CREN = 0;
				RCSTA2bits.CREN = 1;
			}
			ClrWdt(); // reset the WDT timer
		}
	}

	if (emulat_type == VIISION) {
		/* Loop forever */
		while (TRUE) { // busy loop BSG style
			if (j++ >= BLINK_RATE_V80) { // delay a bit ok
				LATHbits.LATH0 = !LATHbits.LATH0; // flash onboard led
				LATEbits.LATE0 = !LATEbits.LATE0; // flash external led
				LATEbits.LATE7 = LATEbits.LATE0; // flash external led

				if (LCD_OK) { // screen status feedback
					Cylon_Eye(LCD_OK); // roll leds cylon style
					timer0_off = TIMEROFFSET;
				} else {
					Cylon_Eye(LCD_OK); // roll leds cylon style (inverted)
					timer0_off = TIMERFAST;
				}
				j = 0;
				if ((screen_type == DELL_E224864) && (SCREEN_INIT && !PIE1bits.TX1IE)) { // if this flag is set send elo commands
					elocmdout_v80(elocodes_s_v); // send touchscreen setup data, causes a frame size report to be send from screen
					SCREEN_INIT = FALSE; // commands sent, now wait for reply to set LCD_OK flag
					LATEbits.LATE3 = 1; // init  led OFF
				}
			}
			/*	check for port errors and clear if needed	*/
			if (RCSTA1bits.OERR) {
				LATF = 0xFF;
				RCSTA1bits.CREN = 0;
				RCSTA1bits.CREN = 1;
			}
			if (RCSTA2bits.OERR) {
				LATF = 0xFF;
				RCSTA2bits.CREN = 0;
				RCSTA2bits.CREN = 1;
			}
			ClrWdt(); // reset the WDT timer
		}
	}

	if (emulat_type == OTHER_MECH) {
		/* Loop forever */
		while (TRUE) { // busy loop BSG style
			if (j++ >= BLINK_RATE_OTHER) { // delay a bit ok
				LATHbits.LATH0 = !LATHbits.LATH0; // flash onboard led
				LATEbits.LATE0 = !LATEbits.LATE0; // flash external led
				LATEbits.LATE7 = LATEbits.LATE0; // flash external led
				Cylon_Eye((screen_type == DELL_E224864));
				j = 0;
			}
			/*	check for port errors and clear if needed	*/
			if (RCSTA1bits.OERR) {
				LATF = 0xFF;
				RCSTA1bits.CREN = 0;
				RCSTA1bits.CREN = 1;
			}
			if (RCSTA2bits.OERR) {
				LATF = 0xFF;
				RCSTA2bits.CREN = 0;
				RCSTA2bits.CREN = 1;
			}
			ClrWdt(); // reset the WDT timer
		}
	}
}
