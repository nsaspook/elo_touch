/*
 IMPLANTER terminal code */
/*
 * This program converts the rs-232 output from a ELO controller type LCD monitor
 * to a format that can be used with Varian Implanters with CRT touch monitors
 * The LCD touchscreen will be  programmed to the correct touch response and configuration at program start
 *
 * USART1 is the host comm port
 * USART2 is the touch-screen comm port
 *
 * Microchip Inc , May 2022
 * Gresham, Oregon
 *
 *
 * This application is designed for use with the
 * PIC18F14Q41 board and  device
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
 * V3.31	bug fixes, cleanup
 * V3.32	PORT G jumpers for smartset configuation.
 * V3.33	display screen/machine config on LATJ with data 8 bit config on PORTB, program structure
 * V3.34	merge state machine refactor update
 * V3.35	"" ""
 * V3.38	fix G 3&4 jumpers for proper monitor and tool selection, update eeprom
 * V4.00	Q43 version, USART operated in bare mode using custom ISR code
 * V4.03	adjust HV scaling values for better calibration on new windows systems
 * V4.04	Small bug fixes version
 * V4.05	receive and parse touch-screen ID codes
 * V5.00	Q41 version, remove CarrollTouch code and options. obsolete
 *
 *
 *
 * HOST RS-232  5-1     uC port1
 * Female       2-2-tx
 *              3-3-rx
 * LCD  RS-232  5-1     uC port2
 * Male         2-3-rx
 *              3-2-tx
 *
 * DIO2/SPI plug
 * 1..2 MIN0 jumper: DELL_E215546 VIISION
 * 3..4 MIN1 jumper: DELL_E215546 E220
 *
 * HFBR-0501Z light link converter for VIISION front controller
 * connected to host USART1 xmit output pin
 */

/* E220/E500 terminal code */
/*
 * This program converts the rs-232 output from a ELO touch-screen controller
 * to a format that can be used with the Varian E220/E500 Implanter
 * The touch controller must be first programmed
 * USART1 is the host comm port
 * USART2 is the touch-screen comm port
 *
 * Microchip Inc , Aug 2009,2018,2021,2022
 * Gresham, Oregon
 *
 *
 * This application is designed for use with the
 * MCHP Q41 touch-board and device.
 * HOST RS-232  Black 5-1     uC port1
 * Female       Red   2-2-tx
 *              White 3-3-rx
 * LCD  RS-232  Black 5-1     uC port2
 * Male         Red   2-3-rx
 *              White 3-2-tx
 *
 * relay output 
 * DIO3 pin1 relay power +5vdc
 *	pin2 relay ground
 * VGA converter box relay
 * Omron
 * G6k-2P bottom view
 * Pin		8 - gnd, wire tag 0/stripe
 * Pin		1 + 5vdc signal
 */

//#define DEBUG_CAM

#pragma warning disable 520
#pragma warning disable 1498

#include <xc.h>
#include <stdlib.h>
#include <stdint.h>
#include "vtouch.h"
#include "vtouch_build.h"
#include "eadog.h"

const char *build_date = __DATE__, *build_time = __TIME__;

typedef struct reporttype {
	uint8_t headder, status;
	uint16_t x_cord, y_cord, z_cord;
	uint8_t checksum;
	volatile uint8_t tohost;
	uint8_t id_type, id_io, id_features, id_minor, id_major, id_p, id_class;
} reporttype;

typedef struct statustype {
	int32_t alive_led, touch_count, resync_count, rawint_count, status_count, ticks;
	uint8_t host_write : 1;
	uint8_t scrn_write : 1;
	uint8_t do_cap : 1;
	uint8_t comm_check, init_check, touch_good, cam_time;
	uint16_t restart_delay;
} statustype;

typedef struct disp_state_t {
	uint8_t CATCH, TOUCH, UNTOUCH, LCD_OK,
	SCREEN_INIT,
	CATCH46, CATCH37, TSTATUS,
	DATA1, DATA2, CAM;
	uint16_t c_idx;
	int16_t speedup;
} disp_state_t;

volatile disp_state_t S = {FALSE};

/*
 * Old monitors
 * E757389	CarrollTouch obsolete
 * E224864	CarrollTouch obsolete
 * E779866	SecureTouch  use DELL_E215546 setting
 * E215546	IntelliTouch use DELL_E215546 setting
 * NEW REPLACEMENT MONITORS 1990L 1991L
 * E328700	IntelliTouch use DELL_E215546 setting
 * E328497	IntelliTouch use DELL_E215546 setting
 * E483757	new remote OSD
 * E005277	power brick
 */
enum screen_type_t {
	DELL_E215546, OTHER_SCREEN
};

enum emulat_type_t {
	VIISION, E220, OTHER_MECH
};

enum screen_type_t screen_type;
enum emulat_type_t emulat_type;

int32_t j = 0;
float xs = X_SCALE, ys = Y_SCALE, xs_ss = X_SCALE_SS, ys_ss = Y_SCALE_SS; // defaults
uint16_t timer0_off = TIMEROFFSET;

volatile uint8_t elobuf[BUF_SIZE], elobuf_out[BUF_SIZE_V80], elobuf_in[BUF_SIZE_V80], xl = X_LOGICAL, yl = Y_LOGICAL;
uint8_t ssbuf[BUF_SIZE];

reporttype ssreport;
volatile struct statustype status = {
	.ticks = 0,
};

const uint8_t elocodes[ELO_SEQ_V80][ELO_SIZE_I_V80] = {// elo 2210/2216 program codes
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

volatile uint8_t idx = 0;
uint16_t x_tmp, y_tmp, uvalx, lvalx, uvaly, lvaly;

char buffer[64], opbuffer[24];

void tmr0isr(void);
void uart1risr(void);
void uart2work(void);

void uart2work(void)
{
	uint8_t c, i, uchar;
	static uint8_t sum = 0xAA + 'U';

	if (emulat_type == E220) {
		// is data from touchscreen COMM2
		PIR3bits.TMR0IF = 0;
		// Write to the Timer0 register
		if (S.CAM && (status.cam_time > MAX_CAM_TIME)) {
			CAM_RELAY = 0; // clear video switch
			S.CAM = FALSE;
		}

		c = UART2_Read(); // read data from touchscreen
		if (status.do_cap) {
		} else {
			if (screen_type == DELL_E215546) { // IntelliTouch

				ssbuf[idx] = c;
				switch (idx++) {
				case 0: // start of touch controller packet, save data and compute checksum
					sum = 0xaa;
					if (c != 'U') {
						idx = 0;

					}
					break;
				case 9: // end of touch controller packet

					idx = 0;
					if (c != sum) { // bad checksum

						break;
					}
					if (ssbuf[1] == 'T') {
						status.restart_delay = 0;
						S.CATCH = TRUE;
						if (!ssreport.tohost) {
							ssreport.x_cord = (ELO_REV_H - (((uint16_t) ssbuf[3])+(((uint16_t) ssbuf[4]) << 8))) >> (uint16_t) 4;
							ssreport.y_cord = (((uint16_t) ssbuf[5])+(((uint16_t) ssbuf[6]) << 8)) >> 4;
						}
					} else {
						if (ssbuf[1] == 'I') {
							status.restart_delay = 0;
							ssreport.id_type = ssbuf[2];
							ssreport.id_io = ssbuf[3];
							ssreport.id_features = ssbuf[4];
							ssreport.id_minor = ssbuf[5];
							ssreport.id_major = ssbuf[6];
							ssreport.id_p = ssbuf[7];
							ssreport.id_class = ssbuf[8];
						}
						if (ssbuf[1] == 'A') {
							status.restart_delay = 0;
							RLED_SetHigh(); // connect  led ON
							S.speedup = -10000;
						}
					}
					break;
				default:
					break;
				}
				sum += c;
				S.DATA2 = TRUE; // usart is connected to data

			}
		}
	}

	if (emulat_type == VIISION) {
		if (screen_type == DELL_E215546) { // This is for the newer SMARTSET Intellitouch screens
			/* Get the character received from the USART */
			c = UART2_Read();
			status.status_count++;

			if (((c & 0xc0) == 0xc0) || S.CATCH) { // start of touch sequence
				S.CATCH = TRUE; // found elo touch command start of sequence
				j = 0; // reset led timer
				elobuf[i++] = c; // start stuffing the command buffer
			}
			if (i == CMD_SIZE_SS_V80) { // see if we should send it

				i = 0; // reset i to start of cmd
				uchar = 0; /* check for proper touch format */
				if ((elobuf[0]& 0xc0) == 0xc0) /* binary start code? */ {
					uchar = TRUE;
				}


				S.CATCH = FALSE; // reset buffering now

				/* munge the data for proper Varian format */
				if (elobuf[5]) { // S.TOUCH
					S.TOUCH = TRUE; // first touch sequence has been sent
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
					elobuf_in[1] = (uint8_t) x_tmp; // X to 8-bit var
					elobuf_in[2] = (uint8_t) y_tmp; // Y
					elobuf_out[0] = 0xc0 + ((elobuf_in[1]&0xc0) >> (uint8_t) 6); // stuff into binary 4002 format
					elobuf_out[1] = 0x80 + (elobuf_in[1]&0x3f);
					elobuf_out[2] = 0x40 + ((elobuf_in[2]&0xc0) >> (uint8_t) 6);
					elobuf_out[3] = 0x00 + (elobuf_in[2]&0x3f);
					elobuf_out[4] = 0x00;
					elobuf_out[5] = 0x0f;
				}

				if (!elobuf[5]) { //S.UNTOUCH
					S.UNTOUCH = TRUE; // untouch sequence found
					elobuf_out[0] = 0xc0; // restuff the buffer with needed varian untouch sequence
					elobuf_out[1] = 0x80;
					elobuf_out[2] = 0x40;
					elobuf_out[3] = 0x00;
					elobuf_out[4] = 0x00;
					elobuf_out[5] = 0x00;
				}

				if (S.TOUCH || S.UNTOUCH) { // send both

					if (uchar) { /* only send valid data to HOST */
						for (int e = 0; e < HOST_CMD_SIZE_V80; e++) { // send buffered data
							putc1(elobuf_out[e]); // send to host
						}
						status.touch_count++;
					}
					S.LCD_OK = TRUE; // looks like a screen controller is connected
					S.SCREEN_INIT = FALSE; // command code has been received by lcd controller
					status.init_check = 0; // reset init code timer

					if (S.UNTOUCH) { // After untouch is sent dump buffer and clear all.
						S.TOUCH = FALSE;
						S.UNTOUCH = FALSE;
						S.CATCH = FALSE;
						// Write to the Timer0 register
					}
				}
			}

			/* Clear the interrupt flag */
			if (i > CMD_OVERFLOW_V80) {
				i = 0; // just incase i is greater than CMD_SIZE*2 somehow
				S.CATCH = FALSE;
				S.TOUCH = FALSE;
				S.UNTOUCH = FALSE;
			}
		}

		if (emulat_type == OTHER_SCREEN) {
			/* Get the character received from the USART */
			c = UART2_Read();
		}
	}
};

/*
 * check host transmit data
 */
void uart1risr(void)
{
	uint8_t tchar;

	tchar = U1RXB; // read from host
	S.DATA1 = TRUE; // usart is connected to data
	if (tchar == (uint8_t) 0x46) { // send one report to host
		S.CATCH46 = TRUE;
		status.touch_good = 0;
	}
	if (tchar == (uint8_t) 0x37) { // start of touch scan read
		S.CATCH37 = TRUE;
		status.touch_good = 0;
	}
	if (tchar == (uint8_t) 0x3C) { // touch reset from host
		// a possible setup command
	}
};

/*
 * check timer0 irq 1 second time
 */
void tmr0isr(void)
{
	//check for TMR0 overflow
	idx = 0; // reset packet char index counter
	ssreport.tohost = FALSE; // when packets stop allow for next updates
	RLED_Toggle();

	if (S.LCD_OK) {
		DB1_SetHigh();
	} else {
		if ((status.init_check++ >LCD_CHK_TIME)) {
			status.init_check = 0; // reset screen init code counter
			S.SCREEN_INIT = TRUE; // set init code flag so it can be sent in main loop
			DB0_SetHigh();
		}
	}

	if ((status.comm_check++ >COMM_CHK_TIME) && !S.CATCH) { // check for LCD screen connection
		status.comm_check = 0; // reset connect heartbeat counter
		S.LCD_OK = FALSE; // reset the connect flag while waiting for response from controller.
		DB1_SetHigh();
	}
}

void touch_cam(void)
{
	//	check for corner presses
	if (S.CATCH) {
		if ((elobuf[0] <= (uint8_t) 0x06) && (elobuf[1] >= (uint8_t) 0x5a)) { // check for left bottom corner
			touch_corner1++;
			touch_corner_timed = TRUE;
		};

		if ((elobuf[0] >= (uint8_t) 0x72) && (elobuf[1] >= (uint8_t) 0x5a)) { // check for right bottom corner
			touch_corner1++;
		};
	};


	if (touch_corner1 >= MAX_CAM_TOUCH) { // we have several corner presses
		S.CAM = TRUE;
		status.cam_time = 0;
		touch_corner1 = 0;
		CAM_RELAY = 1; // set primary VGA/CAM switch
		elobuf[0] = 0; // clear last touch values
		elobuf[1] = 0;
	};
}

/*
 * busy loop delay with WDT reset
 */
void wdtdelay(const uint32_t delay)
{
	uint32_t dcount;
	
	for (dcount = 0; dcount <= delay; dcount++) { // delay a bit
		ClrWdt(); // reset the WDT timer
	};
}

void elocmdout(uint8_t * elostr)
{
	putc2(elostr[0]);
	wdtdelay(2000);
}

void eloSScmdout(const uint8_t elostr)
{
	putc2(elostr);
	wdtdelay(2000); // inter char delay
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
			break;
		}
		eloSScmdout(c);
	};
	if (slow) {
		wdtdelay(30000);
	}
}

void elocmdout_v80(const uint8_t * elostr)
{
	uint8_t elo_char;
	for (int e = 0; e < ELO_SIZE_V80; e++) { // send buffered data
		elo_char = elostr[e];
		putc2(elo_char); // send to LCD touch
		wdtdelay(2000); // inter char delay
	}
	wdtdelay(50000); // wait for LCD controller reset
}

void setup_lcd(void)
{
	if (screen_type == DELL_E215546) {
		elopacketout(elocodes_e5, ELO_SEQ, 0); // ask for ID
		wdtdelay(30000);
		elopacketout(elocodes_e3, ELO_SEQ, 0); // reset to default smartset
		wdtdelay(700000); // wait for LCD touch controller reset
		elopacketout(elocodes_e0, ELO_SEQ, 0); // set touch packet spacing and timing
		elopacketout(elocodes_e2, ELO_SEQ, 0); // nvram save
	}
}

void putc1(const uint8_t c)
{
	while (!UART1_is_tx_done()) {
	}; // wait until the usart is clear
	U1TXB = c;
}

void putc2(const uint8_t c)
{
	while (!UART2_is_tx_done()) {
	}; // wait until the usart is clear
	U2TXB = c;
}

void start_delay(void)
{
	wdtdelay(100000);
}

uint8_t Test_Screen(void)
{
	putc2(0x46);
	wdtdelay(30000);
	setup_lcd(); // send lcd touch controller setup codes
	return S.DATA2;
}

void main(void)
{
	uint8_t z, check_byte, ts_type = TS_TYPE;
	uint16_t eep_ptr, update_screen = 0;
	uint8_t scaled_char;
	float rez_scale_h = 1.0, rez_parm_h, rez_scale_v = 1.0, rez_parm_v;
	float rez_scale_h_ss = ELO_SS_H_SCALE, rez_scale_v_ss = ELO_SS_V_SCALE;

	SYSTEM_Initialize();

	status.do_cap = DO_CAP;
	S.c_idx = 0;
	S.speedup = 0;

	// default interface
	screen_type = DELL_E215546;
	emulat_type = E220;
	z = 0b11111101;
	sprintf(opbuffer, "E220 DELL_E215546");

	/*
	 * set touchscreen emulation type code
	 */

	check_byte = DATAEE_ReadByte(0);

	if (check_byte != 0x57) { // invalid eeprom display data
		DATAEE_WriteByte(0, 0x57);
		DATAEE_WriteByte(1, z);
	}

	check_byte = DATAEE_ReadByte(0);
	if (check_byte == 0x57) { // change config from default settings if needed
		z = DATAEE_ReadByte(1);
		if (z == 0b11111110 || (!MIN0_GetValue())) {
			screen_type = DELL_E215546;
			emulat_type = VIISION;
			z = 0b11111110;
			DATAEE_WriteByte(1, z);
			sprintf(opbuffer, "VIISION DELL_E215546");
		}
		if (z == 0b11111101 || (!MIN1_GetValue())) {
			screen_type = DELL_E215546;
			emulat_type = E220;
			z = 0b11111101;
			DATAEE_WriteByte(1, z);
			sprintf(opbuffer, "E220 DELL_E215546");
		}
		if (z == 0b11111100) {
			screen_type = DELL_E215546;
			emulat_type = OTHER_MECH;
			sprintf(opbuffer, "OTHER DELL_E215546");
		}
	}

	CAM_RELAY = 0;
	status.touch_count = 0;
	S.CAM = 0;
	ssreport.tohost = TRUE;

	wdtdelay(700000); // wait for LCD controller reset on power up
	TMR0_SetInterruptHandler(tmr0isr);
	UART1_SetRxInterruptHandler(uart1risr);
	// Enable high priority global interrupts
	INTERRUPT_GlobalInterruptHighEnable();

	// Enable low priority global interrupts.
	INTERRUPT_GlobalInterruptLowEnable();

	init_display();
	sprintf(buffer, "%s ", "          ");
	eaDogM_WriteStringAtPos(0, 0, buffer);
	sprintf(buffer, "%s ", build_version);
	eaDogM_WriteStringAtPos(0, 0, buffer);
	sprintf(buffer, "%s ", build_date);
	eaDogM_WriteStringAtPos(1, 0, buffer);
	/* display build time and boot status codes 67 34 07, WDT reset 67 24 07 */
	sprintf(buffer, "%s B:%X %X %X", build_time, STATUS, PCON0, PCON1);
	eaDogM_WriteStringAtPos(2, 0, buffer);
	eaDogM_WriteStringAtPos(3, 0, opbuffer);

	if (emulat_type == OTHER_MECH) {
		/*
		 * Open the USART configured as
		 * 8N1, 9600 baud, in /transmit/receive INT mode
		 */
		/* Host */
	}

	if (emulat_type == VIISION) {
		/*
		 * Open the USART configured as
		 * 8N1, 9600 baud, in /transmit/receive INT mode
		 */
		/* Host */

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
		 * Open the USART configured as
		 * 8N1, 9600 baud, in receive INT mode
		 */
		setup_lcd(); // send lcd touch controller setup codes
	}

	if (emulat_type == E220) {
		S.DATA1 = FALSE; // reset COMM flags.
		S.DATA2 = FALSE; // reset touch COMM flag
		// leds from outputs to ground via resistor.
		while (status.do_cap) {
			ClrWdt();
			if (status.host_write || status.scrn_write) {
			}
		}

		Test_Screen(); // send touch init commands
		/* Loop forever */
		while (TRUE) {
			if (UART2_is_rx_ready()) {
				uart2work();
			}
			if (j++ >= (BLINK_RATE_E220 + S.speedup)) { // delay a bit ok
#ifdef	DEBUG_CAM
				CAM_RELAY = !CAM_RELAY;
#endif
				if (status.cam_time > MAX_CAM_TIMEOUT) {
					if (touch_corner_timed) {
						touch_corner_timed = FALSE;
						CAM_RELAY = 0; // clear video switch
						S.CAM = FALSE;
					}
				}
				status.cam_time++;

				/*		For the auto-restart switch						*/
				//FIXME
				if (AUTO_RESTART) { // enable auto-restarts
					if ((status.restart_delay++ >= (uint16_t) 60) && (!S.TSTATUS)) { // try and reinit lcd after delay
						start_delay();
						setup_lcd(); // send lcd touch controller setup codes
						start_delay();
						while (TRUE) {
						}; // lockup WDT counter to restart
					} else {
						if ((status.restart_delay >= (uint16_t) 150) && (S.TSTATUS)) { // after delay restart TS status.
							S.TSTATUS = FALSE; // lost comms while connected
							S.speedup = 0;
							status.restart_delay = 0;
						};
					};
				}
				j = 0;
				if (update_screen++ >= SCREEN_UPDATE) {
					update_screen = 0;
					sprintf(buffer, "E %lu %lu %lu %2.2x %u.%u %2.2x ", status.status_count, status.touch_count, status.ticks++, ssreport.id_type, ssreport.id_major, ssreport.id_minor, ssreport.id_class);
					eaDogM_WriteStringAtPos(0, 0, buffer);
					if (ssreport.id_type == 0) {
						sprintf(opbuffer, "NO SCREEN DETECTED  ");
					}
					if (ssreport.id_type == 0x32) {
						sprintf(opbuffer, "E220 DELL_E215546");
					}
					if (ssreport.id_type == 0x33) {
						sprintf(opbuffer, "E220 DELL_E224864");
					}
					eaDogM_WriteStringAtPos(3, 0, opbuffer);
				}
			}

			touch_cam(); // always check the cam touch

			if (S.CATCH46) { // flag to send report to host
				if (S.CATCH) { // send the buffered touch report
					__delay_ms(75);
					putc1(0xFE); // send position report header to host
					if (screen_type == DELL_E215546) {
						ssreport.tohost = TRUE;
						rez_parm_h = ((float) (ssreport.x_cord)) * rez_scale_h_ss;
						rez_parm_v = ((float) (ssreport.y_cord)) * rez_scale_v_ss;
						ssreport.tohost = FALSE;
						scaled_char = ((uint8_t) (rez_parm_h));
						elobuf[0] = scaled_char;
						putc1(scaled_char); // send h scaled touch coord
						scaled_char = ((uint8_t) (rez_parm_v));
						elobuf[1] = scaled_char;
						putc1(scaled_char); // send v scaled touch coord
					}
					putc1(0xFF); // send end of report to host
					status.touch_count++;
					S.CATCH = FALSE;
					S.CATCH46 = FALSE;
				} else { // just send status
					__delay_ms(65);
					putc1(0xF5); // send status report
					putc1(0xFF); // end of report
					status.status_count++;
					S.CATCH46 = FALSE;
				};
			};

			if (S.CATCH37) { // send screen size codes
				__delay_ms(75);
				rez_scale_h = 1.0; // LCD touch screen real H/V rez
				rez_scale_v = 1.0;
				if (!(screen_type == DELL_E215546)) {
					putc2(0x3D); // send clear buffer to touch
				}

				putc1(0xF4); // send status report
				switch (ts_type) {
				case 1:
					// new LCD type screens
					putc1(0x71); // touch parm 113
					putc1(0x59); // touch parm 89
					break;
				default:
					// CRT type screens
					putc1(0x77); // touch parm
					putc1(0x5f); // touch parm
					break;
				}
				putc1(0xFF); // end of report
				status.resync_count++;
				S.CATCH37 = FALSE;
			};

			if (S.TOUCH) {
				// do nothing now.
			};

			ClrWdt(); // reset the WDT timer
		}
	}

	if (emulat_type == VIISION) {
		/* Loop forever */
		while (TRUE) { // busy loop BSG style
			if (UART2_is_rx_ready()) {
				uart2work();
			}
			if (j++ >= BLINK_RATE_V80) { // delay a bit ok

				if (S.LCD_OK) { // screen status feedback
					timer0_off = TIMEROFFSET;
				} else {
					timer0_off = TIMERFAST;
				}
				j = 0;
				if (update_screen++ >= SCREEN_UPDATE) {
					update_screen = 0;
					sprintf(buffer, "V %lu %lu %lu            ", status.status_count, status.touch_count, status.ticks++);
					eaDogM_WriteStringAtPos(0, 0, buffer);
				}
			}
			ClrWdt(); // reset the WDT timer
		}
	}

	if (emulat_type == OTHER_MECH) {
		/* Loop forever */
		while (TRUE) { // busy loop BSG style
			if (j++ >= BLINK_RATE_OTHER) { // delay a bit ok
				j = 0;
			}
			ClrWdt(); // reset the WDT timer
		}
	}
}
