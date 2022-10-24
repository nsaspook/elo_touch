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
 * V1.04	xmit RS232 uses int mode in ISR
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
 * V5.10	Q41 fixed for V80 and E220 output codes and 20X4 status character display via SPI
 * V5.11	add invert host TX pin invert jumper on RELAY pin
 * V5.12        fix codes to work with ACCUTOUCH screens
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
 * Both jumpers for auto touch test mode
 * 
 *
 * HFBR-0501Z light link converter for VIISION front controller
 * connected to host USART1 xmit output pin
 * Jumper DIO3 from RELAY pin#1 to GND pin#2 sets UART1 TX inverted output for possible light-link issues at boot
 */

/* E220/E500 terminal code */
/*
 * This program converts the RS-232 output from a ELO touch-screen controller
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
 * DIO3 pin1 relay power PIn +5vdc
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

#include "vtouch.h"
#include "vtouch_build.h"
#include "eadog.h"

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

enum screen_type_t screen_type;
enum emulat_type_t emulat_type;

int32_t j = 0;
float xs = X_SCALE, ys = Y_SCALE, xs_ss = X_SCALE_SS, ys_ss = Y_SCALE_SS; // defaults
uint16_t timer0_off = TIMEROFFSET;

uint8_t elobuf[BUF_SIZE], elobuf_out[BUF_SIZE_V80], elobuf_in[BUF_SIZE_V80], xl = X_LOGICAL, yl = Y_LOGICAL;
uint8_t ssbuf[BUF_SIZE];
volatile uint8_t offset = 0x1B; // touch origin offset for accutouch

reporttype ssreport;
volatile disp_state_t S = {
    .TEST_MODE = false,
};
volatile struct statustype status = {
    .ticks = 0,
};

volatile bool xy_flip = true;

/*
 * Viision setup codes
 */
const uint8_t elocodes[ELO_SEQ_V80][ELO_SEQ] = {// elo 2210/2216 program codes
    'U', 'M', 0x00, 0x85, 0x40, '0', '0', '0', '0', '0', // initial touch, NO stream Point,untouch,Z-axis,no scaling, tracking
    'U', 'B', 5, 20, 0x00, 0x00, 0x0f, '0', '0', '0', // packet delays to match old terminal
    'U', 'N', '1', '7', '0', '0', '0', '0', '0', '0', // nvram save
    'U', 'R', '2', '0', '0', '0', '0', '0', '0', '0', // nvram reset
}; // initial intelli-touch codes

/*
 * E220/E500 setup codes
 */
// SmartSet codes 0 command, 1 status, 2 low byte, 3 high byte, etc ...
uint8_t elocodes_e0[] = {
    'U', 'B', 0x01, 0x4, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 // smartset timing and spacing setup
};
uint8_t elocodes_e2[] = {
    'U', 'N', '1', '7', 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};
uint8_t elocodes_e3[] = {
    'U', 'R', '2', 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};
uint8_t elocodes_e5[] = {
    'U', 'i', 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};


uint8_t testcodes[ELO_TEST][ELO_SEQ] = {
    0x55, 0x54, 0x81, 0x54, 0x00, 0x88, 0x00, 0x2d, 0x00, 0xdd, // touch corners X,Y 54, 136
    0x55, 0x54, 0x81, 0x45, 0x00, 0x83, 0x0f, 0x1a, 0x00, 0xc5, // 69, 3971
    0x55, 0x54, 0x81, 0xbf, 0x0f, 0x81, 0x0f, 0x1d, 0x00, 0x4f, // 4031, 29
    0x55, 0x54, 0x81, 0xb3, 0x0f, 0x8c, 0x00, 0x1d, 0x00, 0x3f, // 4019, 3869
    0x55, 0x54, 0x84, 0x57, 0x0f, 0x36, 0x08, 0x4f, 0x00, 0xca //  untouch 3927,2102
};

uint16_t touch_corner1 = 0, touch_corner_timed = 0, x_tmp, y_tmp;
uint8_t idx = 0;

char buffer[64], opbuffer[24];

static void tmr0isr(void);
static void delayisr(void);
static void uart1risr(void);
uint32_t get_ticks(void);
void clear_ticks(void);
void uart2work(void);
uint8_t uart_stuff(uint8_t);
void setup_lcd_e220(void);

/*
 * push touchscreen presses to serial buffer
 */
uint8_t uart_stuff(uint8_t touch) {
    static uint8_t line = 0; // keep track of current touch string

    if (touch >= ELO_TEST) {
        touch = 0;
    }

    if (line >= ELO_TEST) {
        line = touch;
    }

    for (uint8_t i = 0; i < ELO_SEQ; i++) {
        UART2_RxDataStuff(testcodes[line][i]);
    }
    return line++;
}

/*
 * protocol processing functions
 */
void uart2work(void) {
    uint8_t c, i, uchar;
    static uint8_t sum = 0xAA + 'U';

    MLED_Toggle();

    if (emulat_type == E220) {
        // is data from Touchscreen COMM2
        // Write to the Timer0 register
        TMR0_Reload();
        if (S.CAM && (status.cam_time > MAX_CAM_TIME)) {
            CAM_RELAY = 0; // clear video switch
            S.CAM = false;
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
                            status.id++;
                            break;
                        }
                        if (ssbuf[1] == 'T') {
                            status.restart_delay = 0;
                            if (ssbuf[2] & 0x80) { // check for accutouch format screen
                                xy_flip = false;
                            } else {
                                xy_flip = true; // FLIP with accutouch
                            }
                            S.CATCH = true;
                            if (!status.tohost) {
                                ssreport.x_cord = (ELO_REV_H - (((uint16_t) ssbuf[3])+(((uint16_t) ssbuf[4]) << 8))) >> (uint16_t) 4;
                                ssreport.y_cord = (((uint16_t) ssbuf[5])+(((uint16_t) ssbuf[6]) << 8)) >> 4;
                                if (xy_flip) {
                                    ssreport.x_cord = ELO_REV_H - ssreport.x_cord; // FLIP X
                                    ssreport.y_cord = ELO_REV_H - ssreport.y_cord; // FLIP Y
                                }
                                x_tmp = (((uint16_t) ssbuf[3])+(((uint16_t) ssbuf[4]) << 8)); // get 12-bit X result
                                y_tmp = (((uint16_t) ssbuf[5])+(((uint16_t) ssbuf[6]) << 8)); // get 12-bit Y result
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
                            }
                        }
                        break;
                    default:
                        break;
                }
                sum += c;
                S.DATA2 = true; // usart is connected to data
                if (idx >= BUF_SIZE) {
                    idx = 0; // just in case i is greater than buffer size
                    S.CATCH = false;
                    S.TOUCH = false;
                    S.UNTOUCH = false;
                }
            }
        }
    }

    /*
     * emulation of E281A-4002 Binary (Z=1-255 on touch, Z=0 on untouch)
     */
    if (emulat_type == VIISION) {
        if (screen_type == DELL_E215546) { // IntelliTouch smart to binary
            c = UART2_Read(); // read data from touchscreen
            status.status_count++;

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
                        status.id++;
                        break;
                    }
                    if (ssbuf[1] == 'T') {
                        status.restart_delay = 0;
                        if (ssbuf[2] & 0x80) { // check for accutouch format screen
                            xy_flip = true;
                        } else {
                            xy_flip = false; // DON'T FLIP with accutouch
                            xs_ss = X_SCALE_SS_A;
                            ys_ss = Y_SCALE_SS_A;
                        }
                        // Touch
                        ssreport.x_cord = (((uint16_t) ssbuf[3])+(((uint16_t) ssbuf[4]) << 8)); // get 12-bit X result
                        ssreport.y_cord = (((uint16_t) ssbuf[5])+(((uint16_t) ssbuf[6]) << 8)); // get 12-bit Y result
                        S.TOUCH = true; // first touch sequence has been sent
                        x_tmp = ssreport.x_cord;
                        y_tmp = ssreport.y_cord;
                        if (xy_flip) {
                            x_tmp = ELO_REV_H - x_tmp; // FLIP X
                            y_tmp = ELO_REV_H - y_tmp; // FLIP Y
                            offset = 0; // fix for accutouch screen offsets
                        }
                        x_tmp = (uint16_t) ((float) x_tmp * (float) xs_ss); // X rescale range
                        y_tmp = (uint16_t) ((float) y_tmp * (float) ys_ss); // Y rescale
                        x_tmp = (x_tmp >> (uint16_t) 4) - offset; // rescale x to 8-bit value
                        y_tmp = (y_tmp >> (uint16_t) 4) - offset; // rescale y
                        elobuf_in[1] = (uint8_t) x_tmp; // X to 8-bit var
                        elobuf_in[2] = (uint8_t) y_tmp; // Y
                        elobuf_out[0] = 0xc0 + ((elobuf_in[1]&0xc0) >> (uint8_t) 6); // stuff into special Varian binary 4002 format
                        elobuf_out[1] = 0x80 + (elobuf_in[1]&0x3f);
                        elobuf_out[2] = 0x40 + ((elobuf_in[2]&0xc0) >> (uint8_t) 6);
                        elobuf_out[3] = 0x00 + (elobuf_in[2]&0x3f);
                        elobuf_out[4] = 0x00;
                        elobuf_out[5] = 0xff;

                        if ((ssbuf[2] | 0x80) == ELO_UT) { // Untouch
                            S.UNTOUCH = true; // untouch sequence found
                            elobuf_out[0] = 0xc0; // restuff the buffer with needed varian untouch sequence
                            elobuf_out[1] = 0x80;
                            elobuf_out[2] = 0x40;
                            elobuf_out[3] = 0x00;
                            elobuf_out[4] = 0x00;
                            elobuf_out[5] = 0x00;
                        }
                        if (S.TOUCH || S.UNTOUCH) { // send both

                            for (int e = 0; e < HOST_CMD_SIZE_V80; e++) { // send buffered data
                                putc1(elobuf_out[e]); // send to host
                            }
                            status.touch_count++;

                            S.LCD_OK = true; // looks like a screen controller is connected
                            S.SCREEN_INIT = false; // command code has been received by lcd controller
                            status.init_check = 0; // reset init code timer

                            if (S.UNTOUCH) { // After untouch is sent dump buffer and clear all.
                                S.TOUCH = false;
                                S.UNTOUCH = false;
                                S.CATCH = false;
                                // Write to the Timer0 register
                                TMR0_Reload();
                            }
                            RLED_Toggle();
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
                        }
                    }
                    break;
                default:
                    break;
            }
            sum += c;
            S.DATA2 = true; // usart is connected to data

            if (idx >= BUF_SIZE) {
                idx = 0; // just in case i is greater than buffer size
                S.CATCH = false;
                S.TOUCH = false;
                S.UNTOUCH = false;
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
static void uart1risr(void) {
    uint8_t tchar;

    tchar = U1RXB; // read from host
    S.DATA1 = true; // usart is connected to data
    if (tchar == (uint8_t) 0x46) { // send one report to host
        S.CATCH46 = true;
    }
    if (tchar == (uint8_t) 0x37) { // start of touch scan read
        S.CATCH37 = true;
    }
    if (tchar == (uint8_t) 0x3C) { // touch reset from host
        // a possible setup command
    }
};

/*
 * check timer0 irq 1 second time
 */
static void tmr0isr(void) {
    //check for TMR0 overflow
    idx = 0; // reset packet char index counter
    status.tohost = false; // when packets stop allow for next updates
    if (S.TEST_MODE) {
        TMR0_WriteTimer(0x01ff);
    }
    RLED_Toggle();

    if (S.LCD_OK) {
        DB1_SetHigh();
    } else {
        if ((status.init_check++ >LCD_CHK_TIME)) {
            status.init_check = 0; // reset screen init code counter
            S.SCREEN_INIT = true; // set init code flag so it can be sent in main loop
            DB0_SetHigh();
        }
    }

    if ((status.comm_check++ >COMM_CHK_TIME) && !S.CATCH) { // check for LCD screen connection
        status.comm_check = 0; // reset connect heartbeat counter
        S.LCD_OK = false; // reset the connect flag while waiting for response from controller.
        DB1_SetHigh();
    }
}

/*
 * check timer1 irq 500us time
 * update ticks
 */
static void delayisr(void) {
    status.ticks++;
}

uint32_t get_ticks(void) {
    static uint32_t tmp = 0;

    PIE3bits.TMR1IE = 0;
    tmp = (uint32_t) status.ticks;
    PIE3bits.TMR1IE = 1;
    return tmp;
}

void clear_ticks(void) {
    PIE3bits.TMR1IE = 0;
    status.ticks = 0;
    PIE3bits.TMR1IE = 1;
}

void touch_cam(void) {
    //	check for corner presses
    if (S.CATCH) {
        if ((elobuf[0] <= (uint8_t) 0x06) && (elobuf[1] >= (uint8_t) 0x5a)) { // check for left bottom corner
            touch_corner1++;
            touch_corner_timed = true;
        };

        if ((elobuf[0] >= (uint8_t) 0x72) && (elobuf[1] >= (uint8_t) 0x5a)) { // check for right bottom corner
            touch_corner1++;
        };
    };

    if (touch_corner1 >= MAX_CAM_TOUCH) { // we have several corner presses
        S.CAM = true;
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
void wdtdelay(const uint32_t delay) {
    uint32_t dcount;

    for (dcount = 0; dcount <= delay; dcount++) { // delay a bit
        ClrWdt(); // reset the WDT timer
    };
}

void eloSScmdout(const uint8_t elostr) {
    putc2(elostr);
    wdtdelay(2000); // inter char delay
}

void elopacketout(uint8_t *strptr, uint8_t strcount, uint8_t slow) {
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

void setup_lcd_e220(void) {
    if (screen_type == DELL_E215546) {
        elopacketout(elocodes_e5, ELO_SEQ, 0); // ask for ID
        wdtdelay(100000);
        elopacketout(elocodes_e3, ELO_SEQ, 0); // reset to default smartset
        wdtdelay(1000000); // wait for LCD touch controller reset
        elopacketout(elocodes_e0, ELO_SEQ, 0); // set touch packet spacing and timing
        elopacketout(elocodes_e2, ELO_SEQ, 0); // nvram save
    }
}

void setup_lcd_v80(void) {
    if (screen_type == DELL_E215546) {
        elopacketout(elocodes_e5, ELO_SEQ, 0); // ask for ID
        wdtdelay(100000);
        elopacketout(elocodes_e3, ELO_SEQ, 0); // reset;
        wdtdelay(1000000); // wait for LCD touch controller reset
        /* program the display */
        elopacketout((uint8_t*) & elocodes[0][0], ELO_SEQ, 0); // initial touch,stream Point,untouch,Z-axis,no scaling, tracking
        elopacketout((uint8_t*) & elocodes[1][0], ELO_SEQ, 0); // packet delays to match old terminal
        elopacketout((uint8_t*) & elocodes[2][0], ELO_SEQ, 0); // nvram save
    }
}

void putc1(const uint8_t c) {
    UART1_Write(c);
}

void putc2(const uint8_t c) {
    UART2_Write(c);
}

void start_delay(void) {
    wdtdelay(100000);
}

uint8_t Test_Screen(void) {
    putc2(0x46);
    wdtdelay(30000);
    setup_lcd_e220(); // send lcd touch controller setup codes
    return S.DATA2;
}

void main(void) {
    uint8_t z, check_byte, ts_type = TS_TYPE;
    uint16_t update_screen = 0;
    uint8_t scaled_char;
    float rez_scale_h = 1.0, rez_parm_h, rez_scale_v = 1.0, rez_parm_v;
    float rez_scale_h_ss = ELO_SS_H_SCALE, rez_scale_v_ss = ELO_SS_V_SCALE;
    bool TX1INV = false;

    SYSTEM_Initialize();
    //    CRC_StartScanner();

    /*
     * default state is ON
     */
    RLED_SetLow();
    MLED_SetLow();

    status.do_cap = DO_CAP;
    S.c_idx = 0;

    // default interface
    screen_type = DELL_E215546;
    emulat_type = E220;
    z = 0b11111101;
    sprintf(opbuffer, "E220 E215546");

    /*
     * set touchscreen emulation type code
     */

    check_byte = DATAEE_ReadByte((uint16_t) 0x380000);

    if (check_byte != 0x57) { // invalid eeprom display data
        DATAEE_WriteByte((uint16_t) 0x380000, 0x57);
        DATAEE_WriteByte((uint16_t) 0x380001, z);
    }

    check_byte = DATAEE_ReadByte((uint16_t) 0x380000);
    if (check_byte == 0x57) { // change config from default settings if needed
        z = DATAEE_ReadByte((uint16_t) 0x380001);
        if (z == 0b11111110 || (!MIN0_GetValue())) {
            screen_type = DELL_E215546;
            emulat_type = VIISION;
            z = 0b11111110;
            DATAEE_WriteByte((uint16_t) 0x380001, z);
            sprintf(opbuffer, "VIISION E215546");
            if (!MIN1_GetValue()) { // check for double jumper to start test-mode
                S.TEST_MODE = true;
                ssreport.id_type = 0x32;
            }
            MLED_SetHigh();
        }
        if (z == 0b11111101 || (!MIN1_GetValue() && MIN0_GetValue())) {
            screen_type = DELL_E215546;
            emulat_type = E220;
            z = 0b11111101;
            DATAEE_WriteByte((uint16_t) 0x380001, z);
            sprintf(opbuffer, "E220 E215546  ");
            MLED_SetLow();
        }
        if (z == 0b11111100) {
            screen_type = DELL_E215546;
            emulat_type = OTHER_MECH;
            sprintf(opbuffer, "OTHER E215546 ");
        }
    }

    CAM_RELAY = 0;
    status.touch_count = 0;
    S.CAM = 0;
    status.tohost = true;

    wdtdelay(700000); // wait for LCD controller reset on power up
    TMR0_SetInterruptHandler(tmr0isr);
    TMR1_SetInterruptHandler(delayisr);
    UART1_SetRxInterruptHandler(uart1risr);
#ifdef USE_DMA
    DMA1_SetSCNTIInterruptHandler(clear_lcd_done);
#endif
    // Enable high priority global interrupts
    INTERRUPT_GlobalInterruptHighEnable();
    // Enable low priority global interrupts.
    INTERRUPT_GlobalInterruptLowEnable();
    TMR0_StartTimer();
    TMR1_StartTimer();

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
        setup_lcd_v80();
    }

    if (emulat_type == E220) {
        setup_lcd_e220(); // send lcd touch controller setup codes
    }

    if (emulat_type == E220) {
        S.DATA1 = false; // reset COMM flags.
        S.DATA2 = false; // reset touch COMM flag
        // leds from outputs to ground via resistor.
        while (status.do_cap) {
            ClrWdt();
            if (status.host_write || status.scrn_write) {
            }
        }

        Test_Screen(); // send touch init commands
        /* Loop forever */
        while (true) {
            if (UART2_is_rx_ready()) {
                uart2work();
            }
            if (j++ >= (BLINK_RATE_E220)) { // delay a bit ok
#ifdef DEBUG_CAM
                CAM_RELAY = !CAM_RELAY;
#endif
                if (status.cam_time > MAX_CAM_TIMEOUT) {
                    if (touch_corner_timed) {
                        touch_corner_timed = false;
                        CAM_RELAY = 0; // clear video switch
                        S.CAM = false;
                    }
                }
                status.cam_time++;

                /*		For the auto-restart switch						*/
                //FIXME
                if (AUTO_RESTART) { // enable auto-restarts
                    if ((status.restart_delay++ >= (uint16_t) 60) && (!S.TSTATUS)) { // try and reinit lcd after delay
                        start_delay();
                        setup_lcd_e220(); // send lcd touch controller setup codes
                        start_delay();
                        while (true) {
                        }; // lockup WDT counter to restart
                    } else {
                        if ((status.restart_delay >= (uint16_t) 150) && (S.TSTATUS)) { // after delay restart TS status.
                            S.TSTATUS = false; // lost comms while connected
                            status.restart_delay = 0;
                        };
                    };
                }
                j = 0;
                if (update_screen++ >= SCREEN_UPDATE) {
                    update_screen = 0;
                    sprintf(buffer, "E %lu %lu %lu %2.2x %u.%u %2.2x ", status.id, status.status_count, status.touch_count, ssreport.id_type, ssreport.id_major, ssreport.id_minor, ssreport.id_class);
                    eaDogM_WriteStringAtPos(0, 0, buffer);
                    if (ssreport.id_type == 0) {
                        sprintf(opbuffer, "E220 DEFAULT ");
                    }
                    if (ssreport.id_type == 0x32) {
                        sprintf(opbuffer, "E220 E215546");
                    }
                    if (ssreport.id_type == 0x33) {
                        sprintf(opbuffer, "E220 E224864");
                    }
                    eaDogM_WriteStringAtPos(3, 0, opbuffer);
                    sprintf(buffer, "T-R: X%4u Y%4u           ", x_tmp, y_tmp);
                    eaDogM_WriteStringAtPos(1, 0, buffer);
                    sprintf(buffer, "T-S: X%4u Y%4u           ", ssreport.x_cord, ssreport.y_cord);
                    eaDogM_WriteStringAtPos(2, 0, buffer);
                    MISC_Toggle();
                }
            }

            touch_cam(); // always check the cam touch

            if (S.CATCH46) { // flag to send report to host
                if (S.CATCH) { // send the buffered touch report
                    __delay_ms(75);
                    putc1(0xFE); // send position report header to host
                    if (screen_type == DELL_E215546) {
                        status.tohost = true;
                        rez_parm_h = ((float) (ssreport.x_cord)) * rez_scale_h_ss;
                        rez_parm_v = ((float) (ssreport.y_cord)) * rez_scale_v_ss;
                        status.tohost = false;
                        scaled_char = ((uint8_t) (rez_parm_h));
                        elobuf[0] = scaled_char;
                        putc1(scaled_char); // send h scaled touch coord
                        scaled_char = ((uint8_t) (rez_parm_v));
                        elobuf[1] = scaled_char;
                        putc1(scaled_char); // send v scaled touch coord
                        RLED_Toggle();
                    }
                    putc1(0xFF); // send end of report to host
                    status.touch_count++;
                    S.CATCH = false;
                    S.CATCH46 = false;
                } else { // just send status
                    __delay_ms(65);
                    putc1(0xF5); // send status report
                    putc1(0xFF); // end of report
                    status.status_count++;
                    S.CATCH46 = false;
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
                S.CATCH37 = false;
            };

            if (S.TOUCH) {
                // do nothing now.
            };

            ClrWdt(); // reset the WDT timer
        }
    }

    if (emulat_type == VIISION) {
        setup_lcd_v80();
        RELAY_SetHigh();
        RELAY_SetDigitalInput();
        RELAY_SetPullup();
        if (RELAY_GetValue() == 0) { // we have a jumper on the RELAY output line to round, invert UART1 TX line
            U1CON2 = 0x04; // TX1 inverted
            TX1INV = true;
        }
        RELAY_SetLow(); // pull the RELAY line to zero
        RELAY_SetDigitalOutput();

        /* Loop forever */
        while (true) { // busy loop
            if (S.TEST_MODE && (get_ticks() > TEST_UPDATE)) {
                uart_stuff(TUL);
                clear_ticks();
            }
            if (UART2_is_rx_ready()) {
                uart2work();
            }
            if (j++ >= BLINK_RATE_V80) { // delay a bit ok
                j = 0;
                if (update_screen++ >= SCREEN_UPDATE) {
                    //                    if (!CRC_IsCrcBusy()) {
                    //                        status.crc = (uint32_t) CRCOUTL;
                    //                    }
                    update_screen = 0;
                    if (!TX1INV) {
                        sprintf(buffer, "E %lu %lu %lu %2.2x %u.%u %2.2x ", status.id, status.status_count, status.touch_count, ssreport.id_type, ssreport.id_major, ssreport.id_minor, ssreport.id_class);
                    } else {
                        sprintf(buffer, "I %lu %lu %lu %2.2x %u.%u %2.2x ", status.id, status.status_count, status.touch_count, ssreport.id_type, ssreport.id_major, ssreport.id_minor, ssreport.id_class);
                    }
                    eaDogM_WriteStringAtPos(0, 0, buffer);
                    if (ssreport.id_type == 0) {
                        sprintf(opbuffer, "VIISION DEFAULT ");
                    }
                    // 0x30 ID code for accutouch screen
                    if (ssreport.id_type == 0x32) {
                        sprintf(opbuffer, "VIISION E215546 ");
                    }
                    if (ssreport.id_type == 0x33) {
                        sprintf(opbuffer, "VIISION E224864 ");
                    }
                    eaDogM_WriteStringAtPos(3, 0, opbuffer);
                    sprintf(buffer, "T-R: X%4u Y%4u           ", ssreport.x_cord, ssreport.y_cord);
                    eaDogM_WriteStringAtPos(1, 0, buffer);
                    sprintf(buffer, "T-S: X%4u Y%4u           ", x_tmp, y_tmp);
                    eaDogM_WriteStringAtPos(2, 0, buffer);
                    MISC_Toggle();
                }
            }
            ClrWdt(); // reset the WDT timer
        }
    }

    if (emulat_type == OTHER_MECH) {
        /* Loop forever */
        while (true) { // busy loop BSG style
            if (j++ >= BLINK_RATE_OTHER) { // delay a bit ok
                j = 0;
            }
            ClrWdt(); // reset the WDT timer
        }
    }
}
