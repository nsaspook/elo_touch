/*
 * File:   vtouch.h
 * Author: root
 *
 * Created on September 20, 2017, 12:27 PM
 */

#ifndef VTOUCH_H
#define	VTOUCH_H

#ifdef	__cplusplus
extern "C" {
#endif

#define	DO_CAP      FALSE       // E220 save data for usarts 1&2, save to eeprom
#define	TS_TYPE     0		// E220 original screen type: 0 for old CRT type screens, 1 for newer Varian LCD screens with Carroll-Touch
#define BUF_SIZE    64
#define	CAP_SIZE    256

#define	CMD_SIZE    2
#define	CMD_OVERFLOW	CMD_SIZE*12
#define ELO_SIZE    12
#define ELO_SEQ     10
#define ELO_REV_H	4096
#define ELO_SS_H_SCALE	0.483
#define ELO_SS_V_SCALE	0.380
#define	BLINK_RATE_E220	20000
#define AUTO_RESTART	FALSE
#define SINGLE_TOUCH	FALSE
#define GOOD_MAX        128		// max number of chars from TS without expected frames seen
#define MAX_CAM_TIME	5
#define MAX_CAM_TIMEOUT	30
#define MAX_CAM_TOUCH	5
#define CAM_RELAY       LATAbits.LATA1
#define CAM_RELAY_AUX	LATEbits.LATE1
#define CAM_RELAY_TIME	LATEbits.LATE2
#define	TIMERPACKET     41000

#define BUF_SIZE_V80        16
#define	CMD_SIZE_V80        4		// CT size of command in bytes from touch screen
#define	CMD_SIZE_SS_V80	    6		// E281A-4002 software emulation Binary size of command
#define	HOST_CMD_SIZE_V80   6		// tool command size
#define	CMD_OVERFLOW_V80    HOST_CMD_SIZE_V80*2
#define ELO_SEQ_V80         10		// max smartset sequences
#define ELO_SIZE	    12
#define ELO_SIZE_V80        14		// number of bytes to send from elocodes_s configuration string
#define ELO_SIZE_I_V80      10		// max smartset sequence size
#define FALSE	0
#define TRUE	1
#define	BLINK_RATE_V80	25000		// BSG timing
#define	X_SCALE         1.90		// scaling factor to host screen X logical coords
#define	Y_SCALE         1.75		// scaling factor to host screen Y logical coords
#define	X_SCALE_SS      0.905		// scaling factor to host screen X logical coords
#define	Y_SCALE_SS      0.650		// scaling factor to host screen Y logical coords
#define	X_LOGICAL       119		// LCD touchscreen logical X frame coords
#define	Y_LOGICAL       94		// LCD touchscreen logical Y frame coords
#define	X_TOOL          202
#define	Y_TOOL          164

#define	BLINK_RATE_OTHER	15000	// BSG timing

#define	TIMEROFFSET	26474		// timer0 16bit counter value for 1 second to overflow
#define	TIMERFAST	58974		// fast flash or testing
#define	COMM_CHK_TIME	30		// LCD comm heartbeat
#define	LCD_CHK_TIME	36		// LCD heartbeat timeout

#ifdef	__cplusplus
}
#endif

#endif	/* VTOUCH_H */

