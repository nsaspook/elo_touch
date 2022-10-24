/*
 * File:   vtouch.h
 * Author: root
 *
 * Created on September 20, 2017, 12:27 PM
 */

#ifndef VTOUCH_H
#define VTOUCH_H

#ifdef __cplusplus
extern "C" {
#endif

#include <xc.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>

#define DO_CAP   false       // E220 save data for usarts 1&2, save to eeprom
#define TS_TYPE   0  // E220 original screen type: 0 for old CRT type screens, 1 for newer Varian LCD screens with Carroll-Touch
#define BUF_SIZE  32
#define CAP_SIZE  8

#define CMD_SIZE  2
#define CMD_OVERFLOW  CMD_SIZE*12
#define ELO_SIZE  12
#define ELO_SEQ   10
#define ELO_TEST  5
#define ELO_IT   0x81 // ELO T response status byte for inital touch, for smart-touch, accu-touch has zero instead of 8
#define ELO_ST   0x82 // ELO T response status byte for steam touch
#define ELO_UT   0x84 // ELO T response status byte for untouch
#define ELO_REV_H  4095
#define ELO_SS_H_SCALE  0.470 // old value 0.483 V4.03
#define ELO_SS_V_SCALE  0.380
#define BLINK_RATE_E220  100
#define BLINK_RATE_V80  100
#define AUTO_RESTART  false
#define GOOD_MAX  128  // max number of chars from TS without expected frames seen
#define MAX_CAM_TIME  5
#define MAX_CAM_TIMEOUT  30
#define MAX_CAM_TOUCH  5
#define CAM_RELAY  RELAY_LAT
#define TIMERPACKET  41000
#define SCREEN_UPDATE  500
#define TEST_UPDATE  66 // 40 min timing value for 20ms per touch string, 666 gives scrolling X,Y display

#define BUF_SIZE_V80  16
#define CMD_SIZE_SS_V80  6  // E281A-4002 software emulation Binary size of command
#define HOST_CMD_SIZE_V80 6  // tool command size
#define CMD_OVERFLOW_V80 HOST_CMD_SIZE_V80*2
#define ELO_SEQ_V80  4  // max smartset sequences
#define X_SCALE   1.90  // scaling factor to host screen X logical coords
#define Y_SCALE   1.75  // scaling factor to host screen Y logical coords
#define X_SCALE_SS  0.905  // scaling factor to host screen X logical coords
#define Y_SCALE_SS  0.650  // scaling factor to host screen Y logical coords
#define X_SCALE_SS_A  1.137  // scaling factor to host screen X logical coords for ACCUTOUCH
#define Y_SCALE_SS_A  0.865  // scaling factor to host screen Y logical coords for ACCUTOUCH
#define X_LOGICAL  119  // LCD touchscreen logical X frame coords
#define Y_LOGICAL  94  // LCD touchscreen logical Y frame coords
#define X_TOOL   202
#define Y_TOOL   164

#define BLINK_RATE_OTHER 15000 // BSG timing

#define TIMEROFFSET  26474  // timer0 16bit counter value for 1 second to overflow
#define TIMERFAST  58974  // fast flash or testing
#define COMM_CHK_TIME  30  // LCD comm heartbeat
#define LCD_CHK_TIME  36  // LCD heartbeat timeout


    char *build_date = __DATE__, *build_time = __TIME__;

    typedef struct reporttype {
        uint8_t headder, status;
        uint16_t x_cord, y_cord, z_cord;
        uint8_t checksum;
        uint8_t id_type, id_io, id_features, id_minor, id_major, id_p, id_class;
    } reporttype;

    typedef struct statustype {
        uint32_t alive_led, touch_count, resync_count, rawint_count, status_count, ticks, id, crc;
        bool host_write;
        bool scrn_write;
        bool do_cap;
        bool tohost;
        uint8_t comm_check, init_check, cam_time;
        uint16_t restart_delay;
    } statustype;

    typedef struct disp_state_t {
        bool CATCH, TOUCH, UNTOUCH, LCD_OK,
        SCREEN_INIT,
        CATCH46, CATCH37, TSTATUS, CAM;
        bool DATA1, DATA2, TEST_MODE;
        uint16_t c_idx;
    } disp_state_t;

    enum screen_type_t {
        DELL_E215546, OTHER_SCREEN
    };

    enum emulat_type_t {
        VIISION, E220, OTHER_MECH
    };

    enum test_type {
        TUL = 0, // touch upper left
        TUR,
        TLL,
        TLR,
        TUT, // touch untouch
    };

#ifdef __cplusplus
}
#endif

#endif /* VTOUCH_H */

