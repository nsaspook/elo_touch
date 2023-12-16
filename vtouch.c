/*
 * 
 */

#pragma warning disable 520
#pragma warning disable 1498
#pragma warning disable 1090

#include <xc.h>
#include <stdlib.h>
#include "vtouch.h"
#include "vtouch_build.h"
#include "eadog.h"

const char *build_date = __DATE__, *build_time = __TIME__;

char buffer[512], opbuffer[256];
uint32_t count = 0;

volatile bool BUTTON_PRESSED = false, BLINK = false;
volatile unsigned long button_intcount, sample_intcount, time_intcount, sw_intcount;

void Led_Blink(void);
void SW_CHECK(void);
int sw_work(void);

void wdtdelay(const uint32_t delay)
{
	uint32_t dcount;
	for (dcount = 0; dcount <= delay; dcount++) { // delay a bit
		ClrWdt(); // reset the WDT timer
	};
}

void start_delay(void)
{
	wdtdelay(100000);
}

void main(void)
{
	SYSTEM_Initialize();

	LED1 = LEDON; // light all switch leds
	LED2 = LEDON;
	LED3 = LEDON;
	LED4 = LEDON;

	wdtdelay(700000); // wait for LCD controller reset on power up
	TMR0_SetInterruptHandler(Led_Blink);
	TMR5_SetInterruptHandler(SW_CHECK);

	// Enable high priority global interrupts
	INTERRUPT_GlobalInterruptHighEnable();

	// Enable low priority global interrupts.
	//	INTERRUPT_GlobalInterruptLowEnable();

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

	while (true) {
		sw_work();
	};
}

void Led_Blink(void)
{
	time_intcount++;
	BLED_Toggle();

	if (BLINK) {
		if (SW1 == 0) {
			if (time_intcount % 2) {
				LED1 = LEDON;
			} else {
				LED1 = LEDOFF;
			}
		}
		if (SW2 == 0) {
			if (time_intcount % 2) {
				LED2 = LEDON;
			} else {
				LED2 = LEDOFF;
			}
		}
		if (SW3 == 0) {
			if (time_intcount % 2) {
				LED3 = LEDON;
			} else {
				LED3 = LEDOFF;
			}
		}
		if (SW4 == 0) {
			if (time_intcount % 2) {
				LED4 = LEDON;
			} else {
				LED4 = LEDOFF;
			}
		}
	} else {
		LED1 = LEDON;
		LED2 = LEDON;
		LED3 = LEDON;
		LED4 = LEDON;
	}
}

void SW_CHECK(void)
{
	static unsigned char debo = DEBOUNCE, bell_count = BELL_TIME;

	// check for mag switch contact
	if (SW1 && SW2 && SW3 && SW4) { // screen status feedback
		BUTTON_PRESSED = false;
		BLINK = false;
		debo = 0;
		BELL = false;
		bell_count = 0;
	} else {
		if (debo++ > DEBOUNCE) {
			sw_intcount++;
			// (switch debounced)
			if (bell_count++ < BELL_TIME) { // Sonalert for audio feedback
				BELL = true;
			} else {
				BELL = false;
				bell_count = BELL_TIME;
			}
			BUTTON_PRESSED = true;
			BLINK = true;
			debo = DEBOUNCE;
		}
	}
}

int sw_work(void)
{
	static bool ONCE = true;
	static uint8_t times = 0;

	if (BUTTON_PRESSED) {
		if (SW1 == 0) {
			if (ONCE) {
				LED1 = LEDOFF;
				if (UART1_is_tx_ready()) {
					/*
					 * format data to JSON
					 */
					snprintf(buffer, 510, "{\r\n     \"Qname\": \"%s\",\r\n     \"Qsequence\": %ld,\r\n     \"Qbuild_date\": \"%s\",\r\n     \"Qbuild_time\": \"%s\"\r\n}",
						build_version, count++, build_date, build_time);
					//					snprintf(buffer,510,"fred %u",times++);
					/*
					 * STDIO redirected to UART1
					 */
					printf("%s", buffer);
				}
			}
			ONCE = false;
		} else {
			LED1 = LEDON;
		}
		if (SW2 == 0) {
			if (ONCE) {
				LED2 = LEDOFF;
				if (UART1_is_tx_ready()) {
					/*
					 * format data to JSON
					 */
					snprintf(buffer, 510, "{\r\n     \"Qname\": \"%s\",\r\n     \"Qsequence\": %ld,\r\n     \"Qbuild_date\": \"%s\",\r\n     \"Qbuild_time\": \"%s\"\r\n}",
						build_version, count++, build_date, build_time);
					/*
					 * STDIO redirected to UART1
					 */
					printf("%s", buffer);
				}
			}
			ONCE = false;
		} else {
			LED2 = LEDON;
		}
		if (SW3 == 0) {
			if (ONCE) {
				LED3 = LEDOFF;

			}
			ONCE = false;
		} else {
			LED3 = LEDON;
		}
		if (SW4 == 0) {
			if (ONCE) {
				LED4 = LEDOFF;

			}
			ONCE = false;
		} else {
			LED4 = LEDON;
		}
	} else {
		ONCE = true;

	}
	return 0;
}
