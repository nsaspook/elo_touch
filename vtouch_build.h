/* 
 * File:   vtouch_build.h
 * Author: root
 *
 * Created on September 20, 2017, 12:30 PM
 */

#ifndef VTOUCH_BUILD_H
#define	VTOUCH_BUILD_H

#ifdef	__cplusplus
extern "C" {
#endif

	const rom int8_t *build_date = __DATE__, *build_time = __TIME__,
		build_version[] = " V3.34 8722 Varian VE touch-screen converter. Fred Brooks, Microchip Inc.";

	//				***
	//				E0.94		Clean up the code and comments
	//				E0.95		Add startup delay for remote service terminals
	//				E0.96		status reporting and monitor testing
	//				E0.97		port bit testing
	//				E0.98		fix Cylon led roll.
	//				E0.99		debug 1,2 Screen size code results changed with portD bit 0,1
	//				E1.00
	//				E1.01		debug 8 on single/tracking touch modes portD bit 7
	//				E1.02		debug 7 on flash LCD while processing. portD bit 6
	//				E1.03		code fixes/updates
	//				E1.04		add delay in status/touched host send routines
	//				E1.05		add interlocks for touch input from screen
	//				E1.06		add WDT counter test switch input and checks for valid ts inputs.
	//				E1.07		screen connect restart code via WDT timeout.
	//				E1.08		Learn touches and set with special touch sequence.
	//				E1.09		Code for 2 special touches and remove the delay switch. bit3 learn1, bit2 learn2
	//				E1.10		External output to led/relay on PORTE, RE0,RE7 mirrors HA0 led
	//				E1.11		add JB define for switch board missing.
	//				E1.12-13	fix LCD display
	//				E1.14		fix rs-232 flags
	//				E1.15		Small coding  cleanups
	//				E1.16		remove WDT calls in ISR, check for proper comms with the touch screen and controller.
	//				E1.17		auto init touchscreen code.
	//				E1.18		Code for new LCD screens and debug capture.
	//				E1.19		recode ISR to remove library define functions
	//				E1.20		VGA/CAM switcher code.
	//				E1.21		Timed camera for left press, software smells
	//				E1.22		Support for SmartSet commands on newer touch panels
	//				E1.23		refactor
	//				E1.24		adjust newer screen size for better touch fit
	//				V3.30		converted to unified driver
	//				V3.31		bug fixes, cleanup
	//				V3.32		PORT G jumpers for smartset configuation.
	//				V3.33		display screen/machine config on LATJ with data 8 bit config on PORTB, program structure rewrite
	//				V3.34		subroutine clean up		
	//				***

#ifdef	__cplusplus
}
#endif

#endif	/* VTOUCH_BUILD_H */

