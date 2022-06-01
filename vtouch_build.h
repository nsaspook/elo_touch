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

#include "vtouch_8722.X/mcc_generated_files/mcc.h"
#include "vtouch_8722.X/mcc_generated_files/interrupt_manager.h"
#include "vtouch_8722.X/mcc_generated_files/pin_manager.h"
#include "vtouch_8722.X/mcc_generated_files/memory.h"

	const char build_version[] = "V5.10 Vtouch Q41 ";

	void wdtdelay(const uint32_t);
	void putc1(const uint8_t);
	void putc2(const uint8_t);

#ifdef	__cplusplus
}
#endif

#endif	/* VTOUCH_BUILD_H */

