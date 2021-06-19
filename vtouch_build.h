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

	const char *build_date = __DATE__, *build_time = __TIME__,
		build_version[] = "V4.01 Vtouch Q43 ";

	void wdtdelay(const uint32_t);
	void putc1(const uint8_t);
	void putc2(const uint8_t);

#ifdef	__cplusplus
}
#endif

#endif	/* VTOUCH_BUILD_H */

