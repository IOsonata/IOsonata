/*
 * BlueIOICM20948.h
 *
 *  Created on: Jul 25, 2018
 *      Author: hoan
 */

#ifndef __BLUEIOICM20948_H__
#define __BLUEIOICM20948_H__


#ifdef __cplusplus

#include "device_intrf.h"
#include "coredev/timer.h"

extern "C" {
#endif

void ICM20948EnableFeature(uint32_t Feature);

#ifdef __cplusplus
}
#endif

#endif // __BLUEIOMPU9250_H__
