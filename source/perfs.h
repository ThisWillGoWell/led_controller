/*
 * perfs.h
 *
 *  Created on: Jun 21, 2019
 *      Author: wggowell
 */

#ifndef PERFS_H_
#define PERFS_H_

#include "stdint.h"
#include "colors.h"
#define NUM_LEDS_PER_BUS 1800U
#define NUM_LEDS NUM_LEDS_PER_BUS * 3
#define TRANSFER_BAUDRATE 1400000U //1400000U /* Transfer baudrate - 14.5MHz */
#define END_BYTES 0

void initPerfs(void);
void setLed(uint32_t i,uint8_t r, uint8_t g, uint8_t b);

typedef void (*msgeq07_callback)(uint32_t spectrum[]);
void readMsgeq07(msgeq07_callback);




#endif /* PERFS_H_ */
