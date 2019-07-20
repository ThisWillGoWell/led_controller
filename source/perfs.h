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

uint8_t* getMode();

uint32_t get_rand_uint32();
uint32_t get_rand_uint32_range(uint32_t start, uint32_t end);
uint8_t get_rand_uint8();
uint32_t get_rand_uint8_range(uint8_t start, uint8_t end);

#endif /* PERFS_H_ */
