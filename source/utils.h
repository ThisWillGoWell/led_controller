/*
 * utils.h
 *
 *  Created on: Jun 21, 2019
 *      Author: wggowell
 */

#ifndef UTILS_H_
#define UTILS_H_
#include "stdint.h"

uint32_t map_uint32(uint32_t input,uint32_t in_min, uint32_t in_max, uint32_t out_min, uint32_t out_max);
uint8_t bound_uint8(uint8_t input, uint8_t min, uint8_t max);

#endif /* UTILS_H_ */
