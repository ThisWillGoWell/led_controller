/*
 * utils.c
 *
 *  Created on: Jun 21, 2019
 *      Author: wggowell
 */


#include "utils.h"

uint32_t map_uint32(uint32_t input,uint32_t in_min, uint32_t in_max, uint32_t out_min, uint32_t out_max){
	if(input < in_min) {
		return out_min;
	}
	if(input > in_max){
		return in_min;
	}
	return (input - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

uint8_t bound_uint8(uint8_t input, uint8_t min, uint8_t max){
	if(input > max){
		return max;
	} else if(input < min){
		return min;
	} else{
		return input;
	}
}
