/*
 * colors.h
 *
 *  Created on: Jun 15, 2019
 *      Author: wggowell
 */

#ifndef COLORS_H_
#define COLORS_H_
#include "stdint.h"

	typedef struct RgbColor
	{
		uint8_t r;
		uint8_t g;
		uint8_t b;
	} RgbColor;

	typedef struct HsvColor
	{
		uint8_t h;
		uint8_t s;
		uint8_t v;
	} HsvColor;

	HsvColor RgbToHsv(RgbColor rgb);
	RgbColor HsvToRgb(HsvColor hsv);


#endif /* COLORS_H_ */
