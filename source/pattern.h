/*
 * pattern.h
 *
 *  Created on: Jun 17, 2019
 *      Author: wggowell
 */

#ifndef PATTERN_H_
#define PATTERN_H_
#include "stdint.h"

	void runPatterns(void);
	void initPatterns(void);

	typedef void (*PatternFunc)(uint32_t counter);

	typedef struct Pattern
	{
		uint32_t defaultMsDealy;
		uint32_t counter;
		PatternFunc func;
	} Pattern;

#endif /* PATTERN_H_ */
