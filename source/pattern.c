/*
 * pattern.c
 *
 *  Created on: Jun 17, 2019
 *      Author: wggowell
 */


#include "pattern.h"
#include "colors.h"
#include "perfs.h"
#include "utils.h"
#include "clock_config.h"

#define DELAY 10U
#define UPDATES_PER_SECOND 1000U / DELAY
#define AUDIO_DELAY 0.5

#define AUDIO_DELAY_BUFFER_LEN 1


/*****************************************************************************
 * Helper Prototypes
*****************************************************************************/
void setLedHsv(uint32_t i, HsvColor c);
void setLedRgb(uint32_t i, RgbColor c);

/*****************************************************************************
 * Patterns
*****************************************************************************/
void rainbowPattern(uint32_t count);
void fade(uint32_t count );
void randomLight(uint32_t count);
void spectrumBrightness(uint32_t counter);
void secondTrySpectrum(uint32_t counter);

uint32_t currentDelay;
uint8_t brightness;

/*****************************************************************************
 * variables
*****************************************************************************/
uint8_t gamma[256] = {
		0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
		0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
		1, 2, 2, 2, 2, 2, 2, 2, 2, 3, 3, 3, 3, 3, 4, 4,
		4, 4, 4, 5, 5, 5, 5, 6, 6, 6, 6, 7, 7, 7, 7, 8,
		8, 8, 9, 9, 9, 10, 10, 10, 11, 11, 12, 12, 12, 13, 13, 14,
		14, 15, 15, 15, 16, 16, 17, 17, 18, 18, 19, 19, 20, 20, 21, 22,
		22, 23, 23, 24, 25, 25, 26, 26, 27, 28, 28, 29, 30, 30, 31, 32,
		33, 33, 34, 35, 36, 36, 37, 38, 39, 40, 40, 41, 42, 43, 44, 45,
		46, 46, 47, 48, 49, 50, 51, 52, 53, 54, 55, 56, 57, 58, 59, 60,
		61, 62, 63, 64, 65, 67, 68, 69, 70, 71, 72, 73, 75, 76, 77, 78,
		80, 81, 82, 83, 85, 86, 87, 89, 90, 91, 93, 94, 95, 97, 98, 99,
		101, 102, 104, 105, 107, 108, 110, 111, 113, 114, 116, 117, 119, 121, 122, 124,
		125, 127, 129, 130, 132, 134, 135, 137, 139, 141, 142, 144, 146, 148, 150, 151,
		153, 155, 157, 159, 161, 163, 165, 166, 168, 170, 172, 174, 176, 178, 180, 182,
		184, 186, 189, 191, 193, 195, 197, 199, 201, 204, 206, 208, 210, 212, 215, 217,
		219, 221, 224, 226, 228, 231, 233, 235, 238, 240, 243, 245, 248, 250, 253, 255
	};

HsvColor hsv;
uint32_t clockCounter;
uint32_t g_systickCounter;

RgbColor currentLedsRgb[NUM_LEDS];
HsvColor currentLedsHsv[NUM_LEDS];

/*****************************************************************************
 * Helper Functions
*****************************************************************************/
void SysTick_Handler(void)
{
	if (g_systickCounter != 0U)
	{
		g_systickCounter--;
	}

}

void SysTick_DelayTicks(uint32_t n)
{
    g_systickCounter = n;
    while(g_systickCounter != 0U)
    {
    }
}

void setLedHsv(uint32_t i, HsvColor c){
	currentLedsHsv[i] = c;
	setLedRgb(i, HsvToRgb(c));
}
void setLedRgb(uint32_t i, RgbColor c){
	currentLedsRgb[i]  = c;
	setLed(i, gamma[c.r], gamma[c.g], gamma[c.b]);
}


void initPatterns(){
	SysTick_Config(SystemCoreClock / 1000U);

}

void runPatterns(){
	while(1){
		//rainbowPattern(clockCounter);
		spectrumBrightness(clockCounter);
		SysTick_DelayTicks(10U); //delay 10 ms
		clockCounter += 1;
	}
}


//Pattern rainbow = { 10, rainbowPattern };

void rainbowPattern( uint32_t count){
	hsv.h = count % 255;
	hsv.v = 100;
	hsv.s = 255;
	for(int i=0;i<NUM_LEDS;i++){
		setLedHsv(i, hsv);
		hsv.h += 5 % 255;
	}
}

uint32_t currentSpectrum[7] = {0};
uint32_t enhansedSpectrum[13] = {0};
uint32_t currentEnhansedSpectrum[13] = {0};

uint32_t currentValue = 0;
HsvColor currentHsv;
int currentIndex;

int currentBufferIndex = 0;

uint32_t spectrumBuffer[AUDIO_DELAY_BUFFER_LEN][7];

void spectrumBrightnessCallback(uint32_t spectrum[]){

	int i;
	uint32_t decay;
	for(i = 0; i<6;i++){
		if(spectrumBuffer[currentBufferIndex] [i] < 800){
			spectrumBuffer[currentBufferIndex] [i] = 0;
		}
		enhansedSpectrum[i*2]= spectrumBuffer[currentBufferIndex] [i];
		enhansedSpectrum[i*2+1] = spectrumBuffer[currentBufferIndex] [i]/2 ;
		if(i != 0){
			enhansedSpectrum[i*2-1] += spectrumBuffer[currentBufferIndex] [i]/2;
		}
		spectrumBuffer[currentBufferIndex][i] = spectrum[i];
	}
	currentBufferIndex = (currentBufferIndex + 1) % AUDIO_DELAY_BUFFER_LEN;

	hsv.s = 255;
	for(i=0; i<13;i++){
		if(enhansedSpectrum[i] > currentEnhansedSpectrum[i]){
			currentEnhansedSpectrum[i] = enhansedSpectrum[i];
		} else {
			// current specturm = current specturm [99-90]%
			// the higher the number, the slower the decay
			if(currentEnhansedSpectrum[i] != 0){
				decay = (currentEnhansedSpectrum[i] * (100 - map_uint32(currentEnhansedSpectrum[i], 0, 4096, 30, 100))) / 1000;
				if(decay > currentEnhansedSpectrum[i]){
					currentEnhansedSpectrum[i] = 0;
				}else{
					currentEnhansedSpectrum[i] -= decay;
				}

			}
		}
		//hsv.v = currentEnhansedSpectrum[i] >> 4U;

		currentIndex = 100 + i * 7 + 2;
		hsv.v = currentEnhansedSpectrum[i] >> 4U; // spectrum is 0-4095, so to map it to 0-255 shift it over 4 (divide by 16)
		if(hsv.v < 50){
			hsv.v = 50;
		}
		hsv.h = 255 - (255 * i / 13);
		setLedHsv(currentIndex, hsv);
		setLedHsv(currentIndex-1, hsv);
		setLedHsv(currentIndex+1, hsv);
		hsv.v = hsv.v >> 1;
		setLedHsv(currentIndex-2, hsv);
		setLedHsv(currentIndex+2, hsv);
		hsv.v = hsv.v >> 1;
		setLedHsv(currentIndex-3, hsv);
		setLedHsv(currentIndex+3, hsv);
	}
}

void spectrumBrightness(uint32_t counter){
	readMsgeq07(spectrumBrightnessCallback);
}


void secondTrySpectrum(uint32_t counter){

}

