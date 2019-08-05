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
void sparkelPattern(uint32_t counter);
void fadeOff(uint32_t counter);
void brightnessTestPattern(uint32_t counter);

void fade_all_and_set(uint8_t amount);

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
	//setLed(i, gamma[c.r], gamma[c.g], gamma[c.b]);
	setLed(i, c.r, c.g, c.b);
}


void initPatterns(){
	SysTick_Config(SystemCoreClock / 1000U);

}
/*
 *       off_mode_num = 0
        rainbow_mode_num = 1
        sparkle_mode_num = 2
        theater_mode_num = 3
        aux_mode_num = 4
        mic_mode_num = 5
        test_brightness_mode_num = 6
        test_color_mode_num = 7
        test_write_speed = 8
 */

// Patterns
#define num_patterns 9
PatternFunc patterns[] = {
		rainbowPattern,
		rainbowPattern,
		sparkelPattern,
		fadeOff,
		spectrumBrightness,
		fadeOff,
		brightnessTestPattern,
		fadeOff,
		fadeOff,
};

RgbColor brightness_currentRgb;
void brightnessTestPattern(uint32_t counter){
	int i;

	brightness_currentRgb.r = counter % 256;
	brightness_currentRgb.g = counter % 256;
	brightness_currentRgb.b = counter % 256;

	for(int i=0;i<NUM_LEDS;i++){
		setLedRgb(i, brightness_currentRgb);
	}

	if(counter % 20 == 0 || counter % 255 == 0){
		for(i =0; i< 10000000; i++){

		}
			i = 0;
	}

}



void fadeOff(uint32_t counter){
	if(counter % 20 == 0){
		fade_all_and_set(1);
	}
}

void runPatterns(){
	uint8_t *mode = getMode();
	while(1){
		//rainbowPattern(clockCounter);
		patterns[bound_uint8(*mode, 0, num_patterns-1)](clockCounter);
		SysTick_DelayTicks(10U); //delay 10 ms
		clockCounter += 1;
	}
}


//Pattern rainbow = { 10, rainbowPattern };

void rainbowPattern( uint32_t count){
	hsv.h = count % 255;
	hsv.v = 50;
	hsv.s = 255;
	for(int i=0;i<(68*7) ;i++){
		setLedHsv(i, hsv);
		hsv.h += 5 % 255;
	}
}

void fade_all_and_set(uint8_t amount){
	for(int i=0; i<NUM_LEDS; i++){
		if(amount >= currentLedsHsv[i].v){
			 currentLedsHsv[i].v = 0;
		}else{
			 currentLedsHsv[i].v -= amount;
		}
		setLedHsv(i, currentLedsHsv[i]);
	}
}


HsvColor sparkle_currentHsv = {0, 255, 0};
void sparkelPattern(uint32_t counter){
	for(int i=0; i < (counter % 10); i++){
		sparkle_currentHsv.v = get_rand_uint8_range(100, 200);
		sparkle_currentHsv.h = get_rand_uint8_range(0, 255);

		setLedHsv(get_rand_uint32_range(0, NUM_LEDS), sparkle_currentHsv);
	}
	fade_all_and_set(5);
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
		//enhansedSpectrum[i*2+1] = spectrumBuffer[currentBufferIndex] [i]/2 ;
		if(i != 0){
			//enhansedSpectrum[i*2-1] += spectrumBuffer[currentBufferIndex] [i]/2;
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
				decay = (currentEnhansedSpectrum[i] * (120 - map_uint32(currentEnhansedSpectrum[i], 0, 4096, 20, 120))) / 1000;
				if(decay > currentEnhansedSpectrum[i]){
					currentEnhansedSpectrum[i] = 0;
				}else{
					currentEnhansedSpectrum[i] -= decay;
				}

			}
		}
		//hsv.v = currentEnhansedSpectrum[i] >> 4U;

		currentIndex = 4 + i*5;
		hsv.v = currentEnhansedSpectrum[i] >> 4U; // spectrum is 0-4095, so to map it to 0-255 shift it over 4 (divide by 16)
//		if(hsv.v < 50){
//			hsv.v = 50;
//		}
		hsv.h = 255 - (255 * i / 13);
		setLedHsv(currentIndex, hsv);
		hsv.v = hsv.v >> 1;
		setLedHsv(currentIndex-1, hsv);
		setLedHsv(currentIndex+1, hsv);
//		hsv.v = hsv.v >> 2;
//		setLedHsv(currentIndex-2, hsv);
//		setLedHsv(currentIndex+2, hsv);
//		hsv.v = hsv.v >> 1;
//		setLedHsv(currentIndex-3, hsv);
//		setLedHsv(currentIndex+3, hsv);
	}
}

void spectrumBrightness(uint32_t counter){
	readMsgeq07(spectrumBrightnessCallback);
}


void secondTrySpectrum(uint32_t counter){

}






