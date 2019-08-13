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

#define NUM_TUBES 15
#define LEDS_PER_TUBE 68


/*****************************************************************************
 * Helper Prototypes
*****************************************************************************/
void setLedHsv(int tube, uint32_t i, HsvColor c);
void setLedRgb(int tube, uint32_t i, RgbColor c);

/*****************************************************************************
 * Patterns
*****************************************************************************/
void rainbowPattern(uint32_t count);
void fade(uint32_t count );
void randomLight(uint32_t count);
void steroVuMeter(uint32_t counter);
void monoVuMeter(uint32_t counter);
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

RgbColor currentLedsRgb[NUM_TUBES][LEDS_PER_TUBE];
HsvColor currentLedsHsv[NUM_TUBES][LEDS_PER_TUBE];

/*****************************************************************************
 * Helper Functionsoff
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

void setLedHsv(int tube, uint32_t i, HsvColor hsv){
	if(tube == -1 ){
		tube = i / LEDS_PER_TUBE;
	} else if(tube%2) {
		i = LEDS_PER_TUBE - i - 1;
	}

	currentLedsHsv[tube][i] = hsv;
	RgbColor c = HsvToRgb(hsv);
	i = tube * LEDS_PER_TUBE + i;
	setLed(i, c.r, c.g, c.b);
}

void setLedRgb(int tube, uint32_t i, RgbColor c){
	// odd tubes are reversed indexed
	if(tube%2) {
		i = LEDS_PER_TUBE - i - 1;
	}
	currentLedsRgb[tube][i] = c;
	i = tube * LEDS_PER_TUBE + i;
	//

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
		steroVuMeter,
		fadeOff,
		brightnessTestPattern,
		fadeOff,
		fadeOff,
		monoVuMeter,
};

RgbColor brightness_currentRgb;
void brightnessTestPattern(uint32_t counter){
	int i;
//
//	brightness_currentRgb.r = counter % 256;
//	brightness_currentRgb.g = counter % 256;
//	brightness_currentRgb.b = counter % 256;
//
//	for(int i=0;i<NUM_LEDS;i++){
//		setLedRgb(i, brightness_currentRgb);
//	}
//
//	if(counter % 20 == 0 || counter % 255 == 0){
//		for(i =0; i< 10000000; i++){
//
//		}
//			i = 0;
//	}

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
		SysTick_DelayTicks(20U); //delay 10 ms
		clockCounter += 1;
	}
}


//Pattern rainbow = { 10, rainbowPattern };

void rainbowPattern( uint32_t count){
	uint8_t startHue = count % 255;
	uint8_t downDelta = 255 / NUM_TUBES;
	uint8_t sideDelta = 255 / LEDS_PER_TUBE;


	hsv.v = 20;
	hsv.s = 255;

	for(int t=0;t<NUM_TUBES; t++){
		hsv.h = startHue + (t * downDelta);
		for(int i=0;i<LEDS_PER_TUBE ;i++){
			setLedHsv(t, i, hsv);
			hsv.h += sideDelta;
		}
	}

}

void fade_all_and_set(uint8_t amount){
	HsvColor current;
	for(int tube=0;tube<NUM_TUBES;tube++){

		for(int i=0; i<LEDS_PER_TUBE; i++){
			current = currentLedsHsv[tube][i];
			if(amount >= current.v){
				current.v = 0;
			}else{
				current.v -= amount;
			}
			setLedHsv(tube, i, current);
		}
	}

}

HsvColor sparkle_currentHsv = {0, 255, 0};

void sparkelPattern(uint32_t counter){
	for(int i=0; i < (counter % 10); i++){
		sparkle_currentHsv.v = get_rand_uint8_range(100, 200);
		sparkle_currentHsv.h = get_rand_uint8_range(0, 255);

		setLedHsv(get_rand_uint8_range(0, NUM_TUBES),  get_rand_uint32_range(0, NUM_LEDS), sparkle_currentHsv);
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


void steroVuMeterCallback(uint32_t spectrum[]){
	HsvColor off;

	hsv.v = 20;
	hsv.s = 255;
	hsv.h = 0;

	uint8_t hueDelta = (255 / (NUM_TUBES + 1));
	int middle = LEDS_PER_TUBE / 2;
	int tube = 0;
	for(int freq=0;freq<14;freq+=2){

		uint32_t amountOnLeft = map_uint32(spectrum[freq], 800, 4096, 0, middle -1 );
		uint32_t amountOnRight = map_uint32(spectrum[freq + 1 ], 800, 4096, 0, middle -1 );
		for(int i=0; i<LEDS_PER_TUBE; i++){
			if(i < amountOnLeft || i > LEDS_PER_TUBE - amountOnRight){
				setLedHsv(tube, i, hsv);
				setLedHsv(tube+1, i, hsv);

			} else {
				setLedHsv(tube, i, off);
				setLedHsv(tube+1, i, off);
			}
		}
		tube += 2;
		hsv.h += hueDelta;

	}
}


void steroVuMeter(uint32_t counter){
	readMsgeq07(steroVuMeterCallback);
}

void monoVuMeterCallback(uint32_t spectrum[]){
	HsvColor off;

	hsv.v = 40;
	hsv.s = 255;
	hsv.h = 0;
	int j;
	uint8_t hueDelta = (255 / (NUM_TUBES + 1));
	int middle = LEDS_PER_TUBE / 2;
	int tube = 0;

	//decay whats already there
	fade_all_and_set(4);

	for(int freq=0;freq<14;freq+=2){

		uint32_t amountOn = map_uint32(spectrum[freq], 500, 4096, 0, middle -1 );
		uint32_t amountOnRight = map_uint32(spectrum[freq + 1 ], 500, 4096, 0, middle -1 );
		if(amountOnRight > amountOn){
			amountOn = amountOnRight;
		}
		for(int i=0; i<LEDS_PER_TUBE; i++){
			if(i >= middle - amountOn && i < middle + amountOn){
				setLedHsv(tube, i, hsv);
				setLedHsv(tube+1, i, hsv);

			} else {
//				setLedHsv(tube, i, off);
//				setLedHsv(tube+1, i, off);
			}
		}
		tube += 2;
		hsv.h += hueDelta*2;
	}

}


void monoVuMeter(uint32_t counter){
	readMsgeq07(monoVuMeterCallback);
}



#define UPPER_THREASHOLD 1000
#define LOWER_THREASHOLD 500 // same as noise floor

#define LEDS_PER_BAND 10
// two state machine
#define SPECTRUM_POP_STATE_ACTIVE_LOW 1
#define SPECTRUM_POP_STATE_ACTIVE_HIGH 2
#define NEW_LEDS 3

//uint8_t sepctrumPopStates[14] = {SPECTRUM_PROCESSOR_STATE_IDLE}; // one state for each band
//uint32_t spectrumPopLeds[14][LEDS_PER_BAND] = {0}; // place to store the leds currenly being tracked
//HsvColor spectrumPopHsv[14];
//
//void spectrumProcessStateMachine(uint8_t currentState, uint32_t spectrumValue ){
//
//}
//
//void specrumPopWriteLeds(){
//
//}
//
//void spectrumPopCallback(uint32_t spectrum[]){
//	for(int i=0;i<14;i++){
//		if( sepctrumPopSta esstate == SPECTRUM_POP_STATE_ACTIVE_LOW && spectrum[i] > UPPER_THREASHOLD){
//			state = SPECTRUM_POP_STATE_ACTIVE_HIGH;
//		} else if( state == SPECTRUM_POP_STATE_ACTIVE_HIGH && sepctrum[i] < LOWER_THREASHOLD){
//			// pick new leds
//			populdateRandomArray(spectrumPopLeds[i], LEDS_PER_BAND, 0, NUM_LEDS);
//			state = SPECTRUM_POP_STATE_ACTIVE_LOW;
//		}
//		for(int j=0;j<LEDS_PER_BAND;j++){
//			spectrumPopHsv[i].v = map_uint32(spectrum[freq], 500, 4096, 0, 255);
//			setLedHsv(-1, spectrumPopLeds[i][j], spectrumPopHsv[i]);
//		}
//	}
//}
//
//void spectrumPop(uint32_t spectrum[]){
//
//}



////	int i;
//	uint32_t decay;
//	for(i = 0; i<6;i++){
//		if(spectrumBuffer[currentBufferIndex] [i] < 800){
//			spectrumBuffer[currentBufferIndex] [i] = 0;
//		}
//		enhansedSpectrum[i*2]= spectrumBuffer[currentBufferIndex] [i];
//		//enhansedSpectrum[i*2+1] = spectrumBuffer[currentBufferIndex] [i]/2 ;
//		if(i != 0){
//			//enhansedSpectrum[i*2-1] += spectrumBuffer[currentBufferIndex] [i]/2;
//		}
//		spectrumBuffer[currentBufferIndex][i] = spectrum[i];
//	}
//	currentBufferIndex = (currentBufferIndex + 1) % AUDIO_DELAY_BUFFER_LEN;
//
//	hsv.s = 255;
//	for(i=0; i<13;i++){
//		if(enhansedSpectrum[i] > currentEnhansedSpectrum[i]){
//			currentEnhansedSpectrum[i] = enhansedSpectrum[i];
//		} else {
//			// current specturm = current specturm [99-90]%
//			// the higher the number, the slower the decay
//			if(currentEnhansedSpectrum[i] != 0){
//				decay = (currentEnhansedSpectrum[i] * (120 - map_uint32(currentEnhansedSpectrum[i], 0, 4096, 20, 120))) / 1000;
//				if(decay > currentEnhansedSpectrum[i]){
//					currentEnhansedSpectrum[i] = 0;
//				}else{
//					currentEnhansedSpectrum[i] -= decay;
//				}
//
//			}
//		}
//		//hsv.v = currentEnhansedSpectrum[i] >> 4U;
//
//		currentIndex = 4 + i*5;
//		hsv.v = currentEnhansedSpectrum[i] >> 4U; // spectrum is 0-4095, so to map it to 0-255 shift it over 4 (divide by 16)
////		if(hsv.v < 50){
////			hsv.v = 50;
////		}
//		hsv.h = 255 - (255 * i / 13);
//		setLedHsv(currentIndex, hsv);
//		hsv.v = hsv.v >> 1;
//		setLedHsv(currentIndex-1, hsv);
//		setLedHsv(currentIndex+1, hsv);
////		hsv.v = hsv.v >> 2;
////		setLedHsv(currentIndex-2, hsv);
////		setLedHsv(currentIndex+2, hsv);
////		hsv.v = hsv.v >> 1;
////		setLedHsv(currentIndex-3, hsv);
//////		setLedHsv(currentIndex+3, hsv);
//	}






