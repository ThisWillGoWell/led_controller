/*
 * This is for configuring all the onboard perfs
 *
 * - TPM
 * - DMA
 * - SPI
 *
 * edma.c
 *
 *  Created on: Jun 18, 2019
 *      Author: wggowell
 */

#ifndef PERFS_C_
#define PERFS_C_

#include "perfs.h"
#include "board.h"
#include "fsl_device_registers.h"
#include "fsl_dspi_edma.h"
#include "fsl_edma.h"
#include "fsl_dmamux.h"
#include "fsl_adc16.h"
#include "fsl_tpm.h"
#include "clock_config.h"
#include "fsl_dspi.h"
#include "pin_mux.h"
#include "fsl_debug_console.h"


/*******************************************************************************
 *
 *
 * All Perfs
 *
 *
 ******************************************************************************/

/*******************************************************************************
 * Prototypes
 ******************************************************************************/
void initEdma(void);
void initDspi(void);
void initMsgeq07Timer(void);
void initMsgeq07Adc(void);
void initMsgEq07Pins(void);


/*******************************************************************************
 *
 *
 * DMA STUFFS
 *
 *
 *
 ******************************************************************************/

/*******************************************************************************
 * DMA Defines
 *  - DSPI
 ******************************************************************************/

#define BRIGHTNESS 0xFF

#define DMA_MUX_BASEADDR DMAMUX
#define DSPI_MASTER_DMA_BASEADDR DMA0
#define DSPIn_MASTER_PCS_FOR_INIT kDSPI_Pcs0
#define DSPI_MASTER_PCS_FOR_TRANSFER kDSPI_MasterPcs0
#define TRANSFER_SIZE NUM_LEDS_PER_BUS * 4 + 4 + END_BYTES /* Transfer dataSize */

#define DSPI0_MASTER_CLK_SRC DSPI0_CLK_SRC
#define DSPI0_MASTER_CLK_FREQ CLOCK_GetFreq(DSPI0_CLK_SRC)

#define DSPI0_MASTER_BASEADDR SPI0
#define DSPI0_MASTER_DMA_RX_REQUEST_SOURCE kDmaRequestMux0SPI0Rx
#define DSPI0_MASTER_DMA_TX_REQUEST_SOURCE kDmaRequestMux0SPI0Tx

#define DSPI1_MASTER_BASEADDR SPI1
#define DSPI1_MASTER_DMA_RX_REQUEST_SOURCE kDmaRequestMux0SPI1Rx
#define DSPI1_MASTER_DMA_TX_REQUEST_SOURCE kDmaRequestMux0SPI1Tx

#define DSPI2_MASTER_BASEADDR SPI2
#define DSPI2_MASTER_DMA_RX_REQUEST_SOURCE kDmaRequestMux0SPI2Rx
#define DSPI2_MASTER_DMA_TX_REQUEST_SOURCE kDmaRequestMux0SPI2Tx

#define SPI0_RX_CHANNEL 0U
#define SPI0_TX_INTER_CHANNEL 1U
#define SPI0_TX_CHANNEL 2U

#define SPI1_RX_CHANNEL 3U
#define SPI1_TX_INTER_CHANNEL 4U
#define SPI1_TX_CHANNEL 5U

#define SPI2_RX_CHANNEL 6U
#define SPI2_TX_INTER_CHANNEL 7U
#define SPI2_TX_CHANNEL 8U

/*******************************************************************************
 * Prototypes
 ******************************************************************************/
/* DSPI user callback */
void DSPI0_Callback(SPI_Type *base, dspi_master_edma_handle_t *handle, status_t status, void *userData);
void DSPI1_Callback(SPI_Type *base, dspi_master_edma_handle_t *handle, status_t status, void *userData);
void DSPI2_Callback(SPI_Type *base, dspi_master_edma_handle_t *handle, status_t status, void *userData);


/******************************************************************************
 * DMA Variables
 ******************************************************************************/
edma_handle_t dspiEdmaMasterRxRegToRxDataHandle;
edma_handle_t dspiEdmaMasterTxDataToIntermediaryHandle;
edma_handle_t dspiEdmaMasterIntermediaryToTxRegHandle;

edma_handle_t dspi1EdmaMasterRxRegToRxDataHandle;
edma_handle_t dspi1EdmaMasterTxDataToIntermediaryHandle;
edma_handle_t dspi1EdmaMasterIntermediaryToTxRegHandle;

edma_handle_t dspi2EdmaMasterRxRegToRxDataHandle;
edma_handle_t dspi2EdmaMasterTxDataToIntermediaryHandle;
edma_handle_t dspi2EdmaMasterIntermediaryToTxRegHandle;

dspi_master_edma_handle_t g_dspi0_edma_m_handle;
dspi_master_edma_handle_t g_dspi1_edma_m_handle;
dspi_master_edma_handle_t g_dspi2_edma_m_handle;

// DSPI transfer objects
dspi_transfer_t spi0Xfer;
dspi_transfer_t spi1Xfer;
dspi_transfer_t spi2Xfer;

// DSPI transfer arrays
uint8_t leds0[TRANSFER_SIZE] = {0};
uint8_t leds1[TRANSFER_SIZE] = {0};
uint8_t leds2[TRANSFER_SIZE] = {0};

void DSPI0_Callback(SPI_Type *base, dspi_master_edma_handle_t *handle, status_t status, void *userData)
{
	 DSPI_MasterTransferEDMA(DSPI0_MASTER_BASEADDR, &g_dspi0_edma_m_handle, &spi0Xfer);
}

void DSPI1_Callback(SPI_Type *base, dspi_master_edma_handle_t *handle, status_t status, void *userData)
{
	DSPI_MasterTransferEDMA(DSPI1_MASTER_BASEADDR, &g_dspi1_edma_m_handle, &spi1Xfer);
}

void DSPI2_Callback(SPI_Type *base, dspi_master_edma_handle_t *handle, status_t status, void *userData)
{
	DSPI_MasterTransferEDMA(DSPI2_MASTER_BASEADDR, &g_dspi2_edma_m_handle, &spi2Xfer);
}


void initEdma(void){

    uint32_t DSPI0TxChannel,DSPI0RxChannel, DSPI0IntermediaryChannel;
	uint32_t DSPI1TxChannel, DSPI1RxChannel, DSPI1IntermediaryChannel;
	uint32_t DSPI2TxChannel, DSPI2RxChannel, DSPI2IntermediaryChannel;

	DSPI0TxChannel = 1U;
	DSPI0IntermediaryChannel = 2U;
	DSPI0RxChannel = 0U;

	DSPI1TxChannel = 3U;
	DSPI1IntermediaryChannel = 4U;
	DSPI1RxChannel = 5U;

	DSPI2TxChannel = 6U;
	DSPI2IntermediaryChannel = 7U;
	DSPI2RxChannel = 8U;


	DMAMUX_Init(DMA_MUX_BASEADDR);


    // enable DMA channels
    // SPI0 Source
	// SPI0 Rx
    DMAMUX_SetSource(DMA_MUX_BASEADDR, DSPI0RxChannel,
                     (uint8_t)DSPI0_MASTER_DMA_RX_REQUEST_SOURCE);
    DMAMUX_EnableChannel(DMA_MUX_BASEADDR, DSPI0RxChannel);
    // SP0 TX
    DMAMUX_SetSource(DMA_MUX_BASEADDR, DSPI0TxChannel,
                     (uint8_t)DSPI0_MASTER_DMA_TX_REQUEST_SOURCE);
    DMAMUX_EnableChannel(DMA_MUX_BASEADDR, DSPI0TxChannel);

    // SPI1 Source
    // SPI1 Rx
    DMAMUX_SetSource(DMA_MUX_BASEADDR, DSPI1RxChannel,
    	                     (uint8_t)DSPI1_MASTER_DMA_RX_REQUEST_SOURCE);
	DMAMUX_EnableChannel(DMA_MUX_BASEADDR, DSPI1RxChannel);
	//SPI1Tx
    DMAMUX_SetSource(DMA_MUX_BASEADDR, DSPI1TxChannel,
   	                     (uint8_t)DSPI1_MASTER_DMA_TX_REQUEST_SOURCE);
	DMAMUX_EnableChannel(DMA_MUX_BASEADDR, DSPI1TxChannel);


	// SPI2 Source
	// SPI2 Rx
	DMAMUX_SetSource(DMA_MUX_BASEADDR, DSPI2RxChannel,
							 (uint8_t)DSPI2_MASTER_DMA_RX_REQUEST_SOURCE);
	DMAMUX_EnableChannel(DMA_MUX_BASEADDR, DSPI2RxChannel);
	//SPI2Tx
	DMAMUX_SetSource(DMA_MUX_BASEADDR, DSPI2TxChannel,
						 (uint8_t)DSPI2_MASTER_DMA_TX_REQUEST_SOURCE);
	DMAMUX_EnableChannel(DMA_MUX_BASEADDR, DSPI2TxChannel);

	/* EDMA init */
	/*
	 * userConfig.enableRoundRobinArbitration = false;
	 * userConfig.enableHaltOnError = true;
	 * userConfig.enableContinuousLinkMode = false;
	 * userConfig.enableDebugMode = false;
	 */
    edma_config_t userConfig;
	EDMA_GetDefaultConfig(&userConfig);

	EDMA_Init(DSPI_MASTER_DMA_BASEADDR, &userConfig);



	// configure DataHandle
		    // SPI0 RX
		    memset(&(dspiEdmaMasterRxRegToRxDataHandle), 0, sizeof(dspiEdmaMasterRxRegToRxDataHandle));
		    // SPI0 TX: Data -> intermediary
		    memset(&(dspiEdmaMasterTxDataToIntermediaryHandle), 0, sizeof(dspiEdmaMasterTxDataToIntermediaryHandle));
		    // SPI0 TX: intermediary -> TX
		    memset(&(dspiEdmaMasterIntermediaryToTxRegHandle), 0, sizeof(dspiEdmaMasterIntermediaryToTxRegHandle));


		    EDMA_CreateHandle(&(dspiEdmaMasterRxRegToRxDataHandle), DSPI_MASTER_DMA_BASEADDR, DSPI0RxChannel);
		    EDMA_CreateHandle(&(dspiEdmaMasterTxDataToIntermediaryHandle), DSPI_MASTER_DMA_BASEADDR,
		                      DSPI0IntermediaryChannel);
		    EDMA_CreateHandle(&(dspiEdmaMasterIntermediaryToTxRegHandle), DSPI_MASTER_DMA_BASEADDR, DSPI0TxChannel);

		    // Create the MAsterTransferHandle
		    DSPI_MasterTransferCreateHandleEDMA(DSPI0_MASTER_BASEADDR, &g_dspi0_edma_m_handle, DSPI0_Callback,
		                                        NULL, &dspiEdmaMasterRxRegToRxDataHandle,
		                                        &dspiEdmaMasterTxDataToIntermediaryHandle,
		                                        &dspiEdmaMasterIntermediaryToTxRegHandle);

		    // SPI1 RX
		    memset(&(dspi1EdmaMasterRxRegToRxDataHandle), 0, sizeof(dspi1EdmaMasterRxRegToRxDataHandle));
			EDMA_CreateHandle(&(dspi1EdmaMasterRxRegToRxDataHandle), DSPI_MASTER_DMA_BASEADDR, DSPI1RxChannel);

			//SPI1: Data -> intermediary
		    memset(&(dspi1EdmaMasterTxDataToIntermediaryHandle), 0, sizeof(dspi1EdmaMasterTxDataToIntermediaryHandle));
		    EDMA_CreateHandle(&(dspi1EdmaMasterTxDataToIntermediaryHandle), DSPI_MASTER_DMA_BASEADDR, DSPI1IntermediaryChannel);

		    //SPI0 TX: intermediary -> TX
		    memset(&(dspi1EdmaMasterIntermediaryToTxRegHandle), 0, sizeof(dspi1EdmaMasterIntermediaryToTxRegHandle));
		    EDMA_CreateHandle(&(dspi1EdmaMasterIntermediaryToTxRegHandle), DSPI_MASTER_DMA_BASEADDR, DSPI1TxChannel);
		    // Create the MasterTransferHandle
		    DSPI_MasterTransferCreateHandleEDMA(DSPI1_MASTER_BASEADDR, &g_dspi1_edma_m_handle, DSPI1_Callback,
		                                        NULL, &dspi1EdmaMasterRxRegToRxDataHandle,
		                                        &dspi1EdmaMasterTxDataToIntermediaryHandle,
		                                        &dspi1EdmaMasterIntermediaryToTxRegHandle);

		    // SPI2 RX
			memset(&(dspi2EdmaMasterRxRegToRxDataHandle), 0, sizeof(dspi2EdmaMasterRxRegToRxDataHandle));
			EDMA_CreateHandle(&(dspi2EdmaMasterRxRegToRxDataHandle), DSPI_MASTER_DMA_BASEADDR, DSPI2RxChannel);

			//SPI2: Data -> intermediary
			memset(&(dspi2EdmaMasterTxDataToIntermediaryHandle), 0, sizeof(dspi2EdmaMasterTxDataToIntermediaryHandle));
			EDMA_CreateHandle(&(dspi2EdmaMasterTxDataToIntermediaryHandle), DSPI_MASTER_DMA_BASEADDR, DSPI2IntermediaryChannel);

			//SPI2 TX: intermediary -> TX
			memset(&(dspi2EdmaMasterIntermediaryToTxRegHandle), 0, sizeof(dspi2EdmaMasterIntermediaryToTxRegHandle));
			EDMA_CreateHandle(&(dspi2EdmaMasterIntermediaryToTxRegHandle), DSPI_MASTER_DMA_BASEADDR, DSPI2TxChannel);
			// Create the MasterTransferHandle
			DSPI_MasterTransferCreateHandleEDMA(DSPI2_MASTER_BASEADDR, &g_dspi2_edma_m_handle, DSPI2_Callback,
												NULL, &dspi2EdmaMasterRxRegToRxDataHandle,
												&dspi2EdmaMasterTxDataToIntermediaryHandle,
												&dspi2EdmaMasterIntermediaryToTxRegHandle);
}


void initDspi(void){
	 /* Port B Clock Gate Control: Clock enabled */
	CLOCK_EnableClock(kCLOCK_PortB);
	/* Port D Clock Gate Control: Clock enabled */
	CLOCK_EnableClock(kCLOCK_PortD);

    /* PORTD1 (pin D4) is configured as SPI0_SCK */
    PORT_SetPinMux(PORTD, 1U, kPORT_MuxAlt2); // SPI0 Clock
    PORT_SetPinMux(PORTD, 2U, kPORT_MuxAlt2); // SPI0 Data

	PORT_SetPinMux(PORTD, 6U, kPORT_MuxAlt7); // SPI1 Data
	PORT_SetPinMux(PORTB, 11U, kPORT_MuxAlt2);// SPI1 Clock

	PORT_SetPinMux(PORTD, 12U, kPORT_MuxAlt2); //SPI2 clock
	PORT_SetPinMux(PORTD, 13U, kPORT_MuxAlt2); // SPI2 Data

	dspi_master_config_t masterConfig;
	uint32_t srcClock_Hz;

	masterConfig.whichCtar = kDSPI_Ctar0;
	masterConfig.ctarConfig.baudRate = TRANSFER_BAUDRATE;
	masterConfig.ctarConfig.bitsPerFrame = 8;
	masterConfig.ctarConfig.cpol = kDSPI_ClockPolarityActiveHigh;
	masterConfig.ctarConfig.cpha = kDSPI_ClockPhaseFirstEdge;
	masterConfig.ctarConfig.direction = kDSPI_MsbFirst;
	masterConfig.ctarConfig.pcsToSckDelayInNanoSec = 1000000000U / TRANSFER_BAUDRATE;
	masterConfig.ctarConfig.lastSckToPcsDelayInNanoSec = 1000000000U / TRANSFER_BAUDRATE;
	masterConfig.ctarConfig.betweenTransferDelayInNanoSec = 0;
	masterConfig.whichPcs = DSPIn_MASTER_PCS_FOR_INIT;
	masterConfig.pcsActiveHighOrLow = kDSPI_PcsActiveLow;
	masterConfig.enableContinuousSCK = false;
	masterConfig.enableRxFifoOverWrite = false;
	masterConfig.enableModifiedTimingFormat = false;
	masterConfig.samplePoint = kDSPI_SckToSin0Clock;
	srcClock_Hz = DSPI0_MASTER_CLK_FREQ;

	DSPI_MasterInit(DSPI0_MASTER_BASEADDR, &masterConfig, srcClock_Hz);
	DSPI_MasterInit(DSPI1_MASTER_BASEADDR, &masterConfig, srcClock_Hz);
	DSPI_MasterInit(DSPI2_MASTER_BASEADDR, &masterConfig, srcClock_Hz);

	// init transfer objects
	spi0Xfer.txData = leds0;
	spi0Xfer.rxData = NULL;
	spi0Xfer.dataSize = TRANSFER_SIZE;
	spi0Xfer.configFlags = kDSPI_MasterCtar0 | DSPI_MASTER_PCS_FOR_TRANSFER | kDSPI_MasterPcsContinuous;

	spi1Xfer.txData = leds1;
	spi1Xfer.rxData = NULL;
	spi1Xfer.dataSize = TRANSFER_SIZE;
	spi1Xfer.configFlags = kDSPI_MasterCtar0 | DSPI_MASTER_PCS_FOR_TRANSFER | kDSPI_MasterPcsContinuous;


	spi2Xfer.txData = leds2;
	spi2Xfer.rxData = NULL;
	spi2Xfer.dataSize = TRANSFER_SIZE;
	spi2Xfer.configFlags = kDSPI_MasterCtar0 | DSPI_MASTER_PCS_FOR_TRANSFER | kDSPI_MasterPcsContinuous;




}

void setLed(uint32_t i,uint8_t r, uint8_t g, uint8_t b){

	int index = 4 + (i % NUM_LEDS_PER_BUS) * 4;
	switch(i / NUM_LEDS_PER_BUS){
	case 0:
		leds0[index + 1] =  b;
		leds0[index + 2] = g;
		leds0[index + 3] = r;
		break;
	case 1:
		leds1[index + 1] = b;
		leds1[index + 2] = g;
		leds1[index + 3] = r;
		break;
	case 2:
		leds2[index + 1] = b;
		leds2[index + 2] = g;
		leds2[index + 3] = r;
	}
}



//RgbColor getLedRgb(int i){
//	RgbColor rgbColor;
//
//	int index = 4 + (i % NUM_LEDS_PER_BUS) * 4;
//	switch(i / NUM_LEDS_PER_BUS){
//	case 0:
//		rgbColor.b = leds0[index + 1];
//		rgbColor.g = leds0[index + 2] ;
//		rgbColor.r = leds0[index + 3];
//		break;
//	case 1:
//		rgbColor.b = leds1[index + 1];
//		rgbColor.g = leds1[index + 2] ;
//		rgbColor.r = leds1[index + 3];
//		break;
//	case 2:
//		rgbColor.b = leds2[index + 1];
//		rgbColor.g = leds2[index + 2] ;
//		rgbColor.r = leds2[index + 3];
//	}
//	return rgbColor;
//}
//
//HsvColor getLedHsv(int i){
//	return RgbToHsv(getLedRgb(i));
//}

/*******************************************************************************
 *
 *
 *
 * MSGEQ07 STUFFS
 *
 *
 *
 ******************************************************************************/
/*******************************************************************************
 * ADC
 ******************************************************************************/
#define ADC16_BASE ADC1
#define ADC16_CHANNEL_GROUP 0U
#define ADC16_USER_CHANNEL 18U

#define ADC16_IRQn ADC1_IRQn
#define ADC16_IRQ_HANDLER_FUNC ADC1_IRQHandler

/*******************************************************************************
 * GPIO defines
 ******************************************************************************/
#define STOBE_RESET_PORT GPIOC
#define RESET_PIN 10U
#define STROBE_PIN 11U

/*******************************************************************************
 * Timer/Pulse Width Modulation Module
 ******************************************************************************/
#define BOARD_TPM TPM2
#define BOARD_TPM_IRQ_NUM TPM2_IRQn
#define BOARD_TPM_HANDLER TPM2_IRQHandler
#define TPM_SOURCE_CLOCK (CLOCK_GetFreq(kCLOCK_PllFllSelClk)/4)
#define TPM_PRESCALER kTPM_Prescale_Divide_4

/*******************************************************************************
 * MSGEQQ07 Timing
 ******************************************************************************/
#define OUTPUT_SETTLE_US 36U
#define RESET_PULSE_WIDTH_US 100U
#define STROBE_TO_STROBE_US 72U

/************************
 * MSGEQQ07 Read State Machine
 ************************/
#define RESET_HIGH_STATE 0
#define STROBE_HIGH_STATE 1
#define STROBE_LOW_STATE 2
#define STROBE_OUTPUT_SETTLE 3

/*******************************************************************************
 * Prototypes
 ******************************************************************************/
void setResetPin(uint8_t onOff);
void setStrobePin(uint8_t onOff);
void printSpectrum(void);
void startRead(void);
void setNextTimerInterrupt(uint32_t);

/*******************************************************************************
 * Variables
 ******************************************************************************/
adc16_channel_config_t adc16ChannelConfigStruct;
uint8_t state;
uint32_t spectrum[7] = {0};
uint8_t currentRead = 0;
msgeq07_callback current_callback;
void ADC16_IRQ_HANDLER_FUNC(void)
{
	/* Read conversion result to clear the conversion completed flag. */
	spectrum[currentRead] = ADC16_GetChannelConversionValue(ADC16_BASE, ADC16_CHANNEL_GROUP);
	setStrobePin(1);

	if (currentRead != 6) {
		currentRead += 1;
		state = STROBE_HIGH_STATE;
		setNextTimerInterrupt(STROBE_TO_STROBE_US);
	} else{
		currentRead = 0;
		current_callback(spectrum);
	}

	 __DSB();
}


void BOARD_TPM_HANDLER(void)
{
	// Stop the timer for now
    TPM_StopTimer(BOARD_TPM);

    switch(state){
    case RESET_HIGH_STATE: // the only diffrence between a first read and subsequent is you set reset 0
    	setResetPin(0);
    case STROBE_HIGH_STATE:
    	setStrobePin(0);
    	state = OUTPUT_SETTLE_US;
    	setNextTimerInterrupt(OUTPUT_SETTLE_US);
    	break;
    case OUTPUT_SETTLE_US:
    	// trigger a read
    	ADC16_SetChannelConfig(ADC16_BASE, ADC16_CHANNEL_GROUP, &adc16ChannelConfigStruct);
    	break;
    }
    /* Clear interrupt flag.*/
    TPM_ClearStatusFlags(BOARD_TPM, kTPM_TimeOverflowFlag);

    __DSB();
}



void readMsgeq07(msgeq07_callback callback){

	current_callback = callback;
	state = RESET_HIGH_STATE;
	//reset high
	setResetPin(1);
	setStrobePin(0);
	setNextTimerInterrupt(RESET_PULSE_WIDTH_US);
}

/*******************************************************************************
 * Timer
 ******************************************************************************/

void initMsgeq07Timer(void){
	/* TPM known issue of KL81, enable clock of TPM0 to use other TPM module */
	CLOCK_EnableClock(kCLOCK_Tpm2);
	/* Select the clock source for the TPM counter as kCLOCK_PllFllSelClk */
	CLOCK_SetTpmClock(1U);

	tpm_config_t tpmInfo;
	TPM_GetDefaultConfig(&tpmInfo);
	/* TPM clock divide by TPM_PRESCALER */
	tpmInfo.prescale = TPM_PRESCALER;


	TPM_Init(BOARD_TPM, &tpmInfo);/* Initialize TPM module */

	// enable the TPM interrupt out of all interrupts
	EnableIRQ(BOARD_TPM_IRQ_NUM);
}

void initMsgEq07Pins(void){

	gpio_pin_config_t output_config = {
		kGPIO_DigitalOutput, 0,
	};

	CLOCK_EnableClock(kCLOCK_PortC);
	PORT_SetPinMux(PORTC, RESET_PIN, kPORT_MuxAsGpio);
	PORT_SetPinMux(PORTC, STROBE_PIN, kPORT_MuxAsGpio);

	GPIO_PinInit(STOBE_RESET_PORT, RESET_PIN, &output_config);
	GPIO_PinInit(STOBE_RESET_PORT, STROBE_PIN, &output_config);
}

void initMsgeq07Adc(){
	adc16_config_t adc16ConfigStruct;

	EnableIRQ(ADC16_IRQn);
	ADC16_GetDefaultConfig(&adc16ConfigStruct);
#ifdef BOARD_ADC_USE_ALT_VREF
	adc16ConfigStruct.referenceVoltageSource = kADC16_ReferenceVoltageSourceValt;
#endif
	ADC16_Init(ADC16_BASE, &adc16ConfigStruct);
	ADC16_EnableHardwareTrigger(ADC16_BASE, false); /* Make sure the software trigger is used. */
#if defined(FSL_FEATURE_ADC16_HAS_CALIBRATION) && FSL_FEATURE_ADC16_HAS_CALIBRATION
	if (kStatus_Success == ADC16_DoAutoCalibration(ADC16_BASE))
	{
		PRINTF("ADC16_DoAutoCalibration() Done.\r\n");
	}
	else
	{
		PRINTF("ADC16_DoAutoCalibration() Failed.\r\n");
	}
#endif /* FSL_FEATURE_ADC16_HAS_CALIBRATION */
	adc16ChannelConfigStruct.channelNumber = ADC16_USER_CHANNEL;
	adc16ChannelConfigStruct.enableInterruptOnConversionCompleted = true; /* Enable the interrupt. */
#if defined(FSL_FEATURE_ADC16_HAS_DIFF_MODE) && FSL_FEATURE_ADC16_HAS_DIFF_MODE
	adc16ChannelConfigStruct.enableDifferentialConversion = false;
#endif /* FSL_FEATURE_ADC16_HAS_DIFF_MODE */

}

void setNextTimerInterrupt(uint32_t us) {
    TPM_SetTimerPeriod(BOARD_TPM, USEC_TO_COUNT(us, TPM_SOURCE_CLOCK));
    TPM_EnableInterrupts(BOARD_TPM, kTPM_TimeOverflowInterruptEnable);
    TPM_StartTimer(BOARD_TPM, kTPM_SystemClock);
}


void setStrobePin(uint8_t onOff){
	if(onOff){
		GPIO_PortSet(STOBE_RESET_PORT, 1u << STROBE_PIN);
	} else {
		GPIO_PortClear(STOBE_RESET_PORT, 1u << STROBE_PIN);
	}
}

void setResetPin(uint8_t onOff){
	if(onOff){
		GPIO_PortSet(STOBE_RESET_PORT, 1u << RESET_PIN);
	} else {
		GPIO_PortClear(STOBE_RESET_PORT, 1u << RESET_PIN);
	}
}



void initPerfs(){
	initDspi();
	initEdma();

	initMsgeq07Timer();
	initMsgeq07Adc();
	initMsgEq07Pins();

	// set the global brightness of each bus
	for(int i=0;i<NUM_LEDS_PER_BUS;i++){
		leds0[4+i*4] = BRIGHTNESS;
		leds1[4+i*4] = BRIGHTNESS;
		leds2[4+i*4] = BRIGHTNESS;
	}

	// initlize the end frame for each bus
	for(int i=TRANSFER_SIZE-1; i > TRANSFER_SIZE - END_BYTES-1; i--){
		leds0[i]=0xFF;
		leds1[i]=0xFF;
		leds2[i]=0xFF;
	}


	if (kStatus_Success != DSPI_MasterTransferEDMA(DSPI0_MASTER_BASEADDR, &g_dspi0_edma_m_handle, &spi0Xfer))
	{
		PRINTF("There is error when start DSPI_MasterTransferEDMA \r\n ");
	}
	if (kStatus_Success != DSPI_MasterTransferEDMA(DSPI1_MASTER_BASEADDR, &g_dspi1_edma_m_handle, &spi1Xfer))
	{
		PRINTF("There is error when start DSPI_MasterTransferEDMA \r\n ");
	}

	if (kStatus_Success != DSPI_MasterTransferEDMA(DSPI2_MASTER_BASEADDR, &g_dspi2_edma_m_handle, &spi2Xfer))
	{
		PRINTF("There is error when start DSPI_MasterTransferEDMA \r\n ");
	}
}



#endif /* EDMA_C_ */
