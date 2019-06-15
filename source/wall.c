/*
 * Copyright (c) 2015, Freescale Semiconductor, Inc.
 * Copyright 2016-2017 NXP
 * All rights reserved.
 * 
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "fsl_device_registers.h"
#include "fsl_debug_console.h"
#include "fsl_dspi.h"
#include "board.h"
#include "fsl_dspi_edma.h"
#include "fsl_edma.h"
#include "fsl_dmamux.h"

#include "pin_mux.h"
#include "clock_config.h"
/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define NUM_LEDS 1800U
#define END_BYTES 75
#define BRIGHTNESS 0xFF
/*******************************************************************************
 * Perf Definitions
 ******************************************************************************/

#define DMA_MUX_BASEADDR DMAMUX
#define DSPI_MASTER_DMA_BASEADDR DMA0

#define DSPIn_MASTER_PCS_FOR_INIT kDSPI_Pcs0
#define DSPI_MASTER_PCS_FOR_TRANSFER kDSPI_MasterPcs0
#define TRANSFER_SIZE NUM_LEDS * 4 + 4 + END_BYTES /* Transfer dataSize */
#define TRANSFER_BAUDRATE 2000000U /* Transfer baudrate - 500k */

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

#define ENABLE_PORT GPIOC
#define ENABLE_PIN_0 10U
#define ENABLE_PIN_1 11U

/*******************************************************************************
 * Prototypes
 ******************************************************************************/
/* DSPI user callback */
void DSPI0_Callback(SPI_Type *base, dspi_master_edma_handle_t *handle, status_t status, void *userData);
void DSPI1_Callback(SPI_Type *base, dspi_master_edma_handle_t *handle, status_t status, void *userData);
void DSPI2_Callback(SPI_Type *base, dspi_master_edma_handle_t *handle, status_t status, void *userData);
/******************************************************************************
 * Variables
 ******************************************************************************/
uint8_t masterRxData[TRANSFER_SIZE] = {0};
uint8_t masterTxData[TRANSFER_SIZE] = {0};

dspi_master_edma_handle_t g_dspi0_edma_m_handle;
dspi_master_edma_handle_t g_dspi1_edma_m_handle;
dspi_master_edma_handle_t g_dspi2_edma_m_handle;



edma_handle_t dspiEdmaMasterRxRegToRxDataHandle;
edma_handle_t dspiEdmaMasterTxDataToIntermediaryHandle;
edma_handle_t dspiEdmaMasterIntermediaryToTxRegHandle;

edma_handle_t dspi1EdmaMasterRxRegToRxDataHandle;
edma_handle_t dspi1EdmaMasterTxDataToIntermediaryHandle;
edma_handle_t dspi1EdmaMasterIntermediaryToTxRegHandle;

edma_handle_t dspi2EdmaMasterRxRegToRxDataHandle;
edma_handle_t dspi2EdmaMasterTxDataToIntermediaryHandle;
edma_handle_t dspi2EdmaMasterIntermediaryToTxRegHandle;

volatile bool isTransferCompleted = false;
volatile uint32_t g_systickCounter;

/*******************************************************************************
 * Code
 ******************************************************************************/

uint8_t leds[TRANSFER_SIZE] = {0};
dspi_transfer_t masterXfer;

int write_counter=0;

void setLed(int i,uint8_t r, uint8_t g, uint8_t b){
	leds[4 + i * 4 + 1] = b;
	leds[4 + i * 4 + 2] = g;
	leds[4 + i * 4 + 3] = r;
}


void DSPI0_Callback(SPI_Type *base, dspi_master_edma_handle_t *handle, status_t status, void *userData)
{
	DSPI_MasterTransferEDMA(DSPI0_MASTER_BASEADDR, &g_dspi0_edma_m_handle, &masterXfer);
}

void DSPI1_Callback(SPI_Type *base, dspi_master_edma_handle_t *handle, status_t status, void *userData)
{

	DSPI_MasterTransferEDMA(DSPI1_MASTER_BASEADDR, &g_dspi1_edma_m_handle, &masterXfer);
}

void DSPI2_Callback(SPI_Type *base, dspi_master_edma_handle_t *handle, status_t status, void *userData)
{
	DSPI_MasterTransferEDMA(DSPI2_MASTER_BASEADDR, &g_dspi2_edma_m_handle, &masterXfer);
}


// for generating interrupts
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


void edmaSpiInit(){
	 /* DMA Mux setting and EDMA init */


	    edma_config_t userConfig;


	    /* DMA MUX init */
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
	    // SPI0 Rx
	    DMAMUX_SetSource(DMA_MUX_BASEADDR, DSPI1RxChannel,
	    	                     (uint8_t)DSPI1_MASTER_DMA_RX_REQUEST_SOURCE);
		DMAMUX_EnableChannel(DMA_MUX_BASEADDR, DSPI0RxChannel);
		//SPI0Tx
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
	    EDMA_GetDefaultConfig(&userConfig);

	    EDMA_Init(DSPI_MASTER_DMA_BASEADDR, &userConfig);

	    uint32_t srcClock_Hz;


	    /*******************************************************************************
	     * SPI
	     * Config
	     ******************************************************************************/
	    dspi_master_config_t masterConfig;
	    masterConfig.whichCtar = kDSPI_Ctar0;
	    masterConfig.ctarConfig.baudRate = TRANSFER_BAUDRATE;
	    masterConfig.ctarConfig.bitsPerFrame = 8;
	    masterConfig.ctarConfig.cpol = kDSPI_ClockPolarityActiveHigh;
	    masterConfig.ctarConfig.cpha = kDSPI_ClockPhaseFirstEdge;
	    masterConfig.ctarConfig.direction = kDSPI_MsbFirst;
	    masterConfig.ctarConfig.pcsToSckDelayInNanoSec = 1000000000U / TRANSFER_BAUDRATE;
	    masterConfig.ctarConfig.lastSckToPcsDelayInNanoSec = 1000000000U / TRANSFER_BAUDRATE;
	    masterConfig.ctarConfig.betweenTransferDelayInNanoSec =  1000000000U / TRANSFER_BAUDRATE;
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


int main(void)
{
    BOARD_InitPins();
    BOARD_BootClockRUN();
    BOARD_InitDebugConsole();
    edmaSpiInit();
    uint32_t errorCount;
    uint32_t loopCount = 1U;
    uint32_t i;

    gpio_pin_config_t output_config = {
		kGPIO_DigitalOutput, 0,
    };
    CLOCK_EnableClock(kCLOCK_PortC);
    PORT_SetPinMux(PORTC, ENABLE_PIN_0, kPORT_MuxAsGpio);
    PORT_SetPinMux(PORTC, ENABLE_PIN_1, kPORT_MuxAsGpio);

    GPIO_PinInit(ENABLE_PORT, ENABLE_PIN_0, &output_config);
    GPIO_PinInit(ENABLE_PORT, ENABLE_PIN_1, &output_config);
    GPIO_PortClear(ENABLE_PORT, 0xFFFFFFFF);
    GPIO_PortSet(ENABLE_PORT, 1u << ENABLE_PIN_1);

    /* Set systick reload value to generate 1ms interrupt */
	if(SysTick_Config(SystemCoreClock / 1000U))
	{
		PRINTF("Failed to start systick");
		while(1)
		{
		}
	}


    for(int i=0;i<NUM_LEDS;i++){
    	leds[4+i*4] = BRIGHTNESS;
    }
    for(int i=TRANSFER_SIZE-1; i > TRANSFER_SIZE - END_BYTES-1; i--){
    	leds[i]=0xFF;
    }
    masterXfer.txData = leds;
	masterXfer.rxData = NULL;
	masterXfer.dataSize = TRANSFER_SIZE;
	masterXfer.configFlags = kDSPI_MasterCtar0 | DSPI_MASTER_PCS_FOR_TRANSFER | kDSPI_MasterPcsContinuous;

	if (kStatus_Success != DSPI_MasterTransferEDMA(DSPI0_MASTER_BASEADDR, &g_dspi0_edma_m_handle, &masterXfer))

	{
		PRINTF("There is error when start DSPI_MasterTransferEDMA \r\n ");
	}
	if (kStatus_Success != DSPI_MasterTransferEDMA(DSPI1_MASTER_BASEADDR, &g_dspi1_edma_m_handle, &masterXfer))
	{
		PRINTF("There is error when start DSPI_MasterTransferEDMA \r\n ");
	}

	if (kStatus_Success != DSPI_MasterTransferEDMA(DSPI2_MASTER_BASEADDR, &g_dspi2_edma_m_handle, &masterXfer))
	{
		PRINTF("There is error when start DSPI_MasterTransferEDMA \r\n ");
	}

    while(1){

		for(int color=0;color<3;color++){
			 for(loopCount=0;loopCount<NUM_LEDS;loopCount++)
			 {
				switch(color){
				case 0:
					setLed(loopCount % NUM_LEDS, 100,0,0);
					break;
				case 1:
					setLed(loopCount % NUM_LEDS, 0,100,0);
					break;
				case 2:
					setLed(loopCount % NUM_LEDS, 0,0,100);
					break;
				}

				/* Delay 10 ms */
				SysTick_DelayTicks(10U);

//				isTransferCompleted = false;


				/* Wait until transfer completed */
//				while (!isTransferCompleted)
//				{
//				}

				/* Delay to wait slave is ready */


			}
		 }
	}


//        /* Set up the transfer data */
//        for (i = 0U; i < TRANSFER_SIZE; i++)
//        {
//
//            masterTxData[i] = (i + loopCount) % 256U;
//            masterRxData[i] = 0U;
//        }

//        /* Print out transmit buffer */
//        PRINTF("\r\n Master transmit:\r\n");
//        for (i = 0; i < TRANSFER_SIZE; i++)
//        {
//            /* Print 16 numbers in a line */
//            if ((i & 0x0FU) == 0U)
//            {
//                PRINTF("\r\n");
//            }
//            PRINTF(" %02X", masterTxData[i]);
//        }
//        PRINTF("\r\n");

        /* Start master transfer, send data to slave */


}

