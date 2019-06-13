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
#define NUM_LEDS 4800U
#define END_BYTES 300
#define BRIGHTNESS 0xFF
/*******************************************************************************
 * Perf Definitions
 ******************************************************************************/
#define EXAMPLE_DSPI_MASTER_BASEADDR SPI0
#define EXAMPLE_DSPI_MASTER_DMA_MUX_BASEADDR DMAMUX
#define EXAMPLE_DSPI_MASTER_DMA_BASEADDR DMA0
#define EXAMPLE_DSPI_MASTER_DMA_RX_REQUEST_SOURCE kDmaRequestMux0SPI0Rx
#define EXAMPLE_DSPI_MASTER_DMA_TX_REQUEST_SOURCE kDmaRequestMux0SPI0Tx
#define DSPI_MASTER_CLK_SRC DSPI0_CLK_SRC
#define DSPI_MASTER_CLK_FREQ CLOCK_GetFreq(DSPI0_CLK_SRC)
#define EXAMPLE_DSPI_MASTER_PCS_FOR_INIT kDSPI_Pcs0
#define EXAMPLE_DSPI_MASTER_PCS_FOR_TRANSFER kDSPI_MasterPcs0
#define EXAMPLE_DSPI_DEALY_COUNT 0x00ffU
#define TRANSFER_SIZE NUM_LEDS * 4 + 4 + END_BYTES /* Transfer dataSize */
#define TRANSFER_BAUDRATE 500000U /* Transfer baudrate - 500k */

/*******************************************************************************
 * Prototypes
 ******************************************************************************/
/* DSPI user callback */
void DSPI_MasterUserCallback(SPI_Type *base, dspi_master_edma_handle_t *handle, status_t status, void *userData);

/*******************************************************************************
 * Variables
 ******************************************************************************/
uint8_t masterRxData[TRANSFER_SIZE] = {0};
uint8_t masterTxData[TRANSFER_SIZE] = {0};

dspi_master_edma_handle_t g_dspi_edma_m_handle;
edma_handle_t dspiEdmaMasterRxRegToRxDataHandle;
#if (!(defined(FSL_FEATURE_DSPI_HAS_GASKET) && FSL_FEATURE_DSPI_HAS_GASKET))
edma_handle_t dspiEdmaMasterTxDataToIntermediaryHandle;
#endif
edma_handle_t dspiEdmaMasterIntermediaryToTxRegHandle;

volatile bool isTransferCompleted = false;
volatile uint32_t g_systickCounter;

/*******************************************************************************
 * Code
 ******************************************************************************/

uint8_t leds[TRANSFER_SIZE] = {0};
dspi_transfer_t masterXfer;


void setLed(int i,uint8_t r, uint8_t g, uint8_t b){
	leds[4 + i * 4 + 1] = b;
	leds[4 + i * 4 + 2] = g;
	leds[4 + i * 4 + 3] = r;
}


void DSPI_MasterUserCallback(SPI_Type *base, dspi_master_edma_handle_t *handle, status_t status, void *userData)
{
//    if (status == kStatus_Success)
//    {
//        PRINTF("This is DSPI master edma transfer completed callback. \r\n\r\n");
//    }
	DSPI_MasterTransferEDMA(EXAMPLE_DSPI_MASTER_BASEADDR, &g_dspi_edma_m_handle, &masterXfer);
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
	    uint32_t masterRxChannel, masterTxChannel;
	    edma_config_t userConfig;

	    masterRxChannel = 0U;
	    masterTxChannel = 1U;
	/* If DSPI instances support Gasket feature, only two channels are needed. */
	#if (!(defined(FSL_FEATURE_DSPI_HAS_GASKET) && FSL_FEATURE_DSPI_HAS_GASKET))
	    uint32_t masterIntermediaryChannel;
	    masterIntermediaryChannel = 2U;
	#endif
	    /* DMA MUX init */
	    DMAMUX_Init(EXAMPLE_DSPI_MASTER_DMA_MUX_BASEADDR);

	    DMAMUX_SetSource(EXAMPLE_DSPI_MASTER_DMA_MUX_BASEADDR, masterRxChannel,
	                     (uint8_t)EXAMPLE_DSPI_MASTER_DMA_RX_REQUEST_SOURCE);
	    DMAMUX_EnableChannel(EXAMPLE_DSPI_MASTER_DMA_MUX_BASEADDR, masterRxChannel);

	#if (defined EXAMPLE_DSPI_MASTER_DMA_TX_REQUEST_SOURCE)
	    DMAMUX_SetSource(EXAMPLE_DSPI_MASTER_DMA_MUX_BASEADDR, masterTxChannel,
	                     (uint8_t)EXAMPLE_DSPI_MASTER_DMA_TX_REQUEST_SOURCE);
	    DMAMUX_EnableChannel(EXAMPLE_DSPI_MASTER_DMA_MUX_BASEADDR, masterTxChannel);
	#endif

	    /* EDMA init */
	    /*
	     * userConfig.enableRoundRobinArbitration = false;
	     * userConfig.enableHaltOnError = true;
	     * userConfig.enableContinuousLinkMode = false;
	     * userConfig.enableDebugMode = false;
	     */
	    EDMA_GetDefaultConfig(&userConfig);

	    EDMA_Init(EXAMPLE_DSPI_MASTER_DMA_BASEADDR, &userConfig);

	    uint32_t srcClock_Hz;
	    dspi_master_config_t masterConfig;

	    /* Master config */
	    masterConfig.whichCtar = kDSPI_Ctar0;
	    masterConfig.ctarConfig.baudRate = TRANSFER_BAUDRATE;
	    masterConfig.ctarConfig.bitsPerFrame = 8;
	    masterConfig.ctarConfig.cpol = kDSPI_ClockPolarityActiveHigh;
	    masterConfig.ctarConfig.cpha = kDSPI_ClockPhaseFirstEdge;
	    masterConfig.ctarConfig.direction = kDSPI_MsbFirst;
	    masterConfig.ctarConfig.pcsToSckDelayInNanoSec = 1000000000U / TRANSFER_BAUDRATE;
	    masterConfig.ctarConfig.lastSckToPcsDelayInNanoSec = 1000000000U / TRANSFER_BAUDRATE;
	    masterConfig.ctarConfig.betweenTransferDelayInNanoSec =  1000000000U / TRANSFER_BAUDRATE;

	    masterConfig.whichPcs = EXAMPLE_DSPI_MASTER_PCS_FOR_INIT;
	    masterConfig.pcsActiveHighOrLow = kDSPI_PcsActiveLow;

	    masterConfig.enableContinuousSCK = false;
	    masterConfig.enableRxFifoOverWrite = false;
	    masterConfig.enableModifiedTimingFormat = false;
	    masterConfig.samplePoint = kDSPI_SckToSin0Clock;

	    srcClock_Hz = DSPI_MASTER_CLK_FREQ;
	    DSPI_MasterInit(EXAMPLE_DSPI_MASTER_BASEADDR, &masterConfig, srcClock_Hz);

	    /* Set up dspi master */
	    memset(&(dspiEdmaMasterRxRegToRxDataHandle), 0, sizeof(dspiEdmaMasterRxRegToRxDataHandle));
	#if (!(defined(FSL_FEATURE_DSPI_HAS_GASKET) && FSL_FEATURE_DSPI_HAS_GASKET))
	    memset(&(dspiEdmaMasterTxDataToIntermediaryHandle), 0, sizeof(dspiEdmaMasterTxDataToIntermediaryHandle));
	#endif
	    memset(&(dspiEdmaMasterIntermediaryToTxRegHandle), 0, sizeof(dspiEdmaMasterIntermediaryToTxRegHandle));

	    EDMA_CreateHandle(&(dspiEdmaMasterRxRegToRxDataHandle), EXAMPLE_DSPI_MASTER_DMA_BASEADDR, masterRxChannel);
	#if (!(defined(FSL_FEATURE_DSPI_HAS_GASKET) && FSL_FEATURE_DSPI_HAS_GASKET))
	    EDMA_CreateHandle(&(dspiEdmaMasterTxDataToIntermediaryHandle), EXAMPLE_DSPI_MASTER_DMA_BASEADDR,
	                      masterIntermediaryChannel);
	#endif
	    EDMA_CreateHandle(&(dspiEdmaMasterIntermediaryToTxRegHandle), EXAMPLE_DSPI_MASTER_DMA_BASEADDR, masterTxChannel);
	#if (defined(FSL_FEATURE_DSPI_HAS_GASKET) && FSL_FEATURE_DSPI_HAS_GASKET)
	    DSPI_MasterTransferCreateHandleEDMA(EXAMPLE_DSPI_MASTER_BASEADDR, &g_dspi_edma_m_handle, DSPI_MasterUserCallback,
	                                        NULL, &dspiEdmaMasterRxRegToRxDataHandle, NULL,
	                                        &dspiEdmaMasterIntermediaryToTxRegHandle);
	#else
	    DSPI_MasterTransferCreateHandleEDMA(EXAMPLE_DSPI_MASTER_BASEADDR, &g_dspi_edma_m_handle, DSPI_MasterUserCallback,
	                                        NULL, &dspiEdmaMasterRxRegToRxDataHandle,
	                                        &dspiEdmaMasterTxDataToIntermediaryHandle,
	                                        &dspiEdmaMasterIntermediaryToTxRegHandle);
	#endif
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
	masterXfer.configFlags = kDSPI_MasterCtar0 | EXAMPLE_DSPI_MASTER_PCS_FOR_TRANSFER | kDSPI_MasterPcsContinuous;
	if (kStatus_Success != DSPI_MasterTransferEDMA(EXAMPLE_DSPI_MASTER_BASEADDR, &g_dspi_edma_m_handle, &masterXfer))

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
