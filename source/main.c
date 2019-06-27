/*
 * main.c
 *
 *  Created on: Jun 17, 2019
 *      Author: wggowell
 */
#include "clock_config.h"
#include "board.h"
#include "perfs.h"
#include "pattern.h"
#include "fsl_common.h"
#include "fsl_port.h"
#include "pin_mux.h"

void initPins(){
	 /* Port B Clock Gate Control: Clock enabled */
	    CLOCK_EnableClock(kCLOCK_PortB);

	/* PORTB16 (pin E10) is configured as UART0_RX */
	PORT_SetPinMux(PORTB, 16U, kPORT_MuxAlt3);

	/* PORTB17 (pin E9) is configured as UART0_TX */
	PORT_SetPinMux(PORTB, 17U, kPORT_MuxAlt3);

    SIM->SOPT5 = ((SIM->SOPT5 &
                   /* Mask bits to zero which are setting */
                   (~(SIM_SOPT5_UART0TXSRC_MASK)))

                  /* UART 0 transmit data source select: UART0_TX pin. */
                  | SIM_SOPT5_UART0TXSRC(SOPT5_UART0TXSRC_UART_TX));
}

int main(void){
	initPins();
    BOARD_BootClockRUN();
    BOARD_InitDebugConsole();
    initPerfs();
    initPatterns();
    runPatterns();
    while(1){

    }
    return 0;

}

