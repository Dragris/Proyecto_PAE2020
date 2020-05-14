/*
 * hal_dyn_uart_emu.c
 *
 *  Created on: 22 mar. 2020
 *      Author: droma
 */

#include <stdio.h>
#include <stdint.h>
#include <unistd.h>
#include "dyn_test/b_queue.h"
#include "main.h"
#include "fake_msp.h"
#include "dyn/dyn_frames.h"

volatile uint8_t UCA2TXBUF = 0;
volatile uint8_t UCA2RXBUF = 0;
volatile uint16_t UCA2IFG = UCTXIFG;
volatile uint16_t UCA2STATW = 0;

void Sentit_Dades_Rx_emu(void) {
    printf("Changed direction to RX\n");
}

void Sentit_Dades_Tx_emu(void) {
    printf("Changed direction to TX\n");
}

void TxUAC2_emu(byte bTxdData) {
    QUEUE_RET ret = QUEUE_ERR;
	UCA2IFG &= ~UCTXIFG; //Buffer de transmision = lleno
	UCA2STATW |= UCBUSY; //Linea = ocupada
    while (ret != QUEUE_OK) {
        ret = queue_push(bTxdData, &q_tx);
    }
    usleep(20);
}

/**
 * Read a byte from the thread-safe queue and place it
 * inside the Statuspacket of RxReturn
 */
void rx_uart_byte_emu(struct RxReturn *respuesta) {
    QUEUE_RET ret = QUEUE_ERR;
    while (ret != QUEUE_OK) {
        ret = queue_pop(&(respuesta->StatusPacket[respuesta->idx]), &q_rx);
    }
    respuesta->idx++;
    usleep(20);
}
