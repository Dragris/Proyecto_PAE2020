/*
 * hal_dyn_uart_emu.h
 *
 *  Created on: 22 mar. 2020
 *      Author: droma
 */

#ifndef HAL_DYN_UART_EMU_H_
#define HAL_DYN_UART_EMU_H_

#include <stdbool.h>
#include <stdint.h>
#include "dyn/dyn_frames.h"

void Sentit_Dades_Rx_emu(void);

void Sentit_Dades_Tx_emu(void);

void TxUAC2_emu(byte bTxdData);

void rx_uart_byte_emu(struct RxReturn *respuesta);

#endif /* HAL_DYN_UART_EMU_H_ */

