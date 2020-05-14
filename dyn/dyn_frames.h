/*
 * hal_uart.h
 *
 *  Created on: 18 mar. 2020
 *      Author: droma
 */

#ifndef DYN_FRAMES_H_
#define DYN_FRAMES_H_

#include <stdint.h>
#include <stdbool.h>

typedef uint8_t byte;

typedef struct RxReturn {
	byte StatusPacket[32];
	uint8_t idx;
	bool time_out;
	bool tx_err;
} RxReturn;

/* funcions per canviar el sentit de les comunicacions */
void Sentit_Dades_Rx(void);
void Sentit_Dades_Tx(void);
void TxUAC2(byte bTxdData);

void Activa_Timer_TimeOut();
void Reset_Timeout();
bool TimeOut(uint16_t cnt);

//TxPacket()  3 paràmetres: ID del Dynamixel, Mida dels paràmetres, Instruction byte. torna la mida del "Return packet"
byte TxPacket(byte bID, byte bParameterLength, byte bInstruction,
		const byte * Parametros);
struct RxReturn RxPacket(void);

struct RxReturn RxTxPacket(byte bID, byte bParameterLength, byte bInstruction,
		const byte *Parametros);

#endif /* DYN_FRAMES_H_ */

