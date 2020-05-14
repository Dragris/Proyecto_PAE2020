/*
 * msp.h
 *
 *  Created on: 18 mar. 2020
 *      Author: droma
 */

#ifndef FAKE_MSP_H_
#define FAKE_MSP_H_

#include <stdbool.h>
#include <stdint.h>

////#define UCA2STATW true
////#define UCBUSY false
#define UCBUSY true
#define UCTXIFG true

extern volatile uint8_t UCA2TXBUF;
extern volatile uint8_t UCA2RXBUF;
extern volatile uint16_t UCA2IFG;
extern volatile uint16_t UCA2STATW;

#endif /* FAKE_MSP_H_ */
