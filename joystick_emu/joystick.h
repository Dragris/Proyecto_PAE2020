/*
 * joystick.h
 *
 *  Created on: 25 mar. 2020
 *      Author: C. Serre, UB
 */

#ifndef JOYSTICK_H_
#define JOYSTICK_H_

#include <stdint.h>

enum {
    Ninguno, Up, Down, Left, Right, Center, Sw1, Sw2, Otro, Quit
};

void Get_estado(uint8_t *estado, uint8_t *anterior);

void Set_estado_anterior(uint8_t valor);

void Set_estado_actual(uint8_t valor);

void *joystick_emu(void *vargp);

#endif /* JOYSTICK_H_ */
