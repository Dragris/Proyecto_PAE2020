/*
 * dyn_app_motor.h
 *
 *      Author: Dragris
 *
 */
#ifndef DYNAMIXELLIB_H_
#define DYNAMIXELLIB_H_

#include <stdio.h>
#include <stdint.h>
#include "dyn_instr.h"

#define LEFT_ENGINE 2
#define RIGHT_ENGINE 3


enum dir{
	FORWARD,
	REVERSE,
	RIGHT,
	LEFT,
	BOTH,
	CW,
	CCW
};

void engineLEDOn(enum dir direction);
void engineLEDOff(void);
void engineLEDRead(void);
void endlessMove(int speed, enum dir direction);
void stopEngines(void);
void wheelUnlock(int id_module);
void endlessDorifto(int speed, enum dir direction);
int readRegister(int id_module, int reg_addr, uint8_t* ret_val);
void readSpeed(void);
void turnAmount(int degree, enum dir direction);
void turn(int speed, enum dir direction);

#endif /* DYNAMIXELLIB_H_ */
