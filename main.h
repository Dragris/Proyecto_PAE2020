/*
 * main.h
 *
 *  Created on: 19 mar. 2020
 *      Author: droma
 */
#include <dyn_app_motor.h>
#include <dyn_app_sensor.h>

//For sensor and engine ID go to each .h file.

enum wall{
    LEFT_W,
    RIGHT_W
};
/**
 * NW = No Wall
 * W = Wall
 */
enum state{
    NONE,
    NW_FAR_LEFT,
    NW_FAR_RIGHT,
    NW_FAR_FRONT,
    NW_CLOSE_LEFT,
    NW_CLOSE_FRONT,
    NW_CLOSE_RIGHT,
    W_FAR_LEFT,
    W_MIN_LEFT,
    W_MAX_LEFT,
    W_FRONT_LEFT,
    W_FAR_RIGHT,
    W_MIN_RIGHT,
    W_MAX_RIGHT,
    W_FRONT_RIGHT
};

#ifndef MAIN_H_
#define MAIN_H_

void cambiaEstado(int, int, int);
/* Set project wide debug level
 *   1.
 *   2. Store movement positions
 *   3. Print distance information
 *   4.
 */
#define DEBUG_LEVEL 5

#endif /* MAIN_H_ */
