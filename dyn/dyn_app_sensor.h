/*
 * dyn_app_sensor.h
 *
 *      Author: Dragris
 *
 */
#ifndef DYN_SENSOR_H_
#define DYN_SENSOR_H_

#include <stdio.h>
#include <stdint.h>
#include "dyn_instr.h"

#define SENSOR_ID 1
typedef uint8_t byte;

enum dir{
    left,
    right,
    center,
    left_right,
    left_center,
    center_right,
    all
};

void setDistanceReading(unsigned int distance);
int readObsDetDistance(byte position);
int sensorRead(byte sensor);
int getObstacleFlag();


#endif /* DYN_SENSOR_H_ */
