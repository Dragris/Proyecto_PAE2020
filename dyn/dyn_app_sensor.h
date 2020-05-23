#ifndef DYN_SENSOR_H_
#define DYN_SENSOR_H_

#include <stdio.h>
#include <stdint.h>
#include "dyn_instr.h"

typedef uint8_t byte;


void setDistanceReading(unsigned int distance);
int readObsDetDistance(byte position);
int sensorRead(byte sensor);
int getObstacleFlag();


#endif /* DYN_SENSOR_H_ */
