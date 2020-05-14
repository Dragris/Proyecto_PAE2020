#ifndef DYN_SENSOR_H_
#define DYN_SENSOR_H_

#include <stdio.h>
#include <stdint.h>
#include "dyn_instr.h"

#define READ 0x02
typedef uint8_t byte;


void setDistanceReading(byte ID, unsigned int distance);
int readObsDetDistance(int ID, byte position);
int sensorRead(byte ID, byte sensor);
int getObstacleFlag(byte ID);


#endif /* DYN_SENSOR_H_ */
