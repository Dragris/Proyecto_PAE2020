/*
 * dyn_app_sensor.c
 *
 *      Author: Dragris
 *
 * Sensor reading functions
 */

#include "dyn_app_sensor.h"

/**
 * Funcion para ponerle una distancia en la que los sensores activan el flag de obstáculo
 */
void setDistanceReading(unsigned int distance){
	uint8_t param;
	//Dirección de memoria de la distancia en la ROM
	// 0x34 en la RAM
	param= distance;
	dyn_write_byte(SENSOR_ID, 0x14, param);
}

/**
 * Funcion para leer la distancia de detección de los obstáculos.
 * Uso para printear la distancia puesta en setDistanceReading()
 */
int readObsDetDistance(byte position){
	uint8_t distance;

	dyn_read_byte(SENSOR_ID, position, &distance);
	return distance;
}

/**
 * Funcion para leer los sensores individualmente
 */
int sensorRead(byte sensor){
	uint8_t distance;
	dyn_read_byte(SENSOR_ID, sensor, &distance);
	return distance;
}

/**
 * Función para leer múltiples sensores a la vez en un array resultado.
 * Feed array needed length.
 *
 * @param valuearray
 * @param id
 * @param sens
 */
void multiSensorRead(uint8_t *valuearray, uint8_t id, enum dir sens){
    uint8_t tmp[3];
    switch (sens) {
        case left:
            dyn_read_byte(id, 0x1A, valuearray);
            break;
        case right:
            dyn_read_byte(id, 0x1B, valuearray);
            break;
        case center:
            dyn_read_byte(id, 0x1C, valuearray);
            break;
        case left_right:
            dyn_read(id, 0x1A, tmp,3);
            valuearray[0] = tmp[0];
            valuearray[1] = tmp[2];
            break;
        case left_center:
            dyn_read(id, 0x1A, valuearray,2);
            break;
        case center_right:
            dyn_read(id, 0x1B, valuearray,2);
            break;
        case all:
            dyn_read(id, 0x1A, valuearray, 3);
            break;
        default:
            //TODO: Print error/control error
            return;
    }
}

/**
 * Funcion para obtener flag de obstáculo detectado
 */
int getObstacleFlag(){
	byte flags;

	dyn_read_byte(SENSOR_ID, 0x20, &flags);
	return flags;
}
