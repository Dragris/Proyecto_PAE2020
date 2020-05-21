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
 * Funcion para obtener flag de obstáculo detectado
 */
int getObstacleFlag(){
	byte flags;

	dyn_read_byte(SENSOR_ID, 0x20, &flags);
	return flags;
}
