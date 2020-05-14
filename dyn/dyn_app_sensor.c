#include "dyn_app_sensor.h"

/**
 * Funcion para ponerle una distancia en la que los sensores activan el flag de obstáculo
 */
void setDistanceReading(byte ID, unsigned int distance){
	uint8_t param;
	//Dirección de memoria de la distancia en la ROM
	// 0x34 en la RAM
	param= distance;
	dyn_write_byte(ID, 0x14, param);
}

/**
 * Funcion para leer la distancia de detección de los obstáculos.
 * Uso para printear la distancia puesta en setDistanceReading()
 */
int readObsDetDistance(int ID, byte position){
	uint8_t distance;

	dyn_read_byte(ID, position, &distance);
	return distance;
}

/**
 * Funcion para leer los sensores individualmente
 */
int sensorRead(byte ID, byte sensor){
	uint8_t distance;

	dyn_read_byte(ID, sensor, &distance);
	return distance;
}

/**
 * Funcion para obtener flag de obstáculo detectado
 */
int getObstacleFlag(byte ID){
	byte flags;

	dyn_read_byte(ID, 0x20, &flags);
	return flags;
}
