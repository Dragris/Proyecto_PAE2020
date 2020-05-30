#include <zconf.h>
#include "dyn_app_motor.h"
/**
 *
 * LAS DIRECCIONES RIGHT Y LEFT ESTAN DADAS POR LA DIRECCION
 * DE GIRO, NO POR SU POSICION EN EL ROBOT.
 */

/**
 * Funcion para encender los LEDs de los motores
 * @param direction LEFT || RIGHT || BOTH
 */
void engineLEDOn(enum dir direction) {
	uint8_t param[16];
    //Hacemos una lista auxiliar porque la función de escritura modifica los datos enviados
	uint8_t param_aux[16];
	//0x19 dirección del LED en AX-12
	param[0] = 1;							//Ponemos los LEDs a 1
	param_aux[0] = 1;

	switch(direction) {
	case LEFT:
		dyn_write(3, 0x19, 1, param);	//Enviamos al motor 2 y comprobamos que se reciben los datos
		break;

	case RIGHT:
		dyn_write(2, 0x19, 1, param);	//Enviamos al motor 3 y comprobamos que se reciben los datos
		break;

	case BOTH:
		dyn_write(2, 0x19, 1, param);	//Enviamos al motor 2 y comprobamos que se reciben los datos
		dyn_write(3, 0x19, 1, param_aux);	//Enviamos al motor 3 y comprobamos que se reciben los datos
		break;
	default:
		break;

	}
}

/**
 * Funcion para apagar todos los LEDs de los motores
 */
void engineLEDOff(void) {
	 uint8_t param[16];
     //Hacemos una lista auxiliar porque la función de escritura modifica los datos enviados
	 uint8_t param_aux[16];
	 //0x19 dirección del LED en AX-12
	 param[0] = 0;		//Ponemos los LEDs a 0
	 param_aux[0] = 0;


	 dyn_write(LEFT_ENGINE, 0x19, 1, param);   //Enviamos al motor 2 y comprobamos que se reciben los datos
	 dyn_write(RIGHT_ENGINE, 0x19, 1, param_aux);   //Enviamos al motor 3 y comprobamos que se reciben los datos
}

/**
 * Función auxiliar para ver el estado de los LEDs
 */
void engineLEDRead(void){
	uint8_t left;
    uint8_t right;

    //El registro que indica los LEDs se encuentra en 0x19
	dyn_read_byte(RIGHT_ENGINE, 0x19,  &left);
	dyn_read_byte(LEFT_ENGINE, 0x19, &right);

	printf("\n LED engine 2: %d", right);
	printf("\n LED engine 3: %d\n", left);
}
/**
 * Método para desbloquear los ángulos del motor
 * @param id_module
 */
void wheelUnlock(int id_module){
    uint8_t bParameters[4];

    // Datos a escribir
    bParameters[0] = 0;  // CW_ANGLE_LIMIT_L = 0
    bParameters[1] = 0;  // CW_ANGLE_LIMIT_H = 0
    bParameters[2] = 0;  // CCW_ANGLE_LIMIT_L = 0
    bParameters[3] = 0;  // CCW_ANGLE_LIMIT_H = 0

    // Enviamos los datos
    // Empezamos por la dirección 0x06, CW_ANGLE_LIMIT_L
    dyn_write(id_module, 0x06, 4, bParameters);
}

/**
 * Función que se llamará para darle a cada modulo dynamixel los datos de movimiento necesarios.
 * @param id_module Modulo al que queremos llevar la instruccion
 * @param speed  Velocidad (0, 1023)
 * @param direction CW || CCW
 */
void setEngine(int id_module, int speed, enum dir direction){
    if (speed<1024){ //Límite de velocidad 0x3ff = 1023
        //Partimos la velocidad en dos bytes como en los registros
        uint8_t speed_L, speed_H;
        speed_L = speed;

        //Según la dirección que queramos elegir speed_H cambiará
        //Es el bit nº3 el que controla que sea CW o CCW 0100
        switch(direction){
            case CW:
                //Bit 3 == 0
                speed_H = (speed >> 8);
                break;
            case CCW:
                //Bit 3 == 1
                speed_H = (speed >> 8) + 4;
                break;
            default:
                return;
        }

        uint8_t  bParameters[2];
        bParameters[0] = speed_L;
        bParameters[1] = speed_H;
        /*
         * Para la correcta escritura de las velocidades, según el manual, deben
         * escribirse a la vez 0x20 y 0x21 (MOVING_SPEED_L y MOVING_SPEED_H).
         */
        dyn_write(id_module, 0x20, 3, bParameters);
    }
}

/**
 * Función con la que movemos en línea recta el robot
 * @param speed int [0, 1023]
 * @param direction
 */
void endlessMove(int speed, enum dir direction){

	switch(direction) {
	case FORWARD: //Nos movemos hacia adelante
	    setEngine(LEFT_ENGINE, speed, CW);
	    setEngine(RIGHT_ENGINE, speed, CW);
		engineLEDOn(BOTH);
		break;

	case REVERSE: //Nos movemos hacia atrás
        setEngine(LEFT_ENGINE, speed, CCW);
        setEngine(RIGHT_ENGINE, speed, CCW);
		engineLEDOn(BOTH);
		break;

	default: //En caso de enviar algo diferente paramos al robot por seguridad
        printf("\nCHECK GIVEN DIRECTION!!\n");
        stopEngines();
		break;
	}
}

/**
 * Funcion para parar los motores del robot
 */
void stopEngines(void){
    //Ponemos la velocidad de los motores a 0 y les damos orientación como si fuese adelante
    //La orientación no es necesaria
    setEngine(LEFT_ENGINE, 0, CW);
    setEngine(RIGHT_ENGINE, 0, CW);
    engineLEDOff();
}

/**
 * Función para girar el robot haciendo uso de un solo motor
 * @param speed
 * @param direction  RIGHT || LEFT
 */
void endlessDorifto(int speed, enum dir direction){
	//Paramos los motores y apagamos los LEDs
	engineLEDOff();
	stopEngines();

	switch(direction){
	case RIGHT:
		setEngine(LEFT_ENGINE, speed, CW);
		setEngine(RIGHT_ENGINE, speed, CCW);
		engineLEDOn(RIGHT);
		break;

	case LEFT:
        setEngine(LEFT_ENGINE, speed, CCW);
        setEngine(RIGHT_ENGINE, speed, CW);
        engineLEDOn(LEFT);
		break;

	default:
	    printf("\nCHECK GIVEN DIRECTION!!\n");
	    //Paramos el robot por motivos de seguridad
	    stopEngines();
		break;
	}
}

/**
 * Only use with even speeds
 * @param speed
 * @param direction
 */
void turn(int speed, enum dir direction){
    engineLEDOff();
    stopEngines();

    switch(direction){
        case RIGHT:
            setEngine(LEFT_ENGINE, speed, CW);
            setEngine(RIGHT_ENGINE, speed/2, CW);
            engineLEDOn(RIGHT);
            break;

        case LEFT:
            setEngine(LEFT_ENGINE, speed/2, CW);
            setEngine(RIGHT_ENGINE, speed, CW);
            engineLEDOn(LEFT);
            break;

        default:
            printf("\nCHECK GIVEN DIRECTION!!\n");
            //Paramos el robot por motivos de seguridad
            stopEngines();
            break;
    }
}

/**
 * Funcion auxiliar para leer la velocidad de los motores.
 * Da la velocidad por pantalla, para testear con assserts usar readRegister.
 */
void readSpeed(void){
    uint8_t left_L;
    uint8_t left_H;
    uint8_t right_L;
    uint8_t right_H;
    //Pedimos la información indiviual de cada registro.
    //Esto lo separa en speed_L y speed_H
	dyn_read_byte(RIGHT_ENGINE, 0x20, &left_L);
	dyn_read_byte(LEFT_ENGINE, 0x20, &right_L);
	dyn_read_byte(RIGHT_ENGINE, 0x21, &left_H);
    dyn_read_byte(LEFT_ENGINE, 0x21, &right_H);

	printf("\n Speed engine 2: %d \tDireccion: %d", right_L, right_H);
	printf("\n Speed engine 3: %d \tDireccion: %d\n", left_L, left_H);
}

/**
 * Funcion para leer un registro especificado de manera individual.
 * Solo usar para testing.
 * @param id_module
 * @param reg_addr
 * @return
 */
int readRegister(int id_module, int reg_addr, uint8_t* ret_val){
    return dyn_read_byte(id_module, reg_addr, ret_val);
}

void turnAmount(int degree, enum dir direction){
    endlessDorifto(150, direction);
    //Esperamos X grados * 28.5
    usleep(degree*28.5*1000); //Esperamos microsegundos *1000 para ms
    stopEngines();
}