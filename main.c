#include <pthread.h>
#include <signal.h>
#include <assert.h>
#include <posicion.h>
#include <stdio.h>
#include <math.h>

#include "main.h"
#include "dyn/dyn_app_common.h"
#include "dyn_test/dyn_emu.h"
#include "dyn_test/b_queue.h"
#include "joystick_emu/joystick.h"
#include "habitacion_001.h"

uint8_t estado = Ninguno, estado_anterior = Ninguno, finalizar = 0;
uint32_t indice;
bool activate = true, corner = false;
int norm_speed = 100, maneuver_speed = 10, found_wall;
int obstacle_dist = 7, min_obstacle_dist = 4; //8mm and 4mm
enum wall thisWall = LEFT_W; //Default wall must be to robot's left

/**
 * main.c
 */
int main(void) {
    pthread_t tid, jid;
    uint8_t tmp;

    //Init queue for TX/RX data
    init_queue(&q_tx);
    init_queue(&q_rx);

    //Start thread for dynamixel module emulation
    // Passing the room information to the dyn_emu thread
    pthread_create(&tid, NULL, dyn_emu, (void *) datos_habitacion);
    pthread_create(&jid, NULL, joystick_emu, (void *) &jid);

    //Testing some high level function
    printf("\nSetting LED to 0 \n");
    dyn_led_control(1, 0);
    printf("\nGetting LED value \n");
    dyn_led_read(1, &tmp);
    assert(tmp == 0);
    printf("\nSetting LED to 1 \n");
    dyn_led_control(1, 1);
    printf("\nGetting LED value \n");
    dyn_led_read(1, &tmp);
    assert(tmp == 1);

    printf("\n************************\n");
    printf("Test passed successfully\n");

    printf("\nDimensiones habitacion %d ancho x %d largo mm2\n", ANCHO, LARGO);
    printf("En memoria: %I64u B = %I64u MiB\n", sizeof(datos_habitacion), sizeof(datos_habitacion) >> 20);

    printf("Pulsar 'q' para terminar, qualquier tecla para seguir\n");
    fflush(stdout);//	return 0;

    //Unlock infinite spin of engines and we stop them
    wheelUnlock(LEFT_ENGINE);
    wheelUnlock(RIGHT_ENGINE);
    stopEngines();



    //endlessMove(norm_speed, FORWARD);
    while (estado != Quit) {
        if (simulator_finished) {
            break;
        }
        /**
         * Revisamos que tengamos el bot activado.
         * Si está desactivado se quedará quieto.
         */
        if(activate) {
            //Fetching sensor reads
            int sens_L = sensorRead(0X1A);
            int sens_C = sensorRead(0X1B);
            int sens_R = sensorRead(0X1C);


            /**
             * En todas los "subestados" en los que se realiza un giro, después del giro damos orden de ir hacia adelante
             * esto es porque el giro es sobre el centro del robot, no sobre la marcha.
             */

            if (found_wall == true) {
                switch (thisWall) {
                    case LEFT_W:
                        if(sens_L > 40){
                            //La pared a la izquierda ha desaparecido, giramos porque la tratamos de esquina
                            turn(norm_speed, LEFT);
                            endlessMove(norm_speed, FORWARD);
                        }else if(sens_L >= obstacle_dist){
                            //TODO REVISAR PARA QUE NO DEPENDA DE SENS_C POR SI TIENE UNA PARED DELANTE
                            int aux_dist;
                            aux_dist = sens_C < 220 ? obstacle_dist : 220;
                            turn(maneuver_speed+20,LEFT);
                            while (sens_C > obstacle_dist || sens_L <= min_obstacle_dist){
                                sens_C = sensorRead(0x1B);
                                sens_L = sensorRead(0x1A);
                            }
                            //endlessMove(20, FORWARD);
                            endlessDorifto(maneuver_speed, RIGHT);
                            sens_C = sensorRead(0x1B);

                            while(sens_C < aux_dist){
                                sens_C = sensorRead(0x1B);
                            }
                            endlessMove(norm_speed, FORWARD);

                        } else if(sens_L <= min_obstacle_dist){
                            turn(maneuver_speed+10, RIGHT);
                            while(sens_C < obstacle_dist){
                                sens_C = sensorRead(0X1B);
                            }
                            endlessMove(norm_speed,FORWARD);
                        }

                        sens_C = sensorRead(0x1B);
                        if(sens_C <= obstacle_dist){
                            //Tenemos una pared delante, giramos a la derecha en el sitio hasta ponernos en paralelo
                            endlessDorifto(maneuver_speed, RIGHT);
                            while(sens_C < obstacle_dist){
                                sens_C = sensorRead(0X1B);
                            }
                            endlessMove(norm_speed, FORWARD);
                        }
                        break;

                    case RIGHT_W:

                        break;

                }
            } else if (found_wall == false) {
                //Si se activa cualquiera de estos if es porque hemos encontrado una pared así que pasaremos al modo "pared"
                if (sens_C < sens_L && sens_C < sens_R){
                   //Do nothing (será el else en la comparación de mínimos)
                } else if (sens_L < sens_C && sens_L < sens_R){
                    endlessDorifto(maneuver_speed, LEFT);
                } else if (sens_R < sens_C && sens_R < sens_L) {
                    endlessDorifto(maneuver_speed, RIGHT);
                }

                if (sens_L <= min_obstacle_dist) {
                    endlessDorifto(maneuver_speed, RIGHT); //Nos ponemos en paralelo con la pared
                    while (sens_C < 200) {
                        sens_C = sensorRead(0X1B);
                    }
                    endlessMove(norm_speed, FORWARD);
                    found_wall = true;
                } else if (sens_C <= obstacle_dist-2) { //Si nos encontramos una pared de frente priorizamos dejarla a la izquierda del robot
                    endlessDorifto(maneuver_speed, RIGHT);
                    while (sens_C < 200) {
                        sens_C = sensorRead(0X1B);
                    }
                    endlessMove(norm_speed, FORWARD);
                    found_wall = true;
                } else if (sens_R <= min_obstacle_dist) { //Si nos encontramos la pared por la derecha cambiamos la pared a la derecha
                    endlessDorifto(maneuver_speed, RIGHT); //Nos ponemos en paralelo con la pared
                    while (sens_C < obstacle_dist) {
                        sens_C = sensorRead(0X1B);
                    }
                    endlessMove(norm_speed, FORWARD);
                    found_wall = true;
                } else {
                    endlessMove(norm_speed, FORWARD);
                }
            } else {
                stopEngines();
                //TODO PRINT ERROR AND STOP
            }
        }


/*
        Get_estado(&estado, &estado_anterior);
        if (estado != estado_anterior) {
            Set_estado_anterior(estado);
            printf("estado = %d\n", estado);
            switch (estado) {
                case Sw1:
                    printf("Boton Sw1 ('a') apretado\n");
                    dyn_led_control(1, 1); //Probaremos de encender el led del motor 2
                    printf("\n");
                    break;
                case Sw2:
                    printf("Boton Sw2 ('s') apretado\n");
                    dyn_led_control(1, 0); //Probaremos de apagar el led del motor 2
                    printf("\n");
                    break;
                case Up:
                    //Activamos el bot
                    activate = 1;
                    break;
                case Down:
                    //Desactivamos y paramos el bot
                    activate = 0;
                    stopEngines();
                    break;
                case Left:
                    //Comprobaremos si detectamos las esquinas de la pared izquierda:
                    printf("Esquina inferior izquierda:\n");
                    printf("(1, 1): %d (fuera pared)\n", obstaculo(1, 1, datos_habitacion));
                    printf("(0, 1): %d (pared izq.)\n", obstaculo(0, 1, datos_habitacion));
                    printf("(1, 0): %d (pared del.)\n", obstaculo(1, 0, datos_habitacion));
                    printf("(0, 0): %d (esquina)\n", obstaculo(0, 0, datos_habitacion));
                    printf("Esquina superior izquierda:\n");
                    printf("(1, 4094): %d (fuera pared)\n", obstaculo(1, 4094, datos_habitacion));
                    printf("(0, 4094): %d (pared izq.)\n", obstaculo(0, 4094, datos_habitacion));
                    printf("(1, 4095): %d (pared fondo.)\n", obstaculo(1, 4095, datos_habitacion));
                    printf("(0, 4095): %d (esquina)\n", obstaculo(0, 4095, datos_habitacion));
                    break;
                case Right:
                    //Comprobaremos si detectamos las esquinas de la pared derecha:
                    printf("Esquina inferior derecha:\n");
                    printf("(4094, 1): %d (fuera pared)\n", obstaculo(4094, 1, datos_habitacion));
                    printf("(4094, 0): %d (pared del.)\n", obstaculo(4094, 0, datos_habitacion));
                    printf("(4095, 1): %d (pared der.)\n", obstaculo(4095, 1, datos_habitacion));
                    printf("(4095, 0): %d (esquina)\n", obstaculo(4095, 0, datos_habitacion));
                    printf("Esquina superior derecha:\n");
                    printf("(4094, 4094): %d (fuera pared)\n", obstaculo(4094, 4094, datos_habitacion));
                    printf("(4094, 4095): %d (pared fondo)\n", obstaculo(4094, 4095, datos_habitacion));
                    printf("(4095, 4094): %d (pared der.)\n", obstaculo(4095, 4094, datos_habitacion));
                    printf("(4095, 4095): %d (esquina)\n", obstaculo(4095, 4095, datos_habitacion));
                    break;
                case Center:

                    break;
                case Quit:
                    printf("Adios!\n");
                    break;
                    //etc, etc...
            }
            fflush(stdout);
        }*/
    }
    //Signal the emulation thread to stop
    pthread_kill(tid, SIGTERM);
    pthread_kill(jid, SIGTERM);
    printf("Programa terminado\n");
    fflush(stdout);
}
