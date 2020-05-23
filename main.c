#include <pthread.h>
#include <signal.h>
#include <assert.h>
#include <posicion.h>
#include <stdio.h>

#include "main.h"
#include "dyn/dyn_app_common.h"
#include "dyn_test/dyn_emu.h"
#include "dyn_test/b_queue.h"
#include "joystick_emu/joystick.h"
#include "habitacion_001.h"

uint8_t estado = Ninguno, estado_anterior = Ninguno, finalizar = 0;
uint32_t indice;
bool activate = true, found_wall = false, aux_dead_end = false, dead_end = false;
int norm_speed = 500, maneuver_speed = 300;
int obstacle_dist = 10, min_obstacle_dist = 5; //Let's hope this is 10mm and 5mm
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
            if(sens_L == 0 && sens_C == 0 && sens_R == 0){
                continue; //Si todos los sensores leen 0 por algún error en lectura saltamos el bucle
            }


            if (found_wall){
                switch(thisWall){
                    case LEFT_W:
                        if (sens_L <= min_obstacle_dist && sens_C <= min_obstacle_dist && sens_R <= min_obstacle_dist ){
                            stopEngines(); //Paramos los motores y pasamos al modo camino sin salida
                            dead_end = true;
                            found_wall = false;
                            break; //Salimos del switch
                        }

                        if (sens_L <= min_obstacle_dist){
                            endlessDorifto(maneuver_speed, RIGHT); //Giramos un poco a la derecha para no rozar la pared
                        } else if (sens_C <= obstacle_dist) {
                            endlessDorifto(norm_speed, RIGHT); //Giramos mucho a la derecha para no estrellarnos
                        } else if (sens_R <= min_obstacle_dist){
                            endlessDorifto(maneuver_speed, LEFT); //Giramos a la izquierda un poco para no rozar por la derecha
                        } else if (sens_L >= obstacle_dist){
                            endlessDorifto(maneuver_speed, LEFT); //Giramos a la izquierda para no alejarnos de la pared
                        }
                        break;

                    case RIGHT_W:
                        if (sens_L <= min_obstacle_dist && sens_C <= min_obstacle_dist && sens_R <= min_obstacle_dist ){
                            stopEngines(); //Paramos los motores y pasamos al modo camino sin salida
                            dead_end = true;
                            found_wall = false;
                            break; //Salimos del switch
                        }

                        if (sens_R <= min_obstacle_dist){
                            endlessDorifto(maneuver_speed, LEFT); //Giramos un poco a la derecha para no rozar la pared
                        } else if (sens_C <= obstacle_dist) {
                            endlessDorifto(norm_speed, LEFT); //Giramos mucho a la derecha para no estrellarnos
                        } else if (sens_L <= min_obstacle_dist){
                            endlessDorifto(maneuver_speed, RIGHT); //Giramos a la izquierda un poco para no rozar por la derecha
                        } else if (sens_R >= obstacle_dist){
                            endlessDorifto(maneuver_speed, RIGHT); //Giramos a la izquierda para no alejarnos de la pared
                        }
                        break;
                }


            } else if (dead_end){
                if (sens_L >= min_obstacle_dist && sens_C >= min_obstacle_dist && sens_R >= min_obstacle_dist && !aux_dead_end){
                    //Giramos 180º y seguimos con la pared
                    //TODO giro 180º
                    endlessDorifto(norm_speed,LEFT);
                    found_wall = true;
                    dead_end = false;
                } else {
                    //Nos movemos marcha atrás
                    aux_dead_end = true;
                    endlessMove(norm_speed, REVERSE);
                    switch (thisWall) {
                        case LEFT_W:
                            if (sens_R > min_obstacle_dist){
                                //TODO Giramos 90º a la derecha
                                //Provisional
                                endlessDorifto(norm_speed, RIGHT);
                                aux_dead_end = false;
                            }
                            break;

                        case RIGHT_W:
                            if (sens_L > min_obstacle_dist){
                                //TODO Giramos 90º a la izquierda
                                //Provisional
                                endlessDorifto(norm_speed, LEFT);
                                aux_dead_end = false;
                            }
                            break;
                    }
                }


            } else if (!found_wall) {
                //Si se activa cualquiera de estos if es porque hemos encontrado una pared así que pasaremos al modo "pared"
                if (sens_L <= obstacle_dist) {
                    endlessDorifto(maneuver_speed, LEFT); //Obligamos el encuentro con la pared
                    found_wall = true;
                } else if(sens_C <= obstacle_dist){ //Si nos encontramos una pared de frente priorizamos dejarla a la izquierda del robot
                    endlessDorifto(norm_speed, RIGHT);
                    found_wall = true;
                }
                else if(sens_R <= obstacle_dist){ //Si nos encontramos la pared por la derecha cambiamos la pared a la derecha
                    thisWall = RIGHT_W;
                    found_wall = true;
                }else {
                    endlessMove(norm_speed, FORWARD);
                }
            } else{
                stopEngines();
                //TODO PRINT ERROR
            }

        }


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
        }
    }
    //Signal the emulation thread to stop
    pthread_kill(tid, SIGTERM);
    pthread_kill(jid, SIGTERM);
    printf("Programa terminado\n");
    fflush(stdout);
}
