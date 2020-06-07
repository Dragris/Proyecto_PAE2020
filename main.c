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
enum state newState = NONE;
enum state oldState = NONE;

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
             * W
             *
             * Miramos si ya hemos encontrado una pared.
             */
            if (found_wall == true) {
                /**
                * Seguimos la pared izquierda.
                */
                switch (thisWall) {
                    case LEFT_W:
                        /**
                         * W_FAR_LEFT
                         *
                         * En este estado la pared se ha alejado demasiado de la pared.
                         * Como es más de lo que se podría alejar el robot significa que hemos pasado por una esquina.
                         * Para corregir nuestro rumbo giramos hacia la izquierda mientras seguimos moviéndonos hacia
                         * adelante.
                         */
                        if(sens_L > 40) {
                            turn(norm_speed, LEFT);
                        }
                        /**
                         * W_MAX_LEFT
                         *
                         * En este estado nos hemos alejado un poco de la pared, superando el límite máximo establecido.
                         * Para corregirlo hemos pensado en aplicar una solución que, en la mayoría de casos, nos permita
                         * ponernos bastante en paralelo para que el robot no haga zig-zag en exceso.
                         *
                         * Para esto lo primero que hacemos es ver si delante detectamos ya una pared, luego hacemos que
                         * el robot gire hacia la pared mientras se mueve hacia adelante y cuando esté la pared a la
                         * distancia que consideramos como obstáculo haremos que gire a la derecha hasta que se pase de
                         * la distancia especificada según si hemos detectado una pared previamente o no.
                         *
                         * Esto nos permite volver a estar en el intervalo de distancia que deseamos y cuando ya estamos
                         * en dicho intervalo nos vuelve a dejar en una trayectoria cerca del pararelo que nos permite
                         * seguir recto durante mayor tiempo.
                         *
                         * Esto, sin embargo, presenta un problema. Cuando hay una pared delante, aunque la detecte, el
                         * robot empezará a hacer zig-zag, esto reduce bastante la velocidad pero, en las simulaciones
                         * que hemos hecho, no permite que el robot se choque ni se aleje en exceso.
                         */
                        else if(sens_L >= obstacle_dist) {
                            int aux_dist;
                            aux_dist = sens_C < 220 ? obstacle_dist : 220;
                            turn(maneuver_speed+20,LEFT);
                            while (sens_C > obstacle_dist || sens_L <= min_obstacle_dist){
                                sens_C = sensorRead(0x1B);
                                sens_L = sensorRead(0x1A);
                            }
                            endlessDorifto(maneuver_speed, RIGHT);
                            sens_C = sensorRead(0x1B);
                            while(sens_C < aux_dist){
                                sens_C = sensorRead(0x1B);
                            }
                            endlessMove(norm_speed, FORWARD);
                        }
                        /**
                         * W_MIN_LEFT
                         *
                         * Nos estamos acercando demasiado a la pared.
                         *
                         * Para corregir esto hacemos que el robot gire un poco a la derecha.
                         * Como nos estamos acercando a la pared, el sensor delantero también debe estar leyendo una
                         * cercanía a la pared, así que para desviarnos de la pared pero sin que sea un angulo demasiado
                         * abierto lo que hacemos es girar hacia la derecha mientras el robot se mueve hacia adelante
                         * hasta que el sensor central lea un valor superior al de la distancia máxima que puede estar
                         * el robot de la pared.
                         */
                        else if(sens_L <= min_obstacle_dist){
                            turn(maneuver_speed+10, RIGHT);
                            while(sens_C < obstacle_dist){
                                sens_C = sensorRead(0X1B);
                            }
                        }

                        sens_C = sensorRead(0x1B);
                        /**
                         * W_FRONT_LEFT
                         *
                         * Nos estamos acercando demasiado a un obstáculo por el frente.
                         *
                         * Para evitar chocar contra el obstáculo que tenemos en frente haremos que el robot gire sobre
                         * sí mismo hasta que deje de detectarlo.
                         */
                        if(sens_C <= obstacle_dist){
                            //Tenemos una pared delante, giramos a la derecha en el sitio hasta ponernos en paralelo
                            endlessDorifto(maneuver_speed, RIGHT);
                            while(sens_C < obstacle_dist){
                                sens_C = sensorRead(0X1B);
                            }
                        }
                        endlessMove(norm_speed, FORWARD);
                        break;

                    /**
                     * Seguimos la pared derecha.
                     */
                    case RIGHT_W:
                        /**
                         * W_FAR_RIGHT
                         *
                         * Esta función es un mirror de W_FAR_LEFT. Ha sido adaptado para seguir la pared derecha.
                         */
                        if(sens_R > 40){ //
                            //La pared a la derecha ha desaparecido, giramos porque la tratamos de esquina
                            turn(norm_speed, RIGHT);
                        }
                        /**
                         * W_MAX_RIGHT
                         *
                         * Esta función es un mirror de W_MAX_LEFT. Ha sido adaptado para seguir la pared derecha.
                         */
                        else if(sens_R >= obstacle_dist){
                            int aux_dist;
                            aux_dist = sens_C < 220 ? obstacle_dist : 220;
                            turn(maneuver_speed+20,RIGHT);
                            while (sens_C > obstacle_dist || sens_R <= min_obstacle_dist){
                                sens_C = sensorRead(0x1B);
                                sens_R = sensorRead(0x1C);
                            }
                            endlessDorifto(maneuver_speed, RIGHT);
                            sens_C = sensorRead(0x1B);

                            while(sens_C < aux_dist){
                                sens_C = sensorRead(0x1B);
                            }
                            endlessMove(norm_speed, FORWARD);

                        }
                        /**
                         * W_MIN_RIGHT
                         *
                         * Esta función es un mirror de W_MIN_LEFT. Ha sido adaptado para seguir la pared derecha.
                         */
                        else if(sens_R <= min_obstacle_dist){
                            turn(maneuver_speed+10, LEFT);
                            while(sens_C < obstacle_dist){
                                sens_C = sensorRead(0X1B);
                            }
                        }

                        sens_C = sensorRead(0x1B);
                        /**
                         * W_FRONT_RIGHT
                         *
                         * Esta función es un mirror de W_FRONT_LEFT. Ha sido adaptado para seguir la pared derecha.
                         */
                        if(sens_C <= obstacle_dist && newState){
                            //Tenemos una pared delante, giramos a la izquierda en el sitio hasta ponernos en paralelo
                            endlessDorifto(maneuver_speed, LEFT);
                            while(sens_C < obstacle_dist){
                                sens_C = sensorRead(0X1B);
                            }
                        }

                        endlessMove(norm_speed, FORWARD);
                        break;

                    default:
                        stopEngines();
                        //TODO PRINT ERROR AND STOP
                        printf("Error: Something went wrong");
                        exit(1);

                }
            }
            /**
             * NW
             *
             * Si no tenemos pared.
             */
            else if (found_wall == false) {
                /*
                 * En este primer bloque miramos qué pared tenemos más cerca.
                 */

                /**
                 * NW_FAR_LEFT
                 *
                 * Si la pared más cercana es la izquierda giraremos hacia ella.
                 * El robot girará sobre sí mismo.
                 */
                if (sens_L < sens_C && sens_L < sens_R){ //
                    endlessDorifto(maneuver_speed, LEFT);
                }
                /**
                 * NW_FAR_RIGHT
                 *
                 * Si la pared más cercana es la derecha giraremos hacia ella.
                 * El robot girará sobre sí mismo.
                 */
                else if (sens_R < sens_C && sens_R < sens_L) {
                    endlessDorifto(maneuver_speed, RIGHT);
                }

                /**
                 * NW_CLOSE_LEFT
                 *
                 * Si se nos acerca un obstáculo por la izquierda, por debajo de la distancia mínima permitida, lo trataremos
                 * como la pared que debemos seguir. Para esto giraremos hacia la derecha, sobre sí mismo, hasta que se
                 * ponga más o menos en paralelo y nos moveremos hacia adelante. Tras esto marcamos que nos hemos encontrado
                 * una pared. Nos es necesario indicar que la pared es la izquierda porque es la pared que seguimos
                 * por default.
                 */
                if (sens_L <= obstacle_dist) {
                    endlessDorifto(maneuver_speed, RIGHT); //Nos ponemos en paralelo con la pared
                    while (sens_C < 200) {
                        sens_C = sensorRead(0X1B);
                    }
                    endlessMove(norm_speed, FORWARD);
                    found_wall = true;
                }
                /**
                 * NW_CLOSE_FRONT
                 *
                 * Si se nos acerca un obstáculo por delante, hacia el centro del intervalo de distancia permitida, lo
                 * trataremos como la pared que vamos a seguir. Como la pared que seguimos por lo general es la izquierda
                 * maniobraremos para dejarla a este lado. Para ello haremos que el robot gire sobre sí mismo hacia la
                 * derecha hasta que no detecte una pared cercana el sensor central. Esto dejará al robot en una ruta
                 * más o menos paralela a la pared. Tras esto el robot se desplazará hacia adelante y marcaremos que hemos
                 * encontrado una pared.
                 *
                 * Hemos distinguido entre este caso y el anterior porque, pese a hacer lo mismo, en un principio iban
                 * a tener diferentes comportamientos, pero luego nos dimos cuenta de que no era necesario y no los
                 * llegamos a unir.
                 */
                else if (sens_C <= obstacle_dist-2) { //
                    //Si nos encontramos una pared de frente priorizamos dejarla a la izquierda del robot
                    endlessDorifto(maneuver_speed, RIGHT);
                    while (sens_C < 200) {
                        sens_C = sensorRead(0X1B);
                    }
                    endlessMove(norm_speed, FORWARD);
                    found_wall = true;
                }
                /**
                 * NW_CLOSE_RIGHT
                 *
                 * Si se nos acerca un obstáculo por la derecha, por debajo de la distancia mínima permitida, lo trataremos
                 * como la pared que debemos seguir. Para esto giraremos hacia la izquierda, sobre sí mismo, hasta que
                 * se ponga más o menos en paralelo y nos moveremos hacia adelante. Tras esto marcamos que nos hemos
                 * encontrado una pared y que es la derecha.
                 */
                else if (sens_R <= obstacle_dist) {
                    //Si nos encontramos la pared por la derecha cambiamos la pared a la derecha
                    endlessDorifto(maneuver_speed, LEFT); //Nos ponemos en paralelo con la pared
                    while (sens_C < obstacle_dist) {
                        sens_C = sensorRead(0X1B);
                    }
                    endlessMove(norm_speed, FORWARD);
                    thisWall = RIGHT_W;
                    found_wall = true;
                }
                /**
                 * NW_FAR_FRONT
                 *
                 * Si ninguna de las anteriores condiciones se cumple suponemos que o no tenemos ningún obstáculo cerca
                 * o que el obstáculo más cercano se encuentra delante pero fuera del intervalo permitido.
                 * En ambos casos hacemos que el robot se mueva hacia adelante, así que lo hemos reducido a un solo estado.
                 */
                else {
                    endlessMove(norm_speed, FORWARD);
                }
            }
            /**
             * ERROR
             *
             * En caso de que ocurra un problema y que perdamos el valor de la variable found_wall pararemos el robot
             * y saldremos de la simulación porque lo más probable es que haya ocurrido un error muy grave.
             */
            else {
                stopEngines();
                //TODO PRINT ERROR AND STOP
                printf("Error: Something went wrong");
                exit(1);
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
