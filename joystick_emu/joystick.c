/*
 * joystick.c
 *
 *  Created on: 25 mar. 2020
 *      Author: C. Serre, UB.
 */

#include "joystick.h"
#include <pthread.h>

#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <conio.h>
#include <string.h>
#include <signal.h>
#include "fake_msp.h"
#include "b_queue.h"
#include "main.h"

uint8_t Estado_actual = Ninguno, Anterior = Ninguno, protegido = 0;

static pthread_mutex_t *joystick_mutex;

void Set_estado_anterior(uint8_t valor) {
    pthread_mutex_lock(joystick_mutex); //para empezar a detectar teclas
    Anterior = valor;
    pthread_mutex_unlock(joystick_mutex); //para empezar a detectar teclas
}

void Set_estado_actual(uint8_t valor) {
    pthread_mutex_lock(joystick_mutex); //para empezar a detectar teclas
    Estado_actual = valor;
    pthread_mutex_unlock(joystick_mutex); //para empezar a detectar teclas
}

void Get_estado(uint8_t *estado, uint8_t *anterior) {
    pthread_mutex_lock(joystick_mutex); //para empezar a detectar teclas
    *estado = Estado_actual;
    *anterior = Anterior;
    pthread_mutex_unlock(joystick_mutex); //para empezar a detectar teclas
}

/**
 * Handler to exit the current thread under the appropiate signal
 */
static void handler(int signum) {
    pthread_exit(NULL);
}

/**
 * Thread to emulate the joystick
 */
void *joystick_emu(void *vargp) {
    uint8_t tecla, estado;

    pthread_mutex_t mutex;
    joystick_mutex = &mutex;

    pthread_mutex_init(joystick_mutex, NULL);

    // Add SIGTERM handler to kill the current thread
    signal(SIGTERM, handler);
    while (1) {

        scanf("%c", &tecla);
        switch (tecla) {
            case 0x0A:
            case 0x0D:
                continue; //omitir el retorno, yendo directamente a la siguiente iteracion del while.
            case 'i':
                estado = Up;
                break;
            case 'j':
                estado = Left;
                break;
            case 'k':
                estado = Center;
                break;
            case 'l':
                estado = Right;
                break;
            case 'm':
                estado = Down;
                break;
            case 'a':
                estado = Sw1;
                break;
            case 's':
                estado = Sw2;
                break;
            case 'q':
                estado = Quit;
                break;
        }
        pthread_mutex_lock(joystick_mutex);
        Estado_actual = estado;
        Anterior = Otro;//Me aseguro de que anterior sea diferente de estado}
        pthread_mutex_unlock(joystick_mutex);
    }
}

