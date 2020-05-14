/*
 * dyn_vector_emu.h
 *
 *  Created on: 19 mar. 2020
 *      Author: droma
 */

#ifndef DYN_EMU_H_
#define DYN_EMU_H_

#include <stdint.h>
#include <stdbool.h>

// Number of dynamixel devices to be emulated
#define N_DEVICES 3
// Size of the dynamixel memory
#define DYN_MAX_POS 49

//Local variable containing a matrix emulating the dynamixel memories
typedef uint8_t dyn_matrix_t[N_DEVICES][DYN_MAX_POS];
dyn_matrix_t dyn_mem;

bool simulator_finished;

#define ID_MOTOR_L 1
#define MOTOR_L_MEM_ROW (ID_MOTOR_L-1)
#define ID_MOTOR_R 2
#define MOTOR_R_MEM_ROW (ID_MOTOR_R-1)
#define ID_SENSOR 3
#define SENSOR_MEM_ROW (ID_SENSOR-1)

void *dyn_emu(void *vargp);

#endif /* DYN_EMU_H_ */
