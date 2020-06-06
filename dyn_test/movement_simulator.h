/*
 * movement_simulator.h
 *
 *  Created on: 16 d�abr. 2020
 *      Author: droma
 */

#ifndef DYN_TEST_MOVEMENT_SIMULATOR_H_
#define DYN_TEST_MOVEMENT_SIMULATOR_H_

#define DYN_REG__GOAL_SPEED_L 0x20
#define DYN_REG__GOAL_SPEED_H 0x21

#define DYN_REG__IR_LEFT 0x1A
#define DYN_REG__IR_CENTER 0x1B
#define DYN_REG__IR_RIGHT 0x1C

#include <stdbool.h>
#include <stdint.h>
#include <time.h>
#include "dyn_emu.h"

#define SIM_STEP_MS_TIME 10
#define MAX_SIM_STEPS 150000

typedef struct _robot_pos {
    uint16_t x;
    uint16_t y;
    float theta;
    int16_t iv_l;
    int16_t iv_r;
    float v_l;
    float v_r;
    double r;
    double w;
    double icc_x;
    double icc_y;
    double x_p;
    double y_p;
    uint64_t sim_step;
    const uint32_t *world;
} _robot_pos_t;

void init_movement_simulator(const uint32_t *world);

void update_movement_simulator_values();

void end_simulator();

void check_colision();

void check_simulation_end();

void check_out_of_bounds();

int32_t timediff(clock_t t1, clock_t t2);

bool elapsed_time(clock_t t1, uint32_t miliseconds, int32_t *true_elapsed_time);

#endif /* DYN_TEST_MOVEMENT_SIMULATOR_H_ */
