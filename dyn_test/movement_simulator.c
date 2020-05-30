/*
 * movement_simulator.c
 *
 *  Created on: 16 d’abr. 2020
 *      Author: droma
 */

#include <time.h>
#include <stdbool.h>
#include <stdint.h>
#include <math.h>
#include <stdio.h>
#include <pthread.h>

#include "main.h"
#include "movement_simulator.h"
#include "posicion.h"
#include "dyn_instr.h"
#include "dyn_emu.h"

#define OUTPUT_FILE "../movement.log"
FILE *fichero;

static clock_t t_last_upd;

static _robot_pos_t robot_pos_str;

#define L_AXIS ((float) 1)
#define DELTA_T ((float) SIM_STEP_MS_TIME/1000)

//114 rpm -> 12 rad/s, r = 2cm -> v_lin_max ~ 240 mm/s
//#define CNTS_2_MM (240/5/1023)
#define CNTS_2_MM ((float) 1.0/(DELTA_T*1023))

/** Returns millisecond difference between t2 and t1, i.e. t2-t1
 *
 * @param t1
 * @param t2
 * @return Millisecond time difference
 */
int32_t timediff(clock_t t1, clock_t t2) {
    int32_t elapsed;
    elapsed = (int32_t) ((((double) t2 - t1) / CLOCKS_PER_SEC) * 1000);
    return elapsed;
}

/** Verify if a given number of milliseconds has passed between current time and externally given time
 *
 * @param t1 External time
 * @param miliseconds Number of milliseconds required to have passed to return true
 * @param true_elapsed_time Real elapsed time
 * @return True if t1 - now > milliseconds
 */
bool elapsed_time(clock_t t1, uint32_t milliseconds, int32_t *true_elapsed_time) {
    clock_t t2 = clock();
    *true_elapsed_time = timediff(t1, t2);
    if (*true_elapsed_time > milliseconds) {
        return true;
    } else {
        return false;
    }
}

/** Reads from the dynamixel memory the speed of a endless turning wheel
 *
 * @param v Pointer were the signed speed is stored
 * @param motor_id ID of the Dynamixel module
 */
void _speed_dyn_2_speed_int(int16_t *v, uint8_t motor_id) {
    *v = dyn_mem[motor_id][DYN_REG__GOAL_SPEED_L];
    *v |= ((dyn_mem[motor_id][DYN_REG__GOAL_SPEED_H] & 0x03) << 8);
    if (dyn_mem[motor_id][DYN_REG__GOAL_SPEED_H] & 0x04) {
        *v *= -1;
    }
}

/** Read the speed of the dynamixel modules and store them inside the position structure
 *
 */
void read_speed() {
    _speed_dyn_2_speed_int(&robot_pos_str.iv_l, MOTOR_L_MEM_ROW);
    _speed_dyn_2_speed_int(&robot_pos_str.iv_r, MOTOR_R_MEM_ROW);

//    robot_pos_str.iv_l = 1023;
//    robot_pos_str.iv_r = 1023;
}

/** Update the position and orientation of the robot using two wheel differential drive kinematics
 *
 */
void calculate_new_position() {
    // http://www.cs.columbia.edu/~allen/F15/NOTES/icckinematics.pdf
    read_speed();

    robot_pos_str.v_l = CNTS_2_MM * robot_pos_str.iv_l;
    robot_pos_str.v_r = CNTS_2_MM * robot_pos_str.iv_r;

    //int test_L = robot_pos_str.v_l;
    //int test_R = robot_pos_str.v_r;

    if (robot_pos_str.iv_l == robot_pos_str.iv_r) {
        robot_pos_str.x_p += robot_pos_str.v_l * DELTA_T * cos(robot_pos_str.theta);
        robot_pos_str.y_p += robot_pos_str.v_r * DELTA_T * sin(robot_pos_str.theta);
    } else {
        robot_pos_str.r = (L_AXIS / 2) * (robot_pos_str.v_l + robot_pos_str.v_r)
                          / (robot_pos_str.v_r - robot_pos_str.v_l);
        robot_pos_str.w = (robot_pos_str.v_r - robot_pos_str.v_l) / L_AXIS;
        robot_pos_str.icc_x = robot_pos_str.x_p
                              - robot_pos_str.r * sin(robot_pos_str.theta);
        robot_pos_str.icc_y = robot_pos_str.y_p
                              + robot_pos_str.r * cos(robot_pos_str.theta);
        robot_pos_str.x_p = cos(robot_pos_str.w * DELTA_T) * (robot_pos_str.x_p - robot_pos_str.icc_x)
                            - sin(robot_pos_str.w * DELTA_T) * (robot_pos_str.y_p - robot_pos_str.icc_y)
                            + robot_pos_str.icc_x;
        robot_pos_str.y_p = sin(robot_pos_str.w * DELTA_T) * (robot_pos_str.x_p - robot_pos_str.icc_x)
                            + cos(robot_pos_str.w * DELTA_T) * (robot_pos_str.y_p - robot_pos_str.icc_y)
                            + robot_pos_str.icc_y;

        robot_pos_str.theta += robot_pos_str.w * DELTA_T;
        if (robot_pos_str.theta < -M_PI) {
            robot_pos_str.theta += 2 * M_PI;
        } else if (robot_pos_str.theta > M_PI) {
            robot_pos_str.theta -= 2 * M_PI;
        }
    }

    robot_pos_str.x = (uint16_t) round(robot_pos_str.x_p);
    robot_pos_str.y = (uint16_t) round(robot_pos_str.y_p);

#if DEBUG_LEVEL > 3
    fprintf(fichero, "#%d, %d\n", robot_pos_str.x, robot_pos_str.y);
#endif
#if DEBUG_LEVEL > 2
    fprintf(fichero, "%.2f, %.2f, %.3f, %.2f, %.2f\n", robot_pos_str.x_p, robot_pos_str.y_p,
            robot_pos_str.theta, robot_pos_str.v_l, robot_pos_str.v_r);
    fflush(fichero);
#endif

}

/** Update the sensor data taking into account the new position
 *
 */
void update_sensor_data() {
    distance(&robot_pos_str, &dyn_mem[SENSOR_MEM_ROW][DYN_REG__IR_LEFT],
             &dyn_mem[SENSOR_MEM_ROW][DYN_REG__IR_CENTER],
             &dyn_mem[SENSOR_MEM_ROW][DYN_REG__IR_RIGHT]);
}


/** Initialization function for the simulator robot kinematics
 *
 */
void init_movement_simulator(const uint32_t *world) {
    //TODO: If required, change to appropriate initial conditions!!
    robot_pos_str.x = 50;
    robot_pos_str.y = 980;
    robot_pos_str.x_p = (float) robot_pos_str.x;
    robot_pos_str.y_p = (float) robot_pos_str.y;
    robot_pos_str.theta = M_PI_2;
    robot_pos_str.sim_step = 0;
    robot_pos_str.world = world;
    simulator_finished = false;

    t_last_upd = clock();

#if DEBUG_LEVEL > 2
    fichero = fopen(OUTPUT_FILE, "w+"); //creacion del fichero de salida, en escritura
    fprintf(fichero, "# x, y, theta, v_l, v_r\n");
#endif
}

/** Update, if required, the position and sensor information
 *
 */
void update_movement_simulator_values() {
    int32_t true_elapsed_time;
    uint32_t objective_delay = SIM_STEP_MS_TIME;

    if (elapsed_time(t_last_upd, objective_delay, &true_elapsed_time)) {
        objective_delay -= (true_elapsed_time - SIM_STEP_MS_TIME);
        t_last_upd = clock();
        robot_pos_str.sim_step++;

        calculate_new_position();
        check_out_of_bounds();
        update_sensor_data();
        check_colision();
#if DEBUG_LEVEL > 2
        check_simulation_end();
#endif
    }
}

/** Verify if a collision happens at the new position
 *
 */
void check_colision() {
    if (obstaculo(robot_pos_str.x, robot_pos_str.y, robot_pos_str.world)) {
        printf("***** COLLISION DETECTED AT (%u, %u) simulator step %I64u\n", robot_pos_str.x, robot_pos_str.y,
               robot_pos_str.sim_step);
    }
}

/** Verify we are not getting outside of the room
 *
 */
void check_out_of_bounds() {
    if (robot_pos_str.x > ANCHO || robot_pos_str.y > ANCHO ) {
        printf("***** LEAVING ROOM... STOPPING SIMULATOR\n");
        fflush(stdout);
        end_simulator();
    }
}

/** Check if the maximum simulation time has been reached
 *
 */
void check_simulation_end() {
    if (MAX_SIM_STEPS != 0 && robot_pos_str.sim_step >= MAX_SIM_STEPS) {
        printf("***** SIMULATION END REACHED. STOPPING SIMULATOR\n");
        fflush(stdout);
        end_simulator();
    }
}

/** Simulation clean-up
 *
 */
void end_simulator() {
    simulator_finished = true;
    fclose(fichero);
    pthread_exit(NULL);
}

