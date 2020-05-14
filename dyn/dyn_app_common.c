/*
 * dyn_motor.c
 *
 *  Created on: 18 mar. 2020
 *      Author: droma
 *
 * High-level functions common for both dynamixel motors and sensors modules.
 *
 */

#include <stdint.h>

#include "dyn_instr.h"

/**
 * Turn on or off the LED of a given dynamixel module
 *
 * @param[in] id Id of the dynamixel moduel
 * @param[in] val Boolean value for the LED objective state
 */
int dyn_led_control(uint8_t id, bool val) {
	return dyn_write_byte(id, DYN_REG__LED, (uint8_t) val);
}

/**
 * Read the current LED status
 *
 * @param[in] id Id of the dynamixel moduel
 * @param[out] val Current LED status
 */
int dyn_led_read(uint8_t id, uint8_t *val) {
	return dyn_read_byte(id, DYN_REG__LED, val);
}
