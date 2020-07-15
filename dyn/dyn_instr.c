/*
 * hal_dyn.c
 *
 *  Created on: 18 mar. 2020
 *      Author: droma
 *
 * This file implements the different dynamixel instructions
 */

#include "dyn_instr.h"
#include "dyn_frames.h"

/**
 * Single byte write instruction
 *
 * This function sends a write instruction for a single address position
 * to a given dynamixel module.
 *
 * @param[in] module_id Id of the dynamixel module
 * @param[in] reg_addr Address where the write is performed
 * @param[in] reg_write_val Value written to the previous address
 * @return Error code to be treated at higher levels.
 */
int dyn_write_byte(uint8_t module_id, DYN_REG_t reg_addr, uint8_t reg_write_val) {
	uint8_t parameters[2] ;
	struct RxReturn reply;

	parameters[0] = reg_addr;
	parameters[1] = reg_write_val;
	reply = RxTxPacket(module_id, 2, DYN_INSTR__WRITE, parameters);

	return (reply.tx_err < 1) | reply.time_out;
}

/**
 * Single byte read instruction
 *
 * This function sends a read instruction for a single address position
 * to a given dynamixel module.
 *
 * @param[in] module_id Id of the dynamixel module
 * @param[in] reg_addr Address where the read is performed
 * @param[out] reg_read_val Pointer where the read value is stored
 * @return Error code to be treated at higher levels.
 */
int dyn_read_byte(uint8_t module_id, DYN_REG_t reg_addr, uint8_t* reg_read_val) {
	uint8_t parameters[2];
	struct RxReturn reply;

	parameters[0] = reg_addr;
	parameters[1] = 1;
	reply = RxTxPacket(module_id, 2, DYN_INSTR__READ, parameters);
	*reg_read_val = reply.StatusPacket[5];

	return (reply.tx_err << 1) | reply.time_out;
}

int dyn_read(uint8_t module_id, DYN_REG_t reg_addr, uint8_t reg_read_val[], uint8_t length){
    uint8_t parameters[2]; //Longitud de parámetros está fijada a dos como indica el manual
    struct RxReturn reply; //Struct de la respuesta
    int i; //Counter

    /**
     * Parámetros vienen en orden address + length
     */
    parameters[0] = reg_addr;
    parameters[1] = length;
    reply = RxTxPacket(module_id, 2, DYN_INSTR__READ, parameters);

    for(i = 0; i < length; i++){ //Añadimos los valores al array que recibirá los datos
        reg_read_val[i] = reply.StatusPacket[5+i];
    }
    //Devolvemos el posible error generado
    return (reply.tx_err << 1) | reply.time_out;
}


/**
 * Multi-byte write instruction
 *
 * This function sends a write instruction starting at a given address position
 * with a given length for a dynamixel module.
 *
 * @param[in] module_id Id of the dynamixel module
 * @param[in] reg_addr Address where the write is performed
 * @param[in] val Pointer to the byte array to be written
 * @param[in] len Number of position to be written
 * @return Error code to be treated at higher levels.
 */
int dyn_write(uint8_t module_id, uint8_t reg_address, int param_length, uint8_t *param) {

    uint8_t bParam[param_length+1];
    struct RxReturn response;

    uint8_t bLength = param_length+1;
    uint8_t bInstruction = DYN_INSTR__WRITE;
    bParam[0] = reg_address;
    //Escribimos desde la segunda posición del array, la dirección está en pos[0]
    int i;
    for (i = 1; i < param_length + 1; i++){
        bParam[i] = param[i-1];
    }

    response = RxTxPacket(module_id, bLength, bInstruction, bParam);

    return (response.tx_err < 1) | response.time_out;

}

