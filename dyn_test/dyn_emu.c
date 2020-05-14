/*
 * dyn_emu.c
 *
 *  Created on: 19 mar. 2020
 *      Author: droma
 *
 * Emulate the Dynamixel modules
 */

#include "dyn_emu.h"

#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <signal.h>
#include <time.h>

#include "movement_simulator.h"
#include "fake_msp.h"
#include "b_queue.h"
#include "main.h"
#include "dyn/dyn_instr.h"

// State machine to implement the data transmission/reception
typedef enum _rx_fsm_type {
    FSM_RX__HEADER_1 = 0,
    FSM_RX__HEADER_2,
    FSM_RX__ID,
    FSM_RX__LEN,
    FSM_RX__INSTR,
    FSM_RX__PARAMS,
    FSM_RX__CHKSM,
    FSM_TX__HEADER_1,
    FSM_TX__HEADER_2,
    FSM_TX__ID,
    FSM_TX__LEN,
    FSM_TX__ERR,
    FSM_TX__PARAMS,
    FSM_TX__CHKSM,
    FSM__LEN,
} FSM_t;

//Definition of the possible status packet error bits
typedef enum _err_bits {
    ERR_INSTR = 0x40,
    ERR_OVRLD = 0x20,
    ERR_CHKSUM = 0x10,
    ERR_RANGE = 0x08,
    ERR_OVRHT = 0x04,
    ERR_ANGLMT = 0x02,
    ERR_VOLTG = 0x01,
} ERR_BITS_t;

//Start of frame of the dynamixel modules
static const uint8_t dyn_header[] = {0xFF, 0xFF};

// Structure of the header from the dynamixel instruction packets
typedef struct __attribute__((__packed__)) _instr_packet_header {
    uint8_t header[sizeof(dyn_header)];
    uint8_t id;
    uint8_t len;
    uint8_t instr;
} instr_pck_header_t;

// Structure of the header from the dynamixel status packets
typedef struct __attribute__((__packed__)) _status_packet_header {
    uint8_t header[sizeof(dyn_header)];
    uint8_t id;
    uint8_t len;
    uint8_t err;
} status_pckt_header_t;

//Pointer to the room the robot will move in
static uint32_t *world; //Must be initialized by init_room(), and then passed to movement_simulator
/**
 */
static uint8_t recv_byte() {
    uint8_t tmp_byte;

    QUEUE_RET ret = QUEUE_ERR;
    while (ret != QUEUE_OK) {
        ret = queue_pop(&tmp_byte, &q_tx);
    }
    printf("0x%02X ", tmp_byte);
	UCA2IFG |= UCTXIFG; //Subo el flag "buffer TX libre"
	UCA2STATW &= ~UCBUSY; //Bajo el flag "linea ocupada"

    return tmp_byte;
}

/**
 * Function to send a single byte using the thread-safe queue
 *
 * The function will be blocking until there is enough space in the
 * queue to add the data.
 *
 * @param[in] data Data to be added to the queue
 */
static void tx_byte(uint8_t data) {
    QUEUE_RET ret = QUEUE_ERR;
    while (ret != QUEUE_OK) {
        ret = queue_push(data, &q_rx);
    }
    printf("0x%02X ", data);
	UCA2IFG |= UCTXIFG; //Subo el flag "buffer TX libre"
	UCA2STATW &= ~UCBUSY; //Bajo el flag "linea ocupada"
}

/**
 * Compute the checksum
 *
 * @param[in] header Header of the dynamixel packets
 * @param[in] buff RX or TX parameters field
 * @return Returns the computed checksum byte
 */
uint8_t calc_chk_sum(const uint8_t *header, const uint8_t *buff) {
    uint8_t chk_sum = 0;
    for (int i = 0; i < 3; ++i) {
        chk_sum += header[2 + i];
    }
    for (int i = 0; i < header[3] - 2; ++i) {
        chk_sum += buff[i];
    }
    chk_sum = ~chk_sum;
    return chk_sum;
}

/**
 * Decode the received instruction packet and constructs the status packet
 *
 * @param[in] rx_header Header of the instruction packet
 * @param[in] rx_buff Instruction packet parameters
 * @param[in] rx_chk_err Received packet checksum field
 * @param[out] tx_header Header of the status packet
 * @param[in] tx_buff Status packet parameters
 */
void decode_and_build_reply(instr_pck_header_t rx_header,
                            const uint8_t *rx_buff,
                            bool rx_chk_err, status_pckt_header_t *tx_header, uint8_t *tx_buff) {

    tx_header->id = rx_header.id;
    if (rx_chk_err) {
        tx_header->err = ERR_CHKSUM;
        tx_header->len = 2;
        return;
    } else {
        tx_header->err = 0;
    }

    switch (rx_header.instr) {
        case DYN_INSTR__READ:
            memcpy(tx_buff, &dyn_mem[rx_header.id - 1][rx_buff[0]], rx_buff[1]);
            tx_header->len = rx_buff[1] + 2;
            break;
        case DYN_INSTR__WRITE:
            memcpy(&dyn_mem[rx_header.id - 1][rx_buff[0]], &rx_buff[1],
                   rx_header.len - 3);
            tx_header->len = 2;
            break;
        default:
            printf("ERR: Undefined state reached while decoding frame");
            assert(false);
            break;
    }
}

/**
 * Handler to exit the current thread under the appropiate signal
 */
static void handler(int signum) {
    end_simulator();
    pthread_exit(NULL);
}

/**
 * Thread to emulate the Dynamixel communication
 */
void *dyn_emu(void *vargp) {
    uint8_t i = 0, rx_chk, rx_recv_chk, tmp, tx_chk;
    bool rx_chk_err;
    bool is_rx_state = true;
    // Initialization of the state machine
    FSM_t fsm_state = FSM_RX__HEADER_1;
    uint8_t *p;

    world = (uint32_t *) vargp;

    // Declaration of the headers and buffers of the
    // instruction and status packets
    instr_pck_header_t rx_header;
    status_pckt_header_t tx_header;
    uint8_t rx_buff[32];
    uint8_t tx_buff[32];

    // Initialize the headers start of frame
    memcpy(&rx_header.header, &dyn_header, sizeof(rx_header.header));
    memcpy(&tx_header.header, &dyn_header, sizeof(tx_header.header));

    p = (uint8_t *) dyn_mem;
    // Initialize the ID field of the dynamixel
    for (i = 0; i < N_DEVICES; ++i) {
        *(p + i * DYN_MAX_POS + 3) = i + 1;
    }
    // TODO: Add other fields of interest of the dynamixel registers

    // Add SIGTERM handler to kill the current thread
    signal(SIGTERM, handler);

    init_movement_simulator(world); //World must have been initialized previously from main() by calling init_world()

    while (true) {

        update_movement_simulator_values();

        if (is_rx_state) {
            if (queue_is_empty(&q_tx)) {
                continue;
            }
        } else {
            if (queue_is_full(&q_rx)) {
                continue;
            }
        }

        switch (fsm_state) {
            case FSM_RX__HEADER_1:
//			printf("\n Waiting for new packet\n");
            case FSM_RX__HEADER_2:
                tmp = recv_byte();
                assert(tmp == 0xFF);
                break;
            case FSM_RX__ID:
                rx_header.id = recv_byte();
                // Check the ID is valid for the defined memories
                assert(rx_header.id <= N_DEVICES);
                assert(rx_header.id != 0);
                break;
            case FSM_RX__LEN:
                rx_header.len = recv_byte();
                break;
            case FSM_RX__INSTR:
                rx_header.instr = recv_byte();
                i = 0;
                break;
            case FSM_RX__PARAMS:
                rx_buff[i] = recv_byte();
                i++;
                break;
            case FSM_RX__CHKSM:
                tx_header.err = 0;
                rx_chk = recv_byte();
                rx_recv_chk = calc_chk_sum((uint8_t *) &rx_header, rx_buff);
                if (rx_recv_chk != rx_chk) {
                    printf("Error in RX CHKSUM");
                    rx_chk_err = true;
                } else {
                    rx_chk_err = false;
                }
                decode_and_build_reply(rx_header, rx_buff, rx_chk_err, &tx_header,
                                       tx_buff);
                i = 0;
                is_rx_state = false;
                break;
            case FSM_TX__HEADER_1:
                printf("\n Sending reply\n");
            case FSM_TX__HEADER_2:
            case FSM_TX__ID:
            case FSM_TX__LEN:
                tx_byte(*(((uint8_t *) &tx_header) + i++));
                break;
            case FSM_TX__ERR:
                tx_byte(tx_header.err);
                i = 0;
                break;
            case FSM_TX__PARAMS:
                tx_byte(tx_buff[i++]);
                break;
            case FSM_TX__CHKSM:
                tx_chk = calc_chk_sum((uint8_t *) &tx_header, tx_buff);
                tx_byte(tx_chk);
                is_rx_state = true;
                break;
            default:
                printf("ERR: Undefined state reached while RX/TX frame");
                assert(false);
                break;
        }

        if (fsm_state == FSM_RX__PARAMS) {
            if (i >= (rx_header.len - 2)) {
                fsm_state = FSM_RX__CHKSM;
            }
        } else if (fsm_state == FSM_TX__PARAMS) {
            if (i >= (tx_header.len - 2)) {
                fsm_state = FSM_TX__CHKSM;
            }
        } else {
            fsm_state = (FSM_t) ((fsm_state + 1) % ((int) FSM__LEN));
            // If no reply parameters, jump directly to the next state
            if (fsm_state == FSM_TX__PARAMS && tx_header.len == 2) {
                fsm_state = FSM_TX__CHKSM;
            }
            if (fsm_state == FSM_RX__PARAMS && rx_header.len == 2) {
                fsm_state = FSM_RX__CHKSM;
            }
        }

    }
}

