cmake_minimum_required(VERSION 3.16)
project(p4_pc_joystick2_habitacion C)

set(CMAKE_C_STANDARD 11)

include_directories(.)
include_directories(dyn)
include_directories(dyn_test)
include_directories(hal_dyn_uart)
include_directories(joystick_emu)

add_executable(p4_pc_joystick2_habitacion
        dyn/dyn_app_common.c
        dyn/dyn_app_common.h
        dyn/dyn_app_sensor.c
        dyn/dyn_app_sensor.h
        dyn/dyn_app_motor.c
        dyn/dyn_app_motor.h
        dyn/dyn_app_sensor.c
        dyn/dyn_app_sensor.h
        dyn/dyn_frames.c
        dyn/dyn_frames.h
        dyn/dyn_instr.c
        dyn/dyn_instr.h
        dyn_test/b_queue.c
        dyn_test/b_queue.h
        dyn_test/dyn_emu.c
        dyn_test/dyn_emu.h
        dyn_test/movement_simulator.c
        dyn_test/movement_simulator.h
        hal_dyn_uart/hal_dyn_uart_emu.c
        hal_dyn_uart/hal_dyn_uart_emu.h
        hal_dyn_uart/fake_msp.h
        joystick_emu/joystick.c
        joystick_emu/joystick.h
        habitacion_001.h
        main.c
        main.h
        posicion.c
        posicion.h)
