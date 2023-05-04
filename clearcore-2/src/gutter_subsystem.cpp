 /**
 * @file gutter_subsystem.cpp
 * @copyright Copyright (c) 2023 Vebu Labs. All rights reserved.
 * @brief This library is used to control the gutter subsystem
 * @author Mike Lui
*/

#include "gutter_subsystem.hpp"

bool new_gutter_mb_cmd = false;

uint16_t gutter_hreg_write(TRegister* reg, uint16_t val) {
    new_gutter_mb_cmd = true;
    return val;
}

void GutterFSMClass::setup()
{
    if(!has_setup){
        has_setup = true;

        new_gutter_mb_cmd = false;
        CcIoManager.set_mb_w_hreg_cb(MbRegisterOffsets::GUTTER_CMD, &gutter_hreg_write);

        ptr_gutter_motor = CcIoManager.get_ptr_motor(AutocadoCc2Motors::GUTTER_STEPPER);
        ptr_gutter_motor->ptr_connector->PolarityInvertSDEnable(true);
        ptr_gutter_motor->enable = false;

        state = Gutter::GutterStates::UNINITIATED;
    }
}

void GutterFSMClass::read_interfaces()
{
    estop_input = CcIoManager.get_input(AutocadoCcPins::ESTOP_IN);
    open_sensor_input = CcIoManager.get_input(AutocadoCcPins::GUTTER_SEN1_IN);
    closed_sensor_input = CcIoManager.get_input(AutocadoCcPins::GUTTER_SEN2_IN);

    max_motor_steps = CcIoManager.get_mb_data(MbRegisterOffsets::DB_GUTTER_MAX_MOVE);
    move_timeout_ms = CcIoManager.get_mb_data(MbRegisterOffsets::DB_GUTTER_TIMEOUT_MS);
    motor_speed = CcIoManager.get_mb_data(MbRegisterOffsets::DB_GUTTER_MOT_SPEED);
    motor_accel = CcIoManager.get_mb_data(MbRegisterOffsets::DB_GUTTER_MOT_ACCEL);

    if(max_motor_steps > GUTTER_MAX_MOVE_STEPS){ max_motor_steps = GUTTER_MAX_MOVE_STEPS;}

    mb_command = CcIoManager.get_mb_data(MbRegisterOffsets::GUTTER_CMD);
}

void GutterFSMClass::run()
{
    read_interfaces();

    if (estop_input == ESTOP_ACTIVE && state != Gutter::GutterStates::ESTOP)
    {
        state = Gutter::GutterStates::ESTOP;
    }

    switch (state)
    {
        case Gutter::GutterStates::UNINITIATED:
            ptr_gutter_motor->enable = false;
            
            if(new_gutter_mb_cmd){
                new_gutter_mb_cmd = false;
                ptr_gutter_motor->ptr_connector->ClearAlerts();
                ptr_gutter_motor->ptr_connector->VelMax(motor_speed);
                ptr_gutter_motor->ptr_connector->AccelMax(motor_accel);

                if(mb_command == GUTTER_CMD_OPEN){
                    ptr_gutter_motor->enable = true;

                    if(open_sensor_input == GUTTER_FLAG_DETECTED){
                        state = Gutter::GutterStates::AT_OPEN;
                    }else{
                        ptr_gutter_motor->distance = max_motor_steps * GUTTER_DIR_TO_OPEN;
                        ptr_gutter_motor->new_move_commanded = true;
                        move_start_time_ms = CcIoManager.getSystemTime();

                        state = Gutter::GutterStates::MOVING_OPEN;
                    }

                }else if(mb_command == GUTTER_CMD_CLOSE){
                    ptr_gutter_motor->enable = true;

                    if(closed_sensor_input == GUTTER_FLAG_DETECTED){
                        state = Gutter::GutterStates::AT_CLOSED;
                    }else{
                        ptr_gutter_motor->distance = max_motor_steps * GUTTER_DIR_TO_CLOSE;
                        ptr_gutter_motor->new_move_commanded = true;
                        move_start_time_ms = CcIoManager.getSystemTime();

                        state = Gutter::GutterStates::MOVING_CLOSED;
                    }
                }
            }
            break;
        
        case Gutter::GutterStates::MOVING_OPEN:
            if(open_sensor_input == GUTTER_FLAG_DETECTED){
                // ptr_gutter_motor->enable = false;
                ptr_gutter_motor->stop_abrupt = true;
                state = Gutter::GutterStates::AT_OPEN;
            }else if(CcIoManager.getSystemTime() - move_start_time_ms > move_timeout_ms){
                ptr_gutter_motor->enable = false;
                state = Gutter::GutterStates::ERROR_TIMEOUT;
            }
            break;
        
        case Gutter::GutterStates::MOVING_CLOSED:
            if(closed_sensor_input == GUTTER_FLAG_DETECTED){
                // ptr_gutter_motor->enable = false;                
                ptr_gutter_motor->stop_abrupt = true;
                state = Gutter::GutterStates::AT_CLOSED;
            }else if(CcIoManager.getSystemTime() - move_start_time_ms > move_timeout_ms){
                ptr_gutter_motor->enable = false;
                state = Gutter::GutterStates::ERROR_TIMEOUT;
            }
            break;

        case Gutter::GutterStates::AT_OPEN:
            if(open_sensor_input != GUTTER_FLAG_DETECTED){state = Gutter::GutterStates::UNINITIATED;}

            if(new_gutter_mb_cmd && mb_command == GUTTER_CMD_CLOSE){
                new_gutter_mb_cmd = false;
                // ptr_gutter_motor->enable = true;
                
                ptr_gutter_motor->distance = max_motor_steps * GUTTER_DIR_TO_CLOSE;
                ptr_gutter_motor->new_move_commanded = true;
                move_start_time_ms = CcIoManager.getSystemTime();

                state = Gutter::GutterStates::MOVING_CLOSED;
            }
            break;
        
        case Gutter::GutterStates::AT_CLOSED:
            if(closed_sensor_input != GUTTER_FLAG_DETECTED){state = Gutter::GutterStates::UNINITIATED;}

            if(new_gutter_mb_cmd && mb_command == GUTTER_CMD_OPEN){
                new_gutter_mb_cmd = false;
                // ptr_gutter_motor->enable = true;

                ptr_gutter_motor->distance = max_motor_steps * GUTTER_DIR_TO_OPEN;
                ptr_gutter_motor->new_move_commanded = true;
                move_start_time_ms = CcIoManager.getSystemTime();

                state = Gutter::GutterStates::MOVING_OPEN;                
            }
            break;

        case Gutter::GutterStates::ERROR_MOTOR:
        case Gutter::GutterStates::ERROR_TIMEOUT:
            ptr_gutter_motor->enable = false;
            break;
        
        case Gutter::GutterStates::ESTOP:
        default:
            ptr_gutter_motor->enable = false;
            new_gutter_mb_cmd = false;
            if (estop_input == ESTOP_RELEASED) {
                state = Gutter::UNINITIATED;
            }
            break;
    }

    write_interfaces();
}

void GutterFSMClass::write_interfaces()
{
    CcIoManager.set_mb_data(MbRegisterOffsets::GUTTER_STATE, state);
    CcIoManager.set_mb_data(MbRegisterOffsets::GUTTER_SENSOR_1, open_sensor_input);
    CcIoManager.set_mb_data(MbRegisterOffsets::GUTTER_SENSOR_2, closed_sensor_input);
}

GutterFSMClass gutter;