 /**
 * @file flat_convey_subsystem.cpp
 * @copyright Copyright (c) 2023 Vebu Labs. All rights reserved.
 * @brief This library is used to control the flat conveyor subsystem
 * @author Mike Lui
*/

#include "flat_convey_subsystem.hpp"

bool new_flat_con_mb_cmd = false;

uint16_t flat_con_hreg_write(TRegister* reg, uint16_t val) {
    new_flat_con_mb_cmd = true;
    return val;
}

void FlatConveyorFSMClass::setup()
{
    if(!has_setup){
        has_setup = true;

        new_flat_con_mb_cmd = false;
        CcIoManager.set_mb_w_hreg_cb(MbRegisterOffsets::FLAT_CON_STEPS, &flat_con_hreg_write);

        ptr_flat_con_motor = CcIoManager.get_ptr_motor(AutocadoCc1Motors::FLAT_CON_MOT);
        motor_steps = 0;

        state = FlatConvey::FlatConStates::STOPPED;
    }
}

void FlatConveyorFSMClass::read_interfaces()
{
    estop_input = CcIoManager.get_input(AutocadoCcPins::ESTOP_IN);
    edge_sensor_input = CcIoManager.get_input(AutocadoCcPins::FLAT_CON_EDGE_SEN_IN);
    length_sensor_input = CcIoManager.get_input(AutocadoCcPins::FLAT_CON_LENGTH_SEN_IN);

    motor_steps = CcIoManager.get_mb_data(MbRegisterOffsets::FLAT_CON_STEPS);
    motor_dir = CcIoManager.get_mb_data(MbRegisterOffsets::FLAT_CON_DIR);
    motor_speed = CcIoManager.get_mb_data(MbRegisterOffsets::DB_FLAT_CON_SPEED);
    motor_accel = CcIoManager.get_mb_data(MbRegisterOffsets::DB_FLAT_CON_ACCEL);
}

void FlatConveyorFSMClass::run()
{
    read_interfaces();

    if (estop_input == ESTOP_ACTIVE && state != FlatConvey::FlatConStates::ESTOP)
    {
        state = FlatConvey::FlatConStates::ESTOP;
    }

    switch (state)
    {
        case FlatConvey::FlatConStates::STOPPED:
            ptr_flat_con_motor->enable = false;
            
            if(new_flat_con_mb_cmd && motor_steps != 0){
                new_flat_con_mb_cmd = false;
                ptr_flat_con_motor->ptr_connector->ClearAlerts();
                ptr_flat_con_motor->ptr_connector->VelMax(motor_speed);
                ptr_flat_con_motor->ptr_connector->AccelMax(motor_accel);

                if(motor_steps > FLAT_CON_MAX_MOVE_STEPS){
                    motor_steps = FLAT_CON_MAX_MOVE_STEPS;
                }

                if(motor_dir == 1){
                    ptr_flat_con_motor->distance = motor_steps;
                }else{
                    ptr_flat_con_motor->distance = -1 * motor_steps;
                }

                ptr_flat_con_motor->enable = true;
                ptr_flat_con_motor->new_move_commanded = true;
                move_allowance_ms = motor_steps * FLAT_MOVE_ALLOWANCE_MS;
                move_start_time_ms = CcIoManager.getSystemTime();

                state = FlatConvey::FlatConStates::MOVING;
            }
            break;

        case FlatConvey::FlatConStates::MOVING:
            if(CcIoManager.getSystemTime() - move_start_time_ms > move_allowance_ms && 
            ptr_flat_con_motor->ptr_connector->StepsComplete())
            {
                ptr_flat_con_motor->enable = false;
                state = FlatConvey::FlatConStates::STOPPED;
            }
            
            if(new_flat_con_mb_cmd && motor_steps == 0){
                new_flat_con_mb_cmd = false;
                ptr_flat_con_motor->stop_abrupt = true;
                state = FlatConvey::FlatConStates::STOPPED;
            }            
            break;

        case FlatConvey::FlatConStates::ERROR_MOTOR:
            ptr_flat_con_motor->enable = false;
            break;
        
        case FlatConvey::FlatConStates::ESTOP:
        default:
            ptr_flat_con_motor->enable = false;
            new_flat_con_mb_cmd = false;
            if (estop_input == ESTOP_RELEASED) {
                state = FlatConvey::STOPPED;
            }
            break;
    }

    write_interfaces();
}

void FlatConveyorFSMClass::write_interfaces()
{
    CcIoManager.set_mb_data(MbRegisterOffsets::FLAT_CON_STATE, state);
    CcIoManager.set_mb_data(MbRegisterOffsets::FLAT_CON_EDGE_SENSOR, edge_sensor_input);
    CcIoManager.set_mb_data(MbRegisterOffsets::FLAT_CON_LENGTH_SENSOR, length_sensor_input);

}

FlatConveyorFSMClass flat_con;