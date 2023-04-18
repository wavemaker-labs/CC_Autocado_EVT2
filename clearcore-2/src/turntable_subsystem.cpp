/**
 * @file turntable_subsystem.cpp
 * @copyright Copyright (c) 2023 Vebu Labs. All rights reserved.
 * @brief This library is used to control the wml turntable
 * @author Mike Lui
*/

#include "turntable_subsystem.hpp"

bool new_mb_turntable_cmd = false;

uint16_t turntable_hreg_write(TRegister* reg, uint16_t val) {
    new_mb_turntable_cmd = true;
    return val;
}

void TurntableFSMClass::setup()
{
    if(!has_setup)
    {
        has_setup = true;
        new_mb_turntable_cmd = false;

        ptr_turntable_motor = CcIoManager.get_ptr_motor(AutocadoCc2Motors::TURNTABLE_STEPPER);
        ptr_turntable_motor->enable = false;

        CcIoManager.set_mb_w_hreg_cb(MbRegisterOffsets::TURNTABLE_CMD, &turntable_hreg_write);

        state = Turntable::TurntableStates::STOPPED;
    }
}

void TurntableFSMClass::read_interfaces()
{
        estop_input = CcIoManager.get_input(AutocadoCcPins::ESTOP_IN);
        bowl_sensor_input = CcIoManager.get_input(AutocadoCcPins::BOWL_SENS_IN);

        motor_speed = CcIoManager.get_mb_data(MbRegisterOffsets::DB_TURNT_MOT_SPEED);
        motor_accel = CcIoManager.get_mb_data(MbRegisterOffsets::DB_TURNT_MOT_ACCEL);
        motor_steps = CcIoManager.get_mb_data(MbRegisterOffsets::DB_TURNT_MOVE_STEPS);
}

void TurntableFSMClass::run()
{
        read_interfaces();

        if (estop_input == ESTOP_ACTIVE && state != Turntable::TurntableStates::ESTOP)
        {
            state = Turntable::TurntableStates::ESTOP;
        }

        switch (state)
        {
            case Turntable::TurntableStates::STOPPED:
                ptr_turntable_motor->enable = false;
                
                if(new_mb_turntable_cmd){
                    new_mb_turntable_cmd = false;

                    if(motor_steps != 0){
                        ptr_turntable_motor->ptr_connector->VelMax(motor_speed);
                        ptr_turntable_motor->ptr_connector->AccelMax(motor_accel);

                        if(motor_steps > TURNABLE_MAX_STEPS){
                            motor_steps = TURNABLE_MAX_STEPS;
                        }

                        ptr_turntable_motor->distance = motor_steps;
                        ptr_turntable_motor->enable = true;
                        ptr_turntable_motor->new_move_commanded = true;

                        move_allowance_ms = motor_steps * TURNTABLE_MOVE_ALLOWANCE_MS;
                        move_start_time_ms = CcIoManager.getSystemTime();

                        state = Turntable::TurntableStates::MOVING;
                    }
                }

                break;

            case Turntable::TurntableStates::MOVING:
                if(CcIoManager.getSystemTime() - move_start_time_ms > move_allowance_ms && 
                ptr_turntable_motor->ptr_connector->StepsComplete())
                {
                    ptr_turntable_motor->enable = false;
                    state = Turntable::TurntableStates::STOPPED;
                }
                break;
        
            case Turntable::TurntableStates::ERROR_MOTOR_ALARM:
                /* code */
                break;
            
            case Turntable::TurntableStates::ESTOP:
            default:
                ptr_turntable_motor->enable = false;
                break;
        }

        write_interfaces();
    
}

void TurntableFSMClass::write_interfaces()
{
    CcIoManager.set_mb_data(MbRegisterOffsets::TURNTABLE_STATE, state);
    CcIoManager.set_mb_data(MbRegisterOffsets::TURNTABLE_SENSOR, bowl_sensor_input);    
}

TurntableFSMClass turntable;