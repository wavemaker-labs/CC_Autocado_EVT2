 /**
 * @file release_subsystem.cpp
 * @copyright Copyright (c) 2023 Vebu Labs. All rights reserved.
 * @brief This library is used to control the hopper release subsystem
 * @author Mike Lui
*/

#include "release_subsystem.hpp"

#define RELEASE_DEFAULT_MOTOR_SPEED 1000
#define RELEASE_DEFAULT_MOTOR_ACCEL 1000
#define RELEASE_DEFAULT_OPEN_POS 10
#define RELEASE_DEFAULT_CLOSED_POS 100
#define RELEASE_DEFAULT_TIMEOUT_MS 10000 // 10 seconds

#define TIME_TO_START_MOVE_MS 100

#define RELEASE_OPEN_COMMAND 1
#define RELEASE_CLOSE_COMMAND 0

static bool new_mb_release_cmd = false;

uint16_t release_hreg_write(TRegister* reg, uint16_t val) 
{
    switch(reg->address.address)
    {
        case MbRegisterOffsets::RELEASER_POS_CMD:
            new_mb_release_cmd = true;
            break;
        case MbRegisterOffsets::DB_RELSR_MOTOR_SPEED:
            hopper_release.motor_speed = val;
            break;
        case MbRegisterOffsets::DB_RELSR_MOTOR_ACCEL:
            hopper_release.motor_accel = val;
            break;
        case MbRegisterOffsets::DB_RELSR_CLOSE_POS:
            hopper_release.closed_position = val;
            break;
        case MbRegisterOffsets::DB_RELSR_OPEN_POS:
            hopper_release.open_position = val;
            break;
        case MbRegisterOffsets::DB_RELSR_TIMEOUT:
            hopper_release.timeout_limit_ms = val;
            break;
    }
    return val;
}

void ReleaseFSMClass::setup() 
{
    if(!has_setup){
        has_setup = true;

        ptr_release_motor = CcIoManager.get_ptr_motor(AutocadoCc1Motors::RELEASE_MOT);
        ptr_release_motor->enable =  false;

        CcIoManager.set_mb_w_hreg_cb(MbRegisterOffsets::RELEASER_POS_CMD, &release_hreg_write);
        CcIoManager.set_mb_w_hreg_cb(MbRegisterOffsets::DB_RELSR_MOTOR_SPEED, &release_hreg_write);
        CcIoManager.set_mb_w_hreg_cb(MbRegisterOffsets::DB_RELSR_MOTOR_ACCEL, &release_hreg_write);
        CcIoManager.set_mb_w_hreg_cb(MbRegisterOffsets::DB_RELSR_CLOSE_POS, &release_hreg_write);
        CcIoManager.set_mb_w_hreg_cb(MbRegisterOffsets::DB_RELSR_OPEN_POS, &release_hreg_write);

        hopper_release.motor_speed = RELEASE_DEFAULT_MOTOR_SPEED;
        hopper_release.motor_accel = RELEASE_DEFAULT_MOTOR_ACCEL;
        hopper_release.closed_position = RELEASE_DEFAULT_CLOSED_POS;
        hopper_release.open_position = RELEASE_DEFAULT_OPEN_POS;
        hopper_release.timeout_limit_ms = RELEASE_DEFAULT_TIMEOUT_MS;

        state = Release::ReleaseStates::UNINITIALIZED;
    }
}

void ReleaseFSMClass::read_interfaces()
{
    estop_input = CcIoManager.get_input(AutocadoCcPins::ESTOP_IN);
    release_sensor = CcIoManager.get_input(AutocadoCcPins::RELEASE_SENS_IN);
}

void ReleaseFSMClass::run()
{
    read_interfaces();

    if (estop_input == ESTOP_ACTIVE && state != Release::ReleaseStates::ESTOP)
    {
        state = Release::ReleaseStates::ESTOP;
    }

    switch (state)
    {
        case Release::ReleaseStates::UNINITIALIZED:
            mb_move_request = 0;
            ptr_release_motor->enable =  false;
            ptr_release_motor->ptr_connector->ClearAlerts();
            
            if(new_mb_release_cmd){
                new_mb_release_cmd = false;
                mb_move_request = CcIoManager.get_mb_data(MbRegisterOffsets::RELEASER_POS_CMD);
                ptr_release_motor->ptr_connector->VelMax(hopper_release.motor_speed);
                ptr_release_motor->ptr_connector->AccelMax(hopper_release.motor_accel);
                ptr_release_motor->enable =  true;
                move_start_time_ms = CcIoManager.getSystemTime();
                state = Release::ReleaseStates::HOMING;
            }
            break;

        case Release::ReleaseStates::HOMING:        
            if(ptr_release_motor->hlfb_state == MotorDriver::HLFB_ASSERTED && 
            ptr_release_motor->steps_complete == true && 
            CcIoManager.getSystemTime() - move_start_time_ms > TIME_TO_START_MOVE_MS){

                Serial.println("release motor homed");                
                ptr_release_motor->ptr_connector->PositionRefSet(0);
                
                if(mb_move_request == RELEASE_OPEN_COMMAND){
                    ptr_release_motor->distance = hopper_release.open_position;
                    ptr_release_motor->new_move_commanded = true;
                    move_start_time_ms = CcIoManager.getSystemTime();
                    state = Release::ReleaseStates::MOVING;

                }else if (mb_move_request == RELEASE_CLOSE_COMMAND){
                    ptr_release_motor->distance = hopper_release.closed_position;
                    ptr_release_motor->new_move_commanded = true;
                    move_start_time_ms = CcIoManager.getSystemTime();
                    state = Release::ReleaseStates::MOVING;
                }else {
                    state = Release::ReleaseStates::MOVE_DONE;
                }                    

            }else if (CcIoManager.getSystemTime() - move_start_time_ms > hopper_release.timeout_limit_ms) {
                state = Release::ReleaseStates::ERROR_HOMING;
            }
            break;

        case Release::ReleaseStates::MOVING:
            if(ptr_release_motor->hlfb_state == MotorDriver::HLFB_ASSERTED && 
            ptr_release_motor->steps_complete == true && 
            CcIoManager.getSystemTime() - move_start_time_ms > TIME_TO_START_MOVE_MS){

                Serial.println("release motor done moving");         
                state = Release::ReleaseStates::MOVE_DONE;

            }else if (CcIoManager.getSystemTime() - move_start_time_ms > hopper_release.timeout_limit_ms) {
                state = Release::ReleaseStates::ERROR_MOVING;
            }
            break;

        case Release::ReleaseStates::MOVE_DONE:
            if(new_mb_release_cmd) {
                new_mb_release_cmd = false;
                mb_move_request = CcIoManager.get_mb_data(MbRegisterOffsets::RELEASER_POS_CMD);

                if(ptr_release_motor->ptr_connector->ValidateMove(false)){

                    if(mb_move_request == RELEASE_OPEN_COMMAND){
                        ptr_release_motor->distance = hopper_release.open_position;
                        ptr_release_motor->new_move_commanded = true;
                        move_start_time_ms = CcIoManager.getSystemTime();
                        state = Release::ReleaseStates::MOVING;

                    }else if (mb_move_request == RELEASE_CLOSE_COMMAND){
                        ptr_release_motor->distance = hopper_release.closed_position;
                        ptr_release_motor->new_move_commanded = true;
                        move_start_time_ms = CcIoManager.getSystemTime();
                        state = Release::ReleaseStates::MOVING;
                    }else {
                        state = Release::ReleaseStates::MOVE_DONE;
                    }  

                } else {
                    state = Release::ReleaseStates::ERROR_MOTOR_MOVE_INVALID;
                }
            }
            break;
        
        case Release::ReleaseStates::ESTOP:
            new_mb_release_cmd = false;
            ptr_release_motor->enable = false;
            if (estop_input == ESTOP_RELEASED) {
                state = Release::ReleaseStates::UNINITIALIZED;
            }
            break;

        case Release::ReleaseStates::ERROR_HOMING:
        case Release::ReleaseStates::ERROR_MOVING:
        case Release::ReleaseStates::ERROR_MOTOR_MOVE_INVALID:
            /* code */
            break;
        
        default:
            break;
    }

    write_interfaces();
}

void ReleaseFSMClass::write_interfaces()
{
    CcIoManager.set_mb_data(MbRegisterOffsets::RELEASER_STATE, state);
    CcIoManager.set_mb_data(MbRegisterOffsets::RELEASER_SENSOR, release_sensor);
}


ReleaseFSMClass hopper_release;