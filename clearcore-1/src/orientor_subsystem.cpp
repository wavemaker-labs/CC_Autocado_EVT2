 /**
 * @file orientor_subsystem.cpp
 * @copyright Copyright (c) 2023 Vebu Labs. All rights reserved.
 * @brief This library is used to control the orientor subsystem
 * @author Mike Lui
*/

#include "orientor_subsystem.hpp"

static bool new_mb_orient_pos_cmd = false;
static bool new_mb_orient_torq_cmd = false;

uint16_t orient_pos_hreg_write(TRegister* reg, uint16_t val) 
{
    new_mb_orient_pos_cmd = true;
    return val;
}

uint16_t orient_torq_hreg_write(TRegister* reg, uint16_t val) 
{
    new_mb_orient_torq_cmd = true;
    return val;
}

void OrientorFSMClass::setup()
{
    if(!has_setup){
        has_setup = true;

        ptr_right_orientor_motor = CcIoManager.get_ptr_motor(AutocadoCc1Motors::ORIENTOR_ONE_MOT);
        ptr_right_orientor_motor->enable = false;

        ptr_left_orientor_motor = CcIoManager.get_ptr_motor(AutocadoCc1Motors::ORIENTOR_TWO_MOT);
        ptr_left_orientor_motor->enable = false;

        CcIoManager.set_mb_w_hreg_cb(MbRegisterOffsets::ORIENTOR_POS_CMD, &orient_pos_hreg_write);
        CcIoManager.set_mb_w_hreg_cb(MbRegisterOffsets::ORIENTOR_TORQUE_CMD, &orient_torq_hreg_write);

        state = Orientor::OrientorStates::UNINITIALIZED;

    }
}

void OrientorFSMClass::read_interfaces()
{
    estop_input = CcIoManager.get_input(AutocadoCcPins::ESTOP_IN);

    motor_speed = CcIoManager.get_mb_data(MbRegisterOffsets::DB_ORIENTOR_MOTOR_SPEED);
    motor_accel = CcIoManager.get_mb_data(MbRegisterOffsets::DB_ORIENTOR_MOTOR_ACCEL);
    motor_torque = CcIoManager.get_mb_data(MbRegisterOffsets::DB_ORIENTOR_MOVE_TRQ);
    timeout_limit_ms = CcIoManager.get_mb_data(MbRegisterOffsets::DB_ORIENTOR_TIMEOUT);
    
    position_0 = 0;
    position_1 = CcIoManager.get_mb_data(MbRegisterOffsets::DB_ORIENTOR_POS_1);
    position_2 = CcIoManager.get_mb_data(MbRegisterOffsets::DB_ORIENTOR_POS_2);
    position_3 = CcIoManager.get_mb_data(MbRegisterOffsets::DB_ORIENTOR_POS_3);
    position_4 = CcIoManager.get_mb_data(MbRegisterOffsets::DB_ORIENTOR_POS_4);
    position_5 = CcIoManager.get_mb_data(MbRegisterOffsets::DB_ORIENTOR_POS_5);

    mb_pos_move_request = CcIoManager.get_mb_data(MbRegisterOffsets::ORIENTOR_POS_CMD);
    mb_tq_move_request = CcIoManager.get_mb_data(MbRegisterOffsets::ORIENTOR_TORQUE_CMD);
    
}

void OrientorFSMClass::set_motor_parameters()
{
    if(ptr_right_orientor_motor != nullptr && ptr_left_orientor_motor != nullptr)
    {
        ptr_right_orientor_motor->ptr_connector->AccelMax(motor_accel);
        ptr_right_orientor_motor->ptr_connector->VelMax(motor_speed);
        ptr_left_orientor_motor->ptr_connector->AccelMax(motor_accel);
        ptr_left_orientor_motor->ptr_connector->VelMax(motor_speed);
    }
}

void OrientorFSMClass::enable_motors(bool val)
{
    if(ptr_right_orientor_motor != nullptr && ptr_left_orientor_motor != nullptr)
    {
        ptr_right_orientor_motor->enable = val;
        ptr_left_orientor_motor->enable = val;
    }
}

bool OrientorFSMClass::motor_move_done()
{   
    bool ret = false;
    if(ptr_right_orientor_motor != nullptr && ptr_left_orientor_motor != nullptr)
    {
        ret = (ptr_right_orientor_motor->hlfb_state == MotorDriver::HLFB_ASSERTED && 
            ptr_right_orientor_motor->steps_complete == true) && (ptr_left_orientor_motor->hlfb_state == MotorDriver::HLFB_ASSERTED && 
            ptr_left_orientor_motor->steps_complete == true);
    }

    return ret;
}

bool OrientorFSMClass::motor_at_torq()
{   
    bool ret = false;
    if(ptr_right_orientor_motor != nullptr && ptr_left_orientor_motor != nullptr)
    {   
        ret = (ptr_right_orientor_motor->hlfb_duty >= motor_torque || 
        ptr_left_orientor_motor->hlfb_duty >= motor_torque);
    }

    return ret;
}

void OrientorFSMClass::set_motor_distance(uint16_t distance)
{
    if(ptr_right_orientor_motor != nullptr && ptr_left_orientor_motor != nullptr)
    {
        ptr_right_orientor_motor->distance = distance;
        ptr_left_orientor_motor->distance = distance;
    }
}

void OrientorFSMClass::set_motor_position(uint16_t pos_request)
{
    if(ptr_right_orientor_motor != nullptr && ptr_left_orientor_motor != nullptr)
    {
        switch (pos_request)
        {
            case 1:
                set_motor_distance(position_1);
                break;
            case 2:
                set_motor_distance(position_2);
                break;
            case 3:
                set_motor_distance(position_3);
                break;
            case 4:
                set_motor_distance(position_4);
                break;
            case 5:
                set_motor_distance(position_5); //clamp/close
                break;                    
            default:
            case 0:
                set_motor_distance(position_0); //home
                break;
        }
    }
}


void OrientorFSMClass::run()
{
    read_interfaces();

    if (estop_input == ESTOP_ACTIVE && state != Orientor::OrientorStates::ESTOP)
    {
        state = Orientor::OrientorStates::ESTOP;
    }

    switch (state)
    {
        case Orientor::OrientorStates::UNINITIALIZED:
            enable_motors(false);
            
            if(new_mb_orient_pos_cmd){
                new_mb_orient_pos_cmd = false;
                ptr_right_orientor_motor->ptr_connector->ClearAlerts();
                ptr_left_orientor_motor->ptr_connector->ClearAlerts();
                set_motor_parameters();
                enable_motors(true);
                move_start_time_ms = CcIoManager.getSystemTime();
                
                state = Orientor::OrientorStates::HOMING;
            }
            break;

        case Orientor::OrientorStates::HOMING:
            if(motor_move_done() && 
            CcIoManager.getSystemTime() - move_start_time_ms > ORIENTOR_MOVE_ALLOWANCE_MS){

                Serial.println("release motor homed");                
                ptr_right_orientor_motor->ptr_connector->PositionRefSet(0);
                ptr_left_orientor_motor->ptr_connector->PositionRefSet(0);
                
                set_motor_position(mb_pos_move_request);                
                ptr_right_orientor_motor->new_move_commanded = true;
                ptr_left_orientor_motor->new_move_commanded = true;

                move_start_time_ms = CcIoManager.getSystemTime();

                state = Orientor::OrientorStates::MOVING_TO_POS;
               
            }else if (CcIoManager.getSystemTime() - move_start_time_ms > timeout_limit_ms) {
                state = Orientor::OrientorStates::ERROR_HOMING;
            }
            break;

        case Orientor::OrientorStates::MOVING_TO_POS:
            if(motor_move_done() && 
            CcIoManager.getSystemTime() - move_start_time_ms > ORIENTOR_MOVE_ALLOWANCE_MS){

                Serial.println(" motors at position");               
                state = Orientor::OrientorStates::MOVE_DONE;
               
            }else if (CcIoManager.getSystemTime() - move_start_time_ms > timeout_limit_ms) {
                state = Orientor::OrientorStates::ERROR_MOVING;
            }
            break;

        case Orientor::OrientorStates::MOVING_TO_TQ:
            if((motor_move_done() || motor_at_torq()) &&
            CcIoManager.getSystemTime() - move_start_time_ms > ORIENTOR_MOVE_ALLOWANCE_MS){

                Serial.println(" motors at pos OR torquw");
                ptr_right_orientor_motor->stop_abrupt = true;
                ptr_left_orientor_motor->stop_abrupt = true;
                state = Orientor::OrientorStates::MOVE_DONE;
               
            }else if (CcIoManager.getSystemTime() - move_start_time_ms > timeout_limit_ms) {
                state = Orientor::OrientorStates::ERROR_MOVING;
            }
            break;

        case Orientor::OrientorStates::MOVE_DONE:
            /* code */
            if(new_mb_orient_pos_cmd){
                new_mb_orient_pos_cmd = false;
                set_motor_position(mb_pos_move_request);                
                ptr_right_orientor_motor->new_move_commanded = true;
                ptr_left_orientor_motor->new_move_commanded = true;
                move_start_time_ms = CcIoManager.getSystemTime();
                
                state = Orientor::OrientorStates::MOVING_TO_POS;
            }
            if(new_mb_orient_torq_cmd){
                new_mb_orient_torq_cmd = false;
                set_motor_position(mb_pos_move_request);                
                ptr_right_orientor_motor->new_move_commanded = true;
                ptr_left_orientor_motor->new_move_commanded = true;
                move_start_time_ms = CcIoManager.getSystemTime();
            }
            break;
            
        case Orientor::OrientorStates::ERROR_HOMING:
        case Orientor::OrientorStates::ERROR_MOVING:
        case Orientor::OrientorStates::ERROR_MOTOR_MOVE_INVALID:
            ptr_right_orientor_motor->enable = false;
            ptr_left_orientor_motor->enable = false;
            break;        
        
        case Orientor::OrientorStates::ESTOP:
        default:
            ptr_right_orientor_motor->enable = false;
            ptr_left_orientor_motor->enable = false;
            new_mb_orient_pos_cmd = false;
            new_mb_orient_torq_cmd = false;
            if (estop_input == ESTOP_RELEASED) {
                state = Orientor::OrientorStates::UNINITIALIZED;
            }
            break;
    }

    write_interfaces();
}

void OrientorFSMClass::write_interfaces()
{
    CcIoManager.set_mb_data(MbRegisterOffsets::ORIENTOR_STATE, state);
}

OrientorFSMClass orientor;