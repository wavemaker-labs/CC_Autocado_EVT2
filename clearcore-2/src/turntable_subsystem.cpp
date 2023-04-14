/**
 * @file drink_pickup_subsystem.cpp
 * @copyright Copyright (c) 2023 Vebu Labs. All rights reserved.
 * @brief This library is used to control the drink pickup
 * @author Mike Lui
*/

#include "drink_pickup_subsystem.hpp"

#define DRINK_MOVE_TIMEOUT_MS   20000 //20 secs
#define DRINK_RELAY_TIME_MS   300 //0.30 secs before moving to not jam into locker lock

#define DRINK_MAX_ENC_MOVE 4000
#define DRINK_MIN_ENC_MOVE -4000

bool new_mb_pickup_cmd = false;

DrinkPickup::DrinkPickupStates *ptr_pickup_state;

void limit_new_pickup_cmd_window(bool * ptr_new_cmd){
    if(*ptr_pickup_state == DrinkPickup::READY || *ptr_pickup_state == DrinkPickup::UNINITIALIZED) {
        *ptr_new_cmd =  true;
    }
}

uint16_t pickup_hreg_write(TRegister* reg, uint16_t val) {
    limit_new_pickup_cmd_window(&new_mb_pickup_cmd);
    return val;
}

void DrinkPickupFSMClass::setup()
{
    if(!has_setup){
        has_setup = true;

        ptr_pickup_state = &state;

        CcIoManager.set_mb_w_hreg_cb(MbRegisterOffsets::PICKUP_MOVE_RQ, &pickup_hreg_write);

        ptr_drink_pickup_motor = CcIoManager.get_ptr_motor(BobaCc2Motors::DRINK_PICKUP);
        ptr_drink_pickup_motor->enable = false;

        locker_relay_out = PinStatus::LOW;

        mb_move_request = 0;
        move_start_time_ms = 0;

        move_timeout = false;
        move_done = false;


        state = DrinkPickup::UNINITIALIZED;
    }  
}

void set_move_limits(int16_t* ptr_move_rq)
{
    if(*ptr_move_rq > DRINK_MAX_ENC_MOVE){
        *ptr_move_rq = DRINK_MAX_ENC_MOVE;
    } else if(*ptr_move_rq < DRINK_MIN_ENC_MOVE){
        *ptr_move_rq = DRINK_MIN_ENC_MOVE;
    }
}

void DrinkPickupFSMClass::read_interfaces()
{
    estop_input = CcIoManager.get_input(BobaCcPins::ESTOP_IN);
    drink_sensor_input = CcIoManager.get_input(BobaCcPins::PICKUP_SENS_IN);

    if(new_mb_pickup_cmd){
        mb_move_request = CcIoManager.get_mb_data(MbRegisterOffsets::PICKUP_MOVE_RQ);
        set_move_limits(&mb_move_request);
    }
}

void DrinkPickupFSMClass::run()
{
    read_interfaces();

    if (estop_input == ESTOP_ACTIVE && state != DrinkPickup::ESTOP)
    {
        state = DrinkPickup::ESTOP;
    }

    switch (state)
    {
    case DrinkPickup::UNINITIALIZED:
        ptr_drink_pickup_motor->enable = false;
        locker_relay_out = PinStatus::LOW;
        move_timeout = false;
        move_done = false;

        if(new_mb_pickup_cmd){
            locker_relay_out = PinStatus::HIGH;
            move_start_time_ms = CcIoManager.getSystemTime();
            state = DrinkPickup::WAITING_RELAY_HOME;
        }        
        break;
    
    case DrinkPickup::WAITING_RELAY_HOME:
        if(CcIoManager.getSystemTime() - move_start_time_ms > DRINK_RELAY_TIME_MS){            
            ptr_drink_pickup_motor->enable = true;
            move_timeout = false;
            move_done = false;
            move_start_time_ms = CcIoManager.getSystemTime();
            state = DrinkPickup::INITIALIZING;
        }
        break;

    case DrinkPickup::INITIALIZING:
        if(CcIoManager.getSystemTime() - move_start_time_ms > DRINK_MOVE_TIMEOUT_MS){            
            move_timeout = true;
            move_start_time_ms = 0;
            state = DrinkPickup::ERROR_HOMING;
        } else if (ptr_drink_pickup_motor->hlfb_state == MotorDriver::HLFB_ASSERTED 
                    && ptr_drink_pickup_motor->steps_complete == true){
            Serial.println("drink pick up homed");
            ptr_drink_pickup_motor->ptr_connector->PositionRefSet(0);
            move_timeout = false;
            locker_relay_out = PinStatus::HIGH;
            move_start_time_ms = CcIoManager.getSystemTime();            
            state = DrinkPickup::WAITING_RELAY_MOVE;
        }

        break;
    
    case DrinkPickup::READY:
        if(new_mb_pickup_cmd){
            locker_relay_out = PinStatus::HIGH;
            move_start_time_ms = CcIoManager.getSystemTime();
            state = DrinkPickup::WAITING_RELAY_MOVE;
        }
        break;
    
    case DrinkPickup::MOVING:
        if(CcIoManager.getSystemTime() - move_start_time_ms > DRINK_MOVE_TIMEOUT_MS){            
            move_timeout = true;
            move_start_time_ms = 0;
            state = DrinkPickup::ERROR_MOVING;
        } else if (ptr_drink_pickup_motor->hlfb_state == MotorDriver::HLFB_ASSERTED 
                    && ptr_drink_pickup_motor->steps_complete == true){
            Serial.println("drink pick up done moving");
            move_timeout = false;
            move_done = true;
            locker_relay_out = PinStatus::LOW;            
            state = DrinkPickup::READY;
        }
        break;
    

    case DrinkPickup::WAITING_RELAY_MOVE:
        if(CcIoManager.getSystemTime() - move_start_time_ms > DRINK_RELAY_TIME_MS){    
            ptr_drink_pickup_motor->ptr_connector->Move(mb_move_request, ptr_drink_pickup_motor->move_mode); 
            Serial.println(mb_move_request);       
            move_timeout = false;
            move_done = false;
            move_start_time_ms = CcIoManager.getSystemTime();

            new_mb_pickup_cmd = false;

            state = DrinkPickup::MOVING;
        }
        break;
    
    case DrinkPickup::ESTOP:
        new_mb_pickup_cmd = false;
        locker_relay_out = PinStatus::LOW;
        
        ptr_drink_pickup_motor->enable = false;

        if (estop_input == ESTOP_RELEASED) {
            state = DrinkPickup::UNINITIALIZED;
        }
        
        break;
    
    case DrinkPickup::ERROR_HOMING:
    case DrinkPickup::ERROR_MOVING:
        /* code */
        break;
    
    default:
        break;
    }

    write_interfaces();
}

void DrinkPickupFSMClass::write_interfaces()
{
    CcIoManager.set_mb_data(MbRegisterOffsets::PICKUP_STATE, state);
    CcIoManager.set_mb_data(MbRegisterOffsets::PICKUP_TIMEOUT, move_timeout);
    CcIoManager.set_mb_data(MbRegisterOffsets::PICKUP_MOVEDONE, move_done);
    CcIoManager.set_mb_data(MbRegisterOffsets::PICKUP_DRINK_SENSOR, drink_sensor_input);
    CcIoManager.set_mb_data(MbRegisterOffsets::PICKUP_CMD_POS, mb_move_request);
    CcIoManager.set_mb_data(MbRegisterOffsets::PICKUP_RELAY_STATE, locker_relay_out);

    CcIoManager.set_pin_output_state(BobaCcPins::LOCKER_UNLOCK_OUT, locker_relay_out);
}

DrinkPickupFSMClass drink_pickup;
