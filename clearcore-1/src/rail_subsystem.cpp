 /**
 * @file rail_subsystem.cpp
 * @copyright Copyright (c) 2023 Vebu Labs. All rights reserved.
 * @brief This library is used to control the rail subsystem
 * @author Mike Lui
*/

#include "rail_subsystem.hpp"


bool new_rail_motor_mb_cmd = false;

uint16_t rail_motor_hreg_write(TRegister* reg, uint16_t val) {
    new_rail_motor_mb_cmd = true;
    return val;
}


void RailFSMClass::setup()
{
    if(!has_setup){
        has_setup = true;        

        state = Rail::RailStates::SETUP;
        ptr_5160_rail_stepper = CcIoManager.get_step_ptr(AutocadoCcSteppers::STEPPER_RAIL);
    }
}

void RailFSMClass::read_interfaces()
{
    switch_0_input = CcIoManager.get_input(D0_RAIL_SW_0);
    switch_1_input = CcIoManager.get_input(D1_RAIL_SW_1);
}

void RailFSMClass::run()
{
    read_interfaces();

    // if (estop_input == ESTOP_ACTIVE && state != Rail::RailStates::ESTOP)
    // {
    //     state = Rail::RailStates::ESTOP;
    // }


    switch (state)
    {
        case Rail::RailStates::SETUP:
            
            if(ptr_5160_rail_stepper->config_ready())
            {
                Serial.println("Config ready");
                Serial.println("current ticks");
                Serial.println(ptr_5160_rail_stepper->get_ticks());
                Serial.println("Attempting away from home");
                ptr_5160_rail_stepper->set_target_position(ptr_5160_rail_stepper->get_ticks() + RAIL_STEPS_AWAY_HOME, 10000);
                state = Rail::RailStates::MOVING_AWAY_FROM_HOME;
            }else
            {
                Serial.println("Config being set up");
                Serial.println(ptr_5160_rail_stepper->step_5160_motor_cfg.configIndex);
            }
            break;

        case Rail::RailStates::MOVING_AWAY_FROM_HOME:
            
            Serial.println(ptr_5160_rail_stepper->get_driver_status(), HEX);
            if(ptr_5160_rail_stepper->at_position())
            {
                Serial.println("Rail at position");
                ptr_5160_rail_stepper->set_velocity(RAIL_HOME_VMAX);
                state = Rail::RailStates::SET_SG;
            }
            
            break;

        case Rail::RailStates::SET_SG:

            Serial.println(ptr_5160_rail_stepper->get_driver_status(), HEX);
            if(ptr_5160_rail_stepper->at_vmax())
            {
                Serial.println("Rail at vmax, setting sg");
                ptr_5160_rail_stepper->set_enable_stallgaurd(true);
                state = Rail::RailStates::WAIT_SG_HOME_DONE;
            }
     
            break;

        case Rail::RailStates::WAIT_SG_HOME_DONE:
            
            Serial.println("waiting for home");
            Serial.println(ptr_5160_rail_stepper->get_driver_status(), HEX);
            if(ptr_5160_rail_stepper->at_sg_stall())
            {
                Serial.println("at stall");
                ptr_5160_rail_stepper->set_velocity(0);
                ptr_5160_rail_stepper->set_enable_stallgaurd(false);
                ptr_5160_rail_stepper->zero_xactual();
                state = Rail::RailStates::STOPPED;
            }            
            break;

        case Rail::RailStates::STOPPED:            
            Serial.println("Homed");
            ptr_5160_rail_stepper->set_target_position(DEFAULT_SQUISH_POS, RAIL_MOVE_VMAX);
            break;

        case Rail::RailStates::MOVING:
     
  
            break; 

        case Rail::RailStates::ERROR_MOTOR:

            break;
        
        case Rail::RailStates::ESTOP:
        default:

            break;
    }

    write_interfaces();
}

void RailFSMClass::write_interfaces()
{

}

RailFSMClass rail;