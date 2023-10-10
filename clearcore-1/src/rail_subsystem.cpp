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
        ptr_5160_rail_stepper = CcIoManager.get_step_ptr(AutocadoCcSteppers::STEPPER_3);
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
                state = Rail::RailStates::STOPPED;
            }else
            {
                Serial.println("Config being set up");
                Serial.println(ptr_5160_rail_stepper->step_5160_motor_cfg.configIndex);
            }


            break;
        case Rail::RailStates::STOPPED:            
            Serial.println("current ticks");
            Serial.println(ptr_5160_rail_stepper->get_ticks());
            Serial.println("Attempting move");
            ptr_5160_rail_stepper->set_target_position(ptr_5160_rail_stepper->get_ticks() + 51200, 10000);
            state = Rail::RailStates::MOVING;

            break;

        case Rail::RailStates::MOVING:
            // Serial.println("Velocity + ticks");
            // Serial.println(ptr_5160_rail_stepper->get_velocity());    
            // Serial.println(ptr_5160_rail_stepper->get_ticks());        
  
            break;

        case Rail::RailStates::CYCLE:

            break;

        case Rail::RailStates::CYCLE_WAIT:
   
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