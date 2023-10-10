 /**
 * @file rail_subsystem.cpp
 * @copyright Copyright (c) 2023 Vebu Labs. All rights reserved.
 * @brief This library is used to control the cutter subsystem
 * @author Mike Lui
*/

#include "cutter_subsystem.hpp"


bool new_cutter_motor_mb_cmd = false;

uint16_t cutter_motor_hreg_write(TRegister* reg, uint16_t val) {
    new_cutter_motor_mb_cmd = true;
    return val;
}


void CutterFSMClass::setup()
{
    if(!has_setup){
        has_setup = true;        

        state = Cutter::CutterStates::SETUP;
        ptr_5160_cut_stepper = CcIoManager.get_step_ptr(AutocadoCcSteppers::STEPPER_2);
    }
}

void CutterFSMClass::read_interfaces()
{
    cut_switch_input = CcIoManager.get_input(D6_CUT_BUTTON);
    load_switch_input = CcIoManager.get_input(D7_LOAD_CUT_BUTTON);
}

void CutterFSMClass::run()
{
    read_interfaces();

    // if (estop_input == ESTOP_ACTIVE && state != Cutter::CutterStatesESTOP)
    // {
    //     state = Cutter::CutterStatesESTOP;
    // }


    switch (state)
    {
        case Cutter::CutterStates::SETUP:
            if(ptr_5160_cut_stepper->config_ready())
            {
                Serial.println("Cutter Config ready");
                state = Cutter::CutterStates::STOPPED;
            }else
            {
                Serial.println("Cutter Config being set up");
                Serial.println(ptr_5160_cut_stepper->step_5160_motor_cfg.configIndex);
            }

            break;
        case Cutter::CutterStates::STOPPED: 

            Serial.println("Attempting cutter move");
            ptr_5160_cut_stepper->set_velocity(80000);       
            state = Cutter::CutterStates::WINDING;
            break;

        case Cutter::CutterStates::WINDING:
  
            break;

        case Cutter::CutterStates::WOUND:

            break;

        case Cutter::CutterStates::RELEASE:
   
            break;      
        
        case Cutter::CutterStates::RELEASED:
   
            break;  

        case Cutter::CutterStates::ERROR_MOTOR:

            break;
        
        case Cutter::CutterStates::ESTOP:
        default:

            break;
    }

    write_interfaces();
}

void CutterFSMClass::write_interfaces()
{
    CcIoManager.set_pin_output_value(AutocadoCcPins::D4_CUT_SOLENOID, relay_output);
}

CutterFSMClass cutter;