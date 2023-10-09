 /**
 * @file clamps_subsystem.cpp
 * @copyright Copyright (c) 2023 Vebu Labs. All rights reserved.
 * @brief This library is used to control the cutter subsystem
 * @author Mike Lui
*/

#include "clamps_subsystem.hpp"


bool new_clamps_motor_mb_cmd = false;

uint16_t clamps_motor_hreg_write(TRegister* reg, uint16_t val) {
    new_clamps_motor_mb_cmd = true;
    return val;
}


void ClampsFSMClass::setup()
{
    if(!has_setup){
        has_setup = true;        

        state = Clamp::ClampStates::SETUP;
        ptr_5160_clamp_lt_stepper = CcIoManager.get_step_ptr(AutocadoCcSteppers::STEPPER_1);
        ptr_5160_clamp_lb_stepper = nullptr;
        ptr_5160_clamp_rt_stepper = nullptr;
        ptr_5160_clamp_rb_stepper = nullptr;
    }
}

void ClampsFSMClass::read_interfaces()
{
    open_switch_input = CcIoManager.get_input(A9_CLP_OPEN_BUTTON);
    recieve_switch_input = CcIoManager.get_input(A10_CLP_RECIEVE_BTN);
    clamp_switch_input = CcIoManager.get_input(A11_CLP_CLAMP_BTN);
    squish_switch_input = CcIoManager.get_input(A12_CLP_SQUISH_BTN);
}

void ClampsFSMClass::run()
{
    read_interfaces();

    // if (estop_input == ESTOP_ACTIVE && state != Cutter::CutterStatesESTOP)
    // {
    //     state = Cutter::CutterStatesESTOP;
    // }


    switch (state)
    {
        case Clamp::ClampStates::SETUP:
            ptr_5160_clamp_lt_stepper->set_velocity(-40000);


            break;
        case Clamp::ClampStates::OPENING:           

            break;

        case Clamp::ClampStates::RECIEVING:
  
            break;

        case Clamp::ClampStates::CLAMPING:

            break;

        case Clamp::ClampStates::CLAMPED:
   
            break;      
        
        case Clamp::ClampStates::SQUISHING:
   
            break;  

        case Clamp::ClampStates::SQUISHED:

            break;
        
        case Clamp::ClampStates::ESTOP:
        default:

            break;
    }

    write_interfaces();
}

void ClampsFSMClass::write_interfaces()
{
    // CcIoManager.set_pin_output_value(AutocadoCcPins::D4_CUT_SOLENOID, relay_output);
}

ClampsFSMClass clamps;