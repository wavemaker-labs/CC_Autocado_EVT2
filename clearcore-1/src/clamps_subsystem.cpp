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

        lt_state = Clamp::ClampStates::SETUP;
        lb_state = Clamp::ClampStates::SETUP;
        rt_state = Clamp::ClampStates::SETUP;
        rb_state = Clamp::ClampStates::SETUP;
        ptr_5160_clamp_lt_stepper = CcIoManager.get_step_ptr(AutocadoCcSteppers::STEPPER_CLAMP_LT);
        ptr_5160_clamp_lb_stepper = CcIoManager.get_step_ptr(AutocadoCcSteppers::STEPPER_CLAMP_LB);
        ptr_5160_clamp_rt_stepper = CcIoManager.get_step_ptr(AutocadoCcSteppers::STEPPER_CLAMP_RT);
        ptr_5160_clamp_rb_stepper = CcIoManager.get_step_ptr(AutocadoCcSteppers::STEPPER_CLAMP_RB);
        ptr_5160_clamp_rb_stepper->special_flag = true; //ue this flag to denote bottom clamps, bc they move differently
        ptr_5160_clamp_lb_stepper->special_flag = true;
    }
}

void ClampsFSMClass::read_interfaces()
{
    open_switch_input = CcIoManager.get_input(A9_CLP_OPEN_BUTTON);
    recieve_switch_input = CcIoManager.get_input(A10_CLP_RECIEVE_BTN);
    clamp_switch_input = CcIoManager.get_input(A11_CLP_CLAMP_BTN);
    squish_switch_input = CcIoManager.get_input(D8_CLP_SQUISH_BTN);

}

void ClampsFSMClass::act_on_button(Cc5160Stepper * ptr_stepper, Clamp::ClampStates * ptr_state)
{
    if(open_switch_input == PinStatus::HIGH && 
    (*ptr_state != Clamp::ClampStates::AT_OPEN ||
     *ptr_state != Clamp::ClampStates::MOVING_TO_OPEN)){
        ptr_stepper->set_target_position(open_position, CLAMPS_MOVE_VMAX);
        *ptr_state = Clamp::ClampStates::MOVING_TO_OPEN;
    }else if (recieve_switch_input == PinStatus::HIGH && 
    (*ptr_state != Clamp::ClampStates::AT_RECIEVE ||
     *ptr_state != Clamp::ClampStates::MOVING_TO_RECIEVE)){
        /*check if it's a top or bottom clamp*/
        if(ptr_stepper->special_flag){
            ptr_stepper->set_target_position(recieve_position_bot, CLAMPS_MOVE_VMAX);
        }else{
            ptr_stepper->set_target_position(recieve_position_top, CLAMPS_MOVE_VMAX);
        }        
        *ptr_state = Clamp::ClampStates::MOVING_TO_RECIEVE;
    }else if (clamp_switch_input == PinStatus::HIGH && 
    (*ptr_state != Clamp::ClampStates::AT_CLAMPING||
     *ptr_state != Clamp::ClampStates::MOVING_TO_CLAMPING)){
        ptr_stepper->clear_enc_dev();
        ptr_stepper->set_target_position(clamp_position, CLAMPS_MOVE_VMAX);
        *ptr_state = Clamp::ClampStates::MOVING_TO_CLAMPING;
        Serial.println("moving to clamping");
    }else if (squish_switch_input == PinStatus::HIGH && 
    (*ptr_state != Clamp::ClampStates::AT_SQUISH ||  *ptr_state != Clamp::ClampStates::MOVING_TO_SQUISH)){
        ptr_stepper->set_target_position(squish_position, CLAMPS_MOVE_VMAX);
        *ptr_state = Clamp::ClampStates::MOVING_TO_SQUISH;
    }
}

void ClampsFSMClass::run()
{
    read_interfaces();

    // if (estop_input == ESTOP_ACTIVE && state != Cutter::CutterStatesESTOP)
    // {
    //     state = Cutter::CutterStatesESTOP;
    // }

    Cc5160Stepper * run_ptr_stepper;
    Clamp::ClampStates * run_prt_state;

    // Serial.println("Clamp Encoder counts");

    for(int i = 0; i < 4; i ++){   

        switch(i){
            case 0:
                run_ptr_stepper = ptr_5160_clamp_lt_stepper;
                run_prt_state = &lt_state;
                break;
            case 1:
                run_ptr_stepper = ptr_5160_clamp_lb_stepper;
                run_prt_state = &lb_state;
                break;
            case 2:
                run_ptr_stepper = ptr_5160_clamp_rt_stepper;
                run_prt_state = &rt_state;
                break;
            case 3:
                run_ptr_stepper = ptr_5160_clamp_rb_stepper;
                run_prt_state = &rb_state;
                break;
        }
        

        switch (*run_prt_state)
        {
            case Clamp::ClampStates::SETUP:
                if(run_ptr_stepper->config_ready())
                {
                    Serial.println("Clamp Config ready");
                    Serial.println("current ticks");
                    Serial.println(run_ptr_stepper->get_ticks());
                    Serial.println("Attempting away from home");
                    run_ptr_stepper->set_target_position(run_ptr_stepper->get_ticks() + CLAMPS_STEPS_AWAY_HOME, CLAMPS_HOME_VMAX);
                    *run_prt_state = Clamp::ClampStates::MOVING_AWAY_FROM_HOME;
                }else
                {
                    Serial.println("Clamp Config being set up");
                    Serial.println(run_ptr_stepper->step_5160_motor_cfg.configIndex);
                }

                break;

            case Clamp::ClampStates::MOVING_AWAY_FROM_HOME:
                
                if(run_ptr_stepper->at_position())
                {
                    Serial.println("clamp at position");
                    run_ptr_stepper->set_velocity(CLAMPS_HOME_VMAX);
                    *run_prt_state = Clamp::ClampStates::SET_SG;
                }
                
                break;

            case Clamp::ClampStates::SET_SG:

                if(run_ptr_stepper->at_vmax())
                {
                    Serial.println("clamp at vmax, setting sg");
                    run_ptr_stepper->set_enable_stallgaurd(true);
                    *run_prt_state = Clamp::ClampStates::WAIT_SG_HOME_DONE;
                }
        
                break;

            case Clamp::ClampStates::WAIT_SG_HOME_DONE:
                
                Serial.println(" waiting for home");
                if(run_ptr_stepper->at_sg_stall())
                {
                    Serial.println("at stall");
                    run_ptr_stepper->set_velocity(0);
                    run_ptr_stepper->set_enable_stallgaurd(false);
                    run_ptr_stepper->zero_xactual();
                    run_ptr_stepper->zero_encoder();
                    run_ptr_stepper->clear_enc_dev();
                    *run_prt_state = Clamp::ClampStates::HOME_DONE;
                }            
                break;

            case Clamp::ClampStates::HOME_DONE:           
            
                act_on_button(run_ptr_stepper, run_prt_state);
                break;

            case Clamp::ClampStates::MOVING_TO_OPEN:

                if(run_ptr_stepper->at_position()){
                    *run_prt_state = Clamp::ClampStates::AT_OPEN;
                }else{
                    act_on_button(run_ptr_stepper, run_prt_state);
                } 
                break;

            case Clamp::ClampStates::AT_OPEN:           
                    act_on_button(run_ptr_stepper, run_prt_state);
                break;

            case Clamp::ClampStates::MOVING_TO_RECIEVE:           
                if(run_ptr_stepper->at_position()){
                    *run_prt_state = Clamp::ClampStates::AT_RECIEVE;
                }else{
                    act_on_button(run_ptr_stepper, run_prt_state);
                } 
                break;

            case Clamp::ClampStates::AT_RECIEVE:
                act_on_button(run_ptr_stepper, run_prt_state);  
                break;

            case Clamp::ClampStates::MOVING_TO_CLAMPING:           
                if(run_ptr_stepper->at_position()){
                    *run_prt_state = Clamp::ClampStates::AT_CLAMPING;
                }else if(run_ptr_stepper->detect_enc_dev()){
                    // if(i == 0){Serial.println("detected dev");
                    // Serial.println(run_ptr_stepper->get_ticks());}
                    run_ptr_stepper->set_velocity(0);
                    *run_prt_state = Clamp::ClampStates::DETECTED_CLAMP;
                }else{
                    act_on_button(run_ptr_stepper, run_prt_state);
                } 
                break;

            case Clamp::ClampStates::DETECTED_CLAMP:           
                if(run_ptr_stepper->at_position() || run_ptr_stepper->get_velocity() == 0){
                    *run_prt_state = Clamp::ClampStates::AT_CLAMPING;
                }else{
                    act_on_button(run_ptr_stepper, run_prt_state);
                } 
                break;

            case Clamp::ClampStates::AT_CLAMPING:
                // if(i == 0){
                //     Serial.println("at clamped");
                //     Serial.println(run_ptr_stepper->get_ticks());
                //     Serial.println(run_ptr_stepper->get_encoder_count());} 
                act_on_button(run_ptr_stepper, run_prt_state); 
                break;

            case Clamp::ClampStates::MOVING_TO_SQUISH:           
                if(run_ptr_stepper->at_position()){
                    *run_prt_state = Clamp::ClampStates::AT_SQUISH;
                }else{
                    act_on_button(run_ptr_stepper, run_prt_state);
                }
                break;

            case Clamp::ClampStates::AT_SQUISH:   
                act_on_button(run_ptr_stepper, run_prt_state);      
                break;
            
            case Clamp::ClampStates::ESTOP:
            default:

                break;
        }

    }

    write_interfaces();
}

void ClampsFSMClass::write_interfaces()
{
    // CcIoManager.set_pin_output_value(AutocadoCcPins::D4_CUT_SOLENOID, relay_output);
}

ClampsFSMClass clamps;