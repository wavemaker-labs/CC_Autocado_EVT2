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
        ptr_5160_clamp_rb_stepper->special_flag = true; //use this flag to denote bottom clamps, bc they move differently
        ptr_5160_clamp_lb_stepper->special_flag = true;

        CcIoManager.set_mb_w_hreg_cb(MbRegisterOffsets::CLAMP_HOME_VEL, &clamps_motor_hreg_write);
        CcIoManager.set_mb_w_hreg_cb(MbRegisterOffsets::CLAMP_INITIAL_CLOSE_VEL, &clamps_motor_hreg_write);
        CcIoManager.set_mb_w_hreg_cb(MbRegisterOffsets::CLAMP_MOVE_VEL, &clamps_motor_hreg_write);
        CcIoManager.set_mb_w_hreg_cb(MbRegisterOffsets::CLAMP_CONTACT_VEL, &clamps_motor_hreg_write);
        CcIoManager.set_mb_w_hreg_cb(MbRegisterOffsets::TOP_RECEIVE_POS, &clamps_motor_hreg_write);
        CcIoManager.set_mb_w_hreg_cb(MbRegisterOffsets::BOTTOM_RECEIVE_POS, &clamps_motor_hreg_write);
        CcIoManager.set_mb_w_hreg_cb(MbRegisterOffsets::CLAMP_SQUISH_POS, &clamps_motor_hreg_write);
        CcIoManager.set_mb_w_hreg_cb(MbRegisterOffsets::PRECLAMP_POS, &clamps_motor_hreg_write);
        CcIoManager.set_mb_w_hreg_cb(MbRegisterOffsets::CLAMP_POS, &clamps_motor_hreg_write);
        CcIoManager.set_mb_w_hreg_cb(MbRegisterOffsets::PRECUT_CLAMP_OFFSET, &clamps_motor_hreg_write);
        CcIoManager.set_mb_w_hreg_cb(MbRegisterOffsets::PRECORE_CLAMP_OFFSET, &clamps_motor_hreg_write);
        CcIoManager.set_mb_w_hreg_cb(MbRegisterOffsets::PRERUB_OPEN_OFFSET, &clamps_motor_hreg_write);
        CcIoManager.set_mb_w_hreg_cb(MbRegisterOffsets::CLAMP_RUB_OFFSET, &clamps_motor_hreg_write);
        CcIoManager.set_mb_w_hreg_cb(MbRegisterOffsets::CLAMP_RUB_VEL, &clamps_motor_hreg_write);
        CcIoManager.set_mb_w_hreg_cb(MbRegisterOffsets::CLAMP_PRESQUISH_DELAY, &clamps_motor_hreg_write);
        CcIoManager.set_mb_w_hreg_cb(MbRegisterOffsets::OPEN_POS, &clamps_motor_hreg_write);
    }
}

void ClampsFSMClass::read_interfaces()
{
    SubCommsClass::SubsystemCommands conductor_cmd;

    conductor_cmd = CcIoManager.IntraComms[SubsystemList::CLAMPS_SUBS].get_ss_cmd();
    
    home_command = (conductor_cmd == SubCommsClass::SubsystemCommands::HOME_COMMAND);

    if (new_clamps_motor_mb_cmd){
        home_velocity = CcIoManager.get_mb_data(MbRegisterOffsets::CLAMP_HOME_VEL);
        initial_close_vmax = CcIoManager.get_mb_data(MbRegisterOffsets::CLAMP_INITIAL_CLOSE_VEL);
        move_velocity = CcIoManager.get_mb_data(MbRegisterOffsets::CLAMP_MOVE_VEL);
        contact_velocity = CcIoManager.get_mb_data(MbRegisterOffsets::CLAMP_CONTACT_VEL);
        receive_position_top = CcIoManager.get_mb_data(MbRegisterOffsets::TOP_RECEIVE_POS);
        receive_position_bot = CcIoManager.get_mb_data(MbRegisterOffsets::BOTTOM_RECEIVE_POS);
        squish_position = CcIoManager.get_mb_data(MbRegisterOffsets::CLAMP_SQUISH_POS);
        pre_clamp_position = CcIoManager.get_mb_data(MbRegisterOffsets::PRECLAMP_POS);
        clamp_position = CcIoManager.get_mb_data(MbRegisterOffsets::CLAMP_POS);
        pre_cut_clamp_offset = CcIoManager.get_mb_data(MbRegisterOffsets::PRECUT_CLAMP_OFFSET);
        pre_core_clamp_offset = CcIoManager.get_mb_data(MbRegisterOffsets::PRECORE_CLAMP_OFFSET);
        pre_rub_open_offset = CcIoManager.get_mb_data(MbRegisterOffsets::PRERUB_OPEN_OFFSET);
        rub_offset = CcIoManager.get_mb_data(MbRegisterOffsets::CLAMP_RUB_OFFSET);
        rub_velocity = CcIoManager.get_mb_data(MbRegisterOffsets::CLAMP_RUB_VEL);
        pre_squish_delay = CcIoManager.get_mb_data(MbRegisterOffsets::CLAMP_PRESQUISH_DELAY);
        open_position = CcIoManager.get_mb_data(MbRegisterOffsets::OPEN_POS);
    }

    #ifndef SINGLE_BUTTON_AUTO_RUN //use buttons else use commands from intracomms
    open_switch_input = CcIoManager.get_input(A9_CLP_OPEN_BUTTON);
    recieve_switch_input = CcIoManager.get_input(A10_CLP_RECIEVE_BTN);
    clamp_switch_input = CcIoManager.get_input(A11_CLP_CLAMP_BTN);
    grab_switch_input = CcIoManager.get_input(D2_CLP_GRAB_BUTTON);
    squish_switch_input = CcIoManager.get_input(D8_CLP_SQUISH_BTN);
    #else
    open_switch_input = (conductor_cmd == CLAMPS_OPEN_CMD);
    recieve_switch_input = (conductor_cmd == CLAMPS_RECIEVE_CMD);
    clamp_switch_input = (conductor_cmd == CLAMPS_CLAMP_CMD);
    grab_switch_input = (conductor_cmd == CLAMPS_GRAB_CMD);
    squish_switch_input = (conductor_cmd == CLAMPS_SQUISH_CMD);
    #endif

}

/*during rub the steps + direction differ due to position
case 0: ptr_5160_clamp_lt_stepper
case 1: ptr_5160_clamp_lb_stepper
case 2: ptr_5160_clamp_rt_stepper
case 3: ptr_5160_clamp_rb_stepper
*/
int32_t clamp_rub_calc(int32_t steps, int motor_pos)
{
    switch (motor_pos)
    {
    case 0:
    case 2:
        return steps;
        break;

    case 1:
    case 3:
        return (-1 * steps);
        break;

    default:
        return 0;
        break;
    }
}

/*Can clean this up with some better functions or enum declarations*/
void ClampsFSMClass::act_on_button(Cc5160Stepper * ptr_stepper, Clamp::ClampStates * ptr_state)
{
    if(open_switch_input == PinStatus::HIGH && 
    (*ptr_state != Clamp::ClampStates::AT_OPEN ||
     *ptr_state != Clamp::ClampStates::MOVING_TO_OPEN)){
        ptr_stepper->set_target_position(open_position, move_velocity);
        *ptr_state = Clamp::ClampStates::MOVING_TO_OPEN;
    }else if (recieve_switch_input == PinStatus::HIGH && 
    (*ptr_state != Clamp::ClampStates::AT_RECIEVE ||
     *ptr_state != Clamp::ClampStates::MOVING_TO_RECIEVE)){
        /*check if it's a top or bottom clamp*/
        if(ptr_stepper->special_flag){
            ptr_stepper->set_target_position(receive_position_bot, move_velocity);
        }else{
            ptr_stepper->set_target_position(receive_position_top, move_velocity);
        }        
        *ptr_state = Clamp::ClampStates::MOVING_TO_RECIEVE;
    }else if (clamp_switch_input == PinStatus::HIGH && 
    (*ptr_state != Clamp::ClampStates::AT_CLAMPING ||
     *ptr_state != Clamp::ClampStates::MOVING_TO_CLAMPING ||
     *ptr_state != Clamp::ClampStates::MOVING_TO_PRE_CLAMPING ||
     *ptr_state != Clamp::ClampStates::WAIT_ALL_PRE_CLAMPING ||
     *ptr_state != Clamp::ClampStates::AT_PRE_CLAMPING ||
     *ptr_state != Clamp::ClampStates::DETECTED_CLAMP||
     *ptr_state != Clamp::ClampStates::MOVING_TO_POST_CLAMP||
     *ptr_state != Clamp::ClampStates::WAITING_POST_CLAMP||
     *ptr_state != Clamp::ClampStates::AT_POST_CLAMP)){
        ptr_stepper->set_target_position(pre_clamp_position, move_velocity);
        *ptr_state = Clamp::ClampStates::MOVING_TO_PRE_CLAMPING;
    }else if (squish_switch_input == PinStatus::HIGH && 
    (*ptr_state != Clamp::ClampStates::AT_SQUISH ||  *ptr_state != Clamp::ClampStates::MOVING_TO_SQUISH)){
        ptr_stepper->set_target_position(squish_position, move_velocity);
        *ptr_state = Clamp::ClampStates::MOVING_TO_SQUISH;
    }else if (grab_switch_input == PinStatus::HIGH && 
    (*ptr_state != Clamp::ClampStates::MOVING_TO_PRE_CORE)){
        *ptr_state = Clamp::ClampStates::MOVING_TO_PRE_CORE;
    }

    if ( 
     *ptr_state == Clamp::ClampStates::HOME_DONE ||
     *ptr_state == Clamp::ClampStates::AT_OPEN ||
     *ptr_state == Clamp::ClampStates::AT_RECIEVE ||
     *ptr_state == Clamp::ClampStates::AT_POST_CLAMP ||
     *ptr_state == Clamp::ClampStates::AT_PRE_CORE ||
     *ptr_state == Clamp::ClampStates::AT_SQUISH)
     {
        led_output = PinStatus::LOW;
    }else
    {
        led_output = PinStatus::HIGH;
    }
}

void ClampsFSMClass::run()
{
    read_interfaces();

    Cc5160Stepper * run_ptr_stepper;
    Clamp::ClampStates * run_prt_state;

    int stepper_number;

    /*runs through the 4 clamps state machines*/
    for(stepper_number = 0; stepper_number < 4; stepper_number ++){   

        switch(stepper_number){
            case 0:
                run_ptr_stepper = ptr_5160_clamp_lt_stepper;
                run_prt_state = &lt_state;

                lt_ticks = run_ptr_stepper->get_old_x();
                lt_encoder = run_ptr_stepper->get_encoder_count();
                break;
            case 1:
                run_ptr_stepper = ptr_5160_clamp_lb_stepper;
                run_prt_state = &lb_state;

                lb_ticks = run_ptr_stepper->get_old_x();
                lb_encoder = run_ptr_stepper->get_encoder_count();
                break;
            case 2:
                run_ptr_stepper = ptr_5160_clamp_rt_stepper;
                run_prt_state = &rt_state;

                rt_ticks = run_ptr_stepper->get_old_x();
                rt_encoder = run_ptr_stepper->get_encoder_count();
                break;
            case 3:
                run_ptr_stepper = ptr_5160_clamp_rb_stepper;
                run_prt_state = &rb_state;

                rb_ticks = run_ptr_stepper->get_old_x();
                rb_encoder = run_ptr_stepper->get_encoder_count();
                break;
        }
        

        switch (*run_prt_state)
        {
            case Clamp::ClampStates::SETUP:
                if(run_ptr_stepper->config_ready())
                {
                    Serial.println("Clamp Config ready");
                    Serial.println(run_ptr_stepper->get_old_x());
                    run_ptr_stepper->set_target_position(run_ptr_stepper->get_old_x() + CLAMPS_STEPS_AWAY_HOME, home_velocity);
                    *run_prt_state = Clamp::ClampStates::MOVING_AWAY_FROM_CLOSE;
                }else
                {
                    Serial.println("Clamp Config being set up");
                    Serial.println(run_ptr_stepper->step_5160_motor_cfg.configIndex);
                }
                break;

            case Clamp::ClampStates::MOVING_AWAY_FROM_CLOSE:

                if(run_ptr_stepper->at_position())
                {
                    run_ptr_stepper->set_velocity(initial_close_vmax);
                    *run_prt_state = Clamp::ClampStates::SET_SG_CLOSING_CLAMP;                    
                }
                break;

             case Clamp::ClampStates::SET_SG_CLOSING_CLAMP:

                if(run_ptr_stepper->at_vmax())
                {
                    // Serial.println("clamp at vmax, setting sg");
                    run_ptr_stepper->set_enable_stallgaurd(true);
                    *run_prt_state = Clamp::ClampStates::SG_CLOSING_CLAMP;
                }

                break;

            case Clamp::ClampStates::SG_CLOSING_CLAMP:

                if(run_ptr_stepper->at_sg_stall())
                {
                    Serial.println("at stall");
                    run_ptr_stepper->set_velocity(0);
                    run_ptr_stepper->set_enable_stallgaurd(false);
                    *run_prt_state = Clamp::ClampStates::WAIT_HOME_CMD;
                }  
                break;

            case Clamp::ClampStates::WAIT_HOME_CMD:
                
                /*wait for ok to home from conductor level*/
                if(home_command){
                    run_ptr_stepper->set_velocity(home_velocity);
                    *run_prt_state = Clamp::ClampStates::SET_SG;
                }
                
                break;

            case Clamp::ClampStates::SET_SG:

                if(run_ptr_stepper->at_vmax())
                {
                    // Serial.println("clamp at vmax, setting sg");
                    run_ptr_stepper->set_enable_stallgaurd(true);
                    *run_prt_state = Clamp::ClampStates::WAIT_SG_HOME_DONE;
                }
                break;

            case Clamp::ClampStates::WAIT_SG_HOME_DONE:
                
                // Serial.println(" waiting for home");
                if(run_ptr_stepper->at_sg_stall())
                {
                    Serial.println("at stall");
                    run_ptr_stepper->set_velocity(0);
                    run_ptr_stepper->set_enable_stallgaurd(false);
                    run_ptr_stepper->zero_xactual();
                    run_ptr_stepper->zero_encoder();
                    run_ptr_stepper->clear_enc_dev();
                    *run_prt_state = Clamp::ClampStates::MOVE_TO_ZERO_REF;
                }            
                break;

            case Clamp::ClampStates::MOVE_TO_ZERO_REF:
                Serial.println("moving to zero");
                if(stepper_number == 0 || stepper_number == 2){
                    run_ptr_stepper->set_target_position(-1*receive_position_top, move_velocity);
                }else{
                    run_ptr_stepper->set_target_position(0, move_velocity);
                }     
                *run_prt_state = Clamp::ClampStates::AT_ZERO_REF;
                break;

            case Clamp::ClampStates::AT_ZERO_REF:           
            
                if(run_ptr_stepper->at_position())
                {
                    Serial.println("at zero");
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
                    run_ptr_stepper->set_velocity(home_velocity);
                    *run_prt_state = Clamp::ClampStates::SET_SG;
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

            case Clamp::ClampStates::MOVING_TO_PRE_CLAMPING:
                if(run_ptr_stepper->at_position()){
                    *run_prt_state = Clamp::ClampStates::WAIT_ALL_PRE_CLAMPING;
                }else{
                    act_on_button(run_ptr_stepper, run_prt_state);
                }
                break;

            case Clamp::ClampStates::WAIT_ALL_PRE_CLAMPING:
                /*all at same state*/
                if(lt_state == Clamp::ClampStates::WAIT_ALL_PRE_CLAMPING &&
                    lb_state == Clamp::ClampStates::WAIT_ALL_PRE_CLAMPING &&
                    rt_state == Clamp::ClampStates::WAIT_ALL_PRE_CLAMPING &&
                    rb_state == Clamp::ClampStates::WAIT_ALL_PRE_CLAMPING){
                        lt_state = Clamp::ClampStates::AT_PRE_CLAMPING;
                        lb_state = Clamp::ClampStates::AT_PRE_CLAMPING;
                        rt_state = Clamp::ClampStates::AT_PRE_CLAMPING;
                        rb_state = Clamp::ClampStates::AT_PRE_CLAMPING;
                }else{
                    act_on_button(run_ptr_stepper, run_prt_state);
                }
                break;
            
             case Clamp::ClampStates::AT_PRE_CLAMPING:
                /*all at same state*/                
                run_ptr_stepper->clear_enc_dev();
                run_ptr_stepper->set_target_position(clamp_position, contact_velocity);
                *run_prt_state = Clamp::ClampStates::MOVING_TO_CLAMPING;
                break;

            case Clamp::ClampStates::MOVING_TO_CLAMPING:

                // Serial.println(stepper_number);
                // Serial.println(run_ptr_stepper->get_old_x());
                // Serial.println(run_ptr_stepper->get_encoder_count());              
                if(run_ptr_stepper->at_position()){
                    *run_prt_state = Clamp::ClampStates::AT_CLAMPING;
                    // Serial.println("Clamp: at clamp position");
                }else if(run_ptr_stepper->detect_enc_dev()){
                    run_ptr_stepper->set_velocity(0);
                    *run_prt_state = Clamp::ClampStates::DETECTED_CLAMP;
                    // Serial.println("Clamp: enc dev detected");
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
                /*all at same state*/
                if(lt_state == Clamp::ClampStates::AT_CLAMPING &&
                    lb_state == Clamp::ClampStates::AT_CLAMPING &&
                    rt_state == Clamp::ClampStates::AT_CLAMPING &&
                    rb_state == Clamp::ClampStates::AT_CLAMPING){

                        /*If there isn't a large difference between encoder and tick at the end of clamping, move back to recieve*/
                        if((run_ptr_stepper->get_encoder_count() - run_ptr_stepper->get_old_x()) < CLAMPS_NO_AVO_IN_CLAMP)
                        {
                            if(run_ptr_stepper->special_flag){
                                run_ptr_stepper->set_target_position(receive_position_bot, move_velocity);
                            }else{
                                run_ptr_stepper->set_target_position(receive_position_top, move_velocity);
                            }        
                            *run_prt_state = Clamp::ClampStates::MOVING_TO_RECIEVE;
                        }
                        else
                        {
                            lt_state = Clamp::ClampStates::MOVING_TO_POST_CLAMP;
                            lb_state = Clamp::ClampStates::MOVING_TO_POST_CLAMP;
                            rt_state = Clamp::ClampStates::MOVING_TO_POST_CLAMP;
                            rb_state = Clamp::ClampStates::MOVING_TO_POST_CLAMP;
                        }
                }else{
                    act_on_button(run_ptr_stepper, run_prt_state);
                } 
                break;

            case Clamp::ClampStates::MOVING_TO_POST_CLAMP:
                /*all at same state*/
                Serial.println(run_ptr_stepper->get_old_x());
                run_ptr_stepper->set_target_position(run_ptr_stepper->get_old_x() + pre_cut_clamp_offset, contact_velocity);
                *run_prt_state = Clamp::ClampStates::WAITING_POST_CLAMP;
                
                break;

            case Clamp::ClampStates::WAITING_POST_CLAMP:
                if(run_ptr_stepper->at_position()){
                    *run_prt_state = Clamp::ClampStates::AT_POST_CLAMP;
                }else{
                    act_on_button(run_ptr_stepper, run_prt_state);
                }                 
                break;

            case Clamp::ClampStates::AT_POST_CLAMP:
                act_on_button(run_ptr_stepper, run_prt_state); 
                break;

            case Clamp::ClampStates::MOVING_TO_PRE_CORE:
                Serial.println(run_ptr_stepper->get_old_x());
                run_ptr_stepper->set_target_position(run_ptr_stepper->get_old_x() + pre_core_clamp_offset, contact_velocity);
                *run_prt_state = Clamp::ClampStates::WAITING_PRE_CORE;
                break;

            case Clamp::ClampStates::WAITING_PRE_CORE:
                if(run_ptr_stepper->at_position()){
                    *run_prt_state = Clamp::ClampStates::AT_PRE_CORE;
                }else{
                    act_on_button(run_ptr_stepper, run_prt_state);
                }                 
                break;

            case Clamp::ClampStates::AT_PRE_CORE:
                act_on_button(run_ptr_stepper, run_prt_state);                
                break;

            case Clamp::ClampStates::MOVING_TO_SQUISH:           
                if(run_ptr_stepper->at_position()){
                    *run_prt_state = Clamp::ClampStates::SYNC_SQUISH;
                }else{
                    act_on_button(run_ptr_stepper, run_prt_state);
                }
                break;

            case Clamp::ClampStates::SYNC_SQUISH:
                /*all at same state*/
                if(lt_state == Clamp::ClampStates::SYNC_SQUISH &&
                    lb_state == Clamp::ClampStates::SYNC_SQUISH &&
                    rt_state == Clamp::ClampStates::SYNC_SQUISH &&
                    rb_state == Clamp::ClampStates::SYNC_SQUISH &&
                    stepper_number == 3){ //only do this on the last pass through
                        lt_state = Clamp::ClampStates::AT_SQUISH;
                        lb_state = Clamp::ClampStates::AT_SQUISH;
                        rt_state = Clamp::ClampStates::AT_SQUISH;
                        rb_state = Clamp::ClampStates::AT_SQUISH;
                }else{
                    act_on_button(run_ptr_stepper, run_prt_state);
                } 
                break;            

            case Clamp::ClampStates::AT_SQUISH:   
                run_ptr_stepper->set_target_position(run_ptr_stepper->get_old_x() + pre_rub_open_offset, contact_velocity);
                *run_prt_state = Clamp::ClampStates::PRE_RUB_OPEN; 
                break;

            case Clamp::ClampStates::PRE_RUB_OPEN:
                if(run_ptr_stepper->at_position()){
                    run_ptr_stepper->set_target_position(run_ptr_stepper->get_old_x() + clamp_rub_calc(rub_offset, stepper_number), rub_velocity);
                    *run_prt_state = Clamp::ClampStates::RUB_OUT_1;
                }else{
                    act_on_button(run_ptr_stepper, run_prt_state);      
                }
                break;

            case Clamp::ClampStates::RUB_OUT_1:
                if(run_ptr_stepper->at_position()){
                    run_ptr_stepper->set_target_position(run_ptr_stepper->get_old_x() + clamp_rub_calc((-2 * rub_offset), stepper_number), rub_velocity);
                    *run_prt_state = Clamp::ClampStates::RUB_IN_1;
                }else{
                    act_on_button(run_ptr_stepper, run_prt_state);      
                }
                break;

            case Clamp::ClampStates::RUB_IN_1:
                if(run_ptr_stepper->at_position()){
                    run_ptr_stepper->set_target_position(run_ptr_stepper->get_old_x() + clamp_rub_calc((2 * rub_offset), stepper_number), rub_velocity);
                    *run_prt_state = Clamp::ClampStates::RUB_OUT_2;
                }else{
                    act_on_button(run_ptr_stepper, run_prt_state);      
                }      
                break;
                
            case Clamp::ClampStates::RUB_OUT_2:   
                if(run_ptr_stepper->at_position()){
                    run_ptr_stepper->set_target_position(run_ptr_stepper->get_old_x() + clamp_rub_calc((-2 * rub_offset), stepper_number), rub_velocity);
                    *run_prt_state = Clamp::ClampStates::RUB_IN_2;
                }else{
                    act_on_button(run_ptr_stepper, run_prt_state);      
                }         
                break;

            case Clamp::ClampStates::RUB_IN_2:   
                if(run_ptr_stepper->at_position()){
                    run_ptr_stepper->set_target_position(run_ptr_stepper->get_old_x() + clamp_rub_calc(rub_offset, stepper_number), rub_velocity);
                    *run_prt_state = Clamp::ClampStates::RUB_OUT_3;
                }else{
                    act_on_button(run_ptr_stepper, run_prt_state);      
                }       
                break;

            case Clamp::ClampStates::RUB_OUT_3:   
                if(run_ptr_stepper->at_position()){
                    *run_prt_state = Clamp::ClampStates::RUB_DONE;
                }else{
                    act_on_button(run_ptr_stepper, run_prt_state);      
                }       
                break;

            case Clamp::ClampStates::RUB_DONE:   
                act_on_button(run_ptr_stepper, run_prt_state);      
                break;
           
            
            case Clamp::ClampStates::ESTOP:
            default:

                break;
        }

    }

    /*Here we map the rail states to states for the higher level controller.*/
    /*need to use a big if else statement, not sure how to do it otherwise with 2 variables
    */
    if(lt_state == Clamp::ERROR_MOTOR || 
        lb_state == Clamp::ERROR_MOTOR ||
        rt_state == Clamp::ERROR_MOTOR ||
        rb_state == Clamp::ERROR_MOTOR){

            CcIoManager.IntraComms[SubsystemList::CLAMPS_SUBS].set_ss_state(SubCommsClass::SubsystemStates::ERROR_MOTOR);

    }else if(lt_state == Clamp::SETUP || 
            lb_state == Clamp::SETUP ||
            rt_state == Clamp::SETUP ||
            rb_state == Clamp::SETUP){

            CcIoManager.IntraComms[SubsystemList::CLAMPS_SUBS].set_ss_state(SubCommsClass::SubsystemStates::SETUP);

    }else if(lt_state == Clamp::WAIT_HOME_CMD && 
            lb_state == Clamp::WAIT_HOME_CMD &&
            rt_state == Clamp::WAIT_HOME_CMD &&
            rb_state == Clamp::WAIT_HOME_CMD){

            CcIoManager.IntraComms[SubsystemList::CLAMPS_SUBS].set_ss_state(SubCommsClass::SubsystemStates::WAITING_HOME_CMD);

    }else if((lt_state == Clamp::HOME_DONE ||
                lt_state == Clamp::AT_OPEN ||
                lt_state == Clamp::AT_RECIEVE ||
                lt_state == Clamp::AT_POST_CLAMP ||
                lt_state == Clamp::AT_PRE_CORE ||
                lt_state == Clamp::RUB_DONE) &&
                (lt_state == lb_state && 
                lb_state == rt_state &&
                rt_state == rb_state )) {

            CcIoManager.IntraComms[SubsystemList::CLAMPS_SUBS].set_ss_state(SubCommsClass::SubsystemStates::WAITING_INPUT);
    }else {
        CcIoManager.IntraComms[SubsystemList::CLAMPS_SUBS].set_ss_state(SubCommsClass::SubsystemStates::MOVING);
    }

    write_interfaces();
}

void ClampsFSMClass::write_interfaces()
{
    CcIoManager.set_pin_output_state (AutocadoCcPins::D3_CLAPS_BUSY_LED, led_output);

    //input registers
    CcIoManager.set_mb_data(MbRegisterOffsets::LEFT_TOP_CLAMP_STATE, lt_state);
    CcIoManager.set_mb_data(MbRegisterOffsets::LEFT_BOTTOM_CLAMP_STATE, lb_state);
    CcIoManager.set_mb_data(MbRegisterOffsets::RIGHT_TOP_CLAMP_STATE, rt_state);
    CcIoManager.set_mb_data(MbRegisterOffsets::RIGHT_BOTTOM_CLAMP_STATE, rb_state);

    CcIoManager.set_mb_data(MbRegisterOffsets::LEFT_TOP_TICK, lt_ticks);
    CcIoManager.set_mb_data(MbRegisterOffsets::LEFT_TOP_ENCODER, lt_encoder);
    CcIoManager.set_mb_data(MbRegisterOffsets::LEFT_TOP_TICK, lb_ticks);
    CcIoManager.set_mb_data(MbRegisterOffsets::LEFT_TOP_ENCODER, lb_encoder);
    CcIoManager.set_mb_data(MbRegisterOffsets::LEFT_TOP_TICK, rt_ticks);
    CcIoManager.set_mb_data(MbRegisterOffsets::LEFT_TOP_ENCODER, rt_encoder);
    CcIoManager.set_mb_data(MbRegisterOffsets::LEFT_TOP_TICK, rb_ticks);
    CcIoManager.set_mb_data(MbRegisterOffsets::LEFT_TOP_ENCODER, rb_encoder);

    //parameters

}

ClampsFSMClass clamps;