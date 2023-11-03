 /**
 * @file rotators_subsystem.cpp
 * @copyright Copyright (c) 2023 Vebu Labs. All rights reserved.
 * @brief This library is used to control the rotators subsystem
 * @author Mike Lui
*/

#include "rotators_subsystem.hpp"


bool new_rots_motor_mb_cmd = false;

uint16_t rots_motor_hreg_write(TRegister* reg, uint16_t val) {
    new_rots_motor_mb_cmd = true;
    return val;
}


void RotsFSMClass::setup()
{
    if(!has_setup){
        has_setup = true;        

        l_state = Rots::RotsStates::SETUP;
        r_state = Rots::RotsStates::SETUP;
        IntraComms[SubsystemList::ROTS_SUBS].set_ss_state(SubCommsClass::SubsystemStates::SETUP);
        ptr_5160_rot_l_stepper = CcIoManager.get_step_ptr(AutocadoCcSteppers::STEPPER_ROT_L);
        ptr_5160_rot_r_stepper = CcIoManager.get_step_ptr(AutocadoCcSteppers::STEPPER_ROT_R);
    }
}

void RotsFSMClass::read_interfaces()
{
    switch_0_input = CcIoManager.get_input(D0_RAIL_SW_0);
    switch_1_input = CcIoManager.get_input(D1_RAIL_SW_1);

    if(switch_0_input == PinStatus::HIGH && switch_1_input == PinStatus::LOW){ 
        cmd_position = Rots::RotsPositions::RECEIVE_POS;
    }else if(switch_0_input == PinStatus::LOW && switch_1_input == PinStatus::LOW){ 
        cmd_position = Rots::RotsPositions::SQUISH_POS;
    }else if(switch_0_input == PinStatus::LOW && switch_1_input == PinStatus::HIGH){
        cmd_position = Rots::RotsPositions::CORE_POS;
    }
}

void RotsFSMClass::act_on_button(Cc5160Stepper * ptr_stepper, Rots::RotsStates * ptr_state)
{
    if(cmd_position == Rots::RotsPositions::RECEIVE_POS &&
        *ptr_state != Rots::RotsStates::AT_RECIEVE && 
        *ptr_state != Rots::RotsStates::MOVING_TO_RECIEVE){

            ptr_stepper->set_target_position(recieve_position, ROTS_MOVE_VMAX);
            *ptr_state = Rots::RotsStates::MOVING_TO_RECIEVE;

    }else if (cmd_position == Rots::RotsPositions::SQUISH_POS &&
        *ptr_state != Rots::RotsStates::AT_SQUISH && 
        *ptr_state != Rots::RotsStates::MOVING_TO_SQUISH){

            ptr_stepper->set_target_position(squish_position, ROTS_MOVE_VMAX);
            *ptr_state = Rots::RotsStates::MOVING_TO_SQUISH;

    }else if (cmd_position == Rots::RotsPositions::CORE_POS &&
        *ptr_state != Rots::RotsStates::AT_CORE && 
        *ptr_state != Rots::RotsStates::MOVING_TO_CORE){

            ptr_stepper->set_target_position(core_position, ROTS_MOVE_VMAX);
            *ptr_state = Rots::RotsStates::MOVING_TO_CORE;
    }
}

void RotsFSMClass::run()
{
    read_interfaces();

    Cc5160Stepper * run_ptr_stepper;
    Rots::RotsStates * run_prt_state;

    for(int i = 0; i < 2; i++)//2 for two axis
    {
        switch(i){
            case 0:
                run_ptr_stepper = ptr_5160_rot_l_stepper;
                run_prt_state = &l_state;
                break;
            case 1:
                run_ptr_stepper = ptr_5160_rot_r_stepper;
                run_prt_state = &r_state;
                break;
        }
    

         /*This state machine is for movements*/
        switch (*run_prt_state)
        {            
            case Rots::RotsStates::SETUP:
                
                if(run_ptr_stepper->config_ready())
                {
                    Serial.println("Rot Config ready");
                    Serial.println("Attempting away from home");
                    Serial.println(run_ptr_stepper->get_old_x());
                    run_ptr_stepper->set_target_position(run_ptr_stepper->get_old_x() + ROTS_STEPS_AWAY_HOME, ROTS_HOME_VMAX);
                    *run_prt_state = Rots::RotsStates::MOVING_AWAY_FROM_HOME;
                }else
                {
                    Serial.println("Rot Config setting up");
                    Serial.println(run_ptr_stepper->step_5160_motor_cfg.configIndex);
                }
                break;

            case Rots::RotsStates::MOVING_AWAY_FROM_HOME:
                
                if(run_ptr_stepper->at_position())
                {
                    run_ptr_stepper->set_velocity(ROTS_HOME_VMAX);
                    *run_prt_state = Rots::RotsStates::SET_SG;
                }
                
                break;

            case Rots::RotsStates::SET_SG:

                if(run_ptr_stepper->at_vmax())
                {
                    // Serial.println("Rail at vmax, setting sg");
                    run_ptr_stepper->set_enable_stallgaurd(true);
                    *run_prt_state = Rots::RotsStates::WAIT_SG_HOME_DONE;
                }
        
                break;

            case Rots::RotsStates::WAIT_SG_HOME_DONE:
                
                // Serial.println("waiting for home");
                if(run_ptr_stepper->at_sg_stall())
                {
                    Serial.println("rot at stall");
                    run_ptr_stepper->set_velocity(0);
                    run_ptr_stepper->set_enable_stallgaurd(false);
                    run_ptr_stepper->zero_xactual();
                    *run_prt_state = Rots::RotsStates::STOPPED;
                }            
                break;

            case Rots::RotsStates::STOPPED:            

                act_on_button(run_ptr_stepper, run_prt_state);

                break;

            case Rots::RotsStates::MOVING_TO_RECIEVE:    
        
                if(run_ptr_stepper->at_position()){
                    *run_prt_state = Rots::RotsStates::AT_RECIEVE;
                }else{
                    act_on_button(run_ptr_stepper, run_prt_state);
                }
                
                break; 

            case Rots::RotsStates::AT_RECIEVE:    
        
                act_on_button(run_ptr_stepper, run_prt_state); 
                break; 

            case Rots::RotsStates::MOVING_TO_SQUISH:    
                if(run_ptr_stepper->at_position()){
                    *run_prt_state = Rots::RotsStates::AT_SQUISH;
                }else{
                    act_on_button(run_ptr_stepper, run_prt_state);
                }
                break; 

            case Rots::RotsStates::AT_SQUISH:    
        
                act_on_button(run_ptr_stepper, run_prt_state);
                break; 

            case Rots::RotsStates::MOVING_TO_CORE:    
        
                if(run_ptr_stepper->at_position()){
                    *run_prt_state = Rots::RotsStates::AT_CORE;
                }else{
                    act_on_button(run_ptr_stepper, run_prt_state);
                }
                break; 

            case Rots::RotsStates::AT_CORE:    
        
                act_on_button(run_ptr_stepper, run_prt_state);
                break; 

            case Rots::RotsStates::ERROR_MOTOR:

                break;
            
            case Rots::RotsStates::ESTOP:
            default:

                break;
        }
    }

    /*Here we map the rail states to states for the higher level controller.*/
    /*need to use a big if else statement, not sure how to do it otherwise with 2 variables
    */
     if(l_state == Rots::ERROR_MOTOR || r_state == Rots::ERROR_MOTOR){

            IntraComms[SubsystemList::ROTS_SUBS].set_ss_state(SubCommsClass::SubsystemStates::ERROR_MOTOR);

     }else if(l_state == Rots::SETUP || r_state == Rots::SETUP){

            IntraComms[SubsystemList::ROTS_SUBS].set_ss_state(SubCommsClass::SubsystemStates::SETUP);

    }else if ((l_state == Rots::MOVING_AWAY_FROM_HOME || r_state == Rots::MOVING_AWAY_FROM_HOME) || 
             (l_state == Rots::SET_SG || r_state == Rots::SET_SG) ||
             (l_state == Rots::WAIT_SG_HOME_DONE || r_state == Rots::WAIT_SG_HOME_DONE)){

                IntraComms[SubsystemList::ROTS_SUBS].set_ss_state(SubCommsClass::SubsystemStates::HOMING);

    }else if((l_state == Rots::STOPPED || r_state == Rots::STOPPED) || 
             (l_state == Rots::AT_RECIEVE || r_state == Rots::AT_RECIEVE) ||
             (l_state == Rots::AT_SQUISH || r_state == Rots::AT_SQUISH) ||
             (l_state == Rots::AT_CORE || r_state == Rots::AT_CORE)){

                IntraComms[SubsystemList::ROTS_SUBS].set_ss_state(SubCommsClass::SubsystemStates::WAITING_INPUT);

    }else if((l_state == Rots::STOPPED || r_state == Rots::MOVING_TO_RECIEVE) || 
             (l_state == Rots::AT_RECIEVE || r_state == Rots::MOVING_TO_SQUISH) ||
             (l_state == Rots::AT_CORE || r_state == Rots::MOVING_TO_CORE)){

                IntraComms[SubsystemList::ROTS_SUBS].set_ss_state(SubCommsClass::SubsystemStates::MOVING);

    }else if((l_state == Rots::STOPPED || r_state == Rots::MOVING_TO_RECIEVE) || 
             (l_state == Rots::AT_RECIEVE || r_state == Rots::MOVING_TO_SQUISH) ||
             (l_state == Rots::AT_CORE || r_state == Rots::MOVING_TO_CORE)){

                IntraComms[SubsystemList::ROTS_SUBS].set_ss_state(SubCommsClass::SubsystemStates::MOVING);
                
    }

    write_interfaces();
}

void RotsFSMClass::write_interfaces()
{

}


RotsFSMClass rotators;