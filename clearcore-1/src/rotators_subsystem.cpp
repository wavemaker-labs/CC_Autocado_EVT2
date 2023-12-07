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
        CcIoManager.IntraComms[SubsystemList::ROTS_SUBS].set_ss_state(SubCommsClass::SubsystemStates::SETUP);
        ptr_5160_rot_l_stepper = CcIoManager.get_step_ptr(AutocadoCcSteppers::STEPPER_ROT_L);
        ptr_5160_rot_r_stepper = CcIoManager.get_step_ptr(AutocadoCcSteppers::STEPPER_ROT_R);
    }
}

void RotsFSMClass::read_interfaces()
{
    SubCommsClass::SubsystemCommands conductor_cmd;

    conductor_cmd = CcIoManager.IntraComms[SubsystemList::ROTS_SUBS].get_ss_cmd();

    home_input = (conductor_cmd == SubCommsClass::SubsystemCommands::HOME_COMMAND);
    ready_input = (conductor_cmd == SubCommsClass::SubsystemCommands::RDY_COMMAND);

    rots_home_vmax = CcIoManager.get_mb_data(MbRegisterOffsets::ROTATOR_HOMING_VEL);
    rots_move_vmax = CcIoManager.get_mb_data(MbRegisterOffsets::ROTATOR_MOVE_VEL);
    receive_position = CcIoManager.get_mb_data(MbRegisterOffsets::ROTATOR_RECEIVE_POS);
    presquish_position = CcIoManager.get_mb_data(MbRegisterOffsets::ROTATOR_PRESQUISH_POS);
    squish_position = CcIoManager.get_mb_data(MbRegisterOffsets::ROTATOR_SQUISH_POS);

    #ifndef SINGLE_BUTTON_AUTO_RUN //use buttons, else use commands from intracomms
    switch_0_input = CcIoManager.get_input(D0_RAIL_SW_0);
    switch_1_input = CcIoManager.get_input(D1_RAIL_SW_1);

    if(conductor_cmd == SubCommsClass::SubsystemCommands::NO_COMMAND){
        if(switch_0_input == PinStatus::LOW && switch_1_input == PinStatus::HIGH){ 
            cmd_position = Rots::RotsPositions::RECEIVE_POS;
        }else if(switch_0_input == PinStatus::LOW && switch_1_input == PinStatus::LOW){ 
            cmd_position = Rots::RotsPositions::PRESQUISH_POS;
        }else if(switch_0_input == PinStatus::HIGH && switch_1_input == PinStatus::LOW){
            cmd_position = Rots::RotsPositions::SQUISH_POS;
        }
    }else{
        switch (conductor_cmd)
        {
        case SubCommsClass::SubsystemCommands::COMMAND_1:
            cmd_position = Rots::RotsPositions::RECEIVE_POS;
            break;
        
        default:
            break;
        }
    }
    #else
    switch (conductor_cmd)
    {
        case ROT_RECIEVE_CMD:
            cmd_position = Rots::RotsPositions::RECEIVE_POS;
            break;

        case ROT_PRESQUISH_CMD:
            cmd_position = Rots::RotsPositions::PRESQUISH_POS;
            break;

        case ROT_SQUISH_CMD:
            cmd_position = Rots::RotsPositions::SQUISH_POS;
            break;
        
        default:
            break;
    }
    #endif

}

void RotsFSMClass::act_on_button(Cc5160Stepper * ptr_stepper, Rots::RotsStates * ptr_state)
{
    if(cmd_position == Rots::RotsPositions::RECEIVE_POS &&
        *ptr_state != Rots::RotsStates::AT_RECIEVE && 
        *ptr_state != Rots::RotsStates::MOVING_TO_RECIEVE){

            ptr_stepper->set_target_position(receive_position, rots_home_vmax);
            *ptr_state = Rots::RotsStates::MOVING_TO_RECIEVE;

    }else if (cmd_position == Rots::RotsPositions::SQUISH_POS &&
        *ptr_state != Rots::RotsStates::AT_SQUISH && 
        *ptr_state != Rots::RotsStates::MOVING_TO_SQUISH){

            ptr_stepper->set_target_position(squish_position, rots_home_vmax);
            *ptr_state = Rots::RotsStates::MOVING_TO_SQUISH;

    }else if (cmd_position == Rots::RotsPositions::PRESQUISH_POS &&
        *ptr_state != Rots::RotsStates::AT_PRESQUISH && 
        *ptr_state != Rots::RotsStates::MOVING_TO_PRESQUISH){

            ptr_stepper->set_target_position(presquish_position, rots_home_vmax);
            *ptr_state = Rots::RotsStates::MOVING_TO_PRESQUISH;
    }
}

void RotsFSMClass::determine_comm_state(){
    /*Here we map the subsystem specific states to generalized states for the higher level controller.*/
    /*need to use a big if else statement, not sure how to do it otherwise with 2 variables
    */
     if(l_state == Rots::ERROR_MOTOR || r_state == Rots::ERROR_MOTOR){

            CcIoManager.IntraComms[SubsystemList::ROTS_SUBS].set_ss_state(SubCommsClass::SubsystemStates::ERROR_MOTOR);

     }else if(l_state == Rots::SETUP || r_state == Rots::SETUP){

            CcIoManager.IntraComms[SubsystemList::ROTS_SUBS].set_ss_state(SubCommsClass::SubsystemStates::SETUP);

    }else if(l_state == Rots::WAIT_FOR_HOME_CMD && r_state == Rots::WAIT_FOR_HOME_CMD){

            CcIoManager.IntraComms[SubsystemList::ROTS_SUBS].set_ss_state(SubCommsClass::SubsystemStates::WAITING_HOME_CMD);

    }else if ((l_state == Rots::MOVING_AWAY_FROM_HOME || r_state == Rots::MOVING_AWAY_FROM_HOME) || 
             (l_state == Rots::SET_SG || r_state == Rots::SET_SG) ||
             (l_state == Rots::WAIT_SG_HOME_DONE || r_state == Rots::WAIT_SG_HOME_DONE)){

                CcIoManager.IntraComms[SubsystemList::ROTS_SUBS].set_ss_state(SubCommsClass::SubsystemStates::HOMING);

    }else if ((l_state == Rots::WAIT_FOR_READY_CMD && r_state == Rots::WAIT_FOR_READY_CMD)){

                CcIoManager.IntraComms[SubsystemList::ROTS_SUBS].set_ss_state(SubCommsClass::SubsystemStates::WAITING_RDY_CMD);

    }else if((l_state == Rots::STOPPED && r_state == Rots::STOPPED) || 
             (l_state == Rots::AT_RECIEVE && r_state == Rots::AT_RECIEVE) ||
             (l_state == Rots::AT_SQUISH && r_state == Rots::AT_SQUISH) ||
             (l_state == Rots::AT_PRESQUISH && r_state == Rots::AT_PRESQUISH)){

                CcIoManager.IntraComms[SubsystemList::ROTS_SUBS].set_ss_state(SubCommsClass::SubsystemStates::WAITING_INPUT);

    }else if((l_state == Rots::MOVING_TO_RECIEVE || r_state == Rots::MOVING_TO_RECIEVE) || 
             (l_state == Rots::MOVING_TO_SQUISH || r_state == Rots::MOVING_TO_SQUISH) ||
             (l_state == Rots::MOVING_TO_PRESQUISH || r_state == Rots::MOVING_TO_PRESQUISH)){

                CcIoManager.IntraComms[SubsystemList::ROTS_SUBS].set_ss_state(SubCommsClass::SubsystemStates::MOVING);

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
                    *run_prt_state = Rots::RotsStates::WAIT_FOR_HOME_CMD;
                }else
                {
                    Serial.println("Rot Config setting up");
                    Serial.println(run_ptr_stepper->step_5160_motor_cfg.configIndex);
                }
                break;

            case Rots::RotsStates::WAIT_FOR_HOME_CMD:

                if(home_input){
                    Serial.println(run_ptr_stepper->get_old_x());
                    run_ptr_stepper->set_target_position(run_ptr_stepper->get_old_x() + ROTS_STEPS_AWAY_HOME, rots_home_vmax);
                    *run_prt_state = Rots::RotsStates::MOVING_AWAY_FROM_HOME;
                }
                break;

            case Rots::RotsStates::MOVING_AWAY_FROM_HOME:
                
                if(run_ptr_stepper->at_position())
                {
                    run_ptr_stepper->set_velocity(rots_home_vmax);
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
                    *run_prt_state = Rots::RotsStates::FINISH_HOME_AT_RECIEVE;
                }            
                break;

            case Rots::RotsStates::FINISH_HOME_AT_RECIEVE:            
                run_ptr_stepper->set_target_position(receive_position, rots_move_vmax);
                if(run_ptr_stepper->at_position())
                {
                    *run_prt_state = Rots::RotsStates::WAIT_FOR_READY_CMD;
                }

                break;

            case Rots::RotsStates::WAIT_FOR_READY_CMD:            
                if(ready_input){
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

            case Rots::RotsStates::MOVING_TO_PRESQUISH:    
        
                if(run_ptr_stepper->at_position()){
                    *run_prt_state = Rots::RotsStates::AT_PRESQUISH;
                }else{
                    act_on_button(run_ptr_stepper, run_prt_state);
                }
                break; 

            case Rots::RotsStates::AT_PRESQUISH:    
        
                act_on_button(run_ptr_stepper, run_prt_state);
                break; 

            case Rots::RotsStates::ERROR_MOTOR:

                break;
            
            case Rots::RotsStates::ESTOP:
            default:

                break;
        }
    }

    determine_comm_state();
    write_interfaces();
}

void RotsFSMClass::write_interfaces()
{
    CcIoManager.set_mb_data(MbRegisterOffsets::LEFT_ROTATOR_STATE, l_state);
    CcIoManager.set_mb_data(MbRegisterOffsets::RIGHT_ROTATOR_STATE, r_state);
}


RotsFSMClass rotators;