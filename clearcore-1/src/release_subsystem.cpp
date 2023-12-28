 /**
 * @file drum_subsystem.cpp
 * @copyright Copyright (c) 2023 Vebu Labs. All rights reserved.
 * @brief This library is used to control the release subsystem
 * @author Mike Lui & Elvis Palma
*/

#include "release_subsystem.hpp"


bool new_release_motor_mb_cmd = false;

uint16_t release_motor_hreg_write(TRegister* reg, uint16_t val) {
    new_release_motor_mb_cmd = true;
    return val;
}


void ReleaseFSMClass::setup()
{
    if(!has_setup){
        has_setup = true;        

        f_state = Release::ReleaseStates::SETUP;
        b_state = Release::ReleaseStates::SETUP;
        ptr_5160_releaseRight_stepper = CcIoManager.get_step_ptr(AutocadoCcSteppers::STEPPER_RELEASE_R);
        ptr_5160_releaseLeft_stepper = CcIoManager.get_step_ptr(AutocadoCcSteppers::STEPPER_RELEASE_L);
    }
}

void ReleaseFSMClass::read_interfaces()
{
    SubCommsClass::SubsystemCommands conductor_cmd;

    conductor_cmd = CcIoManager.IntraComms[SubsystemList::RELEASE_SUBS].get_ss_cmd();
    
    
    ready_input = (conductor_cmd == SubCommsClass::SubsystemCommands::RDY_COMMAND); 

    #ifndef SINGLE_BUTTON_AUTO_RUN //use buttons else use commands from intracomms
    orient_switch_input = CcIoManager.get_input(A9_ORIENT_BUTTON);
    release_switch_input = CcIoManager.get_input(D8_RELEASE_BUTTON);
    #else
    release_switch_input = (conductor_cmd == RELEASE_AVO_CMD);
    #endif

    // if(conductor_cmd == SubCommsClass::SubsystemCommands::NO_COMMAND){
    //     if(release_switch_input == PinStatus::LOW && orient_switch_input == PinStatus::HIGH){ 
    //         Serial.println("release_switch_input low, orient_switch_input high");
    //         cmd_position = Release::ReleasePositions::ORIENT_POS;
    //     }else if(release_switch_input == PinStatus::HIGH && orient_switch_input == PinStatus::LOW){ 
    //         cmd_position = Release::ReleasePositions::RELEASE_POS;
    //     }
    // }else{
    //     switch (conductor_cmd)
    //     {
    //     case SubCommsClass::SubsystemCommands::COMMAND_1:
    //     Serial.println("COMMAND_1");
    //         cmd_position = Release::ReleasePositions::ORIENT_POS;
    //         break;
    //     case SubCommsClass::SubsystemCommands::COMMAND_2:
    //     Serial.println("COMMAND_2");
    //         cmd_position = Release::ReleasePositions::RELEASE_POS;
    //         break;
    //     default:
    //         break;
    //     }
    // }
}

void ReleaseFSMClass::run()
{

read_interfaces();

Cc5160Stepper * run_ptr_stepper;
Release::ReleaseStates * state;

    float trap_up_atp = (197294.7467757156)*log(((TRAP_UP_ANGLE)/220.0)+(9.0/11.0)) + 39591;           //converting up angle to pulses
    int32_t TRAP_UP_POS = (int32_t)trap_up_atp;                                                     //converting up pulses to int

    float trap_2_up_atp = (197294.7467757156)*log((( TRAP_2_UP_ANGLE)/220.0)+(9.0/11.0)) + 39591;           //converting up angle to pulses
    int32_t TRAP_2_UP_POS = (int32_t)trap_2_up_atp;                                                     //converting up pulses to int

    float trap_close_atp = (197294.7467757156)*log((( TRAP_CLOSE_ANGLE)/220.0)+(9.0/11.0)) + 39591;     //converting close angle to pulses
    int32_t TRAP_CLOSE_POS = (int32_t)trap_close_atp;                                               //converting close pulses to int

    float trap_1_open_atp = (197294.7467757156)*log((( TRAP_1_OPEN_ANGLE)/220.0)+(9.0/11.0)) + 39591;       //converting open angle to pulses
    int32_t TRAP_1_OPEN_POS = (int32_t)trap_1_open_atp;          
    
    float trap_2_open_atp = (197294.7467757156)*log((( TRAP_2_OPEN_ANGLE)/220.0)+(9.0/11.0)) + 39591;     //converting close angle to pulses
    int32_t TRAP_2_OPEN_POS = (int32_t)trap_2_open_atp;                                       //converting open pulses to int
    
    float orient_speed1_ptr = (13.7335640138408304*51200.0*MOVE_1_ORIENT_RPM)/(0.7152557373046875*60) + 39591;                                 //converting rpm to pulses per seconds
    uint32_t first_orient_vel = (uint32_t)orient_speed1_ptr;                                                 //converting pulses per seconds to int
    
    float orient_speed2_ptr = (13.7335640138408304*51200.0*MOVE_2_ORIENT_RPM)/(0.7152557373046875*60) + 39591;                                 //converting rpm to pulses per seconds
    uint32_t second_orient_vel = (uint32_t)orient_speed2_ptr;                                                 //converting pulses per seconds to int

    float release_1_speed_ptr = (13.7335640138408304*51200.0*MOVE_1_RELEASE_RPM)/(0.7152557373046875*60) + 39591;                                 //converting rpm to pulses per seconds
    uint32_t release_1_vel = (uint32_t)release_1_speed_ptr;  

    float release_2_speed_ptr = (13.7335640138408304*51200.0*MOVE_2_RELEASE_RPM)/(0.7152557373046875*60) + 39591;                                 //converting rpm to pulses per seconds
    uint32_t release_2_vel = (uint32_t)release_2_speed_ptr;  

for(int i = 0; i < 2; i ++){   

    switch(i){
        case 0:
            run_ptr_stepper = ptr_5160_releaseRight_stepper;
            state = &f_state;
            break;
        case 1:
            run_ptr_stepper = ptr_5160_releaseLeft_stepper;
            state = &b_state;
            break;
    }

    switch (*state)
    {
        case Release::ReleaseStates::SETUP:
            if(run_ptr_stepper->config_ready())
            {
                Serial.println("Release Config ready");
                Serial.println("Attempting moving away from home");
                Serial.println(run_ptr_stepper->get_old_x());
                run_ptr_stepper->set_target_position(run_ptr_stepper->get_old_x() + STEPS_AWAY_HOME, HOME_VMAX);
                *state = Release::ReleaseStates::OFFSETING;

            }else
            {
                Serial.println("Release Config being set up");
                Serial.println(run_ptr_stepper->step_5160_motor_cfg.configIndex);
            }

            break;
        
        case Release::ReleaseStates::OFFSETING:

            if(run_ptr_stepper->at_position())
                {
                    Serial.println("Release at offset position");
                    run_ptr_stepper->set_velocity(HOME_VMAX);
                    *state = Release::ReleaseStates::SET_SG;
                }

            break;

        case Release::ReleaseStates::SET_SG: 

            if(run_ptr_stepper->at_vmax())
                {
                    Serial.println("Homing");
                    run_ptr_stepper->set_enable_stallgaurd(true);
                    *state = Release::ReleaseStates::HOMING;
                }

            break;

        case Release::ReleaseStates::HOMING: 

            if(run_ptr_stepper->at_sg_stall())
            {
                Serial.println("Home reached, going to close position");

                run_ptr_stepper->set_velocity(0);
                run_ptr_stepper->set_enable_stallgaurd(false);
                run_ptr_stepper->zero_xactual();

                *state = Release::ReleaseStates::HOMED;
            }
                       
            break;

        case Release::ReleaseStates::HOMED:

            run_ptr_stepper->set_target_position(TRAP_CLOSE_POS, release_2_vel);

            if (run_ptr_stepper->at_position())
            {
                Serial.println("Release waiting ready cmd");
                CcIoManager.IntraComms[SubsystemList::RELEASE_SUBS].set_ss_state(SubCommsClass::SubsystemStates::WAITING_INPUT);
                *state = Release::ReleaseStates::WAITING_READY_CMD;
            }

            break;

        case Release::ReleaseStates::WAITING_READY_CMD:            
                        
            if (release_switch_input)
            {
                Serial.println("Releasing");
                run_ptr_stepper->set_target_position(TRAP_1_OPEN_POS,release_1_vel);
                *state = Release::ReleaseStates::RELEASING_1;
            }

            break;

        case Release::ReleaseStates::RELEASING_1:

            if(run_ptr_stepper->at_position())
            {
                Serial.println("Released, moving to close position");

                run_ptr_stepper->set_target_position(TRAP_1_OPEN_POS, release_1_vel);
                delay(500);
                *state = Release::ReleaseStates::RELEASING_2;
            }
   
            break; 

        case Release::ReleaseStates::RELEASING_2:

            if(run_ptr_stepper->at_position())
            {
                Serial.println("Released, moving to close position");
                run_ptr_stepper->set_target_position(TRAP_2_OPEN_POS, release_2_vel);

                *state = Release::ReleaseStates::RELEASED;
            }
   
            break;      

        case Release::ReleaseStates::RELEASED:

            if(run_ptr_stepper->at_position())
            {
                run_ptr_stepper->set_target_position(TRAP_CLOSE_POS, release_2_vel);
                //Serial.println("At up position");
                delay(250);
                *state = Release::ReleaseStates::WAITING_READY_CMD;
            }
   
            break;  

        case Release::ReleaseStates::ESTOP:

            break;
        
        case Release::ReleaseStates::ERROR_MOTOR:
        default:

            break;
    }
    
}
    
    if(f_state == Release::ERROR_MOTOR || 
        b_state == Release::ERROR_MOTOR){

            CcIoManager.IntraComms[SubsystemList::RELEASE_SUBS].set_ss_state(SubCommsClass::SubsystemStates::ERROR_MOTOR);

    }else if(f_state == Release::SETUP || 
            b_state == Release::SETUP){

            CcIoManager.IntraComms[SubsystemList::RELEASE_SUBS].set_ss_state(SubCommsClass::SubsystemStates::SETUP);

    }else if(f_state == Release::WAITING_HOME_CMD && 
            b_state == Release::WAITING_HOME_CMD){

            CcIoManager.IntraComms[SubsystemList::RELEASE_SUBS].set_ss_state(SubCommsClass::SubsystemStates::WAITING_HOME_CMD);

    }else if((f_state == Release::WAITING_READY_CMD) &&
                (f_state == b_state)) {

            CcIoManager.IntraComms[SubsystemList::RELEASE_SUBS].set_ss_state(SubCommsClass::SubsystemStates::WAITING_INPUT);
    }else {
        CcIoManager.IntraComms[SubsystemList::RELEASE_SUBS].set_ss_state(SubCommsClass::SubsystemStates::MOVING);
    }
    
    write_interfaces();
}

void ReleaseFSMClass::write_interfaces()
{

}

ReleaseFSMClass release;