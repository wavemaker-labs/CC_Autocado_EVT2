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

int32_t release_rpm_to_ppt(float flo_val)
{
    return (int32_t)(flo_val*RELEASE_MOTOR_GEAR_RATIO*RELEASE_US_PER_REV)/(CLOCK_RATIO*SECS_PER_MIN);
}

int32_t release_angle_to_pulses(float flo_val)
{
    return (int32_t)((197294.7467757156)*log(((flo_val)/220.0)+(9.0/11.0)) + 39591);    //Release exponential formula
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
    release_switch_input = (conductor_cmd == RELEASE_AVO_CMD);
}

void ReleaseFSMClass::run()
{

    read_interfaces();

    Cc5160Stepper * run_ptr_stepper;
    Release::ReleaseStates * state;

    release_1st_vel = release_rpm_to_ppt(MOVE_1_RELEASE_RPM);
    release_2nd_vel = release_rpm_to_ppt(MOVE_2_RELEASE_RPM);

    trap_close_pos =release_angle_to_pulses(TRAP_CLOSE_ANGLE);
    trap_open_pos_1 =release_angle_to_pulses(TRAP_OPEN_ANGLE_1);
    trap_open_pos_2 =release_angle_to_pulses(TRAP_OPEN_ANGLE_2);

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

            run_ptr_stepper->set_target_position(trap_close_pos, release_2nd_vel);

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
                run_ptr_stepper->set_target_position(trap_open_pos_1,release_1st_vel);
                *state = Release::ReleaseStates::RELEASING_1;
            }

            break;

        case Release::ReleaseStates::RELEASING_1:

            if(run_ptr_stepper->at_position())
            {
                Serial.println("Released, moving to close position");

                run_ptr_stepper->set_target_position(trap_open_pos_1, release_1st_vel);
                delay(500);
                *state = Release::ReleaseStates::RELEASING_2;
            }
   
            break; 

        case Release::ReleaseStates::RELEASING_2:

            if(run_ptr_stepper->at_position())
            {
                Serial.println("Released, moving to close position");
                run_ptr_stepper->set_target_position(trap_open_pos_2, release_2nd_vel);

                *state = Release::ReleaseStates::RELEASED;
            }
   
            break;      

        case Release::ReleaseStates::RELEASED:

            if(run_ptr_stepper->at_position())
            {
                run_ptr_stepper->set_target_position(trap_close_pos, release_2nd_vel);
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