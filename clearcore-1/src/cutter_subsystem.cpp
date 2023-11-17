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
        ptr_5160_cut_stepper = CcIoManager.get_step_ptr(AutocadoCcSteppers::STEPPER_CUTTER);
    }
}

void CutterFSMClass::read_interfaces()
{
    SubCommsClass::SubsystemCommands conductor_cmd;

    conductor_cmd = IntraComms[SubsystemList::CUTTER_SUBS].get_ss_cmd();

    cut_switch_input = CcIoManager.get_input(D6_CUT_BUTTON);
    load_switch_input = CcIoManager.get_input(D7_LOAD_CUT_BUTTON);
    ready_input = (conductor_cmd == SubCommsClass::SubsystemCommands::RDY_COMMAND);
}

void CutterFSMClass::determine_comm_state(){
    /*Here we map the subsystem specific states to generalized states for the higher level controller.*/
    switch (state)
    {
        case Cutter::CutterStates::SETUP:
            IntraComms[SubsystemList::CUTTER_SUBS].set_ss_state(SubCommsClass::SubsystemStates::SETUP);
            break;
        case Cutter::CutterStates::STOPPED:
        case Cutter::CutterStates::RELEASED:
        case Cutter::CutterStates::WOUND:
            IntraComms[SubsystemList::CUTTER_SUBS].set_ss_state(SubCommsClass::SubsystemStates::WAITING_INPUT);
            break;

        case Cutter::CutterStates::WINDING:
        case Cutter::CutterStates::RELEASING:
            IntraComms[SubsystemList::CUTTER_SUBS].set_ss_state(SubCommsClass::SubsystemStates::MOVING);
            break;      

        case Cutter::CutterStates::ERROR_MOTOR:
            IntraComms[SubsystemList::CUTTER_SUBS].set_ss_state(SubCommsClass::SubsystemStates::ERROR_MOTOR);
            break;
        
        case Cutter::CutterStates::ESTOP:
        default:
            IntraComms[SubsystemList::CUTTER_SUBS].set_ss_state(SubCommsClass::SubsystemStates::ESTOP);
            break;
    }
}

void CutterFSMClass::run()
{
    read_interfaces();

    switch (state)
    {
        case Cutter::CutterStates::SETUP:
            if(ptr_5160_cut_stepper->config_ready())
            {
                state = Cutter::CutterStates::WAIT_FOR_READY_CMD;
            }else
            {
                Serial.println(ptr_5160_cut_stepper->step_5160_motor_cfg.configIndex);
            }

            break;

        case Cutter::CutterStates::WAIT_FOR_READY_CMD:
            if(ready_input){
                state = Cutter::CutterStates::STOPPED;
            }
            break;

        case Cutter::CutterStates::STOPPED:
        case Cutter::CutterStates::RELEASED:
            if(load_switch_input == PinStatus::HIGH){                
                ptr_5160_cut_stepper->set_enable_right_sw(true);
                ptr_5160_cut_stepper->set_target_position(CUTTER_LOAD_TICKS, CUTTER_VELOCITY);
                state = Cutter::CutterStates::WINDING;
            }            
            break;

        case Cutter::CutterStates::WINDING:
            Serial.println("winding");
            Serial.println(ptr_5160_cut_stepper->at_r_sw());
            Serial.println(ptr_5160_cut_stepper->at_stop());
            if(ptr_5160_cut_stepper->at_r_sw() && ptr_5160_cut_stepper->at_stop()){
                ptr_5160_cut_stepper->set_target_position(0, 1);
                ptr_5160_cut_stepper->zero_xactual();
                ptr_5160_cut_stepper->set_enable_right_sw(false);
                state = Cutter::CutterStates::WOUND;
            }  
            break;

        case Cutter::CutterStates::WOUND:
            if(cut_switch_input == PinStatus::HIGH)
            { 
                ptr_5160_cut_stepper->set_target_position(CUTTER_CUT_TICKS, CUTTER_VELOCITY);
                state = Cutter::CutterStates::RELEASING;
            }
            break;

        case Cutter::CutterStates::RELEASING:
            if(ptr_5160_cut_stepper->at_stop()){
                state = Cutter::CutterStates::STOPPED;
            }  
   
            break;      

        case Cutter::CutterStates::ERROR_MOTOR:

            break;
        
        case Cutter::CutterStates::ESTOP:
        default:

            break;
    }

    determine_comm_state();

    write_interfaces();
}

void CutterFSMClass::write_interfaces()
{
    CcIoManager.set_pin_output_state(AutocadoCcPins::D4_CUT_SOLENOID, relay_output);
}

CutterFSMClass cutter;