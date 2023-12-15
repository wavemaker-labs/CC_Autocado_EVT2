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

        CcIoManager.set_mb_w_hreg_cb(MbRegisterOffsets::CUTTER_VEL, &cutter_motor_hreg_write);
        CcIoManager.set_mb_w_hreg_cb(MbRegisterOffsets::LOADING_REV, &cutter_motor_hreg_write);
        CcIoManager.set_mb_w_hreg_cb(MbRegisterOffsets::CUT_REV, &cutter_motor_hreg_write);

        float flo_val;

        flo_val = (4.25*30*51200.0*CUTTER_VELOCITY/(0.7152557373046875*100))/60;
        cutter_velocity = (int32_t)flo_val; 
        
        flo_val = (CUTTER_LOAD_REV*4.25*30*51200.0)/100;
        cutter_load_rev = (int32_t)flo_val;  
        
        flo_val = (CUTTER_CUT_REV*4.25*30*51200.0)/100;
        cutter_cut_rev = (int32_t)flo_val; 

        CcIoManager.set_mb_holding_data(MbRegisterOffsets::CUTTER_VEL, CUTTER_VELOCITY);  //test, not working
    }
}

void CutterFSMClass::read_interfaces()
{
    SubCommsClass::SubsystemCommands conductor_cmd;

    conductor_cmd = CcIoManager.IntraComms[SubsystemList::CUTTER_SUBS].get_ss_cmd();

    ready_input = (conductor_cmd == SubCommsClass::SubsystemCommands::RDY_COMMAND);
    
    if (new_cutter_motor_mb_cmd){
        //Serial.println(cutter_velocity);
        cutter_velocity = CcIoManager.get_mb_data(MbRegisterOffsets::CUTTER_VEL);
        cutter_load_rev = CcIoManager.get_mb_data(MbRegisterOffsets::LOADING_REV);
        cutter_cut_rev = CcIoManager.get_mb_data(MbRegisterOffsets::CUT_REV);

        float flo_val;

        flo_val = (4.25*30*51200.0*cutter_velocity/(0.7152557373046875*100))/60;
        cutter_velocity = (int32_t)flo_val; 
        
        flo_val = (cutter_load_rev*4.25*30*51200.0)/100;
        cutter_load_rev = (int32_t)flo_val;  
        
        flo_val = (cutter_cut_rev*4.25*30*51200.0)/100;
        cutter_cut_rev = (int32_t)flo_val; 
    }

    #ifndef SINGLE_BUTTON_AUTO_RUN //use buttons else use commands from intracomms
    cut_switch_input = CcIoManager.get_input(D6_CUT_BUTTON);
    load_switch_input = CcIoManager.get_input(D7_LOAD_CUT_BUTTON);
    #else
    cut_switch_input = (conductor_cmd == CUTTER_CUT_CMD);
    load_switch_input = (conductor_cmd == CUTTER_LOAD_CMD);
    #endif
}

void CutterFSMClass::determine_comm_state(){
    /*Here we map the subsystem specific states to generalized states for the higher level controller.*/
    switch (state)
    {
        case Cutter::CutterStates::SETUP:
            CcIoManager.IntraComms[SubsystemList::CUTTER_SUBS].set_ss_state(SubCommsClass::SubsystemStates::SETUP);
            break;
        case Cutter::CutterStates::STOPPED:
        case Cutter::CutterStates::RELEASED:
        case Cutter::CutterStates::WOUND:
            CcIoManager.IntraComms[SubsystemList::CUTTER_SUBS].set_ss_state(SubCommsClass::SubsystemStates::WAITING_INPUT);
            break;

        case Cutter::CutterStates::WINDING:
        case Cutter::CutterStates::RELEASING:
            CcIoManager.IntraComms[SubsystemList::CUTTER_SUBS].set_ss_state(SubCommsClass::SubsystemStates::MOVING);
            break;      

        case Cutter::CutterStates::ERROR_MOTOR:
            CcIoManager.IntraComms[SubsystemList::CUTTER_SUBS].set_ss_state(SubCommsClass::SubsystemStates::ERROR_MOTOR);
            break;
        
        case Cutter::CutterStates::ESTOP:
        default:
            CcIoManager.IntraComms[SubsystemList::CUTTER_SUBS].set_ss_state(SubCommsClass::SubsystemStates::ESTOP);
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
                ptr_5160_cut_stepper->set_target_position(cutter_load_rev, cutter_velocity);
                state = Cutter::CutterStates::WINDING;
            }            
            break;

        case Cutter::CutterStates::WINDING:
            // Serial.println("winding");
            // Serial.println(ptr_5160_cut_stepper->at_r_sw());
            // Serial.println(ptr_5160_cut_stepper->at_stop());
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
                ptr_5160_cut_stepper->set_target_position(cutter_cut_rev, cutter_velocity);
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
    
    CcIoManager.set_mb_data(MbRegisterOffsets::CUTTER_STATE, state); 
}

CutterFSMClass cutter;