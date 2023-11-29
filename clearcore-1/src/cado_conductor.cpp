 /**
 * @file cado_conductor.cpp
 * @copyright Copyright (c) 2023 Vebu Labs. All rights reserved.
 * @brief This library is the "conductor" that schedules the different movements
 * @author Mike Lui
*/

 /**
 * @file ui_cc1_subsystem.cpp
 * @copyright Copyright (c) 2023 Vebu Labs. All rights reserved.
 * @brief This library is used to control ui on clearcore 1
 * @author Mike Lui
*/

#include "cado_conductor.hpp"

void ConductorClass::setup()
{
    if(!has_setup){
        has_setup = true;
    }
}

void ConductorClass::read_interfaces()
{
    #ifdef SINGLE_BUTTON_AUTO_RUN
    run_input = CcIoManager.get_input(D6_CUT_BUTTON);
    unload_cutter_input = CcIoManager.get_input(D7_LOAD_CUT_BUTTON);
    cutter_state = CcIoManager.IntraComms[SubsystemList::CUTTER_SUBS].get_ss_state();
    last_cutter_state = CcIoManager.IntraComms[SubsystemList::CUTTER_SUBS].get_ss_last_transition();
    #endif
}

void ConductorClass::run()
{
    read_interfaces();

    switch (state)
    {
        case Cond::SETUP:
            /* wait for clamps to be ready closed (waiting for home input) */
            Serial.println("Conductor: Setup");
            Serial.println("Conductor: Closing clamps");
            state = Cond::CLOSING_CLAMPS;
            break;

        case Cond::CLOSING_CLAMPS:
            /* Clamps close on start up to get out of the way */

            if(CcIoManager.IntraComms[SubsystemList::CLAMPS_SUBS].get_ss_state() == SubCommsClass::WAITING_HOME_CMD && 
            CcIoManager.IntraComms[SubsystemList::ROTS_SUBS].get_ss_state() == SubCommsClass::WAITING_HOME_CMD){
                Serial.println("Conductor: Clamps closed");
                Serial.println("Conductor: Homing Rotators");
                CcIoManager.IntraComms[SubsystemList::ROTS_SUBS].set_ss_command(SubCommsClass::HOME_COMMAND);
                state = Cond::HOMING_ROTATORS;
            }
            break;

        case Cond::HOMING_ROTATORS:
            /* OK to homing rotators now */
            if(CcIoManager.IntraComms[SubsystemList::ROTS_SUBS].get_ss_state() == SubCommsClass::SubsystemStates::WAITING_RDY_CMD){
                Serial.println("Conductor: Rotators homed, moved to catch posittion");
                Serial.println("Conductor: Rotators homed, home clamps");
                CcIoManager.IntraComms[SubsystemList::CLAMPS_SUBS].set_ss_command(SubCommsClass::HOME_COMMAND);
                state = Cond::HOMING_CLAMPS;
            }
            break;


        case Cond::HOMING_CLAMPS:
            /* OK to home clamps now */
            if(CcIoManager.IntraComms[SubsystemList::CLAMPS_SUBS].get_ss_state() == SubCommsClass::SubsystemStates::WAITING_INPUT){
                Serial.println("Conductor: clamps homed, system ready to run");
                CcIoManager.IntraComms[SubsystemList::ROTS_SUBS].set_ss_command(SubCommsClass::RDY_COMMAND);
                CcIoManager.IntraComms[SubsystemList::CUTTER_SUBS].set_ss_command(SubCommsClass::RDY_COMMAND);
                state = Cond::RUNNING;
            }
            break;

        case Cond::RUNNING:
            /* rest of machine ready to run */
            if(unload_cutter_input){
                CcIoManager.IntraComms[SubsystemList::CUTTER_SUBS].set_ss_command(CUTTER_LOAD_CMD);
                state = Cond::UNLOAD_CUTTER_TO_FLAG;
            }
            break;

        case Cond::UNLOAD_CUTTER_TO_FLAG:

            if(cutter_state == SubCommsClass::SubsystemStates::WAITING_INPUT && 
            last_cutter_state == SubCommsClass::SubsystemStates::MOVING){

                CcIoManager.IntraComms[SubsystemList::CUTTER_SUBS].set_ss_command(CUTTER_CUT_CMD);
                state = Cond::UNLOAD_CUTTER_TO_RELEASE;
            }
            break;

        case Cond::UNLOAD_CUTTER_TO_RELEASE:

            if(cutter_state == SubCommsClass::SubsystemStates::WAITING_INPUT && 
            last_cutter_state == SubCommsClass::SubsystemStates::MOVING){
                
                state = Cond::RUNNING;
            }
            break;
        
        default:
            break;
    }

    write_interfaces();
}

void ConductorClass::write_interfaces()
{

}

ConductorClass conductor;