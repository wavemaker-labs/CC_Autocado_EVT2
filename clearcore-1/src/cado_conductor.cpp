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

    drum_state = CcIoManager.IntraComms[SubsystemList::DRUM_SUBS].get_ss_state();
    last_drum_state = CcIoManager.IntraComms[SubsystemList::DRUM_SUBS].get_ss_last_transition();

    release_state = CcIoManager.IntraComms[SubsystemList::RELEASE_SUBS].get_ss_state();
    last_release_state = CcIoManager.IntraComms[SubsystemList::RELEASE_SUBS].get_ss_last_transition();

    cutter_state = CcIoManager.IntraComms[SubsystemList::CUTTER_SUBS].get_ss_state();
    last_cutter_state = CcIoManager.IntraComms[SubsystemList::CUTTER_SUBS].get_ss_last_transition();

    clamps_state = CcIoManager.IntraComms[SubsystemList::CLAMPS_SUBS].get_ss_state();
    last_clamps_state = CcIoManager.IntraComms[SubsystemList::CLAMPS_SUBS].get_ss_last_transition();

    rotators_state = CcIoManager.IntraComms[SubsystemList::ROTS_SUBS].get_ss_state();
    last_rotators_state = CcIoManager.IntraComms[SubsystemList::ROTS_SUBS].get_ss_last_transition();
    #endif
}

bool finished_move(SubCommsClass::SubsystemStates current, SubCommsClass::SubsystemStates last){
    return (current == SubCommsClass::SubsystemStates::WAITING_INPUT && last == SubCommsClass::SubsystemStates::MOVING);

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
            state = Cond::HOMING_TRAP_DOORS;
            break;

        case Cond::HOMING_TRAP_DOORS:
            
            if(CcIoManager.IntraComms[SubsystemList::RELEASE_SUBS].get_ss_state() == SubCommsClass::SubsystemStates::WAITING_INPUT){
                Serial.println("Conductor: Trap doors homed. Closing clamps");
                CcIoManager.IntraComms[SubsystemList::DRUM_SUBS].set_ss_command(SubCommsClass::RDY_COMMAND);
                CcIoManager.IntraComms[SubsystemList::RELEASE_SUBS].set_ss_command(SubCommsClass::RDY_COMMAND);
                state = Cond::CLOSING_CLAMPS;
            }
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
                Serial.println("Conductor: Rotators homed, home clamps");
                CcIoManager.IntraComms[SubsystemList::CLAMPS_SUBS].set_ss_command(SubCommsClass::HOME_COMMAND);
                state = Cond::HOMING_CLAMPS;
            }
            break;


        case Cond::HOMING_CLAMPS:
            /* OK to home clamps now */
            if(CcIoManager.IntraComms[SubsystemList::CLAMPS_SUBS].get_ss_state() == SubCommsClass::SubsystemStates::WAITING_INPUT){
                Serial.println("Conductor: clamps homed, system getting to ready position");
                CcIoManager.IntraComms[SubsystemList::ROTS_SUBS].set_ss_command(SubCommsClass::RDY_COMMAND);
                CcIoManager.IntraComms[SubsystemList::CUTTER_SUBS].set_ss_command(SubCommsClass::RDY_COMMAND);
                state = Cond::MOVE_TO_READY;
            }
            break;

        case Cond::MOVE_TO_READY:
            /* move to recieve with loaded blade */
            Serial.println("Conductor: Moving to ready");
            CcIoManager.IntraComms[SubsystemList::CUTTER_SUBS].set_ss_command(CUTTER_LOAD_CMD);
            CcIoManager.IntraComms[SubsystemList::CLAMPS_SUBS].set_ss_command(CLAMPS_RECIEVE_CMD);
            CcIoManager.IntraComms[SubsystemList::ROTS_SUBS].set_ss_command(ROT_RECIEVE_CMD);
            state = Cond::WAIT_READY;
            break;

        case Cond::WAIT_READY:
            /* wait for cutter and clamps to finish moving */
            if( finished_move(cutter_state, last_cutter_state) &&
                finished_move(clamps_state, last_clamps_state)&&
                rotators_state == SubCommsClass::SubsystemStates::WAITING_INPUT
            ){
                Serial.println("Conductor: clearing claws");
                CcIoManager.IntraComms[SubsystemList::ROTS_SUBS].set_ss_command(ROT_SQUISH_CMD);
                state = Cond::CLEAR_CLAWS;
            }
            break;

        case Cond::CLEAR_CLAWS:
            /* wait for claws to be moving */
            if( rotators_state == SubCommsClass::SubsystemStates::MOVING && (CcIoManager.getSystemTime() -
                CcIoManager.IntraComms[SubsystemList::ROTS_SUBS].get_ss_last_transition_time_ms() >= CLAW_CLEAR_TIME_MS)){
                Serial.println("Conductor: Moving back");
                CcIoManager.IntraComms[SubsystemList::ROTS_SUBS].set_ss_command(ROT_RECIEVE_CMD);
                state = Cond::BACK_TO_RECIEVE;
            }
            break;

        case Cond::BACK_TO_RECIEVE:
            /* wait for cutter and clamps to finish moving */
            if( finished_move(cutter_state, last_cutter_state) &&
                finished_move(clamps_state, last_clamps_state) &&
                finished_move(rotators_state, last_rotators_state)
            ){
                Serial.println("Conductor: back to recieve, ready");
                state = Cond::AT_READY;
            }
            break;

        case Cond::AT_READY:
        case Cond::RUNNING:
            /* rest of machine ready to run */
            if(unload_cutter_input){
                Serial.println("Conductor: unloading blade");
                CcIoManager.IntraComms[SubsystemList::CUTTER_SUBS].set_ss_command(CUTTER_LOAD_CMD);
                state = Cond::UNLOAD_CUTTER_TO_FLAG;
            }else if(run_input){
                Serial.println("Conductor: running cycle");
                //CcIoManager.IntraComms[SubsystemList::CLAMPS_SUBS].set_ss_command(CLAMPS_CLAMP_CMD);
                //state = Cond::CLAMPING;
                CcIoManager.IntraComms[SubsystemList::DRUM_SUBS].set_ss_command(LOAD_DRUM_CMD);
                state = Cond::LOADING;
            }
            break;
        
        case Cond::LOADING:
            if(
                release_state == SubCommsClass::SubsystemStates::WAITING_INPUT && 
                drum_state == SubCommsClass::SubsystemStates::WAITING_INPUT
            ){
               Serial.println("Conductor: Loaded, now release");
                
                CcIoManager.IntraComms[SubsystemList::RELEASE_SUBS].set_ss_command(RELEASE_AVO_CMD);
                state = Cond::RELEASING;
            }
        break;

        case Cond::RELEASING:
            if(
                release_state == SubCommsClass::SubsystemStates::WAITING_INPUT && 
                last_release_state == SubCommsClass::SubsystemStates::MOVING &&
                drum_state == SubCommsClass::SubsystemStates::WAITING_INPUT
            ){
                Serial.println("Conductor: Released, back to ready");
                state = Cond::CLAMPING;
            }
        break;
        
        case Cond::CLAMPING:
            /* wait for clamping to finish moving and checking for avo*/
            if(
                finished_move(clamps_state, last_clamps_state) &&
                cutter_state == SubCommsClass::SubsystemStates::WAITING_INPUT
            ){
                Serial.println("Conductor: Clamped, now cut");
                CcIoManager.IntraComms[SubsystemList::CUTTER_SUBS].set_ss_command(CUTTER_CUT_CMD);
                state = Cond::CUTTING;
            }else if (clamps_state == SubCommsClass::SubsystemStates::ABORTED_COMMAND_WAITING_INPUT)
            {
                Serial.println("Conductor: Clamped, no avo detected, moving to ready");
                CcIoManager.IntraComms[SubsystemList::CUTTER_SUBS].set_ss_command(CUTTER_LOAD_CMD);
                CcIoManager.IntraComms[SubsystemList::CLAMPS_SUBS].set_ss_command(CLAMPS_RECIEVE_CMD);
                CcIoManager.IntraComms[SubsystemList::ROTS_SUBS].set_ss_command(ROT_RECIEVE_CMD);
                state = Cond::WAIT_READY;
            }
            break;

        case Cond::CUTTING:
            /* wait for cutter to finish moving */
            if(finished_move(cutter_state, last_cutter_state) && 
                rotators_state == SubCommsClass::SubsystemStates::WAITING_INPUT){
                Serial.println("Conductor: Cut, now move to presquish and load cutter");
                CcIoManager.IntraComms[SubsystemList::CUTTER_SUBS].set_ss_command(CUTTER_LOAD_CMD);
                CcIoManager.IntraComms[SubsystemList::ROTS_SUBS].set_ss_command(ROT_PRESQUISH_CMD);                
                state = Cond::MOVE_ROT_PRESQUISH_LOAD_CUT;
            }
            break;

        case Cond::MOVE_ROT_PRESQUISH_LOAD_CUT:
            /* wait for rotator to finish moving */
            if(finished_move(rotators_state, last_rotators_state) && 
                clamps_state == SubCommsClass::SubsystemStates::WAITING_INPUT){
                Serial.println("Conductor: grab");
                CcIoManager.IntraComms[SubsystemList::CLAMPS_SUBS].set_ss_command(CLAMPS_GRAB_CMD);             
                state = Cond::GRAB;
            }
            break;

         case Cond::GRAB:
            /* wait for clamp to finish grab */
            if(finished_move(clamps_state, last_clamps_state) && 
                rotators_state == SubCommsClass::SubsystemStates::WAITING_INPUT){
                Serial.println("Conductor: rots to squish");
                CcIoManager.IntraComms[SubsystemList::ROTS_SUBS].set_ss_command(ROT_SQUISH_CMD);             
                state = Cond::MOVE_ROT_SQUISH;
            }
            break;

         case Cond::MOVE_ROT_SQUISH:
            /* wait for rotators to be at squish */
            if(finished_move(rotators_state, last_rotators_state) && 
                clamps_state == SubCommsClass::SubsystemStates::WAITING_INPUT){
                Serial.println("Conductor: clamps to squish");
                CcIoManager.IntraComms[SubsystemList::CLAMPS_SUBS].set_ss_command(CLAMPS_SQUISH_CMD);             
                state = Cond::CLAMPS_SQUISH;
            }
            break;

         case Cond::CLAMPS_SQUISH:
            /* wait for clamps to finishing squish */
            if(finished_move(clamps_state, last_clamps_state) && 
                rotators_state == SubCommsClass::SubsystemStates::WAITING_INPUT){
                Serial.println("Conductor: rotate to recieve");
                CcIoManager.IntraComms[SubsystemList::ROTS_SUBS].set_ss_command(ROT_RECIEVE_CMD);             
                state = Cond::MOVE_ROT_RECEIVE;
            }
            break;

        case Cond::MOVE_ROT_RECEIVE:
            /* wait for rotators to finishing to recive */
            if(finished_move(rotators_state, last_rotators_state) && 
                clamps_state == SubCommsClass::SubsystemStates::WAITING_INPUT){
                Serial.println("Conductor: open clamps");
                CcIoManager.IntraComms[SubsystemList::CLAMPS_SUBS].set_ss_command(CLAMPS_OPEN_CMD);             
                state = Cond::OPEN_CLAMP;
            }
            break;

        case Cond::OPEN_CLAMP:
            /* wait for clamps to finishing opening */
            if(finished_move(clamps_state, last_clamps_state) && 
                rotators_state == SubCommsClass::SubsystemStates::WAITING_INPUT){
                Serial.println("Conductor: move back to ready");           
                state = Cond::MOVE_TO_READY;
            }
            break;

        case Cond::UNLOAD_CUTTER_TO_FLAG:

            if(finished_move(cutter_state, last_cutter_state)){
                Serial.println("Conductor: blade at flag, releasing.");
                CcIoManager.IntraComms[SubsystemList::CUTTER_SUBS].set_ss_command(CUTTER_CUT_CMD);
                state = Cond::UNLOAD_CUTTER_TO_RELEASE;
            }
            break;

        case Cond::UNLOAD_CUTTER_TO_RELEASE:

            if(finished_move(cutter_state, last_cutter_state)){            
                Serial.println("Conductor: blade released, back to ready");    
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