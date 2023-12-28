 /**
 * @file cado_conductor.hpp
 * @copyright Copyright (c) 2023 Vebu Labs. All rights reserved.
 * @brief This library is the "conductor" that schedules the different movements
 * @author Mike Lui
*/


#ifndef CONDUCTOR_HPP
#define CONDUCTOR_HPP

#include "control_node_1.hpp"

#define CLAW_CLEAR_TIME_MS 600

namespace Cond
{
    typedef enum {        
        SETUP,
        HOMING_TRAP_DOORS,
        CLOSING_CLAMPS,
        HOMING_ROTATORS,
        HOMING_CLAMPS,
        MOVE_TO_READY,
        WAIT_READY,
        CLEAR_CLAWS,
        BACK_TO_RECIEVE,
        AT_READY,
        RUNNING,
        LOADING,
        RELEASING,
        CLAMPING,
        CUTTING,
        MOVE_ROT_PRESQUISH_LOAD_CUT,
        GRAB,
        MOVE_ROT_SQUISH,
        CLAMPS_SQUISH,
        MOVE_ROT_RECEIVE,
        OPEN_CLAMP,
        UNLOAD_CUTTER_TO_FLAG,
        UNLOAD_CUTTER_TO_RELEASE,
        ESTOP = 80,
        ERROR = 90
    } ConductorStates;
}

/*Catch & receive
Clamp
Cut
Go to pre-squish & Load cutter
Grab
Go to squish
Squish
Go to catch
Open
Recieve
Go to "skin drop pos"
Go to catch*/

class ConductorClass{
    public:
        void setup();
        void run();        

        ConductorClass() {
            has_setup = false;
            state = Cond::ConductorStates::SETUP;
            run_input = 0;
            unload_cutter_input = 0;
        }

    private:
        void read_interfaces();
        void write_interfaces();  

        int16_t run_input;
        int16_t unload_cutter_input;

        SubCommsClass::SubsystemStates drum_state;
        SubCommsClass::SubsystemStates last_drum_state;

        SubCommsClass::SubsystemStates release_state;
        SubCommsClass::SubsystemStates last_release_state;  

        SubCommsClass::SubsystemStates cutter_state;
        SubCommsClass::SubsystemStates last_cutter_state;

        SubCommsClass::SubsystemStates clamps_state;
        SubCommsClass::SubsystemStates last_clamps_state;

        SubCommsClass::SubsystemStates rotators_state;
        SubCommsClass::SubsystemStates last_rotators_state;

        bool has_setup;
        Cond::ConductorStates state;
        
};

extern ConductorClass conductor;
 
#endif//CONDUCTOR_HPP