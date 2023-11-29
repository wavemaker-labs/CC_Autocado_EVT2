 /**
 * @file cado_conductor.hpp
 * @copyright Copyright (c) 2023 Vebu Labs. All rights reserved.
 * @brief This library is the "conductor" that schedules the different movements
 * @author Mike Lui
*/


#ifndef CONDUCTOR_HPP
#define CONDUCTOR_HPP

#include "control_node_1.hpp"

namespace Cond
{
    typedef enum {        
        SETUP,
        CLOSING_CLAMPS,
        HOMING_ROTATORS,
        HOMING_CLAMPS,
        RUNNING,
        UNLOAD_CUTTER_TO_FLAG,
        UNLOAD_CUTTER_TO_RELEASE,
        ESTOP = 80,
        ERROR = 90
    } ConductorStates;
}

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

        SubCommsClass::SubsystemStates cutter_state;
        SubCommsClass::SubsystemStates last_cutter_state;
        
        bool has_setup;
        Cond::ConductorStates state;
        
};

extern ConductorClass conductor;
 
#endif//CONDUCTOR_HPP