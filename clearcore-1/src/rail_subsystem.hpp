 /**
 * @file rail_subsystem.hpp
 * @copyright Copyright (c) 2023 Vebu Labs. All rights reserved.
 * @brief This library is used to control the rail subsystem
 * @author Mike Lui
*/

#ifndef RAIL_SUBSYSTEM_HPP
#define RAIL_SUBSYSTEM_HPP

#include "C:\Projects\Autocado\autocado-evt2-ccc-bench\clearcore-1\src\control_node_1.hpp"


namespace Rail
{
    typedef enum {
        STOPPED,
        MOVING,
        SETUP,
        CYCLE,
        CYCLE_WAIT,
        ESTOP = 80,
        ERROR_MOTOR = 90    
    } RailStates;
}

class RailFSMClass {
    public:
        void setup();
        void run();  

        RailFSMClass() {
            has_setup = false;
            ptr_5160_rail_stepper = nullptr;
            state = Rail::RailStates::SETUP;
            estop_input = ESTOP_RELEASED;
        }

    private:
        void read_interfaces();
        void write_interfaces();  

        bool has_setup;
        uint16_t move_timeout_ms;
        Rail::RailStates state;

        Cc5160Stepper * ptr_5160_rail_stepper;

        int16_t estop_input;
        int16_t switch_0_input;
        int16_t switch_1_input;
        
        uint32_t move_start_time_ms;
        uint32_t move_allowance_ms;

};

extern RailFSMClass rail;

#endif //RAIL_SUBSYSTEM_HPP