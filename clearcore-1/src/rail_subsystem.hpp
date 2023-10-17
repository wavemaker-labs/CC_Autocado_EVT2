 /**
 * @file rail_subsystem.hpp
 * @copyright Copyright (c) 2023 Vebu Labs. All rights reserved.
 * @brief This library is used to control the rail subsystem
 * @author Mike Lui
*/

#ifndef RAIL_SUBSYSTEM_HPP
#define RAIL_SUBSYSTEM_HPP

#include "C:\Projects\Autocado\autocado-evt2-ccc-bench\clearcore-1\src\control_node_1.hpp"

#define RAIL_STEPS_AWAY_HOME    -51200
#define RAIL_HOME_VMAX          51200
#define RAIL_MOVE_VMAX          80000

#define DEFAULT_RECEIVE_POS 3000
#define DEFAULT_SQUISH_POS  -100000
#define DEFAULT_CORE_POS    900000


namespace Rail
{
    typedef enum {
        SETUP,
        MOVING_AWAY_FROM_HOME,
        START_HOMING,
        SET_SG,
        WAIT_SG_HOME_DONE,
        STOPPED,
        MOVING_TO_RECIEVE,       
        MOVING_TO_SQUISH,       
        MOVING_TO_CORE,       
        MOVING,       
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