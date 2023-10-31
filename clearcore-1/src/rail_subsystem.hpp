 /**
 * @file rail_subsystem.hpp
 * @copyright Copyright (c) 2023 Vebu Labs. All rights reserved.
 * @brief This library is used to control the rail subsystem
 * @author Mike Lui
*/

#ifndef RAIL_SUBSYSTEM_HPP
#define RAIL_SUBSYSTEM_HPP

#include "control_node_1.hpp"

#define RAIL_STEPS_AWAY_HOME    10000
#define RAIL_HOME_VMAX          -51200
#define RAIL_MOVE_VMAX          1200000

#define RAIL_DEFAULT_RECEIVE_POS 50000
#define RAIL_DEFAULT_SQUISH_POS  400000
#define RAIL_DEFAULT_CORE_POS    1000000

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
        AT_RECIEVE,       
        AT_SQUISH,       
        AT_CORE,       
        ESTOP = 80,
        ERROR_MOTOR = 90    
    } RailStates;

    typedef enum {
        RECEIVE_POS,
        SQUISH_POS,
        CORE_POS  
    } RailPositions;
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
            recieve_position = RAIL_DEFAULT_RECEIVE_POS;
            squish_position = RAIL_DEFAULT_SQUISH_POS;
            core_position = RAIL_DEFAULT_CORE_POS;
        }

    private:
        void read_interfaces();
        void write_interfaces();  

        bool has_setup;
        uint16_t move_timeout_ms;
        Rail::RailStates state;

        Cc5160Stepper * ptr_5160_rail_stepper;

        Rail::RailPositions cmd_position;

        int32_t recieve_position;
        int32_t squish_position;
        int32_t core_position;

        int16_t estop_input;
        int16_t switch_0_input;
        int16_t switch_1_input;
        
        uint32_t move_start_time_ms;
        uint32_t move_allowance_ms;

        void act_on_button(Cc5160Stepper * ptr_stepper, Rail::RailStates * ptr_state);

};

extern RailFSMClass rail;

#endif //RAIL_SUBSYSTEM_HPP