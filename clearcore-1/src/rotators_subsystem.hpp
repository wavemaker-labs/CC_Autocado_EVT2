 /**
 * @file rotators_subsystem.hpp
 * @copyright Copyright (c) 2023 Vebu Labs. All rights reserved.
 * @brief This library is used to control the rotators subsystem
 * @author Mike Lui
*/

#ifndef ROTATORS_SUBSYSTEM_HPP
#define ROTATORS_SUBSYSTEM_HPP

#include "control_node_1.hpp"

#define ROTS_STEPS_AWAY_HOME    10000
#define ROTS_HOME_VMAX          -51200
#define ROTS_MOVE_VMAX          1200000

#define ROTS_DEFAULT_RECEIVE_POS 50000
#define ROTS_DEFAULT_SQUISH_POS  400000
#define ROTS_DEFAULT_CORE_POS    1000000

namespace Rots
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
    } RotsStates;

    typedef enum {
        RECEIVE_POS,
        SQUISH_POS,
        CORE_POS  
    } RotsPositions;
}

class RotsFSMClass {
    public:
        void setup();
        void run();  

        RotsFSMClass() {
            has_setup = false;
            ptr_5160_rot_l_stepper = nullptr;
            ptr_5160_rot_r_stepper = nullptr;
            l_state = Rots::RotsStates::SETUP;
            r_state = Rots::RotsStates::SETUP;
            estop_input = ESTOP_RELEASED;
            recieve_position = ROTS_DEFAULT_RECEIVE_POS;
            squish_position = ROTS_DEFAULT_SQUISH_POS;
            core_position = ROTS_DEFAULT_CORE_POS;
        }

    private:
        void read_interfaces();
        void write_interfaces();  

        bool has_setup;
        uint16_t move_timeout_ms;
        Rots::RotsStates l_state;
        Rots::RotsStates r_state;

        Cc5160Stepper * ptr_5160_rot_l_stepper;
        Cc5160Stepper * ptr_5160_rot_r_stepper;

        Rots::RotsPositions cmd_position;

        int32_t recieve_position;
        int32_t squish_position;
        int32_t core_position;

        int16_t estop_input;
        int16_t switch_0_input;
        int16_t switch_1_input;
        
        uint32_t move_start_time_ms;
        uint32_t move_allowance_ms;

        void act_on_button(Cc5160Stepper * ptr_stepper, Rots::RotsStates * ptr_state);

};

extern RotsFSMClass rotators;

#endif //ROTS_SUBSYSTEM_HPP