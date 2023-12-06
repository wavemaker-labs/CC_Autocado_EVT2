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
#define ROTS_MOVE_VMAX           512000

#define ROTS_DEFAULT_RECEIVE_POS    92.0  //~-90 degrees
#define ROTS_DEFAULT_PRESQUISH_POS  75.0  //~-75 degree
#define ROTS_DEFAULT_SQUISH_POS     0.5   //~-0 degrees


namespace Rots
{
    typedef enum {
        SETUP,
        WAIT_FOR_HOME_CMD,
        MOVING_AWAY_FROM_HOME,
        START_HOMING,
        SET_SG,
        WAIT_SG_HOME_DONE,
        FINISH_HOME_AT_RECIEVE,
        WAIT_FOR_READY_CMD,
        STOPPED,
        MOVING_TO_RECIEVE,       
        MOVING_TO_SQUISH,       
        MOVING_TO_PRESQUISH,       
        AT_RECIEVE,       
        AT_SQUISH,       
        AT_PRESQUISH,       
        ESTOP = 80,
        ERROR_MOTOR = 90    
    } RotsStates;

    typedef enum {
        RECEIVE_POS,
        SQUISH_POS,
        PRESQUISH_POS  
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

            rots_home_vmax = -51200;
            rots_move_vmax = 512000;

            float catch_atp = (ROTS_DEFAULT_RECEIVE_POS/360.0)*51200.0*46.656;       //converting up angle to pulses
            receive_position = (int32_t)catch_atp;                                     //converting up pulses to int

            float presquish_atp = (ROTS_DEFAULT_PRESQUISH_POS/360.0)*51200.0*46.656; //converting close angle to pulses
            presquish_position = (int32_t)presquish_atp;                             //converting close pulses to int

            float squish_atp = (ROTS_DEFAULT_SQUISH_POS/360.0)*51200.0*46.656;       //converting open angle to pulses
            squish_position = (int32_t)squish_atp;                                   //converting open pulses to int
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

        int32_t rots_home_vmax;
        int32_t rots_move_vmax;

        int32_t receive_position;
        int32_t squish_position;
        int32_t presquish_position;

        int16_t estop_input;
        int16_t switch_0_input;
        int16_t switch_1_input;
        int16_t home_input;
        int16_t ready_input;
        
        uint32_t move_start_time_ms;
        uint32_t move_allowance_ms;

        void act_on_button(Cc5160Stepper * ptr_stepper, Rots::RotsStates * ptr_state);
        void determine_comm_state();

};

extern RotsFSMClass rotators;

#endif //ROTS_SUBSYSTEM_HPP