 /**
 * @file release_subsystem.hpp
 * @copyright Copyright (c) 2023 Vebu Labs. All rights reserved.
 * @brief This library is used to control the release subsystem
 * @author Mike Lui & Elvis Palma
*/

#ifndef RELEASE_SUBSYSTEM_HPP
#define RELEASE_SUBSYSTEM_HPP

#include "control_node_1.hpp"

//Homing offset and velocity
#define STEPS_AWAY_HOME        -10000    //pulses
#define HOME_VMAX               20000    //ppt

//Trap doors angles from the vertical (home position)
#define TRAP_UP_ANGLE           0.0      
#define TRAP_2_UP_ANGLE         32.0
#define TRAP_CLOSE_ANGLE        -15.0
#define TRAP_1_OPEN_ANGLE      -80.0
#define TRAP_2_OPEN_ANGLE       -100.0      //min ~ -135

#define TRAP_CLOSE_ANGLE        -15.0
#define TRAP_OPEN_ANGLE_1       -70.0
#define TRAP_OPEN_ANGLE_2       -90.0      //min ~ -135

#define MOVE_1_ORIENT_RPM       25.0     
#define MOVE_2_ORIENT_RPM       7.0     
#define MOVE_1_RELEASE_RPM      2.0     
#define MOVE_2_RELEASE_RPM      100.0     

#define RELEASE_US_PER_REV      51200.0
#define RELEASE_MOTOR_GEAR_RATIO 13.733564013840830449826989619377
#define CLOCK_RATIO             0.7152557373046875
#define SECS_PER_MIN            60

namespace Release
{
    typedef enum {       
        SETUP,
        OFFSETING,
        SET_SG,
        WAITING_HOME_CMD,
        HOMING,
        HOMED,
        WAITING_READY_CMD,
        STOPPED, 
        ORIENT_UP_1,
        ORIENT_UP_1_5,
        ORIENT_UP_2,
        ORIENT_DOWN_1,
        ORIENT_DOWN_2,
        RELEASING_1,
        RELEASING_2,
        ORIENTED_1,
        ORIENTED_2,
        RELEASED,
        ESTOP = 80,
        ERROR_MOTOR = 90    
    } ReleaseStates;

    typedef enum{
        ORIENT_POS,
        RELEASE_POS
    }ReleasePositions;
}


class ReleaseFSMClass {
    public:
        void setup();
        void run();  

        ReleaseFSMClass() {
            has_setup = false;
            ptr_5160_releaseRight_stepper = nullptr;
            ptr_5160_releaseLeft_stepper = nullptr;
            f_state = Release::ReleaseStates::SETUP;
            b_state = Release::ReleaseStates::SETUP;
            estop_input = ESTOP_RELEASED;
        }

    private:
        void read_interfaces();
        void write_interfaces();  

        bool has_setup;
        uint16_t move_timeout_ms;
        Release::ReleaseStates f_state;
        Release::ReleaseStates b_state;

        Cc5160Stepper * ptr_5160_releaseRight_stepper;
        Cc5160Stepper * ptr_5160_releaseLeft_stepper;

        Release::ReleasePositions cmd_position;

        int16_t estop_input;
        int16_t release_switch_input;
        int16_t orient_switch_input;
        
        uint32_t move_start_time_ms;
        uint32_t move_allowance_ms;

        int16_t home_command;
        int16_t home_input;
        int16_t ready_input;

        int32_t release_1st_vel;
        int32_t release_2nd_vel;

        int32_t trap_close_pos;
        int32_t trap_open_pos_1;
        int32_t trap_open_pos_2;
};

extern ReleaseFSMClass release;

#endif //RELEASE_SUBSYSTEM_HPP