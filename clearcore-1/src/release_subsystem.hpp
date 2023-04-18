 /**
 * @file release_subsystem.hpp
 * @copyright Copyright (c) 2023 Vebu Labs. All rights reserved.
 * @brief This library is used to control the hopper release subsystem
 * @author Mike Lui
*/

#ifndef RELEASE_SUBSYSTEM_HPP
#define RELEASE_SUBSYSTEM_HPP

#include "C:\Projects\Autocado\autocado-clearcore\clearcore-1\src\control_node_1.hpp"

namespace Release
{
    typedef enum {
        UNINITIALIZED = 0,
        HOMING,        
        MOVING,
        MOVE_DONE,
        WAITING_HOME = 70,
        WAITING_MOVE,
        WAITING_MOVE_START,
        ESTOP = 80,
        ERROR_HOMING = 90,
        ERROR_MOVING,
        ERROR_MOTOR_MOVE_INVALID
    } ReleaseStates;
}

class ReleaseFSMClass {
    public:
        void setup();
        void run();        

        /*debug tuning variables*/
        uint16_t timeout_limit_ms;
        uint16_t motor_speed;
        uint16_t motor_accel;
        uint16_t position_0;
        uint16_t position_1;
        uint16_t position_2;
        uint16_t position_3;
        uint16_t position_4;
        uint16_t position_5;

        ReleaseFSMClass() {
            has_setup = false;
            state = Release::ReleaseStates::UNINITIALIZED;
            estop_input = PinStatus::LOW;
            move_start_time_ms = 0;
            ptr_release_motor = nullptr;
        }

    private:
        void read_interfaces();
        void write_interfaces();
        void set_motor_position(int16_t move_request); 

        bool has_setup;
        
        Release::ReleaseStates state;

        int16_t mb_move_request;        
        int16_t estop_input;
        int16_t release_sensor;

        uint32_t move_start_time_ms;
        uint32_t delay_move_start_time_ms;        

        MotorIO * ptr_release_motor;
};

extern ReleaseFSMClass hopper_release;

#endif //RELEASE_SUBSYSTEM_HPP