 /**
 * @file orientor_subsystem.hpp
 * @copyright Copyright (c) 2023 Vebu Labs. All rights reserved.
 * @brief This library is used to control the orientor subsystem
 * @author Mike Lui
*/

#ifndef ORIENTOR_SUBSYSTEM_HPP
#define ORIENTOR_SUBSYSTEM_HPP

#define ORIENTOR_MOVE_ALLOWANCE_MS 200

#include "C:\Projects\Autocado\autocado-clearcore\clearcore-1\src\control_node_1.hpp"

namespace Orientor
{
    typedef enum {
        UNINITIALIZED = 0,
        HOMING,        
        MOVING_TO_POS,
        MOVING_TO_TQ,
        MOVE_DONE,
        ESTOP = 80,
        ERROR_HOMING = 90,
        ERROR_MOVING,
        ERROR_MOTOR_MOVE_INVALID
    } OrientorStates;
}

class OrientorFSMClass{
    public:
        void setup();
        void run();        


        OrientorFSMClass() {
            has_setup = false;
            ptr_right_orientor_motor = nullptr;
            ptr_left_orientor_motor = nullptr;
        }

    private:
        void read_interfaces();
        void write_interfaces();  
        void set_motor_parameters();
        void enable_motors(bool val);
        bool motor_move_done();
        void set_motor_distance(uint16_t distance);
        void set_motor_position(uint16_t pos_request);

        bool has_setup;
        
        Orientor::OrientorStates state;

        /*debug tuning variables*/
        uint16_t timeout_limit_ms;
        uint16_t motor_speed;
        uint16_t motor_accel;
        uint16_t motor_torque;
        uint16_t position_0;
        uint16_t position_1;
        uint16_t position_2;
        uint16_t position_3;
        uint16_t position_4;
        uint16_t position_5;

        int16_t mb_pos_move_request;       
        int16_t mb_tq_move_request;       
        int16_t estop_input;

        uint32_t move_start_time_ms;
        uint32_t delay_move_start_time_ms;        

        MotorIO * ptr_right_orientor_motor;
        MotorIO * ptr_left_orientor_motor;
};

extern OrientorFSMClass orientor;

#endif //ORIENTOR_SUBSYSTEM_HPP