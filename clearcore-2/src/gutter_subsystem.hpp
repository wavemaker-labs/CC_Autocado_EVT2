 /**
 * @file gutter_subsystem.hpp
 * @copyright Copyright (c) 2023 Vebu Labs. All rights reserved.
 * @brief This library is used to control the gutter subsystem
 * @author Mike Lui
*/

#ifndef GUTTER_SUBSYSTEM_HPP
#define GUTTER_SUBSYSTEM_HPP

#include "C:\Projects\Autocado\autocado-clearcore\clearcore-2\src\control_node_2.hpp"

#define GUTTER_DEFAULT_MOVE_TIMEOUT_MS 5000
#define GUTTER_MAX_MOVE_STEPS 30000
#define GUTTER_ALLOWANCE_MS 25

#define GUTTER_CMD_OPEN 1
#define GUTTER_CMD_CLOSE 0

#define GUTTER_DIR_TO_OPEN 1
#define GUTTER_DIR_TO_CLOSE -1

namespace Gutter
{
    typedef enum {
        UNINITIATED,
        MOVING_OPEN,
        MOVING_CLOSED,
        AT_OPEN,
        AT_CLOSED,
        ESTOP = 80,
        ERROR_MOTOR = 90,
        ERROR_TIMEOUT
    }GutterStates;
}

class GutterFSMClass {
    public:
        void setup();
        void run();        

        GutterFSMClass() {
            has_setup = false;
        }

    private:
        void read_interfaces();
        void write_interfaces();  

        bool has_setup;
        uint16_t move_timeout_ms;
        Gutter::GutterStates state;
        uint16_t mb_command;

        uint16_t max_motor_steps;
        uint16_t motor_speed;
        uint16_t motor_accel;
        uint16_t motor_dir;

        int16_t estop_input;
        int16_t open_sensor_input;
        int16_t closed_sensor_input;

        MotorIO * ptr_gutter_motor;
        
        uint32_t move_start_time_ms;

};

extern GutterFSMClass gutter;

#endif //GUTTER_SUBSYSTEM_HPP