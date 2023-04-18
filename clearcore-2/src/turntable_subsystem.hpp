/**
 * @file turntable_subsystem.hpp
 * @copyright Copyright (c) 2023 Vebu Labs. All rights reserved.
 * @brief This library is used to control the wml turntable
 * @author Mike Lui
*/

#ifndef TURNTABLE_SUBSYSTEM_HPP
#define TURNTABLE_SUBSYSTEM_HPP

#include "C:\Projects\Autocado\autocado-clearcore\clearcore-2\src\control_node_2.hpp"

namespace Turntable
{
    typedef enum {
        STOPPED = 0,
        MOVING,
        ESTOP = 80,
        ERROR_MOTOR_ALARM = 90
    } TurntableStates;
} // namespace Turntable

class TurntableFSMClass {
    public:
        void setup();
        void run();        

        TurntableFSMClass() {
            has_setup = false;
        }

    private:
        void read_interfaces();
        void write_interfaces();  

        bool has_setup;
        uint16_t move_timeout_ms;
        Turntable::TurntableStates state;

        uint16_t motor_steps;
        int16_t signed_motor_steps;
        uint16_t motor_speed;
        uint16_t motor_accel;
        uint16_t max_steps;

        int16_t estop_input;
        int16_t bowl_sensor_input;
        
        uint32_t move_start_time_ms;
        uint32_t move_allowance_ms;
};

extern TurntableFSMClass turntable;

#endif //TURNTABLE_SUBSYSTEM_HPP