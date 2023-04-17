 /**
 * @file flat_convey_subsystem.hpp
 * @copyright Copyright (c) 2023 Vebu Labs. All rights reserved.
 * @brief This library is used to control the flat conveyor subsystem
 * @author Mike Lui
*/

#ifndef FLAT_CONVEY_SUBSYSTEM_HPP
#define FLAT_CONVEY_SUBSYSTEM_HPP

#include "C:\Projects\Autocado\autocado-clearcore\clearcore-1\src\control_node_1.hpp"

#define FLAT_CON_DEFAULT_MOVE_TIMEOUT_MS 5000
#define FLAT_CON_MAX_MOVE_STEPS 30000
#define FLAT_MOVE_ALLOWANCE_MS 25

namespace FlatConvey
{
    typedef enum {
        STOPPED,
        MOVING,
        ESTOP = 80,
        ERROR_MOTOR = 90    
    } FlatConStates;
}

class FlatConveyorFSMClass {
    public:
        void setup();
        void run();        

        FlatConveyorFSMClass() {
            has_setup = false;
        }

    private:
        void read_interfaces();
        void write_interfaces();  

        bool has_setup;
        uint16_t move_timeout_ms;
        FlatConvey::FlatConStates state;

        uint16_t motor_steps;
        int16_t signed_motor_steps;
        uint16_t motor_speed;
        uint16_t motor_accel;
        uint16_t motor_dir;

        int16_t estop_input;
        int16_t edge_sensor_input;
        int16_t length_sensor_input;

        MotorIO * ptr_flat_con_motor;
        
        uint32_t move_start_time_ms;
        uint32_t move_allowance_ms;

};

extern FlatConveyorFSMClass flat_con;

#endif //FLAT_CONVEY_SUBSYSTEM_HPP