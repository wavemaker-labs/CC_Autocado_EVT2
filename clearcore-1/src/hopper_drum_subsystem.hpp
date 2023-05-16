 /**
 * @file hopper_drum_subsystem.hpp
 * @copyright Copyright (c) 2023 Vebu Labs. All rights reserved.
 * @brief This library is used to control the hopper drum system
 * @author Mike Lui
*/

#ifndef HOPPER_DRUM_SUBSYSTEM_HPP
#define HOPPER_DRUM_SUBSYSTEM_HPP

#include "C:\Projects\Autocado\autocado-clearcore\clearcore-1\src\control_node_1.hpp"

#define DRUM_CMD_MOVE_TO_LOAD 1
#define DRUM_CMD_MOVE_TO_DUMP 2

#define DRUM_DEFAULT_MOVE_TIMEOUT_MS 15000
#define DRUM_CLEAR_SENSOR_TIME_MS 700

#define DRUM_DIR_TO_DUMP PinStatus::HIGH
#define DRUM_DIR_TO_LOAD PinStatus::LOW

namespace HopperDrum
{
    typedef enum {
        UNINITIALIZED = 0,
        MOVING_TO_LOAD,
        MOVING_TO_DUMP,
        AT_LOAD_POS,
        AT_DUMP_POS,
        CLEAR_LOAD = 70,
        CLEAR_DUMP,
        ESTOP = 80,
        ERROR_MOVING = 90     
    } DrumStates;
    
} // namespace HopperDrum

class HopperDrumFSMClass {
    public:
        void setup();
        void run();        

        HopperDrumFSMClass() {
            has_setup = false;
        }

    private:
        void read_interfaces();
        void write_interfaces();  

        bool has_setup;
        uint16_t move_timeout_ms;
        HopperDrum::DrumStates state;

        int16_t estop_input;
        int16_t plate_sensor_input;

        PinStatus run_stop_out;
        PinStatus dir_out;
        
        uint16_t mb_move_request;
        uint32_t move_start_time_ms;
};

extern HopperDrumFSMClass hpr_drum;

#endif //HOPPER_DRUM_SUBSYSTEM_HPP