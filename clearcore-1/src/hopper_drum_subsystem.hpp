 /**
 * @file hopper_drum_subsystem.hpp
 * @copyright Copyright (c) 2023 Vebu Labs. All rights reserved.
 * @brief This library is used to control the hopper drum system
 * @author Mike Lui & Elvis Palma
*/

#ifndef HOPPER_DRUM_SUBSYSTEM_HPP
#define HOPPER_DRUM_SUBSYSTEM_HPP

#include "control_node_1.hpp"

#define drum_vel_rpm     7.5         //RPM
#define dump_angle       60.0        //degrees rotation
#define vibro_vel_rpm    1800.0      //RPM

#define DRUM_US_PER_REV         51200.0
#define DRUM_MOTOR_GEAR_RATIO   13.733564013840830449826989619377
#define DRUM_PULLEY_GEAR_RATIO  1.99
#define CLOCK_RATIO             0.7152557373046875
#define DEG_PER_REV             360.0
#define SECS_PER_MIN            60

namespace HopperDrum
{
    typedef enum {
        SETUP,
        STOPPED,
        WAIT_READY_CMD,
        MOVING_TO_LOAD,
        MOVING_TO_DUMP,
        AT_LOAD_POS,
        AT_DUMP_POS,
        CLEAR_LOAD = 70,
        CLEAR_DUMP,
        DOUBLE_FEED_DETECTED,
        ESTOP = 80,
        ERROR_MOTOR = 90     
    } DrumStates;
    
} // namespace HopperDrum

class HopperDrumFSMClass {
    public:
        void setup();
        void run();        

        HopperDrumFSMClass() {
            has_setup = false;
            ptr_5160_drum_stepper = nullptr;
            ptr_5160_vibro_stepper = nullptr;
            state = HopperDrum::DrumStates::SETUP;
            estop_input = ESTOP_RELEASED;
        }

    private:
        void read_interfaces();
        void write_interfaces();  
        void determine_comm_state();
        
        bool has_setup;
        uint16_t move_timeout_ms;
        HopperDrum::DrumStates state;

        Cc5160Stepper * ptr_5160_drum_stepper;
        Cc5160Stepper * ptr_5160_vibro_stepper;

        int16_t estop_input;
        int16_t loadDrum_input;
        int16_t dumpDrum_input;
        int16_t drum_sensor_input;
        int16_t drum_sensor_input_fall;
        int16_t clear_doublefeed_input;
        int16_t ready_input;

        PinStatus vibrator_motor_out;
        PinStatus run_stop_out;
        PinStatus dir_out;
        PinStatus double_feed_led;
        
        int32_t drum_vel;
        int32_t vibro_vel;
        int32_t dump_offset;

        uint16_t mb_move_request;
        uint32_t move_start_time_ms;
};

extern HopperDrumFSMClass hpr_drum;

#endif //HOPPER_DRUM_SUBSYSTEM_HPP