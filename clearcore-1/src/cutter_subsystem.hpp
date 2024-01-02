 /**
 * @file cutter_subsystem.hpp
 * @copyright Copyright (c) 2023 Vebu Labs. All rights reserved.
 * @brief This library is used to control the cutter subsystem
 * @author Mike Lui
*/

#ifndef CUTTER_SUBSYSTEM_HPP
#define CUTTER_SUBSYSTEM_HPP

#include "control_node_1.hpp"

//4.2A motor, not geared
// #define CUTTER_VELOCITY     66 
// #define CUTTER_LOAD_REV     319 
// #define CUTTER_CUT_REV      7 

//2.8A motor, 1:4.25 gear
#define CUTTER_VELOCITY        263  
#define CUTTER_LOAD_REV        319
#define CUTTER_CUT_REV         7

#define CUTTER_US_PER_REV       51200.0
#define CUTTER_MOTOR_GEAR_RATIO 4.25
#define CUTTER_GEAR_RATIO       30
#define CLOCK_RATIO             0.7152557373046875
#define CUTTER_MODBUS_RATIO     100
#define SECS_PER_MIN            60

namespace Cutter
{
    typedef enum {
        STOPPED,        
        WAIT_FOR_READY_CMD,        
        SETUP,
        HOMING,
        WINDING,
        WOUND,
        RELEASING,
        RELEASED,
        ESTOP = 80,
        ERROR_MOTOR = 90    
    } CutterStates;
}

class CutterFSMClass {
    public:
        void setup();
        void run();

        CutterFSMClass() {
            has_setup = false;
            ptr_5160_cut_stepper = nullptr;
            state = Cutter::CutterStates::SETUP;
            estop_input = ESTOP_RELEASED;
            relay_output = PinStatus::LOW;

            cutter_velocity = CUTTER_VELOCITY; 
            cutter_load_rev = CUTTER_LOAD_REV; 
            cutter_cut_rev = CUTTER_CUT_REV;                      
        }

    private:
        void read_interfaces();
        void write_interfaces();  

        void determine_comm_state();

        bool has_setup;
        uint16_t move_timeout_ms;
        Cutter::CutterStates state;

        Cc5160Stepper * ptr_5160_cut_stepper;

        int16_t estop_input;
        int16_t cut_switch_input;
        int16_t load_switch_input;

        int16_t ready_input;

        PinStatus relay_output;
        
        uint32_t move_start_time_ms;
        uint32_t move_allowance_ms;

        uint32_t cutter_velocity;  
        int32_t cutter_load_rev;
        int32_t cutter_cut_rev;
};

extern CutterFSMClass cutter;

#endif //CUTTER_SUBSYSTEM_HPP