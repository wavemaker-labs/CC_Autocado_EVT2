 /**
 * @file cutter_subsystem.hpp
 * @copyright Copyright (c) 2023 Vebu Labs. All rights reserved.
 * @brief This library is used to control the cutter subsystem
 * @author Mike Lui
*/

#ifndef CUTTER_SUBSYSTEM_HPP
#define CUTTER_SUBSYSTEM_HPP

// #include "C:\Projects\Autocado\autocado-evt2-ccc-bench\clearcore-1\src\control_node_1.hpp"
#include "control_node_1.hpp"



#define CUTTER_VELOCITY     500000 //was 1M  
#define CUTTER_LOAD_TICKS   30600000 // 2000000 * 15.3
#define CUTTER_CUT_TICKS    15300000 // 1000000 * 15.3

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

};

extern CutterFSMClass cutter;

#endif //CUTTER_SUBSYSTEM_HPP