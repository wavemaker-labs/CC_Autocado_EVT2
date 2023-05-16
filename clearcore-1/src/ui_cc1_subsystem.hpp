 /**
 * @file ui_cc1_subsystem.hpp
 * @copyright Copyright (c) 2023 Vebu Labs. All rights reserved.
 * @brief This library is used to control ui on clearcore 1
 * @author Mike Lui
*/

#ifndef UI1_SUBSYSTEM_HPP
#define UI1_SUBSYSTEM_HPP

#include "C:\Projects\Autocado\autocado-clearcore\clearcore-1\src\control_node_1.hpp"

class UiCc1Class{
    public:
        void setup();
        void run();        

        UiCc1Class() {
            has_setup = false;

            avo_size = 0;
            
            alert_mode = 0;
            alert_mode = PinStatus::LOW;

            done_status = 0;
            done_out = PinStatus::LOW;

            buzzer_mode = 0;
            buzzer_started = false;
            buzzer_duration_ms = 0;
            buzzer_start_ms = 0;
            buzzer_out = PinStatus::LOW;
        }

    private:
        void read_interfaces();
        void write_interfaces();  

        bool has_setup;

        uint16_t avo_size;

        uint16_t buzzer_mode;
        uint16_t buzzer_started;
        uint16_t buzzer_duration_ms;
        uint32_t buzzer_start_ms;
        PinStatus buzzer_out;

        uint16_t alert_mode;
        PinStatus alert_out;

        uint16_t done_status;
        PinStatus done_out;

        int16_t estop_input;
        
};

extern UiCc1Class ui_cc1;
 
#endif//UI1_SUBSYSTEM_HPP