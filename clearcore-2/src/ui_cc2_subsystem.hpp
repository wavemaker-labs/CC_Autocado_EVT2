 /**
 * @file ui_cc2_subsystem.hpp
 * @copyright Copyright (c) 2023 Vebu Labs. All rights reserved.
 * @brief This library is used to control ui on clearcore 2
 * @author Mike Lui
*/

#ifndef UI2_SUBSYSTEM_HPP
#define UI2_SUBSYSTEM_HPP

#include "C:\Projects\Autocado\autocado-clearcore\clearcore-2\src\control_node_2.hpp"

class UiCc2Class{
    public:
        void setup();
        void run();        

        UiCc2Class() {
            has_setup = false;
            start_led_out = PinStatus::LOW;
        }

    private:
        void read_interfaces();
        void write_interfaces();  

        bool has_setup;

        uint16_t start_button_in;
        uint16_t start_button_latch;

        uint16_t start_led_cmd;
        PinStatus start_led_out;

        uint16_t door_drawer_sen_in;       
        
};

extern UiCc2Class ui_cc2;
 
#endif//UI2_SUBSYSTEM_HPP