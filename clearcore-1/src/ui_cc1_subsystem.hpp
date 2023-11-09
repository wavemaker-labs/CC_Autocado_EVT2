 /**
 * @file ui_cc1_subsystem.hpp
 * @copyright Copyright (c) 2023 Vebu Labs. All rights reserved.
 * @brief This library is used to control ui on clearcore 1
 * @author Mike Lui
*/

#ifndef UI1_SUBSYSTEM_HPP
#define UI1_SUBSYSTEM_HPP

#include "control_node_1.hpp"
#include "dwin_screen.hpp"

class UiCc1Class{
    public:
        void setup();
        void run();        

        UiCc1Class() {
            has_setup = false;
            new_screen = 0;
            current_screen = 0;            
        }

    private:
        void read_interfaces();
        void write_interfaces();  

        bool has_setup;

        uint16_t new_screen;
        uint16_t current_screen;

        int16_t estop_input;
        
};

extern UiCc1Class ui_cc1;
 
#endif//UI1_SUBSYSTEM_HPP