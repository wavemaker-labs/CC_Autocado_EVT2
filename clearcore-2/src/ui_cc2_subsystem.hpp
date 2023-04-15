/**
 * @file keg_light_subsystem.hpp
 * @copyright Copyright (c) 2023 Vebu Labs. All rights reserved.
 * @brief This library is used to update the sensors and output for the keg and light
 * @author Mike Lui
*/

#ifndef KEG_LIGHT_SUBSYSTEM_HPP
#define KEG_LIGHT_SUBSYSTEM_HPP

#include "C:\Projects\Autocado\autocado-clearcore\clearcore-2\src\control_node_2.hpp"

class KegLightClass {
    public:
        void setup();
        void run();

        KegLightClass() {
            has_setup = false;
            light_out = PinStatus::LOW;
        }

    private:
        void read_interfaces();
        void write_interfaces();
        bool has_setup;

        PinStatus light_out;
        uint16_t mb_light_cmd;
        uint16_t keg_temp_dc;
};

extern KegLightClass keg_light;

#endif // KEG_LIGHT_SUBSYSTEM_HPP