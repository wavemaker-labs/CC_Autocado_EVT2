/**
 * @file peeler_subsystem.hpp
 * @copyright Copyright (c) 2023 Vebu Labs. All rights reserved.
 * @brief This library is used to control the peelers
 * @author Mike Lui
*/

#ifndef PEELER_SUBSYSTEM_HPP
#define PEELER_SUBSYSTEM_HPP

#include "C:\Projects\Autocado\autocado-clearcore\clearcore-2\src\control_node_2.hpp"

#define PEELER_PINCH_CMD 1
#define PEELER_STOP_CMD 0


namespace Peeler
{
    typedef enum {
        STOPPED = 0,
        MOVING,
        ESTOP = 80
    } PeelerStates;
} // namespace Peeler

class PeelerFSMClass {
    public:
        void setup();
        void run();        

        PeelerFSMClass() {
            has_setup = false;
        }

    private:
        void read_interfaces();
        void write_interfaces();  

        bool has_setup;
        Peeler::PeelerStates state;

        uint16_t mb_move_request;

        PinStatus motor_1_out;
        PinStatus motor_2_out;

        uint16_t motor_1_current;
        uint16_t motor_2_current;

        int16_t estop_input;       

};

extern PeelerFSMClass peeler;

#endif //PEELER_SUBSYSTEM_HPP