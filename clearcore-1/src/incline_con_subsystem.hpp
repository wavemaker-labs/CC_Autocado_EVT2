 /**
 * @file incline_con_subsystem.hpp
 * @copyright Copyright (c) 2023 Vebu Labs. All rights reserved.
 * @brief This library is used to control the inclined conveyor system
 * @author Mike Lui
*/

#ifndef INLCINE_CON_SUBSYSTEM_HPP
#define INLCINE_CON_SUBSYSTEM_HPP

#include "C:\Projects\Autocado\autocado-clearcore\clearcore-1\src\control_node_1.hpp"

#define INLCINED_CON_MOVE_CMD 1
#define INLCINED_CON_STOP_CMD 0

namespace InclinedConveyor
{
    typedef enum {
        STOPPED,
        MOVING,
        ESTOP = 80    
    } IncConStates;
    
} // namespace InclinedConveyor

class InclinedConveyorFSMClass {
    public:
        void setup();
        void run();        

        InclinedConveyorFSMClass() {
            has_setup = false;
        }

    private:
        void read_interfaces();
        void write_interfaces();  

        bool has_setup;
        uint16_t move_timeout_ms;
        InclinedConveyor::IncConStates state;

        int16_t estop_input;

        PinStatus run_stop_out;
        PinStatus dir_out;
        
        uint16_t mb_move_request;
        uint32_t move_start_time_ms;
};

extern InclinedConveyorFSMClass incl_con;

#endif //INLCINE_CON_SUBSYSTEM_HPP