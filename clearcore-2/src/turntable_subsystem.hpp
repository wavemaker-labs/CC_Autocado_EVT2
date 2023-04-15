/**
 * @file drink_pickup_subsystem.hpp
 * @copyright Copyright (c) 2023 Vebu Labs. All rights reserved.
 * @brief This library is used to control the drink pickup
 * @author Mike Lui
*/

#ifndef DRINK_PICKUP_SUBSYSTEM_HPP
#define DRINK_PICKUP_SUBSYSTEM_HPP

#include "C:\Autocado\autocado-clearcore\clearcore-2\src\control_node_2.hpp"

namespace DrinkPickup
{
    typedef enum {
        UNINITIALIZED = 0,
        INITIALIZING,        
        READY,
        MOVING,
        WAITING_RELAY_HOME = 70,
        WAITING_RELAY_MOVE,
        WAITING_MOVE,
        ESTOP = 80,
        ERROR_HOMING = 90,
        ERROR_MOVING
    } DrinkPickupStates;
} // namespace DrinkPickup

class DrinkPickupFSMClass {
    public:
        void setup();
        void run();        

        DrinkPickupFSMClass() {
            has_setup = false;
            state = DrinkPickup::DrinkPickupStates::UNINITIALIZED;
            estop_input = PinStatus::LOW;
            locker_relay_out = PinStatus::LOW;
            mb_move_request = 0;
            move_start_time_ms = 0;
            ptr_drink_pickup_motor = nullptr;
        }

    private:
        void read_interfaces();
        void write_interfaces();  

        bool has_setup;
        bool move_done;
        bool move_timeout;
        DrinkPickup::DrinkPickupStates state;
        int16_t estop_input;
        int16_t drink_sensor_input;
        PinStatus locker_relay_out;
        int16_t mb_move_request;
        uint32_t move_start_time_ms;
        MotorIO * ptr_drink_pickup_motor;
};

extern DrinkPickupFSMClass drink_pickup;

#endif //DRINK_PICKUP_SUBSYSTEM_HPP