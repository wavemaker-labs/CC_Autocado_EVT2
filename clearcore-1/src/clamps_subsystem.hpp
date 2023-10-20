 /**
 * @file clamps_subsystem.hpp
 * @copyright Copyright (c) 2023 Vebu Labs. All rights reserved.
 * @brief This library is used to control the clamp subsystem
 * @author Mike Lui
*/

#ifndef CLAMP_SUBSYSTEM_HPP
#define CLAMP_SUBSYSTEM_HPP

#include "C:\Projects\Autocado\autocado-evt2-ccc-bench\clearcore-1\src\control_node_1.hpp"

#define CLAMPS_STEPS_AWAY_HOME    10000
#define CLAMPS_HOME_VMAX          -51200
#define CLAMPS_MOVE_VMAX          -512000

#define CLAMPS_DEFAULT_RECEIVE_TOP_POS 30000
#define CLAMPS_DEFAULT_RECEIVE_BOT_POS 440000
#define CLAMPS_DEFAULT_SQUISH_POS  700000
#define CLAMPS_DEFAULT_CLAMP_POS   560000
#define CLAMPS_DEFAULT_OPEN_POS    30000

namespace Clamp
{
    typedef enum {        
        SETUP,
        MOVING_AWAY_FROM_HOME,
        START_HOMING,
        SET_SG,
        WAIT_SG_HOME_DONE,
        HOME_DONE,
        STOPPED,
        MOVING_TO_OPEN,
        AT_OPEN,
        MOVING_TO_RECIEVE,
        AT_RECIEVE,
        MOVING_TO_CLAMPING,
        DETECTED_CLAMP,
        AT_CLAMPING,
        MOVING_TO_SQUISH,
        AT_SQUISH,
        ESTOP = 80,
        ERROR_MOTOR = 90
    } ClampStates;
}

class ClampsFSMClass {
    public:
        void setup();
        void run();  

        ClampsFSMClass() {
            has_setup = false;
            ptr_5160_clamp_lt_stepper = nullptr;
            ptr_5160_clamp_lb_stepper = nullptr;
            ptr_5160_clamp_rt_stepper = nullptr;
            ptr_5160_clamp_rb_stepper = nullptr;
            lt_state = Clamp::ClampStates::SETUP;
            lb_state = Clamp::ClampStates::SETUP;
            rt_state = Clamp::ClampStates::SETUP;
            rb_state = Clamp::ClampStates::SETUP;
            open_position = CLAMPS_DEFAULT_OPEN_POS;
            recieve_position_top = CLAMPS_DEFAULT_RECEIVE_TOP_POS;
            recieve_position_bot = CLAMPS_DEFAULT_RECEIVE_BOT_POS;
            clamp_position = CLAMPS_DEFAULT_CLAMP_POS;
            squish_position = CLAMPS_DEFAULT_SQUISH_POS;
            estop_input = ESTOP_RELEASED;
        }

    private:
        void read_interfaces();
        void write_interfaces();  

        void act_on_button(Cc5160Stepper * ptr_stepper, Clamp::ClampStates * ptr_state);

        bool has_setup;
        uint16_t move_timeout_ms;
        Clamp::ClampStates lt_state;
        Clamp::ClampStates lb_state;
        Clamp::ClampStates rt_state;
        Clamp::ClampStates rb_state;

        Cc5160Stepper * ptr_5160_clamp_lt_stepper; //left top
        Cc5160Stepper * ptr_5160_clamp_lb_stepper; //left bot
        Cc5160Stepper * ptr_5160_clamp_rt_stepper; //right top
        Cc5160Stepper * ptr_5160_clamp_rb_stepper; //right bot

        int16_t estop_input;
        int16_t open_switch_input;
        int16_t recieve_switch_input;
        int16_t clamp_switch_input;
        int16_t squish_switch_input;

        int32_t open_position;
        int32_t recieve_position_top;
        int32_t recieve_position_bot;
        int32_t clamp_position;
        int32_t squish_position;
        
        uint32_t move_start_time_ms;
        uint32_t move_allowance_ms;

};

extern ClampsFSMClass clamps;

#endif //CLAMP_SUBSYSTEM_HPP