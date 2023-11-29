 /**
 * @file clamps_subsystem.hpp
 * @copyright Copyright (c) 2023 Vebu Labs. All rights reserved.
 * @brief This library is used to control the clamp subsystem
 * @author Mike Lui
*/

#ifndef CLAMP_SUBSYSTEM_HPP
#define CLAMP_SUBSYSTEM_HPP

#include "control_node_1.hpp"

#define CLAMPS_STEPS_AWAY_HOME          -10000
#define CLAMPS_HOME_VMAX                -102400
#define CLAMPS_INITIAL_CLOSE_VMAX       51200
#define CLAMPS_MOVE_VMAX                312000  
#define CLAMPS_CONTACT_VMAX             51200   //velocity after contact is made with avocado

#define CLAMPS_DEFAULT_RECEIVE_TOP_POS  30000
#define CLAMPS_DEFAULT_RECEIVE_BOT_POS  340000
#define CLAMPS_DEFAULT_RECEIVE_BOT_DEG  50.0
#define CLAMPS_DEFAULT_SQUISH_POS       800000
#define CLAMPS_DEFAULT_OPEN_POS         30000

#define CLAMPS_DEFAULT_PRE_CLAMP_POS              400000    //Position before checking encoder
#define CLAMPS_DEFAULT_CLAMP_POS                  800000    //was 600000, Limit if encoders don't stop the clamps
#define CLAMPS_DEFAULT_PRE_CUT_CLAMPING_OFFSET    20000     //was 35,000, moving to close more after clamping stops, this plus the clamp pos should not be more than squish
#define CLAMPS_DEFAULT_PRE_CORE_CLAMPING_OFFSET   30000     //was 60000, moving to close more after pre cut stops, waits until PRE_SQUISH_DELAY is reached

#define CLAMPS_PRE_RUB_OPEN_STEPS   -40000
#define CLAMPS_RUB_STEPS             100000
#define CLAMPS_RUB_VMAX             912000

#define PRE_SQUISH_DELAY                          5000  //timer until pre core offset action   


namespace Clamp
{
    typedef enum {        
        SETUP,
        MOVING_AWAY_FROM_CLOSE,
        SET_SG_CLOSING_CLAMP,
        SG_CLOSING_CLAMP,
        WAIT_HOME_CMD,
        START_HOMING,
        SET_SG,
        WAIT_SG_HOME_DONE,
        HOME_DONE,
        MOVING_TO_OPEN,
        AT_OPEN,
        MOVING_TO_RECIEVE,
        AT_RECIEVE,
        MOVING_TO_PRE_CLAMPING,
        WAIT_ALL_PRE_CLAMPING,
        AT_PRE_CLAMPING,
        MOVING_TO_CLAMPING,
        DETECTED_CLAMP,
        AT_CLAMPING,
        MOVING_TO_POST_CLAMP,
        WAITING_POST_CLAMP,
        AT_POST_CLAMP,
        MOVING_TO_PRE_CORE,
        MOVING_TO_SQUISH,
        WAITING_PRE_CORE,
        AT_PRE_CORE,
        SYNC_SQUISH,
        AT_SQUISH,
        PRE_RUB_OPEN,
        RUB_OUT_1,
        RUB_IN_1,
        RUB_OUT_2,
        RUB_IN_2,
        RUB_DONE,
        RUB_OUT_3,
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
            recieve_position_bot = (int32_t)((CLAMPS_DEFAULT_RECEIVE_BOT_DEG/360.0)*51200.0*46.656); 
            clamp_position = CLAMPS_DEFAULT_CLAMP_POS;
            pre_clamp_position = CLAMPS_DEFAULT_PRE_CLAMP_POS;
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
        int16_t grab_switch_input;
        int16_t squish_switch_input;
        int16_t home_command;

        int32_t open_position;
        int32_t recieve_position_top;
        int32_t recieve_position_bot;
        int32_t clamp_position;
        int32_t pre_clamp_position;
        int32_t squish_position;
        
        uint32_t move_start_time_ms;
        uint32_t move_allowance_ms;

        PinStatus led_output;

};

extern ClampsFSMClass clamps;

#endif //CLAMP_SUBSYSTEM_HPP