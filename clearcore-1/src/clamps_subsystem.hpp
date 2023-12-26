 /**
 * @file clamps_subsystem.hpp
 * @copyright Copyright (c) 2023 Vebu Labs. All rights reserved.
 * @brief This library is used to control the clamp subsystem
 * @author Mike Lui
*/

#ifndef CLAMP_SUBSYSTEM_HPP
#define CLAMP_SUBSYSTEM_HPP

#include "control_node_1.hpp"

#define CLAMPS_STEPS_AWAY_HOME          -10000 //steps

#define CLAMPS_HOME_VMAX                -184
#define CLAMPS_INITIAL_CLOSE_VMAX       92
#define CLAMPS_MOVE_VMAX                561  
#define CLAMPS_CONTACT_VMAX             92   //velocity after contact is made with avocado

#define CLAMPS_NO_AVO_IN_CLAMP                    25000    //less than 1/4 of full spring travel
#define CLAMPS_DEFAULT_PRE_CLAMP_POS              6028    //Position before checking encoder
#define CLAMPS_DEFAULT_CLAMP_POS                  9042    //Limit if encoders don't stop the clamps
#define CLAMPS_DEFAULT_PRE_CUT_CLAMPING_OFFSET    527     //moving to close more after clamping stops, this plus the clamp pos should not be more than squish
#define CLAMPS_DEFAULT_PRE_CORE_CLAMPING_OFFSET   452     //moving to close more after pre cut stops, waits until PRE_SQUISH_DELAY is reached
#define CLAMPS_DEFAULT_SQUISH_POSITION            12056
#define CLAMPS_DEFAULT_OPEN_POSITION              0

#define TOP_CLAMPS_OFFSET           5275      //angle from current 0 position to the vertical (matching bottom clamps)
#define TOP_RECEIVING_POSITION      0
#define BOTTOM_RECEIVING_POSITION   5124

#define CLAMPS_PRE_RUB_OPEN_STEPS   -1025
#define CLAMPS_RUB_STEPS             1507
#define CLAMPS_RUB_VMAX              1638

#define PRE_SQUISH_DELAY            5000  //timer until pre core offset action   

#define CLAMPS_STEPPER_COUNT  4

#define CLAMPS_GEAR_RATIO 46.646
#define CLAMPS_US_PER_REV 51200.0
#define DEG_PER_REV 360.0
#define CLAMP_MODBUS_RATIO 100


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
        NO_AVO_IN_CLAMP,
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

            lt_motor = 0;
            lt_encoder = 0;
            lt_stallguard = 0;
            lb_motor = 0;
            lb_encoder = 0;
            lb_stallguard = 0;
            rt_motor = 0;
            rt_encoder = 0;
            rt_stallguard = 0;
            rb_motor = 0;
            rb_encoder = 0;
            rb_stallguard = 0;

            //velocities
            initial_close_vmax = CLAMPS_INITIAL_CLOSE_VMAX;
            move_velocity = CLAMPS_MOVE_VMAX;
            contact_velocity = CLAMPS_CONTACT_VMAX;
            home_velocity = CLAMPS_HOME_VMAX;
            rub_velocity = CLAMPS_RUB_VMAX;

            //positions
            top_position_offset = TOP_CLAMPS_OFFSET;
            receive_position_top = TOP_RECEIVING_POSITION;       
            receive_position_bot = BOTTOM_RECEIVING_POSITION;   
            pre_clamp_position = CLAMPS_DEFAULT_PRE_CLAMP_POS;
            clamp_position = CLAMPS_DEFAULT_CLAMP_POS;    
            pre_cut_clamp_offset = CLAMPS_DEFAULT_PRE_CUT_CLAMPING_OFFSET;
            pre_core_clamp_offset = CLAMPS_DEFAULT_PRE_CORE_CLAMPING_OFFSET;
            squish_position = CLAMPS_DEFAULT_SQUISH_POSITION;
            pre_rub_open_offset = CLAMPS_PRE_RUB_OPEN_STEPS;
            rub_offset = CLAMPS_RUB_STEPS;
            open_position = CLAMPS_DEFAULT_OPEN_POSITION;

            pre_squish_delay = PRE_SQUISH_DELAY;
            
            estop_input = ESTOP_RELEASED;
            checked_clamp_for_avo = false; 
            avo_in_clamp = false;
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

        int32_t lt_motor;
        int32_t lt_encoder;
        int32_t lt_stallguard;
        int32_t lb_motor;
        int32_t lb_encoder;
        int32_t lb_stallguard;
        int32_t rt_motor;
        int32_t rt_encoder;
        int32_t rt_stallguard;
        int32_t rb_motor;
        int32_t rb_encoder;
        int32_t rb_stallguard;
        
        int32_t home_velocity;
        int32_t initial_close_vmax;
        int32_t move_velocity;
        int32_t contact_velocity;
        int32_t rub_velocity;

        int32_t top_position_offset;
        int32_t receive_position_top;
        int32_t receive_position_bot;
        int32_t squish_position;
        int32_t pre_clamp_position;
        int32_t clamp_position;
        int32_t pre_cut_clamp_offset;
        int32_t pre_core_clamp_offset;
        int32_t pre_rub_open_offset;
        int32_t rub_offset;
        int32_t pre_squish_delay;

        int32_t open_position;

        bool checked_clamp_for_avo;
        bool avo_in_clamp;
        
        uint32_t move_start_time_ms;
        uint32_t move_allowance_ms;

        PinStatus led_output;

};

extern ClampsFSMClass clamps;

#endif //CLAMP_SUBSYSTEM_HPP