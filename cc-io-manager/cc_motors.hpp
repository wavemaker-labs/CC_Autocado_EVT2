 /**
 * @file cc_motors.hpp
 * @copyright Copyright (c) 2023 Vebu Labs. All rights reserved.
 * @brief This library holds motor infomation
 * @author Mike Lui
*/

#ifndef CC_MOTORS_HPP
#define CC_MOTORS_HPP

#include "ClearCore.h"


struct MotorIO
{
   /*Motor Settings*/
   int16_t velocity_limit;
   int16_t accel_limit;
   MotorDriver::HlfbModes hlfb_mode;
   MotorDriver::HlfbCarrierFrequency hlfb_mode_freq;
   Connector::ConnectorModes motor_mode;
   MotorDriver * ptr_connector;

   /*Commands*/
   int16_t distance;
   uint8_t position; // 2, 4, 16 position
   bool enable;
   bool new_move_commanded;   
   bool stop_abrupt;
   StepGenerator::MoveTarget move_mode;

   /*Feedback*/
   MotorDriver::HlfbStates hlfb_state;
   bool steps_complete;
   MotorDriver::StatusRegMotor statusreg;   


   MotorIO(MotorDriver::HlfbModes hlfb_mode_, MotorDriver::HlfbCarrierFrequency hlfb_mode_freq_, int16_t velocity_limit_, int16_t accel_limit_, Connector::ConnectorModes motor_mode_, MotorDriver * ptr_connector_, StepGenerator::MoveTarget move_mode_) { 
        ptr_connector = ptr_connector_;
        hlfb_mode_ = hlfb_mode_;
        hlfb_mode_freq = hlfb_mode_freq_;
        velocity_limit = velocity_limit_;
        accel_limit = accel_limit_;
        motor_mode = motor_mode_;
        distance = 0;
        position = 0;
        enable = false;
        new_move_commanded = false;
        steps_complete = false;
        stop_abrupt = false;
        hlfb_state = MotorDriver::HlfbStates::HLFB_UNKNOWN; 
        statusreg = 0;
        move_mode = move_mode_;
   }
   MotorIO(int16_t velocity_limit_, int16_t accel_limit_, Connector::ConnectorModes motor_mode_, MotorDriver * ptr_connector_, StepGenerator::MoveTarget move_mode_) { 
        ptr_connector = ptr_connector_;
        velocity_limit = velocity_limit_;
        accel_limit = accel_limit_;        
        motor_mode = motor_mode_;
        distance = 0;
        position = 0;
        enable = false;
        new_move_commanded = false;
        steps_complete = false;
        stop_abrupt = false;
        hlfb_state = MotorDriver::HlfbStates::HLFB_UNKNOWN; 
        statusreg = 0;
        hlfb_mode = MotorDriver::HlfbModes::HLFB_MODE_HAS_BIPOLAR_PWM;
        hlfb_mode_freq = MotorDriver::HlfbCarrierFrequency::HLFB_CARRIER_482_HZ;        
        move_mode = move_mode_;
   }
   MotorIO(int16_t velocity_limit_, int16_t accel_limit_, Connector::ConnectorModes motor_mode_, MotorDriver * ptr_connector_) { 
        ptr_connector = ptr_connector_;
        velocity_limit = velocity_limit_;
        accel_limit = accel_limit_;        
        motor_mode = motor_mode_;
        distance = 0;
        position = 0;
        enable = false;
        new_move_commanded = false;
        steps_complete = false;
        stop_abrupt = false;
        hlfb_state = MotorDriver::HlfbStates::HLFB_UNKNOWN; 
        statusreg = 0;
        hlfb_mode = MotorDriver::HlfbModes::HLFB_MODE_HAS_BIPOLAR_PWM;
        hlfb_mode_freq = MotorDriver::HlfbCarrierFrequency::HLFB_CARRIER_482_HZ;        
        move_mode = MotorDriver::MOVE_TARGET_ABSOLUTE;
   }
};


#endif //CC_BOBA_MOTORS_HPP