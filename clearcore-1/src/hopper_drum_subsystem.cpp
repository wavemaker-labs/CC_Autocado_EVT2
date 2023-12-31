 /**
 * @file hopper_drum_subsystem.cpp
 * @copyright Copyright (c) 2023 Vebu Labs. All rights reserved.
 * @brief This library is used to control the hopper drum system
 * @author Mike Lui & Elvis Palma & Anthony Margulis
*/

#include "hopper_drum_subsystem.hpp"

int32_t drum_rpm_to_ppt(float flo_val)
{
    return (int32_t)((flo_val*DRUM_MOTOR_GEAR_RATIO*DRUM_PULLEY_GEAR_RATIO*DRUM_US_PER_REV)/(CLOCK_RATIO*SECS_PER_MIN));
}

int32_t vibro_rpm_to_ppt(float flo_val)
{
    return (int32_t)((flo_val*DRUM_US_PER_REV)/(CLOCK_RATIO*SECS_PER_MIN));
}

int32_t hopper_angle_to_pulses(float flo_val)
{
    return (int32_t)((flo_val/(DEG_PER_REV))*(DRUM_MOTOR_GEAR_RATIO*DRUM_PULLEY_GEAR_RATIO*DRUM_US_PER_REV));
}

void HopperDrumFSMClass::setup()
{
    if(!has_setup){
        has_setup = true;        

        state = HopperDrum::DrumStates::SETUP;
        ptr_5160_drum_stepper = CcIoManager.get_step_ptr(AutocadoCcSteppers::STEPPER_DRUM);
        ptr_5160_vibro_stepper = CcIoManager.get_step_ptr(AutocadoCcSteppers::STEPPER_VIBRO);
    }
    
}

void HopperDrumFSMClass::read_interfaces()
{
    SubCommsClass::SubsystemCommands conductor_cmd;

    conductor_cmd = CcIoManager.IntraComms[SubsystemList::DRUM_SUBS].get_ss_cmd();

    drum_sensor_input = CcIoManager.get_input(AutocadoCcPins::A12_PRESENCE_BTN);
    drum_sensor_input_fall = CcIoManager.get_input(AutocadoCcPins::A12_PRESENCE_FALL);
    clear_doublefeed_input = CcIoManager.get_input(AutocadoCcPins::A10_CLP_RECIEVE_BTN);
    ready_input = (conductor_cmd == SubCommsClass::SubsystemCommands::RDY_COMMAND);
    loadDrum_input = (conductor_cmd == LOAD_DRUM_CMD);
}


void HopperDrumFSMClass::determine_comm_state(){
    /*Here we map the subsystem specific states to generalized states for the higher level controller.*/
    switch (state)
    {
        case HopperDrum::DrumStates::SETUP:
            CcIoManager.IntraComms[SubsystemList::DRUM_SUBS].set_ss_state(SubCommsClass::SubsystemStates::SETUP);
            break;
        case HopperDrum::DrumStates::WAIT_READY_CMD:
        case HopperDrum::DrumStates::STOPPED:
        case HopperDrum::DrumStates::AT_LOAD_POS:
        case HopperDrum::DrumStates::AT_DUMP_POS:
            CcIoManager.IntraComms[SubsystemList::DRUM_SUBS].set_ss_state(SubCommsClass::SubsystemStates::WAITING_INPUT);
            break;

        case HopperDrum::DrumStates::MOVING_TO_LOAD:
        case HopperDrum::DrumStates::MOVING_TO_DUMP:
            CcIoManager.IntraComms[SubsystemList::DRUM_SUBS].set_ss_state(SubCommsClass::SubsystemStates::MOVING);
            break;      

        case HopperDrum::DrumStates::ERROR_MOTOR:
            CcIoManager.IntraComms[SubsystemList::DRUM_SUBS].set_ss_state(SubCommsClass::SubsystemStates::ERROR_MOTOR);
            break;
        
        case HopperDrum::DrumStates::ESTOP:
        default:
            CcIoManager.IntraComms[SubsystemList::DRUM_SUBS].set_ss_state(SubCommsClass::SubsystemStates::ESTOP);
            break;
    }
}

void HopperDrumFSMClass::run()
{
    read_interfaces();

    drum_vel = drum_rpm_to_ppt(drum_vel_rpm);
    dump_offset = hopper_angle_to_pulses(dump_angle);
    vibro_vel = vibro_rpm_to_ppt(vibro_vel_rpm);
    
    if (state != HopperDrum::DrumStates::MOVING_TO_LOAD && drum_sensor_input_fall)
    {
        Serial.println("Double feed detected");
        double_feed_led = PinStatus::HIGH;
        state = HopperDrum::DrumStates::DOUBLE_FEED_DETECTED;
    }

    switch (state)
    {
        case HopperDrum::DrumStates::SETUP:
            if(ptr_5160_drum_stepper->config_ready() && ptr_5160_vibro_stepper->config_ready())
            {
                Serial.println("Hopper Config ready");
                state = HopperDrum::DrumStates::WAIT_READY_CMD;

            }else
            {
                Serial.println("Hopper Config being set up");
                Serial.println(ptr_5160_drum_stepper->step_5160_motor_cfg.configIndex);
                Serial.println(ptr_5160_vibro_stepper->step_5160_motor_cfg.configIndex);
            }

            break;

        case HopperDrum::DrumStates::WAIT_READY_CMD:

            if (loadDrum_input)
            {
                Serial.println("Attempting load drum move");

                ptr_5160_drum_stepper->set_velocity(drum_vel);
                ptr_5160_vibro_stepper->set_velocity(vibro_vel);

                state = HopperDrum::DrumStates::MOVING_TO_LOAD;
            } 

            break;

        case HopperDrum::DrumStates::MOVING_TO_LOAD:

            //if(!drum_sensor_input)     //for sensor on top of drum
            if(drum_sensor_input_fall)  //for sensor besides the drum
            {
                Serial.println("Drum avo presence sensor triggered");

                ptr_5160_drum_stepper->set_velocity(0);
                ptr_5160_vibro_stepper->set_velocity(0);

                state = HopperDrum::DrumStates::AT_LOAD_POS;
            }
  
            break;

        case HopperDrum::DrumStates::AT_LOAD_POS:

            if (ptr_5160_drum_stepper->at_vmax() && ptr_5160_vibro_stepper->at_vmax())
            {
                Serial.println("Avo loaded");
                Serial.println("Hopper ready");

                state = HopperDrum::DrumStates::WAIT_READY_CMD;
            }
   
            break;      

        case HopperDrum::DrumStates::CLEAR_LOAD:
  
            break;

        case HopperDrum::DrumStates::CLEAR_DUMP:

            break;
            
        case HopperDrum::DrumStates::DOUBLE_FEED_DETECTED:
            
            if (clear_doublefeed_input){
                double_feed_led = PinStatus::LOW;
                state = HopperDrum::DrumStates::WAIT_READY_CMD;
            }
            
            break;

        case HopperDrum::DrumStates::ESTOP:

            break;
        
        case HopperDrum::DrumStates::ERROR_MOTOR:
        default:

            break;
    }    

    determine_comm_state();

    write_interfaces();
}

void HopperDrumFSMClass::write_interfaces()
{
    CcIoManager.set_pin_output_state(AutocadoCcPins::D3_DOUBLE_FEED_LED, double_feed_led);
}

HopperDrumFSMClass hpr_drum;