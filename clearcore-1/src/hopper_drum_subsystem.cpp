 /**
 * @file hopper_drum_subsystem.cpp
 * @copyright Copyright (c) 2023 Vebu Labs. All rights reserved.
 * @brief This library is used to control the hopper drum system
 * @author Mike Lui
*/

#include "hopper_drum_subsystem.hpp"

bool new_mb_drum_cmd = false;
uint16_t new_mb_timeout = DRUM_DEFAULT_MOVE_TIMEOUT_MS;

uint16_t hpr_drum_hreg_write(TRegister* reg, uint16_t val) {
    new_mb_drum_cmd = true;
    return val;
}

uint16_t hpr_drum_dbg_hreg_write(TRegister* reg, uint16_t val) {
    new_mb_timeout = val;
    return val;
}

void HopperDrumFSMClass::setup()
{
    if(!has_setup){
        has_setup = true;

        new_mb_drum_cmd = false;
        CcIoManager.set_mb_w_hreg_cb(MbRegisterOffsets::DRUM_MOVE_CMD, &hpr_drum_hreg_write);     
        CcIoManager.set_mb_w_hreg_cb(MbRegisterOffsets::DB_DRUM_TIMEOUT, &hpr_drum_dbg_hreg_write);     

        run_stop_out = PinStatus::LOW;
        dir_out = PinStatus::LOW;

        estop_input = 0;
        plate_sensor_input = 0;

        mb_move_request = 0;
        move_start_time_ms = 0;

        move_timeout_ms = new_mb_timeout;

        state = HopperDrum::DrumStates::UNINITIALIZED;
    }
}

void HopperDrumFSMClass::read_interfaces()
{
    plate_sensor_input = CcIoManager.get_input(AutocadoCcPins::DRUM_SENS_IN);
    estop_input = CcIoManager.get_input(AutocadoCcPins::ESTOP_IN);
}

void HopperDrumFSMClass::run()
{
    read_interfaces();

    if (estop_input == ESTOP_ACTIVE && state != HopperDrum::DrumStates::ESTOP)
    {
        state = HopperDrum::DrumStates::ESTOP;
    }
    
    switch (state)
    {
        case HopperDrum::DrumStates::UNINITIALIZED:
            run_stop_out = PinStatus::LOW;
            dir_out = PinStatus::LOW;

            if(new_mb_drum_cmd) {    
                new_mb_drum_cmd = false;       
                move_timeout_ms = new_mb_timeout;     
                mb_move_request = CcIoManager.get_mb_data(MbRegisterOffsets::DRUM_MOVE_CMD);

                if(plate_sensor_input == PinStatus::LOW){
                    dir_out = DRUM_DIR_TO_DUMP;
                    run_stop_out = PinStatus::HIGH;
                    move_start_time_ms = CcIoManager.getSystemTime();
                    state = HopperDrum::DrumStates::CLEAR_LOAD;
                }else if(mb_move_request == DRUM_CMD_MOVE_TO_LOAD){
                    dir_out = DRUM_DIR_TO_LOAD;
                    run_stop_out = PinStatus::HIGH;
                    move_start_time_ms = CcIoManager.getSystemTime();
                    state = HopperDrum::DrumStates::MOVING_TO_LOAD;
                }else if(mb_move_request == DRUM_CMD_MOVE_TO_DUMP){
                    dir_out = DRUM_DIR_TO_DUMP;
                    run_stop_out = PinStatus::HIGH;
                    move_start_time_ms = CcIoManager.getSystemTime();
                    state = HopperDrum::DrumStates::MOVING_TO_DUMP;
                }
            }
            break;

        case HopperDrum::DrumStates::CLEAR_LOAD:
            if(CcIoManager.getSystemTime() - move_start_time_ms > DRUM_CLEAR_SENSOR_TIME_MS)
            {
                /*if sensor is cleared that means we were at the load pos*/
                if(plate_sensor_input == PinStatus::HIGH){
                    if(mb_move_request == DRUM_CMD_MOVE_TO_LOAD){
                        dir_out = DRUM_DIR_TO_LOAD;
                        run_stop_out = PinStatus::HIGH;
                        move_start_time_ms = CcIoManager.getSystemTime();
                        state = HopperDrum::DrumStates::MOVING_TO_LOAD;
                    }else if(mb_move_request == DRUM_CMD_MOVE_TO_DUMP){
                        dir_out = DRUM_DIR_TO_DUMP;
                        run_stop_out = PinStatus::HIGH;
                        move_start_time_ms = CcIoManager.getSystemTime();
                        state = HopperDrum::DrumStates::MOVING_TO_DUMP;
                    }
                }else{                   
                    /*Try the other way to clear sensor then*/
                    dir_out = DRUM_DIR_TO_LOAD;
                    run_stop_out = PinStatus::HIGH;
                    move_start_time_ms = CcIoManager.getSystemTime();
                    state = HopperDrum::DrumStates::CLEAR_DUMP;
                }
            }
            break;

        case HopperDrum::DrumStates::CLEAR_DUMP:
            if(CcIoManager.getSystemTime() - move_start_time_ms > DRUM_CLEAR_SENSOR_TIME_MS)
            {
                /*if sensor is cleared that means we were at the dump pos*/
                if(plate_sensor_input == PinStatus::HIGH){
                    if(mb_move_request == DRUM_CMD_MOVE_TO_LOAD){
                        dir_out = DRUM_DIR_TO_LOAD;
                        run_stop_out = PinStatus::HIGH;
                        move_start_time_ms = CcIoManager.getSystemTime();
                        state = HopperDrum::DrumStates::MOVING_TO_LOAD;
                    }else if(mb_move_request == DRUM_CMD_MOVE_TO_DUMP){
                        dir_out = DRUM_DIR_TO_DUMP;
                        run_stop_out = PinStatus::HIGH;
                        move_start_time_ms = CcIoManager.getSystemTime();
                        state = HopperDrum::DrumStates::MOVING_TO_DUMP;
                    }
                }else{
                    /*couldn't clear sensor either way - must be movement error*/
                    run_stop_out = PinStatus::LOW;
                    state = HopperDrum::DrumStates::ERROR_MOVING;
                }
            }
            break;


        case HopperDrum::DrumStates::MOVING_TO_LOAD:
            if(plate_sensor_input == PinStatus::LOW && CcIoManager.getSystemTime() - move_start_time_ms > DRUM_CLEAR_SENSOR_TIME_MS){
                run_stop_out = PinStatus::LOW;
                state = HopperDrum::DrumStates::AT_LOAD_POS;
            }else if(CcIoManager.getSystemTime() - move_start_time_ms > move_timeout_ms){
                state = HopperDrum::DrumStates::ERROR_MOVING;
            }
            break;

        case HopperDrum::DrumStates::MOVING_TO_DUMP:
            if(plate_sensor_input == PinStatus::LOW && CcIoManager.getSystemTime() - move_start_time_ms > DRUM_CLEAR_SENSOR_TIME_MS){
                run_stop_out = PinStatus::LOW;
                state = HopperDrum::DrumStates::AT_DUMP_POS;
            }else if(CcIoManager.getSystemTime() - move_start_time_ms > move_timeout_ms){
                state = HopperDrum::DrumStates::ERROR_MOVING;
            }
            break;
        
        case HopperDrum::DrumStates::AT_LOAD_POS:
            if(new_mb_drum_cmd) {    
                new_mb_drum_cmd = false;       
                move_timeout_ms = new_mb_timeout;     
                mb_move_request = CcIoManager.get_mb_data(MbRegisterOffsets::DRUM_MOVE_CMD);

                if(mb_move_request == DRUM_CMD_MOVE_TO_LOAD){
                    run_stop_out = PinStatus::LOW;
                    state = HopperDrum::DrumStates::AT_LOAD_POS;
                }else if(mb_move_request == DRUM_CMD_MOVE_TO_DUMP){
                    dir_out = DRUM_DIR_TO_DUMP;
                    run_stop_out = PinStatus::HIGH;
                    move_start_time_ms = CcIoManager.getSystemTime();
                    state = HopperDrum::DrumStates::MOVING_TO_DUMP;
                }
            }
            break;

        case HopperDrum::DrumStates::AT_DUMP_POS:
            if(new_mb_drum_cmd) {    
                new_mb_drum_cmd = false;       
                move_timeout_ms = new_mb_timeout;     
                mb_move_request = CcIoManager.get_mb_data(MbRegisterOffsets::DRUM_MOVE_CMD);

                if(mb_move_request == DRUM_CMD_MOVE_TO_DUMP){
                    run_stop_out = PinStatus::LOW;
                    state = HopperDrum::DrumStates::AT_DUMP_POS;
                }else if(mb_move_request == DRUM_CMD_MOVE_TO_LOAD){
                    dir_out = DRUM_DIR_TO_LOAD;
                    run_stop_out = PinStatus::HIGH;
                    move_start_time_ms = CcIoManager.getSystemTime();
                    state = HopperDrum::DrumStates::MOVING_TO_LOAD;
                }
            }
            break;

        case HopperDrum::DrumStates::ESTOP:
        default:
            new_mb_drum_cmd = false;
            dir_out = PinStatus::LOW;
            run_stop_out = PinStatus::LOW;
            if (estop_input == ESTOP_RELEASED) {
                state = HopperDrum::DrumStates::UNINITIALIZED;
            }
            break;

        case HopperDrum::DrumStates::ERROR_MOVING:
            run_stop_out = PinStatus::LOW;
            break;

    }    

    write_interfaces();
}

void HopperDrumFSMClass::write_interfaces()
{
    CcIoManager.set_mb_data(MbRegisterOffsets::DRUM_STATE, state);

    CcIoManager.set_pin_output_state(AutocadoCcPins::DRUM_RUN_STOP_OUT, run_stop_out);
    CcIoManager.set_pin_output_state(AutocadoCcPins::DRUM_DIR_OUT, dir_out);
}

HopperDrumFSMClass hpr_drum;