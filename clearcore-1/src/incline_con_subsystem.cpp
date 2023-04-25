 /**
 * @file incline_con_subsystem.cpp
 * @copyright Copyright (c) 2023 Vebu Labs. All rights reserved.
 * @brief This library is used to control the inclined conveyor syste
 * @author Mike Lui
*/

#include "incline_con_subsystem.hpp"

bool new_mb_incline_cmd = false;

uint16_t incline_hreg_write(TRegister* reg, uint16_t val) {
    new_mb_incline_cmd = true;
    return val;
}

void InclinedConveyorFSMClass::setup()
{
    if(!has_setup){
        has_setup = true;
        CcIoManager.set_mb_w_hreg_cb(MbRegisterOffsets::INCLINE_CON_CMD, &incline_hreg_write);
        run_stop_out = PinStatus::LOW;
        state = InclinedConveyor::IncConStates::STOPPED;
    }
}

void InclinedConveyorFSMClass::read_interfaces()
{
    mb_move_request = CcIoManager.get_mb_data(MbRegisterOffsets::INCLINE_CON_CMD);
    estop_input = CcIoManager.get_input(AutocadoCcPins::ESTOP_IN);
}

void InclinedConveyorFSMClass::run()
{
    read_interfaces();

    if (estop_input == ESTOP_ACTIVE && state != InclinedConveyor::IncConStates::ESTOP)
    {
        state = InclinedConveyor::IncConStates::ESTOP;
    }

    switch (state)
    {
        case InclinedConveyor::IncConStates::STOPPED:
            if(new_mb_incline_cmd && mb_move_request == INLCINED_CON_MOVE_CMD)
            {
                new_mb_incline_cmd = false;
                run_stop_out = PinStatus::HIGH;
                state = InclinedConveyor::IncConStates::MOVING;
            }
            break;

        case InclinedConveyor::IncConStates::MOVING:
            if(new_mb_incline_cmd && mb_move_request == INLCINED_CON_STOP_CMD)
            {
                new_mb_incline_cmd = false;
                run_stop_out = PinStatus::LOW;
                state = InclinedConveyor::IncConStates::STOPPED;
            }
            break;

        case InclinedConveyor::IncConStates::ESTOP:
        default:
            new_mb_incline_cmd = false;
            run_stop_out = PinStatus::LOW;
            if (estop_input == ESTOP_RELEASED) {
                state = InclinedConveyor::IncConStates::STOPPED;
            }
            break;
    }

    write_interfaces();

}

void InclinedConveyorFSMClass::write_interfaces()
{
    CcIoManager.set_pin_output_state(AutocadoCcPins::INCLINE_RUN_STOP_OUT, run_stop_out);
    CcIoManager.set_mb_data(MbRegisterOffsets::INCLINE_CON_STATE, state);

}

InclinedConveyorFSMClass incl_con;