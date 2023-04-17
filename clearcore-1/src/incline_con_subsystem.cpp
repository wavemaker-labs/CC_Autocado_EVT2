 /**
 * @file incline_con_subsystem.cpp
 * @copyright Copyright (c) 2023 Vebu Labs. All rights reserved.
 * @brief This library is used to control the inclined conveyor syste
 * @author Mike Lui
*/

#include "incline_con_subsystem.hpp"

void InclinedConveyorFSMClass::setup()
{
    if(!has_setup){
        has_setup = true;
        run_stop_out = PinStatus::LOW;
        state = InclinedConveyor::IncConStates::STOPPED;
    }
}

void InclinedConveyorFSMClass::read_interfaces()
{
    mb_move_request = CcIoManager.get_mb_data(MbRegisterOffsets::INCLINE_CON_CMD);
    estop_input = CcIoManager.get_input(MbRegisterOffsets::E_STOP);
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
            if(mb_move_request == INLCINED_CON_MOVE_CMD)
            {
                run_stop_out = PinStatus::HIGH;
                state = InclinedConveyor::IncConStates::MOVING;
            }
            break;

        case InclinedConveyor::IncConStates::MOVING:
            if(mb_move_request == INLCINED_CON_STOP_CMD)
            {
                run_stop_out = PinStatus::LOW;
                state = InclinedConveyor::IncConStates::STOPPED;
            }
            break;

        case InclinedConveyor::IncConStates::ESTOP:
        default:
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