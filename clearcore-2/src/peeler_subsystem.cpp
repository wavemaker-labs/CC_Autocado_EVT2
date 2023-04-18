/**
 * @file peeler_subsystem.hpp
 * @copyright Copyright (c) 2023 Vebu Labs. All rights reserved.
 * @brief This library is used to control the peelers
 * @author Mike Lui
*/

#include "peeler_subsystem.hpp"

void PeelerFSMClass::setup()
{
    if(!has_setup){
        has_setup = true;
        motor_1_out = PinStatus::LOW;
        motor_2_out = PinStatus::LOW;
        state = Peeler::PeelerStates::STOPPED;
    }
}

void PeelerFSMClass::read_interfaces()
{
    mb_move_request = CcIoManager.get_mb_data(MbRegisterOffsets::PEELER_CMD);
    estop_input = CcIoManager.get_input(AutocadoCcPins::ESTOP_IN);

    motor_1_current = CcIoManager.get_input(AutocadoCcPins::PEELER_I1_AIN);
    motor_2_current = CcIoManager.get_input(AutocadoCcPins::PEELER_I2_AIN);
}

void PeelerFSMClass::run()
{
    read_interfaces();

    if (estop_input == ESTOP_ACTIVE && state != Peeler::PeelerStates::ESTOP)
    {
        state = Peeler::PeelerStates::ESTOP;
    }

    switch (state)
    {
        case Peeler::PeelerStates::STOPPED:

            motor_1_out = PinStatus::LOW;
            motor_2_out = PinStatus::LOW;
            if(mb_move_request == PEELER_PINCH_CMD)
            {
                motor_1_out = PinStatus::HIGH;
                motor_2_out = PinStatus::HIGH;
                state = Peeler::PeelerStates::MOVING;
            }
            break;

        case Peeler::PeelerStates::MOVING:
            if(mb_move_request == PEELER_STOP_CMD)
            {
                motor_1_out = PinStatus::LOW;
                motor_2_out = PinStatus::LOW;
                state = Peeler::PeelerStates::STOPPED;
            }
            break;

        case Peeler::PeelerStates::ESTOP:
        default:
            motor_1_out = PinStatus::LOW;
            motor_2_out = PinStatus::LOW;
            if (estop_input == ESTOP_RELEASED) {
                state = Peeler::PeelerStates::STOPPED;
            }
            break;
    }

    write_interfaces();

}

void PeelerFSMClass::write_interfaces()
{
    CcIoManager.set_mb_data(MbRegisterOffsets::PEELER_STATE, state);
    CcIoManager.set_mb_data(MbRegisterOffsets::PEELER_CURRENT_1, motor_1_current);
    CcIoManager.set_mb_data(MbRegisterOffsets::PEELER_CURRENT_2, motor_2_current);

    CcIoManager.set_pin_output_state(AutocadoCcPins::PEELER_RELAY1_OUT, motor_1_out);
    CcIoManager.set_pin_output_state(AutocadoCcPins::PEELER_RELAY2_OUT, motor_2_out);
}

PeelerFSMClass peeler;