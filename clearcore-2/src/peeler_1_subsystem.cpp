/**
 * @file peeler_subsystem.hpp
 * @copyright Copyright (c) 2023 Vebu Labs. All rights reserved.
 * @brief This library is used to control the peelers
 * @author Mike Lui
*/

#include "peeler_subsystem.hpp"

bool new_mb_peeler_1_cmd = false;

uint16_t peeler1_hreg_write(TRegister* reg, uint16_t val) {
    new_mb_peeler_1_cmd = true;
    return val;
}

void Peeler1FSMClass::setup()
{
    if(!has_setup){
        has_setup = true;
        CcIoManager.set_mb_w_hreg_cb(MbRegisterOffsets::PEELER_M1_CMD, &peeler1_hreg_write);
        motor_out = PinStatus::LOW;
        state = Peeler::PeelerStates::STOPPED;
    }
}

void Peeler1FSMClass::read_interfaces()
{
    mb_move_request = CcIoManager.get_mb_data(MbRegisterOffsets::PEELER_M1_CMD);
    estop_input = CcIoManager.get_input(AutocadoCcPins::ESTOP_IN);

    motor_current = CcIoManager.get_input(AutocadoCcPins::PEELER_I1_AIN);
}

void Peeler1FSMClass::run()
{
    read_interfaces();

    if (estop_input == ESTOP_ACTIVE && state != Peeler::PeelerStates::ESTOP)
    {
        state = Peeler::PeelerStates::ESTOP;
    }

    switch (state)
    {
        case Peeler::PeelerStates::STOPPED:

            motor_out = PinStatus::LOW;
            if( new_mb_peeler_1_cmd && mb_move_request == PEELER_PINCH_CMD)
            {
                new_mb_peeler_1_cmd = false;
                motor_out = PinStatus::HIGH;
                state = Peeler::PeelerStates::MOVING;
            }
            break;

        case Peeler::PeelerStates::MOVING:
            if(new_mb_peeler_1_cmd && mb_move_request == PEELER_STOP_CMD)
            {
                new_mb_peeler_1_cmd = false;
                motor_out = PinStatus::LOW;
                state = Peeler::PeelerStates::STOPPED;
            }
            break;

        case Peeler::PeelerStates::ESTOP:
        default:
            new_mb_peeler_1_cmd = false;
            motor_out = PinStatus::LOW;
            if (estop_input == ESTOP_RELEASED) {
                state = Peeler::PeelerStates::STOPPED;
            }
            break;
    }

    write_interfaces();

}

void Peeler1FSMClass::write_interfaces()
{
    CcIoManager.set_mb_data(MbRegisterOffsets::PEELER_M1_STATE, state);
    CcIoManager.set_mb_data(MbRegisterOffsets::PEELER_CURRENT_1, motor_current);

    CcIoManager.set_pin_output_state(AutocadoCcPins::PEELER_RELAY1_OUT, motor_out);
}

Peeler1FSMClass peeler_m1;