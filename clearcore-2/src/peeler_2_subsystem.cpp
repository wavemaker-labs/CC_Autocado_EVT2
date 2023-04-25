/**
 * @file peeler_subsystem.hpp
 * @copyright Copyright (c) 2023 Vebu Labs. All rights reserved.
 * @brief This library is used to control the peelers
 * @author Mike Lui
*/

#include "peeler_subsystem.hpp"

bool new_mb_peeler_2_cmd = false;

uint16_t peeler2_hreg_write(TRegister* reg, uint16_t val) {
    new_mb_peeler_2_cmd = true;
    return val;
}

void Peeler2FSMClass::setup()
{
    if(!has_setup){
        has_setup = true;
        CcIoManager.set_mb_w_hreg_cb(MbRegisterOffsets::PEELER_M2_CMD, &peeler2_hreg_write);
        motor_out = PinStatus::LOW;
        state = Peeler::PeelerStates::STOPPED;
    }
}

void Peeler2FSMClass::read_interfaces()
{
    mb_move_request = CcIoManager.get_mb_data(MbRegisterOffsets::PEELER_M2_CMD);
    estop_input = CcIoManager.get_input(AutocadoCcPins::ESTOP_IN);

    motor_current = CcIoManager.get_input(AutocadoCcPins::PEELER_I2_AIN);
}

void Peeler2FSMClass::run()
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
            if( new_mb_peeler_2_cmd && mb_move_request == PEELER_PINCH_CMD)
            {
                new_mb_peeler_2_cmd = false;
                motor_out = PinStatus::HIGH;
                state = Peeler::PeelerStates::MOVING;
            }
            break;

        case Peeler::PeelerStates::MOVING:
            if(new_mb_peeler_2_cmd && mb_move_request == PEELER_STOP_CMD)
            {
                new_mb_peeler_2_cmd = false;
                motor_out = PinStatus::LOW;
                state = Peeler::PeelerStates::STOPPED;
            }
            break;

        case Peeler::PeelerStates::ESTOP:
        default:
            new_mb_peeler_2_cmd = false;
            motor_out = PinStatus::LOW;
            if (estop_input == ESTOP_RELEASED) {
                state = Peeler::PeelerStates::STOPPED;
            }
            break;
    }

    write_interfaces();

}

void Peeler2FSMClass::write_interfaces()
{
    CcIoManager.set_mb_data(MbRegisterOffsets::PEELER_M2_STATE, state);
    CcIoManager.set_mb_data(MbRegisterOffsets::PEELER_CURRENT_2, motor_current);

    CcIoManager.set_pin_output_state(AutocadoCcPins::PEELER_RELAY2_OUT, motor_out);
}

Peeler2FSMClass peeler_m2;