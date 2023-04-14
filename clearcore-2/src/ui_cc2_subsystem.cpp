/**
 * @file keg_light_subsystem.cpp
 * @copyright Copyright (c) 2023 Vebu Labs. All rights reserved.
 * @brief This library is used to update the sensors and output for the keg and light
 * @author Mike Lui
*/

#include "keg_light_subsystem.hpp"

bool new_mb_light_cmd = false;

uint16_t light_hreg_write(TRegister* reg, uint16_t val) {
    new_mb_light_cmd = true;
    return val;
}

void KegLightClass::setup()
{   
    if(!has_setup){
        has_setup = true;

        light_out = PinStatus::LOW;

        CcIoManager.set_mb_w_hreg_cb(MbRegisterOffsets::LIGHT_PICKUP, &light_hreg_write);
    }

}


void KegLightClass::read_interfaces()
{
    keg_temp_dc = CcIoManager.get_input(BobaCcPins::KEG_TEMP_AIN);
}

void KegLightClass::run()
{
    read_interfaces();

    if(new_mb_light_cmd) {
        new_mb_light_cmd = false;

        mb_light_cmd = CcIoManager.get_mb_data(MbRegisterOffsets::LIGHT_PICKUP);

        if(mb_light_cmd == 1){
            light_out = PinStatus::HIGH;
        } else if (mb_light_cmd == 0) {
            light_out = PinStatus::LOW;
        }
    }

    write_interfaces();
}

void KegLightClass::write_interfaces()
{
    CcIoManager.set_mb_data(MbRegisterOffsets::KEG_TEMP, keg_temp_dc);
    CcIoManager.set_pin_output_state(BobaCcPins::PICKUP_LIGHT_OUT, light_out);
    CcIoManager.set_mb_data(MbRegisterOffsets::PICKUP_DRINK_SENSOR, CcIoManager.get_input(BobaCcPins::PICKUP_SENS_IN));
}


KegLightClass keg_light;