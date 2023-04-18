 /**
 * @file ui_cc2_subsystem.cpp
 * @copyright Copyright (c) 2023 Vebu Labs. All rights reserved.
 * @brief This library is used to control ui on clearcore 2
 * @author Mike Lui
*/

#include "ui_cc2_subsystem.hpp"

bool mb_reset_start_latch = false;

uint16_t reset_start_hreg_write(TRegister* reg, uint16_t val) {
    mb_reset_start_latch = true;
    return val;
}

void UiCc2Class::setup()
{
    if(!has_setup){
        has_setup = true;
        start_button_latch = 0;
        mb_reset_start_latch = 0;
        start_led_out = PinStatus::LOW;
        
        CcIoManager.set_mb_w_hreg_cb(MbRegisterOffsets::UI_RESET_START, &reset_start_hreg_write);    
    }
}

void UiCc2Class::read_interfaces()
{
    start_button_in = CcIoManager.get_input(AutocadoCcPins::START_BUTTON_IN);
    door_drawer_sen_in = CcIoManager.get_input(AutocadoCcPins::DOORS_DRAWER_SEN_IN);
    start_led_cmd = CcIoManager.get_mb_data(MbRegisterOffsets::UI_START_LED);
}

void UiCc2Class::run()
{
    read_interfaces();

    if(start_led_cmd){
        start_led_out = PinStatus::HIGH;
    }else{
        start_led_out = PinStatus::LOW;
    }

    if(start_button_in && !start_button_latch){
        start_button_latch = true;
    }

    if(mb_reset_start_latch){
        start_button_latch = false;
        mb_reset_start_latch = false;
    }
    

    write_interfaces();
}

void UiCc2Class::write_interfaces()
{
    CcIoManager.set_pin_output_state(AutocadoCcPins::START_LED_OUT, start_led_out);    
    CcIoManager.set_mb_data(MbRegisterOffsets::UI_START_BUTTON, start_button_latch);
    CcIoManager.set_mb_data(MbRegisterOffsets::UI_DOOR_DRAWER_STATUS, door_drawer_sen_in);
}

UiCc2Class ui_cc2;