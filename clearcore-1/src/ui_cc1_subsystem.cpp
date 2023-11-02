 /**
 * @file ui_cc1_subsystem.cpp
 * @copyright Copyright (c) 2023 Vebu Labs. All rights reserved.
 * @brief This library is used to control ui on clearcore 1
 * @author Mike Lui
*/

#include "ui_cc1_subsystem.hpp"


bool new_buzzer_mb_cmd = false;
uint32_t timer;
uint8_t screen_num = 0;


uint16_t buzzer_hreg_write(TRegister* reg, uint16_t val) {
    new_buzzer_mb_cmd = true;
    return val;
}

void UiCc1Class::setup()
{
    if(!has_setup){
        has_setup = true;
        CcIoManager.set_uart_tx(disp_power_up, POWER_UP_SCREEN_LEN);
    }
}

void UiCc1Class::read_interfaces()
{
    // avo_size = CcIoManager.get_input(AutocadoCcPins::AVO_SIZE_AIN);
}

void UiCc1Class::run()
{
    read_interfaces();

    if(IntraComms[ROTS_SUBS].get_ss_state() == SubCommsClass::WAITING_INPUT && 
    IntraComms[CUTTER_SUBS].get_ss_state() == SubCommsClass::WAITING_INPUT){
        CcIoManager.set_uart_tx(disp_ready, READY_SCREEN_LEN);
    }

    if(IntraComms[CUTTER_SUBS].get_ss_state() == SubCommsClass::MOVING && IntraComms[CUTTER_SUBS].get_ss_last_state() == SubCommsClass::WAITING_INPUT){
        CcIoManager.set_uart_tx(play_sound, PLAY_SOUND_LEN);
    }

    write_interfaces();
}

void UiCc1Class::write_interfaces()
{
    // CcIoManager.set_pin_output_state(AutocadoCcPins::BUZZER, buzzer_out);
    // CcIoManager.set_pin_output_state(AutocadoCcPins::ALERT_LED_OUT, alert_out);
    // CcIoManager.set_pin_output_state(AutocadoCcPins::DONE_LED_OUT, done_out);

    // CcIoManager.set_mb_data(MbRegisterOffsets::UI_AVO_SIZE, avo_size);
}

UiCc1Class ui_cc1;