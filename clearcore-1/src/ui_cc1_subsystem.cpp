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
SubsystemComms::SubsystemStates last_cutter_state;


uint16_t buzzer_hreg_write(TRegister* reg, uint16_t val) {
    new_buzzer_mb_cmd = true;
    return val;
}

void UiCc1Class::setup()
{
    if(!has_setup){
        has_setup = true;
    }
}

void UiCc1Class::read_interfaces()
{
    // avo_size = CcIoManager.get_input(AutocadoCcPins::AVO_SIZE_AIN);
    // buzzer_duration_ms = CcIoManager.get_mb_data(MbRegisterOffsets::UI_BUZZER_MODE_CMD);
    // alert_mode = CcIoManager.get_mb_data(MbRegisterOffsets::UI_ALERT_MODE_CMD);
    // done_status = CcIoManager.get_mb_data(MbRegisterOffsets::UI_DONE_STATUS_CMD);
    // estop_input = CcIoManager.get_input(AutocadoCcPins::ESTOP_IN);
}

void UiCc1Class::run(SubsystemComms::SubsystemStates rail_state, SubsystemComms::SubsystemStates cutter_state)
{
    read_interfaces();

    if(rail_state == SubsystemComms::SubsystemStates::WAITING_INPUT){
        CcIoManager.set_uart_tx(disp_ready, READY_SCREEN_LEN);
    }

    if(last_cutter_state == SubsystemComms::SubsystemStates::WAITING_INPUT && cutter_state == SubsystemComms::SubsystemStates::MOVING){
        CcIoManager.set_uart_tx(play_sound, PLAY_SOUND_LEN);
    }

    last_cutter_state = cutter_state;

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