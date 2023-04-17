 /**
 * @file ui_cc1_subsystem.cpp
 * @copyright Copyright (c) 2023 Vebu Labs. All rights reserved.
 * @brief This library is used to control ui on clearcore 1
 * @author Mike Lui
*/

#include "ui_cc1_subsystem.hpp"

#define CADO_SELECTOR_ONE_VAL   500
#define CADO_SELECTOR_TWO_VAL   1000
#define CADO_SELECTOR_THREE_VAL 2000
#define CADO_SELECTOR_FOUT_VAL  3000

void UiCc1Class::setup()
{
    if(!has_setup){
        has_setup = true;
    }
}

uint16_t return_cado_size(int16_t val)
{
    uint16_t ret = 0;
    if(val < CADO_SELECTOR_ONE_VAL){
        ret = 1;
    }else if (val < CADO_SELECTOR_TWO_VAL){
        ret = 2;
    }else if (val < CADO_SELECTOR_THREE_VAL){
        ret = 3;
    }else if (val < CADO_SELECTOR_FOUT_VAL){
        ret = 4;
    }
    
}

void UiCc1Class::read_interfaces()
{
    avo_size = return_cado_size(CcIoManager.get_input(AutocadoCcPins::AVO_SIZE_AIN));
    buzzer_duration_ms = CcIoManager.get_mb_data(MbRegisterOffsets::UI_BUZZER_MODE_CMD);
    alert_mode = CcIoManager.get_mb_data(MbRegisterOffsets::UI_ALERT_MODE_CMD);
    done_status = CcIoManager.get_mb_data(MbRegisterOffsets::UI_DONE_STATUS_CMD);
}

void UiCc1Class::run()
{
    read_interfaces();

    if(buzzer_duration_ms != 0 && buzzer_started == false){
        buzzer_start_ms = CcIoManager.getSystemTime();
        buzzer_out = PinStatus::HIGH;
        buzzer_started = true;
    }

    if(buzzer_started == true && CcIoManager.getSystemTime() - buzzer_start_ms > buzzer_duration_ms){
        buzzer_out = PinStatus::LOW;
        buzzer_duration_ms = 0;
        buzzer_started = false;
    }

    if(alert_mode == 1){
        alert_out = PinStatus::HIGH;
    }else{
        alert_out = PinStatus::LOW;
    }

    if(done_status == 1){
        done_out = PinStatus::HIGH;
    }else{
        done_out = PinStatus::LOW;
    }

    write_interfaces();
}

void UiCc1Class::write_interfaces()
{
    CcIoManager.set_pin_output_state(AutocadoCcPins::BUZZER, buzzer_out);
    CcIoManager.set_pin_output_state(AutocadoCcPins::ALERT_LED_OUT, alert_out);
    CcIoManager.set_pin_output_state(AutocadoCcPins::DONE_LED_OUT, done_out);

    CcIoManager.set_mb_data(MbRegisterOffsets::UI_AVO_SIZE, avo_size);
}

UiCc1Class ui_cc1;