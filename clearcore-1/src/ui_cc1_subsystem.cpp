 /**
 * @file ui_cc1_subsystem.cpp
 * @copyright Copyright (c) 2023 Vebu Labs. All rights reserved.
 * @brief This library is used to control ui on clearcore 1
 * @author Mike Lui
*/

#include "ui_cc1_subsystem.hpp"


bool new_buzzer_mb_cmd = false;
uint32_t timer;
uint8_t change_screen_buf [CHANGE_SCREEN_LEN] = {0x5A, 0xA5, 0x07, 0x82, 0x00, 0x84, 0x5A, 0x01, 0x00, 0x00};


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
    
}

void UiCc1Class::run()
{
    read_interfaces();

    if(IntraComms[ROTS_SUBS].get_ss_state() == SubCommsClass::WAITING_INPUT && 
    IntraComms[CUTTER_SUBS].get_ss_state() == SubCommsClass::WAITING_INPUT){
        new_screen = READY_SCREEN_ADDRESS;
    }

    if(IntraComms[CUTTER_SUBS].get_ss_state() == SubCommsClass::MOVING ||
      IntraComms[ROTS_SUBS].get_ss_state() == SubCommsClass::MOVING ||
      IntraComms[CLAMPS_SUBS].get_ss_state() == SubCommsClass::MOVING ){
        new_screen = PROCESSING_SCREEN_ADDRESS;
    }

    write_interfaces();
}

void UiCc1Class::write_interfaces()
{
    if(current_screen != new_screen){
        change_screen_buf[SCREEN_ADDR_BYTE] = new_screen;
        CcIoManager.set_uart_tx(change_screen_buf, CHANGE_SCREEN_LEN);
        current_screen = new_screen;
    }
}

UiCc1Class ui_cc1;