 /**
 * @file control_node_1.cpp
 * @copyright Copyright (c) 2023 Vebu Labs. All rights reserved.
 * @brief This library is used to implement control 1 specific code.
 * @author Mike Lui
*/


#include "control_node_1.hpp"

/*Ethernet*/
byte mac[] = {0x24, 0x15, 0x10, 0xb0, 0x14, 0x18};
IPAddress clear_core_ip(192, 168, 1, 102); 

SubCommsClass IntraComms[CC1_NUM_SUBSYSTEMS] = {
    {RAIL_SUBS},
    {CLAMPS_SUBS},
    {CUTTER_SUBS}
};

#define adcResolution 12

bool b_ignore_weight = false;

/*
Clearcore 
Link: 
    Inputs: 
    IO-0 - Rail Position 0 (Digital In)
    IO-1 - Rail Position 1 (Digital In)
    DI-6 - Cutting Push Button (Digital In)
    DI-7 - Load Cutter Push Button (Digital In)
    A-9 -  Open Push Button (Digital In)
    A-10 - Receive Push Button (Digital In)
    A-11 - Clamp Push Button (Digital In)
    A-12 - Squish Push Button (Digital In)
Outputs:
   IO2 - 
   IO3 - 
   IO4 - Solenoid
   IO5 -  
Motors:
   M0 - 
   M1 - 
   M2 - 
5160 Drivers:
   D0 - Stepper Motor clamp 1 (top left from front view)
   D1 - Stepper Motor clamp 2 (top right from front view)
   D2 - Stepper Motor clamp 3 (bot left from front view)
   D3 - Stepper Motor clamp 4 (bot right from front view)
   D4 - Stepper Motor Rail
   D5 - Stepper Motor cutter
*/


void CntrlNode1Io::assign_modbus_registers(){
    ptr_mb_reg_array = cc1_modbus;
    mb_reg_array_size = CC1_NUM_MODBUS_REGISTERS;
}

void CntrlNode1Io::assign_io_pins() {
    /*
    Clearcore 
    Inputs: 
    IO-0 - Rail Position 0 (Digital In)
    IO-1 - Rail Position 1 (Digital In)
    DI-6 - Cutting Push Button (Digital In)
    DI-7 - Load Cutter Push Button (Digital In)
    A-9 -  Open Push Button (Digital In)
    A-10 - Receive Push Button (Digital In)
    A-11 - Clamp Push Button (Digital In)
    A-12 - Squish Push Button (Digital In)
    Outputs:
    IO4 - Solenoid
    */

    ptr_io_array = cc1_io_pins;
    ptr_io_array_size = NUM_CC_IO_PIN;

    /*set up inputs*/
    ClearCore::AdcMgr.AdcResolution(adcResolution);
    ConnectorIO0.Mode(ClearCore::Connector::INPUT_DIGITAL);
    ConnectorIO1.Mode(ClearCore::Connector::INPUT_DIGITAL);
    ConnectorIO2.Mode(ClearCore::Connector::INPUT_DIGITAL);
       
    ConnectorIO5.Mode(ClearCore::Connector::INPUT_DIGITAL);
    ConnectorDI6.Mode(ClearCore::Connector::INPUT_DIGITAL);
    ConnectorDI7.Mode(ClearCore::Connector::INPUT_DIGITAL);
    ConnectorDI8.Mode(ClearCore::Connector::INPUT_DIGITAL);
    ConnectorA9.Mode(ClearCore::Connector::INPUT_DIGITAL);
    ConnectorA10.Mode(ClearCore::Connector::INPUT_DIGITAL);
    ConnectorA11.Mode(ClearCore::Connector::INPUT_DIGITAL);

    /*set up outputs*/
    ConnectorIO3.Mode(ClearCore::Connector::OUTPUT_DIGITAL);     
    ConnectorIO4.Mode(ClearCore::Connector::OUTPUT_DIGITAL);
}
void CntrlNode1Io::assign_motor_parameters() {
    ptr_motor_array = nullptr;
    motor_array_size = 0;

    ptr_step_array = &cc_step_mots[0];
    step_motor_array_size = CC_NUM_DAISY_STEP_MOTORS;

    ptr_step_config_array = Cc5160StepperCfg;
    step_config_array_size = CC_NUM_DAISY_STEP_MOTORS;

    SPI.begin(); //daisy chain SPI motors need SPI
}


void CntrlNode1Io::initialize_ethernet(){
    Ethernet.begin(mac, clear_core_ip); // start the Ethernet connection
}

void CntrlNode1Io::initialize_uart(){
    Serial1.begin(115200);
    Serial1.ttl(true);
}


void CntrlNode1Io::read_pin_inputs() {

    /*Inputs: 
    IO-0 - Rail Position 0 (Digital In)
    IO-1 - Rail Position 1 (Digital In)
    DI-6 - Cutting Push Button (Digital In)
    DI-7 - Load Cutter Push Button (Digital In)
    A-9 -  Open Push Button (Digital In)
    A-10 - Receive Push Button (Digital In)
    A-11 - Clamp Push Button (Digital In)
    A-12 - Squish Push Button (Digital In
    */
   // Using rising edge from spec: https://wavemakerlabs.atlassian.net/wiki/spaces/CHIP/pages/1936228353/CCC+Test+Bench+Electrical+Architecture
    ptr_io_array[D0_RAIL_SW_0].value = ConnectorIO0.State();
    ptr_io_array[D1_RAIL_SW_1].value = ConnectorIO1.State();
    ptr_io_array[D2_CLP_GRAB_BUTTON].value = ConnectorIO2.State();
    ptr_io_array[D6_CUT_BUTTON].value = ConnectorDI6.InputRisen();
    ptr_io_array[D7_LOAD_CUT_BUTTON].value = ConnectorDI7.InputRisen();
    ptr_io_array[D8_CLP_SQUISH_BTN].value = ConnectorDI8.InputRisen();
    ptr_io_array[A9_CLP_OPEN_BUTTON].value = ConnectorA9.InputRisen();
    ptr_io_array[A10_CLP_RECIEVE_BTN].value = ConnectorA10.InputRisen();
    ptr_io_array[A11_CLP_CLAMP_BTN].value = ConnectorA11.InputRisen();
    ptr_io_array[A12_NOT_USED].value = ConnectorA12.InputRisen();

}

void CntrlNode1Io::write_pin_outputs () {

    /*
    Outputs:
    IO3 - Clamps are busy LED
    IO4 - Solenoid
    */
    ConnectorIO3.State(ptr_io_array[D3_CLAPS_BUSY_LED].state);
    ConnectorIO4.State(ptr_io_array[D4_CUT_SOLENOID].state);
} 

void CntrlNode1Io::write_screen_output(const uint8_t *buffer, size_t size) {
    Serial1.write(buffer, size);
}

void CntrlNode1Io::update_system_mb () {
    ptr_mb_reg_array[MbRegisterOffsets::WD_COUNTER].data = m_watchdog_counter;
    // ptr_mb_reg_array[MbRegisterOffsets::E_STOP].data = (ptr_io_array[ESTOP_IN].value == ESTOP_ACTIVE);
    ptr_mb_reg_array[MbRegisterOffsets::E_STOP].data = 0;
}


bool CntrlNode1Io::ignore_weight() { return b_ignore_weight;}

uint16_t reset_hreg_write(TRegister* reg, uint16_t val) {
    if (val == 1){
        SysMgr.ResetBoard();
    }
    return val;
}

uint16_t debug_ignore_weight_hreg_write(TRegister* reg, uint16_t val) {
    if (val == 1){
        b_ignore_weight = true;
    } else {
        b_ignore_weight = false;
    }
    return val;
}

void CntrlNode1Io::cc_mb_hooks() {
    set_mb_w_hreg_cb(MbRegisterOffsets::RESET_CC_RQ, &reset_hreg_write);
}


CntrlNode1Io CcIoManager;