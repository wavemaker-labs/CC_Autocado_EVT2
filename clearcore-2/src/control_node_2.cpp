 /**
 * @file control_node_2.cpp
 * @copyright Copyright (c) 2023 Vebu Labs. All rights reserved.
 * @brief This library is used to implement control 2 specific code.
 * @author Mike Lui
*/


#include "control_node_2.hpp"

/*Ethernet*/
byte mac[] = {0x24, 0x15, 0x10, 0xb0, 0x17, 0x49};
IPAddress clear_core_ip(192, 168, 1, 103); 

#define adcResolution 12

bool b_ignore_weight = false;


void CntrlNode2Io::assign_modbus_registers(){
    ptr_mb_reg_array = cc2_modbus;
    mb_reg_array_size = CC2_NUM_MODBUS_REGISTERS;
}

void CntrlNode2Io::assign_io_pins() {

     /*Clearcore 2 specific declarations
        Link: https://wavemakerlabs.atlassian.net/wiki/spaces/BOBA/pages/1742798849/Control+Architecture
        Inputs: 
        Clearcore 2 specific 
        IO0 - Bowl Switch (Digital In)
        IO2 - Doors and Drawer (Digital In)
        IO6 - E stop in
        DI7 - Gutter Switch # 2 (Digital In)
        DI8 - Gutter Switch # 1 (Digital In)
        A9 -  Not used
        A10 - Start button (Digital In) 
        A11 - Peeler Current 1 (Analog In)
        A12 - Peeler Current 2 (Analog In)
        Outputs:
        IO1 - Start LED (Digital Out)
        IO3 - Power disable (Digital Out)
        IO4 - Peeler Relay 1 (Digital Out)
        IO5 - Peeler Relay 2 (Digital Out)
    */
        
    ptr_io_array = cc2_io_pins;
    ptr_io_array_size = NUM_CC_IO_PIN;

    /*set up inputs*/
    ClearCore::AdcMgr.AdcResolution(adcResolution);
    ConnectorIO0.Mode(ClearCore::Connector::INPUT_DIGITAL);
    ConnectorIO2.Mode(ClearCore::Connector::INPUT_DIGITAL);
    ConnectorDI6.Mode(ClearCore::Connector::INPUT_DIGITAL);
    ConnectorDI7.Mode(ClearCore::Connector::INPUT_DIGITAL);
    ConnectorDI8.Mode(ClearCore::Connector::INPUT_DIGITAL);
    ConnectorA10.Mode(ClearCore::Connector::INPUT_DIGITAL);
    ConnectorA11.Mode(ClearCore::Connector::INPUT_ANALOG);
    ConnectorA11.FilterTc(20, AdcManager::FILTER_UNIT_MS); 
    ConnectorA12.Mode(ClearCore::Connector::INPUT_ANALOG);
    ConnectorA12.FilterTc(20, AdcManager::FILTER_UNIT_MS); 
    
    /*set up outputs*/
    ConnectorIO1.Mode(ClearCore::Connector::OUTPUT_DIGITAL);
    ConnectorIO3.Mode(ClearCore::Connector::OUTPUT_DIGITAL);
    ConnectorIO4.Mode(ClearCore::Connector::OUTPUT_DIGITAL);
    ConnectorIO5.Mode(ClearCore::Connector::OUTPUT_DIGITAL);

    /*Not used*/    
    ConnectorA9.Mode(ClearCore::Connector::INPUT_DIGITAL);

}


void CntrlNode2Io::assign_motor_parameters() {
    ptr_motor_array = cc2_motors;
    motor_array_size = CC2_NUM_MOTORS;
    
    /*Motors*/
    MotorMgr.MotorInputClocking(MotorManager::CLOCK_RATE_LOW);
    MotorMgr.MotorModeSet(MotorManager::MOTOR_M0M1,
                          Connector::CPM_MODE_STEP_AND_DIR);
    MotorMgr.MotorModeSet(MotorManager::MOTOR_M2M3,
                          Connector::CPM_MODE_STEP_AND_DIR);
}

void CntrlNode2Io::initialize_ethernet(){
    Ethernet.begin(mac, clear_core_ip); // start the Ethernet connection
}


int16_t current_sensor_to_milliamps(int16_t input)
{
    return ((input*5000/4096));
}

void CntrlNode2Io::read_pin_inputs() {
    /* Clearcore 2 specific 
        Inputs: 
        IO0 - Bowl Switch (Digital In)
        IO2 - Doors and Drawer (Digital In)
        IO6 - E stop in
        DI7 - Gutter Switch # 2 (Digital In)
        DI8 - Gutter Switch # 1 (Digital In)
        A9 -  Not used
        A10 - Start button (Digital In) 
        A11 - Peeler Current 1 (Analog In)
        A12 - Peeler Current 2 (Analog In)*/



    ptr_io_array[BOWL_SENS_IN].value = ConnectorIO0.State();
    ptr_io_array[DOORS_DRAWER_SEN_IN].value = ConnectorIO2.State();
    ptr_io_array[ESTOP_IN].value = ConnectorDI6.State();
    ptr_io_array[GUTTER_SEN2_IN].value = ConnectorDI7.State();
    ptr_io_array[GUTTER_SEN1_IN].value = ConnectorDI8.State();
    ptr_io_array[A9_NOT_USED].value = 0;
    ptr_io_array[START_BUTTON_IN].value = ConnectorA10.InputRisen(); // Or Fallen
    ptr_io_array[PEELER_I1_AIN].value = current_sensor_to_milliamps(ConnectorA11.State());
    ptr_io_array[PEELER_I2_AIN].value = current_sensor_to_milliamps(ConnectorA12.State());
}

void CntrlNode2Io::write_pin_outputs () {
    /*Clearcore 2 specific 
       Outputs:
        IO1 - Start LED (Digital Out)
        IO3 - Power disable (Digital Out)
        IO4 - Peeler Relay 1 (Digital Out)
        IO5 - Peeler Relay 2 (Digital Out) */
    ConnectorIO1.State(ptr_io_array[START_LED_OUT].state);
    ConnectorIO3.State(ptr_io_array[POWER_DISABLE_OUT].state);
    ConnectorIO4.State(ptr_io_array[PEELER_RELAY1_OUT].state);
    ConnectorIO5.State(ptr_io_array[PEELER_RELAY2_OUT].state);
}

void CntrlNode2Io::update_system_mb () {
    ptr_mb_reg_array[MbRegisterOffsets::WD_COUNTER].data = m_watchdog_counter;
    ptr_mb_reg_array[MbRegisterOffsets::E_STOP].data = (ptr_io_array[ESTOP_IN].value == ESTOP_ACTIVE);
}

bool CntrlNode2Io::ignore_weight() { return b_ignore_weight;}

uint16_t reset_hreg_write(TRegister* reg, uint16_t val) {
    if (val == 1){
        SysMgr.ResetBoard();
    }
    return val;
}


void CntrlNode2Io::cc_mb_hooks() {
    set_mb_w_hreg_cb(MbRegisterOffsets::RESET_CC_RQ, &reset_hreg_write);    
}

CntrlNode2Io CcIoManager;