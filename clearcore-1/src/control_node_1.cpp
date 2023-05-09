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

#define adcResolution 12

bool b_ignore_weight = false;

void CntrlNode1Io::assign_modbus_registers(){
    ptr_mb_reg_array = cc1_modbus;
    mb_reg_array_size = CC1_NUM_MODBUS_REGISTERS;
}

void CntrlNode1Io::assign_io_pins() {
    /* Clearcore 1 specific declarations
    Link: https://docs.google.com/spreadsheets/d/1YpkD04bFSpitLLQUb6TSDgcKF5yo8VDTLAZbZOmg5CY/
    Inputs: 
    DI6 - Flat Conveyor Edge Sensor (Digital In)
    DI7 - Hopper Drum Sensor (Digital In)
    DI8 - Hopper Release Sensor (Digital In)
    A-9 - Flat Conveyor Length Sensor (Digital In)
    A-10 - Avocado Size Selection (Analog In)
    A-11 - E-Stop (Digital In)
    A-12 - Not used
    Outputs:
    IO0 - Done LED (Digital Out)
    IO1 - Alert LED (Digital Out)
    IO2 - Drum motor Run/Stop (relay)
    IO3 - Hopper Drum Direction (relay)
    IO4 - Hopper Inclined Motor Run/Stop (relay)
    IO5 - Buzzer (Digital Out) */

    ptr_io_array = cc1_io_pins;
    ptr_io_array_size = NUM_CC_IO_PIN;

    /*set up inputs*/
    ClearCore::AdcMgr.AdcResolution(adcResolution);
    ConnectorDI6.Mode(ClearCore::Connector::INPUT_DIGITAL);
    ConnectorDI7.Mode(ClearCore::Connector::INPUT_DIGITAL);
    ConnectorDI8.Mode(ClearCore::Connector::INPUT_DIGITAL);
    ConnectorA9.Mode(ClearCore::Connector::INPUT_DIGITAL);
    ConnectorA10.Mode(ClearCore::Connector::INPUT_ANALOG);
    ConnectorA11.Mode(ClearCore::Connector::INPUT_DIGITAL);
    ConnectorA11.FilterTc(400, AdcManager::FILTER_UNIT_MS); 

    /*set up outputs*/
    ConnectorIO0.Mode(ClearCore::Connector::OUTPUT_DIGITAL);
    ConnectorIO1.Mode(ClearCore::Connector::OUTPUT_DIGITAL);
    ConnectorIO2.Mode(ClearCore::Connector::OUTPUT_DIGITAL);
    ConnectorIO3.Mode(ClearCore::Connector::OUTPUT_DIGITAL);
    ConnectorIO4.Mode(ClearCore::Connector::OUTPUT_DIGITAL);
    ConnectorIO5.Mode(ClearCore::Connector::OUTPUT_DIGITAL);
}
void CntrlNode1Io::assign_motor_parameters() {
    ptr_motor_array = cc1_motors;
    motor_array_size = CC1_NUM_MOTORS;
    
    /*Motors*/
    MotorMgr.MotorInputClocking(MotorManager::CLOCK_RATE_LOW);
    MotorMgr.MotorModeSet(MotorManager::MOTOR_M0M1,
                          Connector::CPM_MODE_STEP_AND_DIR);
    MotorMgr.MotorModeSet(MotorManager::MOTOR_M2M3,
                          Connector::CPM_MODE_STEP_AND_DIR);
}

void CntrlNode1Io::initialize_ethernet(){
    Ethernet.begin(mac, clear_core_ip); // start the Ethernet connection
}

int16_t adc_to_avo_size(int16_t input)
{
    int16_t out;
    if(input < AVO_SIZE_XLARGE_ADC_LIMIT){
        out = AVO_SIZE_XLARGE;
    }else if(input < AVO_SIZE_SMALL_ADC_LIMIT){
        out = AVO_SIZE_SMALL;
    }else if(input < AVO_SIZE_LARGE_ADC_LIMIT){
        out = AVO_SIZE_LARGE;
    }else if(input < AVO_SIZE_MED_ADC_LIMIT){
        out = AVO_SIZE_MED;
    }else{
        out = AVO_SIZE_ERROR;
    }
    return out;
}

void CntrlNode1Io::read_pin_inputs() {
    /* Clearcore 1 specific 
        Inputs: 
        DI6 - Flat Conveyor Edge Sensor (Digital In)
        DI7 - Hopper Drum Sensor (Digital In)
        DI8 - Hopper Release Sensor (Digital In)
        A-9 - Flat Conveyor Length Sensor (Digital In)
        A-10 - Avocado Size Selection (Analog In)
        A-11 - E-Stop (Digital In)
        A-12 - Not used*/

    ptr_io_array[FLAT_CON_EDGE_SEN_IN].value = ConnectorDI6.State();
    ptr_io_array[DRUM_SENS_IN].value = ConnectorDI7.State();
    ptr_io_array[RELEASE_SENS_IN].value = ConnectorDI8.State();
    ptr_io_array[FLAT_CON_LENGTH_SEN_IN].value = ConnectorA9.State(); 
    /*if statement ignores the middle points between dials (0)*/
    if(ConnectorA10.State() > AVO_SIZE_MIN_ADC){ptr_io_array[AVO_SIZE_AIN].value = adc_to_avo_size(ConnectorA10.State());}
    ptr_io_array[ESTOP_IN].value = ConnectorA11.State();
    ptr_io_array[A12_NOT_USED].value = 0;

}

void CntrlNode1Io::write_pin_outputs () {
    /*Clearcore 1 specific 
       Outputs:
        IO0 - Done LED
        IO1 - Alert LED
        IO2 - Drum motor Run/Stop (relay)
        IO3 - Hopper Drum Direction (relay)
        IO4 - Hopper Inclined Motor Run/Stop (relay)
        IO5 - Buzzer (Digital Out) */

    ConnectorIO0.State(ptr_io_array[DONE_LED_OUT].state);
    ConnectorIO1.State(ptr_io_array[ALERT_LED_OUT].state);
    ConnectorIO2.State(ptr_io_array[DRUM_RUN_STOP_OUT].state);
    ConnectorIO3.State(ptr_io_array[DRUM_DIR_OUT].state);
    ConnectorIO4.State(ptr_io_array[INCLINE_RUN_STOP_OUT].state);
    ConnectorIO5.State(ptr_io_array[BUZZER].value);    
} 

void CntrlNode1Io::update_system_mb () {
    ptr_mb_reg_array[MbRegisterOffsets::WD_COUNTER].data = m_watchdog_counter;
    ptr_mb_reg_array[MbRegisterOffsets::E_STOP].data = (ptr_io_array[ESTOP_IN].value == ESTOP_ACTIVE);
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