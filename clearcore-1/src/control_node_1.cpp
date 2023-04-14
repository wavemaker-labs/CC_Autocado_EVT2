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
    Link: https://wavemakerlabs.atlassian.net/wiki/spaces/BOBA/pages/1742798849/Control+Architecture
    Inputs: 
    IO1 - Cup Prepped Diffuse
    A-9 - Componenti Dispense Switch (Digital In)
    A-10 - Boba temp sensor (Analog In)
    A-11 - E-Stop (Digital In)
    A-12 - Load Cell (Analog In)
    Outputs:
    IO2 - Heater Relay (relay)
    IO3 - Ice Spoof (relay)
    IO4 - Syrup Pump (relay)
    IO5 - Componenti DC motors (Hbridge) */

    ptr_io_array = cc1_io_pins;
    ptr_io_array_size = NUM_CC_IO_PIN;

    /*set up inputs*/
    ClearCore::AdcMgr.AdcResolution(adcResolution);
    ConnectorIO0.Mode(ClearCore::Connector::INPUT_DIGITAL);
    ConnectorA9.Mode(ClearCore::Connector::INPUT_DIGITAL);
    ConnectorA10.Mode(ClearCore::Connector::INPUT_ANALOG);
    ConnectorA11.Mode(ClearCore::Connector::INPUT_DIGITAL);
    ConnectorA12.Mode(ClearCore::Connector::INPUT_ANALOG);
    ConnectorA12.FilterTc(800, AdcManager::FILTER_UNIT_MS); //slow tc to account for spikes in impact

    /*set up outputs*/
    ConnectorIO2.Mode(ClearCore::Connector::OUTPUT_DIGITAL);
    ConnectorIO3.Mode(ClearCore::Connector::OUTPUT_DIGITAL);
    ConnectorIO4.Mode(ClearCore::Connector::OUTPUT_DIGITAL);
    ConnectorIO5.Mode(ClearCore::Connector::OUTPUT_H_BRIDGE);
}
void CntrlNode1Io::assign_motor_parameters() {
    ptr_motor_array = cc1_motors;
    motor_array_size = CC1_NUM_MOTORS;
    
    /*Motors*/
    MotorMgr.MotorInputClocking(MotorManager::CLOCK_RATE_NORMAL);
    MotorMgr.MotorModeSet(MotorManager::MOTOR_M0M1,
                          Connector::CPM_MODE_STEP_AND_DIR);
    MotorMgr.MotorModeSet(MotorManager::MOTOR_M2M3,
                          Connector::CPM_MODE_A_DIRECT_B_DIRECT);
}

void CntrlNode1Io::initialize_ethernet(){
    Ethernet.begin(mac, clear_core_ip); // start the Ethernet connection
}

int16_t temp_sensor_to_decacelcius(int16_t input)
{
    return (((input-809)*2500)/3287); // 4-20mA sensor in 503ohms
}

void CntrlNode1Io::read_pin_inputs() {
    /* Clearcore 1 specific 
        Inputs: 
        IO1 - Cup Prepped Diffuse
        A-9 - Componenti Dispense Switch (Digital In)
        A-10 - Boba temp sensor (Analog In)
        A-11 - E-Stop (Digital In)
        A-12 - Load Cell (Analog In) g*/

    ptr_io_array[IO0_NOT_USED_IN].value = ConnectorIO0.State();
    ptr_io_array[CUP_PREPPED_IN].value = ConnectorIO1.State();
    ptr_io_array[COMPONENTI_SW_IN].value = ConnectorA9.InputRisen(); 
    ptr_io_array[BOBA_TEMP_AIN].value = temp_sensor_to_decacelcius(ConnectorA10.State());
    ptr_io_array[ESTOP_IN].value = ConnectorA11.State();
    ptr_io_array[LOAD_CELL_IN].value = (ConnectorA12.State()/2) - get_mb_data(MbRegisterOffsets::LOAD_CELL_OFFSET);
}

void CntrlNode1Io::write_pin_outputs () {
    /*Clearcore 1 specific 
       Outputs:
        IO2 - Heater Relay (relay)
        IO3 - Ice Spoof (relay)
        IO4 - Syrup Pump (relay)
        IO5 - Componenti DC motors (Hbridge) */
    ConnectorIO2.State(ptr_io_array[HEATER_RELAY_OUT].state);
    ConnectorIO3.State(ptr_io_array[ICE_RELAY_OUT].state);
    ConnectorIO4.State(ptr_io_array[SYRUP_RELAY_OUT].state);
    ConnectorIO5.State(ptr_io_array[COMPONENTI_MOT_OUT].value);    //PWM output
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
    set_mb_w_hreg_cb(MbRegisterOffsets::IGNORE_WEIGHT_CHECKS, &debug_ignore_weight_hreg_write);
}



CntrlNode1Io CcIoManager;