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
#define PULSE_PER_ML_DIVISOR    10

uint16_t milk_pulse_cnt = 0;
uint16_t tea1_pulse_cnt = 0;
uint16_t tea2_pulse_cnt = 0;

bool b_ignore_weight = false;

// void milk_pulse() {milk_pulse_cnt++;}
// void tea1_pulse() {tea1_pulse_cnt++;}
// void tea2_pulse() {tea2_pulse_cnt++;}

/*one method to slow pulse: max flow meter freq is 235Hz, 4.3ms period. So only accept a rising pulse
if the io line has registered low for at least 2 ms. This takes advantage of the built in io filtering set at 2ms*/
// void milk_pulse() {if(ConnectorDI6.State() == PinStatus::LOW) milk_pulse_cnt++;}
// void tea1_pulse() {if(ConnectorDI7.State() == PinStatus::LOW) tea1_pulse_cnt++;}
// void tea2_pulse() {if(ConnectorDI8.State() == PinStatus::LOW) tea2_pulse_cnt++;}

/*second method to slow pulse: track system time and require at least 3 ms since last rising edge, 
this is slower and uses more ram and data, might be too slow even.*/
#define PULSE_MIN_PERIOD 3
uint32_t last_milk_pulse;
uint32_t last_tea1_pulse;
uint32_t last_tea2_pulse;
void milk_pulse() {
    if(millis() - last_milk_pulse > PULSE_MIN_PERIOD){
        milk_pulse_cnt++;
        last_milk_pulse = millis();
    }
} 
void tea1_pulse() {
    if(millis() - last_tea1_pulse > PULSE_MIN_PERIOD){
        tea1_pulse_cnt++;
        last_tea1_pulse = millis();
    }
}
void tea2_pulse() {
    if(millis() - last_tea2_pulse > PULSE_MIN_PERIOD){
        tea2_pulse_cnt++;
        last_tea2_pulse = millis();
    }
}

void CntrlNode2Io::reset_counters() {
    Serial.println("Counters");
    Serial.println(milk_pulse_cnt);
    Serial.println(tea1_pulse_cnt);
    Serial.println(tea2_pulse_cnt);
    milk_pulse_cnt = 0;
    tea1_pulse_cnt = 0;
    tea2_pulse_cnt = 0;
}

void CntrlNode2Io::assign_modbus_registers(){
    ptr_mb_reg_array = cc2_modbus;
    mb_reg_array_size = CC2_NUM_MODBUS_REGISTERS;
}

void CntrlNode2Io::assign_io_pins() {

     /*Clearcore 2 specific declarations
        Link: https://wavemakerlabs.atlassian.net/wiki/spaces/BOBA/pages/1742798849/Control+Architecture
        Inputs: 
        DI6 - Milk Flow
        DI7 - Tea 1 Flow
        DI8 - Tea 2 Flow
        A9 - Pick up Diffuse sensor (Digital In)
        A10 - Cup Top Fill Sensor (Digital In)
        A11 - E-Stop (Digital In)
        A12 - Kegarator Temp (Analog In)
        Outputs:
        IO0 - Locker Solenoid unlock
        IO1 - Milk Solenoid unlock
        IO2 - Tea1 Solenoid unlock
        IO3 - Tea2 Solenoid unlock
        IO5 - Pick up ready light
        Motors:
        M0 - Drink Pick up
    */
        
    ptr_io_array = cc2_io_pins;
    ptr_io_array_size = NUM_CC_IO_PIN;

    /*set up inputs*/
    ClearCore::AdcMgr.AdcResolution(adcResolution);
    ConnectorDI6.Mode(ClearCore::Connector::INPUT_DIGITAL);
    ConnectorDI6.FilterLength(5);
    ConnectorDI7.Mode(ClearCore::Connector::INPUT_DIGITAL);
    ConnectorDI7.FilterLength(5);
    ConnectorDI8.Mode(ClearCore::Connector::INPUT_DIGITAL);
    ConnectorDI8.FilterLength(5);
    ConnectorA9.Mode(ClearCore::Connector::INPUT_DIGITAL);
    ConnectorA10.Mode(ClearCore::Connector::INPUT_DIGITAL);
    ConnectorA11.Mode(ClearCore::Connector::INPUT_DIGITAL);
    ConnectorA12.Mode(ClearCore::Connector::INPUT_ANALOG);
    ConnectorA12.FilterTc(100, AdcManager::FILTER_UNIT_MS); 

    /*These are incremented for each pulse*/
    ClearCore::InputMgr.InterruptHandlerSet(digitalPinToInterrupt(CLEARCORE_PIN_DI6), milk_pulse, ClearCore::InputManager::RISING);
    ClearCore::InputMgr.InterruptHandlerSet(digitalPinToInterrupt(CLEARCORE_PIN_DI7), tea1_pulse, ClearCore::InputManager::RISING);
    ClearCore::InputMgr.InterruptHandlerSet(digitalPinToInterrupt(CLEARCORE_PIN_DI8), tea2_pulse, ClearCore::InputManager::RISING); 

    /*set up outputs*/    
    ConnectorIO0.Mode(ClearCore::Connector::OUTPUT_DIGITAL);
    ConnectorIO1.Mode(ClearCore::Connector::OUTPUT_DIGITAL);
    ConnectorIO2.Mode(ClearCore::Connector::OUTPUT_DIGITAL);
    ConnectorIO3.Mode(ClearCore::Connector::OUTPUT_DIGITAL);
    ConnectorIO5.Mode(ClearCore::Connector::OUTPUT_DIGITAL);

    /*Not used*/
    ConnectorIO4.Mode(ClearCore::Connector::INPUT_DIGITAL);

}


void CntrlNode2Io::assign_motor_parameters() {
    ptr_motor_array = cc2_motors;
    motor_array_size = CC2_NUM_MOTORS;
    
    /*Motors*/
    MotorMgr.MotorInputClocking(MotorManager::CLOCK_RATE_NORMAL);
    MotorMgr.MotorModeSet(MotorManager::MOTOR_M0M1,
                          Connector::CPM_MODE_STEP_AND_DIR);
    MotorMgr.MotorModeSet(MotorManager::MOTOR_M2M3,
                          Connector::CPM_MODE_STEP_AND_DIR);
}

void CntrlNode2Io::initialize_ethernet(){
    Ethernet.begin(mac, clear_core_ip); // start the Ethernet connection
}


int16_t temp_sensor_to_decacelcius(int16_t input)
{
    return (((input-809)*2500)/3287); // 4-20mA sensor in 503ohms
}

void CntrlNode2Io::read_pin_inputs() {
    /* Clearcore 2 specific 
        Inputs: 
        DI6 - Milk Flow
        DI7 - Tea 1 Flow
        DI8 - Tea 2 Flow
        A9 - Pick up Diffuse sensor (Digital In)
        A10 - Cup Top Fill Sensor (Digital In)
        A11 - E-Stop (Digital In)
        A12 - Kegarator Temp (Analog In) g*/

    ptr_io_array[MILK_FLOW_IN].value = (milk_pulse_cnt * PULSE_PER_ML_DIVISOR) / get_mb_data(MbRegisterOffsets::MILK_K_FACTOR); //convert to ml
    ptr_io_array[TEA1_FLOW_IN].value = (tea1_pulse_cnt * PULSE_PER_ML_DIVISOR) / get_mb_data(MbRegisterOffsets::TEA1_K_FACTOR); //convert to ml
    ptr_io_array[TEA2_FLOW_IN].value = (tea2_pulse_cnt * PULSE_PER_ML_DIVISOR) / get_mb_data(MbRegisterOffsets::TEA2_K_FACTOR); //convert to ml     

    ptr_io_array[PICKUP_SENS_IN].value = ConnectorA9.State();
    ptr_io_array[TOP_FILL_SENS_IN].value = ConnectorA10.State();
    ptr_io_array[ESTOP_IN].value = ConnectorA11.State();
    ptr_io_array[KEG_TEMP_AIN].value = temp_sensor_to_decacelcius(ConnectorA12.State());
}

void CntrlNode2Io::write_pin_outputs () {
    /*Clearcore 2 specific 
       Outputs:
        IO0 - Locker Solenoid unlock
        IO1 - Milk Solenoid unlock
        IO2 - Tea1 Solenoid unlock
        IO3 - Tea2 Solenoid unlock
        IO5 - Pick up ready light */
    ConnectorIO0.State(ptr_io_array[LOCKER_UNLOCK_OUT].state);
    ConnectorIO1.State(ptr_io_array[MILK_RELAY_OUT].state);
    ConnectorIO2.State(ptr_io_array[TEA1_RELAY_OUT].state);
    ConnectorIO3.State(ptr_io_array[TEA2_RELAY_OUT].state);
    ConnectorIO5.State(ptr_io_array[PICKUP_LIGHT_OUT].state);

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

uint16_t debug_ignore_weight_hreg_write(TRegister* reg, uint16_t val) {
    if (val == 1){
        b_ignore_weight = true;
    } else {
        b_ignore_weight = false;
    }
    return val;
}

void CntrlNode2Io::cc_mb_hooks() {
    set_mb_w_hreg_cb(MbRegisterOffsets::RESET_CC_RQ, &reset_hreg_write);    
    set_mb_w_hreg_cb(MbRegisterOffsets::IGNORE_WEIGHT_CHECKS, &debug_ignore_weight_hreg_write);
}

CntrlNode2Io CcIoManager;