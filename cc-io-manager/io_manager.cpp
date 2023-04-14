 /**
 * @file io_interface.cpp
 * @copyright Copyright (c) 2023 Vebu Labs. All rights reserved.
 * @brief This library is an interface the subsystem FSMs and is specific to each clearcore
 * @author Mike Lui
*/

#include "ClearCore.h"
#include <io_manager.hpp>


bool cbConn(IPAddress ip) {
  Serial.println(ip);
  return true;
}

void IoManagerClass::setup_modbus_registers() {

    assign_modbus_registers();

    /*Add registers from array above by offset, default data, and register type. Add callback if provided, 
    otherwise callbacks from FSMs can be called when FSMs are set up*/
    for(int i = 0; i < mb_reg_array_size; i++) {
        if (ptr_mb_reg_array[i].register_type == INPUT_REG) {
            m_modbus_interface.addIreg(ptr_mb_reg_array[i].offset, ptr_mb_reg_array[i].data, 1);
        }else if (ptr_mb_reg_array[i].register_type == HOLDING_REG) {
            m_modbus_interface.addHreg(ptr_mb_reg_array[i].offset, ptr_mb_reg_array[i].data, 1);
            if(ptr_mb_reg_array[i].cbModbus != nullptr){
                m_modbus_interface.onSetHreg(ptr_mb_reg_array[i].offset, ptr_mb_reg_array[i].cbModbus, 1);
            }
        }        
    }
}


void IoManagerClass::setup_motor_parameters() {

    assign_motor_parameters();

    if(ptr_motor_array != nullptr){
        for (int i = 0; i < motor_array_size; i++) {
            ptr_motor_array[i].ptr_connector->HlfbMode(ptr_motor_array[i].hlfb_mode);
            ptr_motor_array[i].ptr_connector->HlfbCarrier(ptr_motor_array[i].hlfb_mode_freq);
            ptr_motor_array[i].ptr_connector->VelMax(ptr_motor_array[i].velocity_limit);
            ptr_motor_array[i].ptr_connector->AccelMax(ptr_motor_array[i].accel_limit);
            ptr_motor_array[i].ptr_connector->EnableRequest(ptr_motor_array[i].enable);
            if(ptr_motor_array[i].motor_mode == Connector::ConnectorModes::CPM_MODE_A_DIRECT_B_DIRECT){
                ptr_motor_array[i].ptr_connector->MotorInAState(false);
                ptr_motor_array[i].ptr_connector->MotorInBState(false);
            }            
        }
    }
}



void IoManagerClass::read_motor_parameters() {
    /*Get motorfeedback*/
    if(ptr_motor_array != nullptr) {
        for (int i = 0; i < motor_array_size; i++) {
            ptr_motor_array[i].hlfb_state = ptr_motor_array[i].ptr_connector->HlfbState();
            ptr_motor_array[i].steps_complete = ptr_motor_array[i].ptr_connector->StepsComplete();            
            // ptr_motor_array[i].statusreg = ptr_motor_array[i].ptr_connector->StatusReg();
        }
    }
}

void IoManagerClass::read_modbus_registers() {
    /*Update H-reg shadow registers*/
    for(int i = 0; i < mb_reg_array_size; i++) {
        if (ptr_mb_reg_array[i].register_type == HOLDING_REG) {
            ptr_mb_reg_array[i].data = m_modbus_interface.Hreg(ptr_mb_reg_array[i].offset);
        }     
    }
}


void IoManagerClass::write_motor_cmds() {
    /*Command  motors*/
    if(ptr_motor_array != nullptr) {
        for (int i = 0; i < motor_array_size; i++) {
            /* if enabled then move based on type*/
            if(ptr_motor_array[i].new_move_commanded && ptr_motor_array[i].ptr_connector->EnableRequest()) {
                if(ptr_motor_array[i].motor_mode == Connector::ConnectorModes::CPM_MODE_STEP_AND_DIR){
                    Serial.println("Motor will move");
                    Serial.println(i);
                    ptr_motor_array[i].ptr_connector->Move(ptr_motor_array[i].distance, MotorDriver::MOVE_TARGET_ABSOLUTE);                    
                }else if(ptr_motor_array[i].motor_mode == Connector::ConnectorModes::CPM_MODE_A_DIRECT_B_DIRECT){
                     switch (ptr_motor_array[i].position) {
                        case 1:
                            ptr_motor_array[i].ptr_connector->MotorInAState(false);
                            ptr_motor_array[i].ptr_connector->MotorInBState(false);
                            break;
                        case 2:
                            ptr_motor_array[i].ptr_connector->MotorInAState(true);
                            ptr_motor_array[i].ptr_connector->MotorInBState(false);
                            break;
                        case 3:
                            ptr_motor_array[i].ptr_connector->MotorInAState(false);
                            ptr_motor_array[i].ptr_connector->MotorInBState(true);
                            break;
                        case 4:
                            ptr_motor_array[i].ptr_connector->MotorInAState(true);
                            ptr_motor_array[i].ptr_connector->MotorInBState(true);
                            break;
                        default:
                            // If this case is reached then an incorrect positionNum was entered
                            break;
                    }                    
                }
                ptr_motor_array[i].new_move_commanded = false;
            }

            /*enable request*/
            if(ptr_motor_array[i].enable && !ptr_motor_array[i].ptr_connector->EnableRequest()) {
                ptr_motor_array[i].ptr_connector->EnableRequest(ptr_motor_array[i].enable);
            }

            /*disable request*/
            if(!ptr_motor_array[i].enable && ptr_motor_array[i].ptr_connector->EnableRequest()) {
                ptr_motor_array[i].ptr_connector->EnableRequest(ptr_motor_array[i].enable);
            }

            /*if abrupt stop*/
            if(ptr_motor_array[i].stop_abrupt)
            {
                ptr_motor_array[i].ptr_connector->MoveStopAbrupt();
            }
        }
    }
    
}

void IoManagerClass::write_modbus_registers() {
    /*Write all input registers in array, data in mb array should be updated in FSM calls*/
    for(int i = 0; i < mb_reg_array_size; i++) {
        if (ptr_mb_reg_array[i].register_type == INPUT_REG) {
            m_modbus_interface.Ireg(ptr_mb_reg_array[i].offset, ptr_mb_reg_array[i].data);
        }     
    }    
}


void IoManagerClass::initialize_interfaces() {
    /*Modbus*/
    initialize_ethernet();

    m_modbus_interface.server();        // Act as Modbus TCP server
    m_modbus_interface.onConnect(cbConn);

    setup_modbus_registers();    
    assign_io_pins();
    setup_motor_parameters();
}

void IoManagerClass::service_interfaces() {
    m_modbus_interface.task();    // Service Server Modbus TCP queries
    systemTime = millis();
}


void IoManagerClass::read_interfaces() {   
    read_pin_inputs();
    read_motor_parameters();
    read_modbus_registers();
}

void IoManagerClass::write_interfaces() {
    write_pin_outputs();
    write_motor_cmds();
    write_modbus_registers();
}


void IoManagerClass::set_mb_w_hreg_cb(uint16_t offset, uint16_t (*cbModbus)(TRegister* reg, uint16_t val)) { 
    m_modbus_interface.onSetHreg(offset, cbModbus, 1);
}

uint16_t * IoManagerClass::get_mb_data_pointer(uint16_t offset) {
    uint16_t * ptr_to_mb_data = nullptr;
    
    for(int i = 0; i < mb_reg_array_size; i++) {
        if (ptr_mb_reg_array[i].offset == offset) {
            ptr_to_mb_data = &ptr_mb_reg_array[i].data;
        }     
    } 
    return ptr_to_mb_data;
 }
 
 uint32_t IoManagerClass::getSystemTime()
 {
    return systemTime;
 }

 int16_t * IoManagerClass::map_io_pin_input(uint16_t pin_num)
 {
    int16_t * ptr_to_input_value = nullptr;
    for(int i = 0; i < ptr_io_array_size; i++) {
        if (ptr_io_array[i].pin_num == pin_num && 
            (ptr_io_array[i].pin_mode == SWITCH_SENSOR_IN || 
             ptr_io_array[i].pin_mode == ANALOG_IN ||
             ptr_io_array[i].pin_mode == INTERRUPT_IN)) {
            ptr_to_input_value = &ptr_io_array[i].value;
        }     
    } 
    return ptr_to_input_value;

 }

PinStatus * IoManagerClass::map_io_pin_output(uint16_t pin_num)
{
    PinStatus * ptr_to_output_state = nullptr;
    for(int i = 0; i < ptr_io_array_size; i++) {
        if (ptr_io_array[i].pin_num == pin_num && 
            (ptr_io_array[i].pin_mode == DIGITAL_OUT || 
             ptr_io_array[i].pin_mode == HBRIDGE_OUT ||
             ptr_io_array[i].pin_mode == ANALOG_OUT)) {
            ptr_to_output_state = &ptr_io_array[i].state;
        }     
    } 
    return ptr_to_output_state;
}

int16_t IoManagerClass::get_input(uint16_t pin)
{
    int16_t input = -99;
    if (pin < ptr_io_array_size && 
        (ptr_io_array[pin].pin_num == pin && 
        (ptr_io_array[pin].pin_mode == SWITCH_SENSOR_IN || 
        ptr_io_array[pin].pin_mode == ANALOG_IN ||
        ptr_io_array[pin].pin_mode == INTERRUPT_IN))){
                input = ptr_io_array[pin].value;
    }    

    return input;
}

void IoManagerClass::set_pin_output_state(uint16_t pin, PinStatus output)
{
    if (pin < ptr_io_array_size && 
        (ptr_io_array[pin].pin_num == pin && 
        (ptr_io_array[pin].pin_mode == DIGITAL_OUT))){
                ptr_io_array[pin].state = output;
    }    
}

void IoManagerClass::set_pin_output_value(uint16_t pin, int16_t output)
{
    if (pin < ptr_io_array_size && 
        (ptr_io_array[pin].pin_num == pin && 
        (ptr_io_array[pin].pin_mode == HBRIDGE_OUT ||
        ptr_io_array[pin].pin_mode == ANALOG_OUT))){
                ptr_io_array[pin].value = output;
    }    
}

uint16_t IoManagerClass::get_mb_data(uint16_t offset)
{
    uint16_t data = 0;
    for(int i = 0; i < mb_reg_array_size; i++) {
        if (ptr_mb_reg_array[i].offset == offset && 
            ptr_mb_reg_array[i].register_type == HOLDING_REG) {
            data = ptr_mb_reg_array[i].data;
        }     
    } 
    return data;
}

void IoManagerClass::set_mb_data(uint16_t offset, uint16_t data)
{
    for(int i = 0; i < mb_reg_array_size; i++) {
        if (ptr_mb_reg_array[i].offset == offset && 
            ptr_mb_reg_array[i].register_type == INPUT_REG) {
            ptr_mb_reg_array[i].data = data;
        }     
    } 
}   

MotorIO * IoManagerClass::get_ptr_motor(uint16_t mot_num)
{
    MotorIO * ptr_motor = nullptr;
    if(mot_num < motor_array_size) {
        ptr_motor = &ptr_motor_array[mot_num];
    }
    return ptr_motor;
}

void IoManagerClass::get_motor_state(uint16_t mot_num, MotorIO * ptr_motor){

}

void IoManagerClass::set_motor_state(uint16_t mot_num, MotorIO * ptr_motor){
    
}


void IoManagerClass::reset_watchdog() {
    m_watchdog_counter = 0;
}

void IoManagerClass::kick_watchdog() {
    m_watchdog_counter++;
}


// IoManagerClass CcIoManager;

