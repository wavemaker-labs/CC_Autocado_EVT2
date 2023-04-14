 /**
 * @file io_manager.hpp
 * @copyright Copyright (c) 2023 Vebu Labs. All rights reserved.
 * @brief This library is an interface for the Clearcores .
 * @author Mike Lui
*/

#ifndef IO_MANAGER_HPP
#define IO_MANAGER_HPP

#include "ClearCore.h"
#include <cc_mb_registers.hpp>
#include <cc_io_pins.hpp>
#include <cc_motors.hpp>
#include <Ethernet.h>
#include <ModbusAPI.h>
#include <ModbusTCPTemplate.h>

#define ESTOP_ACTIVE PinStatus::LOW
#define ESTOP_RELEASED PinStatus::HIGH

class ModbusEthernet : public ModbusAPI<ModbusTCPTemplate<EthernetServer, EthernetClient>> {};

class IoManagerClass {

    public:
      void initialize_interfaces();
      void service_interfaces();
      void read_interfaces();
      void write_interfaces();

      /*get the ms counter from start of cycle*/
      uint32_t getSystemTime();

      /*functions to use to interface with modbus registers*/
      void set_mb_w_hreg_cb(uint16_t offset, uint16_t (*cbModbus)(TRegister* reg, uint16_t val));
      uint16_t get_mb_data(uint16_t offset);
      void set_mb_data(uint16_t offset, uint16_t data); 

      /*functions to use to interface pin inputs and outputs*/
      int16_t get_input(uint16_t pin);
      void set_pin_output_state(uint16_t pin, PinStatus output);
      void set_pin_output_value(uint16_t pin, int16_t output);
      
      /*Deprecated, use carefully*/
      int16_t * map_io_pin_input(uint16_t pin);
      /*Deprecated, use carefully*/
      uint16_t * get_mb_data_pointer(uint16_t offset);
      /*Deprecated, use carefully*/
      PinStatus * map_io_pin_output(uint16_t pin);


      void reset_watchdog();
      void kick_watchdog();

      MotorIO * get_ptr_motor(uint16_t mot_num);
      void get_motor_state(uint16_t mot_num, MotorIO * ptr_motor);
      void set_motor_state(uint16_t mot_num, MotorIO * ptr_motor);

    protected:
      MotorIO *ptr_motor_array;
      uint16_t motor_array_size;

      ModbusRegister *ptr_mb_reg_array;      
      uint16_t mb_reg_array_size;

      PinIO *ptr_io_array;
      uint16_t ptr_io_array_size;

      virtual void assign_modbus_registers() = 0;
      virtual void assign_io_pins() = 0;
      virtual void assign_motor_parameters() = 0;

      virtual void initialize_ethernet() = 0;

      virtual void read_pin_inputs() = 0;
      virtual void write_pin_outputs() = 0;

      virtual void update_system_mb() = 0;
      uint16_t m_watchdog_counter; 
     
    private:
      void setup_motor_parameters();
      void setup_modbus_registers(); 

      void read_motor_parameters();
      void read_modbus_registers();

      void write_motor_cmds();
      void write_modbus_registers();   
        
      uint32_t systemTime; 
      
          
      ModbusEthernet m_modbus_interface;  //ModbusTCP instance
};



#endif //IO_MANAGER_HPP



/*
Clearcore 1
Link: https://wavemakerlabs.atlassian.net/wiki/spaces/BOBA/pages/1742798849/Control+Architecture
Inputs: 
   IO1 - Cup Prepped Difuse
   A-9 - Componenti Dispense Switch (Digital In)
   A-10 - Boba temp sensor (Analog In)
   A-11 - E-Stop (Digital In)
   A-13 - Load Cell (Analog In)
Outputs:
   IO2 - Heater Relay
   IO3 - Ice Spoof
   IO4 - Syrup Pump
   IO5 - Componenti DC motors 
Motors:
   M0 - Cup Dispenser
   M1 - Cup Transport
   M2 - Boba Dispense
*/

/*
Clearcore 2
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
   IO0 - Solenoid unlock
   IO1 - Solenoid unlock
   IO3 - Ice Spoof
   IO5 - Pick up ready light
Motors:
   M0 - Drink Pick up
*/