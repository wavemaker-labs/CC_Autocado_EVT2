 /**
 * @file cc_mb_registers.hpp
 * @copyright Copyright (c) 2023 Vebu Labs. All rights reserved.
 * @brief This library hold modbus offsets
 * @author Mike Lui
*/

#ifndef CC_MODBUS_REGS_HPP
#define CC_MODBUS_REGS_HPP

#include <Ethernet.h>
#include <ModbusAPI.h>
#include <ModbusTCPTemplate.h>

typedef enum {
        HOLDING_REG,
        INPUT_REG
} MbRegisterType;

struct ModbusRegister
{
   MbRegisterType register_type;
   uint16_t offset;
   uint16_t data;
   uint16_t (*cbModbus)(TRegister* reg, uint16_t val);
};



#endif