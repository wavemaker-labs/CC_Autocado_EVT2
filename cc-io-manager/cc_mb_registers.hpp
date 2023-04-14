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


// #define CC2_NUM_MODBUS_REGISTERS 25
// #define CC2_NODE_NUM 2

// static ModbusRegister cc2_modbus[CC2_NUM_MODBUS_REGISTERS] =
// {
//         {INPUT_REG, CC_NUMBER, CC2_NODE_NUM, nullptr}, 
//         {INPUT_REG, WD_COUNTER, 0, nullptr},
//         {INPUT_REG, SYSTEM_ERROR, 0, nullptr},
//         {INPUT_REG, FW_VERSION, 1, nullptr},
//         {INPUT_REG, E_STOP, 0, nullptr},

//         /*Keg Temp*/
//         {INPUT_REG, KEG_TEMP, 0, nullptr},

//         /*Tea milk Dispense*/
//         {HOLDING_REG, MTD_MILK_D_RQ, 0, nullptr},
//         {HOLDING_REG, MTD_TEA1_D_RQ, 0, nullptr},
//         {HOLDING_REG, MTD_TEA2_D_RQ, 0, nullptr},
//         {HOLDING_REG, MTD_DRINK_TOLER, 0, nullptr},
//         {HOLDING_REG, MTD_DRINK_TIMEOUT, 0, nullptr},
//         {INPUT_REG, MTD_DRINK_STATE, 0, nullptr},
//         {INPUT_REG, MTD_MILK_DISPENSED, 0, nullptr},
//         {INPUT_REG, MTD_TEA1_DISPENSED, 0, nullptr},
//         {INPUT_REG, MTD_TEA2_DISPENSED, 0, nullptr},
//         {INPUT_REG, MTD_TIME_OUT, 0, nullptr},
//         {INPUT_REG, MTD_LEAK, 0, nullptr},

//         /*Drink Pick Up Dispense*/
//         {HOLDING_REG, PICKUP_MOVE_RQ, 0, nullptr},
//         {INPUT_REG, PICKUP_STATE, 0, nullptr},
//         {INPUT_REG, PICKUP_TIMEOUT, 0, nullptr},
//         {INPUT_REG, PICKUP_MOVEDONE, 0, nullptr},
//         {INPUT_REG, PICKUP_DRINK_SENSOR, 0, nullptr},
//         {INPUT_REG, PICKUP_CMD_POS, 0, nullptr},
//         {INPUT_REG, PICKUP_RELAY_STATE, 0, nullptr},

//         /*Drink Ready Up Light*/
//         {HOLDING_REG, LIGHT_PICKUP, 0, nullptr}
// };

#endif