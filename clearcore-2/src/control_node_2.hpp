 /**
 * @file control_node_2.hpp
 * @copyright Copyright (c) 2023 Vebu Labs. All rights reserved.
 * @brief This library is used to implement control 2 specific code.
 * @author Mike Lui
*/

#ifndef CONTROL_NODE_2
#define CONTROL_NODE_2

#include <io_manager.hpp>

typedef enum
{
    CC_NUMBER = 0,
    WD_COUNTER,
    SYSTEM_ERROR,
    FW_VERSION,
    E_STOP,
    CUP_DISP_RQ,
    CUP_DISP_STATE,
    CUP_DISP_CNT,
    CUP_T_MOVE_RQ,
    CUP_T_STATE,
    CUP_T_TIMEOUT,
    CUP_T_MOVEDONE,
    CUP_T_CMD_POS,
    CUP_T_LOADCELL,
    BOBA_D_RQ,
    BOBA_D_STATE,
    BOBA_D_DISPENSED,
    BOBA_D_HEATER_ST,
    BOBA_D_TEMP,
    ICE_D_RQ,
    ICE_D_STATE,
    ICE_D_DISPENSED,
    SYR_D_RQ,
    SYR_D_STATE,
    SYR_D_DISPENSED,
    KEG_TEMP,
    MTD_MILK_D_RQ,
    MTD_TEA1_D_RQ,
    MTD_TEA2_D_RQ,
    MTD_DRINK_TOLER,
    MTD_DRINK_TIMEOUT,
    MTD_DRINK_STATE,
    MTD_MILK_DISPENSED,
    MTD_TEA1_DISPENSED,
    MTD_TEA2_DISPENSED,
    MTD_TIME_OUT,
    MTD_LEAK,
    PICKUP_MOVE_RQ,
    PICKUP_STATE,
    PICKUP_TIMEOUT,
    PICKUP_MOVEDONE,
    PICKUP_DRINK_SENSOR,
    PICKUP_CMD_POS,
    PICKUP_RELAY_STATE,
    LIGHT_PICKUP,
    RESET_CC_RQ = 50,
    IGNORE_WEIGHT_CHECKS,
    MILK_K_FACTOR,
    TEA1_K_FACTOR,
    TEA2_K_FACTOR,
    LOAD_CELL_OFFSET

} MbRegisterOffsets;


#define CC2_NUM_MODBUS_REGISTERS 30
#define CC2_NODE_NUM 2


static ModbusRegister cc2_modbus[CC2_NUM_MODBUS_REGISTERS] =
{
    {INPUT_REG, CC_NUMBER, CC2_NODE_NUM, nullptr}, 
    {INPUT_REG, WD_COUNTER, 0, nullptr},
    {INPUT_REG, SYSTEM_ERROR, 0, nullptr},
    {INPUT_REG, FW_VERSION, 1, nullptr},
    {INPUT_REG, E_STOP, 0, nullptr},

    /*Keg Temp*/
    {INPUT_REG, KEG_TEMP, 0, nullptr},

    /*Tea milk Dispense*/
    {HOLDING_REG, MTD_MILK_D_RQ, 0, nullptr},
    {HOLDING_REG, MTD_TEA1_D_RQ, 0, nullptr},
    {HOLDING_REG, MTD_TEA2_D_RQ, 0, nullptr},
    {HOLDING_REG, MTD_DRINK_TOLER, 0, nullptr},
    {HOLDING_REG, MTD_DRINK_TIMEOUT, 0, nullptr},
    {INPUT_REG, MTD_DRINK_STATE, 0, nullptr},
    {INPUT_REG, MTD_MILK_DISPENSED, 0, nullptr},
    {INPUT_REG, MTD_TEA1_DISPENSED, 0, nullptr},
    {INPUT_REG, MTD_TEA2_DISPENSED, 0, nullptr},
    {INPUT_REG, MTD_TIME_OUT, 0, nullptr},
    {INPUT_REG, MTD_LEAK, 0, nullptr},

    /*Drink Pick Up Dispense*/
    {HOLDING_REG, PICKUP_MOVE_RQ, 0, nullptr},
    {INPUT_REG, PICKUP_STATE, 0, nullptr},
    {INPUT_REG, PICKUP_TIMEOUT, 0, nullptr},
    {INPUT_REG, PICKUP_MOVEDONE, 0, nullptr},
    {INPUT_REG, PICKUP_DRINK_SENSOR, 0, nullptr},
    {INPUT_REG, PICKUP_CMD_POS, 0, nullptr},
    {INPUT_REG, PICKUP_RELAY_STATE, 0, nullptr},

    /*Drink Ready Up Light*/
    {HOLDING_REG, LIGHT_PICKUP, 0, nullptr},

    /*Reset or stop*/
    {HOLDING_REG, RESET_CC_RQ, 0, nullptr},
    {HOLDING_REG, IGNORE_WEIGHT_CHECKS, 0, nullptr},

    /*flow meter tuning factors*/
    {HOLDING_REG, MILK_K_FACTOR, 60, nullptr},
    {HOLDING_REG, TEA1_K_FACTOR, 60, nullptr},
    {HOLDING_REG, TEA2_K_FACTOR, 60, nullptr}
};


/*control node 2*/
typedef enum {
        LOCKER_UNLOCK_OUT = 0,
        MILK_RELAY_OUT,
        TEA1_RELAY_OUT, 
        TEA2_RELAY_OUT,
        IO4_NOT_USED_IN,
        PICKUP_LIGHT_OUT,
        MILK_FLOW_IN,
        TEA1_FLOW_IN,
        TEA2_FLOW_IN,
        PICKUP_SENS_IN,
        TOP_FILL_SENS_IN,
        ESTOP_IN,
        KEG_TEMP_AIN
} BobaCcPins;


#define NUM_CC_IO_PIN 13
static PinIO cc2_io_pins[NUM_CC_IO_PIN] = {
    PinIO(DIGITAL_OUT, LOCKER_UNLOCK_OUT, nullptr),
    PinIO(DIGITAL_OUT, MILK_RELAY_OUT, nullptr),
    PinIO(DIGITAL_OUT, TEA1_RELAY_OUT, nullptr),
    PinIO(DIGITAL_OUT, TEA2_RELAY_OUT, nullptr),
    PinIO(SWITCH_SENSOR_IN, IO4_NOT_USED_IN, nullptr),
    PinIO(DIGITAL_OUT, PICKUP_LIGHT_OUT, nullptr),
    PinIO(SWITCH_SENSOR_IN, MILK_FLOW_IN, nullptr),
    PinIO(SWITCH_SENSOR_IN, TEA1_FLOW_IN, nullptr),
    PinIO(SWITCH_SENSOR_IN, TEA2_FLOW_IN, nullptr),
    PinIO(SWITCH_SENSOR_IN, PICKUP_SENS_IN, nullptr),
    PinIO(SWITCH_SENSOR_IN, TOP_FILL_SENS_IN, nullptr),
    PinIO(SWITCH_SENSOR_IN, ESTOP_IN, nullptr),
    PinIO(ANALOG_IN, KEG_TEMP_AIN, nullptr)
};


typedef enum
{
    DRINK_PICKUP = 0      
} BobaCc2Motors;


#define CC2_NUM_MOTORS 1
static MotorIO cc2_motors[CC2_NUM_MOTORS] = {
    {1000, 400, Connector::ConnectorModes::CPM_MODE_STEP_AND_DIR, &ConnectorM1, MotorDriver::MOVE_TARGET_ABSOLUTE}
};

class CntrlNode2Io : public IoManagerClass {
    public:
        void update_system_mb() override;
        void cc_mb_hooks();
        void reset_counters();
        bool ignore_weight();
    protected:
        void assign_modbus_registers() override;
        void assign_io_pins() override;
        void assign_motor_parameters() override;

        void initialize_ethernet() override;

        void read_pin_inputs() override;
        void write_pin_outputs() override;           
};

extern CntrlNode2Io CcIoManager;

#endif //CONTROL_NODE_2