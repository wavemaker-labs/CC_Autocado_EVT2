 /**
 * @file control_node_1.hpp
 * @copyright Copyright (c) 2023 Vebu Labs. All rights reserved.
 * @brief This library is used to implement control 1 specific code.
 * @author Mike Lui
*/

#ifndef CONTROL_NODE_1
#define CONTROL_NODE_1

#include <io_manager.hpp>

#define FIRMWARE_VER 1

typedef enum
{
        CC_NUMBER = 0,
        WD_COUNTER,
        SYSTEM_ERROR,
        FW_VERSION,
        E_STOP,
        DRUM_MOVE_CMD,
        DRUM_STATE,
        FLAT_CON_SPEED,
        FLAT_CON_DIR,
        FLAT_CON_STATE,
        FLAT_CON_SENSOR_1,
        FLAT_CON_SENSOR_2,
        INCLINE_CON_CMD,
        INCLINE_CON_STATE,
        ORIENTOR_CMD,
        ORIENTOR_STATE,
        RELEASER_POS_CMD,
        RELEASER_STATE,
        RELEASER_SENSOR,
        UI_AVO_SIZE,
        UI_BUZZER_MODE_CMD,
        UI_ALERT_MODE_CMD,
        UI_DONE_STATUS_CMD,
        RESET_CC_RQ = 50,
        DB_DRUM_TIMEOUT,
        DB_FLAT_CON_PULLBACK,
        DB_FLAT_CON_TIMEOUT,
        DB_ORIENTOR_MOTOR_SPEED,
        DB_ORIENTOR_MOTOR_ACCEL,
        DB_ORIENTOR_MOVE_TRQ,
        DB_RELSR_MOTOR_SPEED,
        DB_RELSR_MOTOR_ACCEL,
        DB_RELSR_CLOSE_POS,
        DB_RELSR_OPEN_POS

} MbRegisterOffsets;

#define CC1_NUM_MODBUS_REGISTERS 28
#define CC1_NODE_NUM 1
static ModbusRegister cc1_modbus[CC1_NUM_MODBUS_REGISTERS] =
{
    {INPUT_REG, CC_NUMBER, CC1_NODE_NUM, nullptr}, 
    {INPUT_REG, WD_COUNTER, 0, nullptr},
    {INPUT_REG, SYSTEM_ERROR, 0, nullptr},
    {INPUT_REG, FW_VERSION, FIRMWARE_VER, nullptr},
    {INPUT_REG, E_STOP, 0, nullptr},

    /*Cup Dispenser*/
    {HOLDING_REG, CUP_DISP_RQ, 0, nullptr}, 
    {INPUT_REG, CUP_DISP_STATE, 0, nullptr},
    {INPUT_REG, CUP_DISP_CNT, 0, nullptr},

    /*Cup Transport*/
    {HOLDING_REG, CUP_T_MOVE_RQ, 0, nullptr}, 
    {INPUT_REG, CUP_T_STATE, 0, nullptr},
    {INPUT_REG, CUP_T_TIMEOUT, 0, nullptr},
    {INPUT_REG, CUP_T_MOVEDONE, 0, nullptr},
    {INPUT_REG, CUP_T_CMD_POS, 0, nullptr},
    {INPUT_REG, CUP_T_LOADCELL, 0, nullptr},

    /*Boba Dispense*/
    {HOLDING_REG, BOBA_D_RQ, 0, nullptr}, 
    {INPUT_REG, BOBA_D_STATE, 0, nullptr},
    {INPUT_REG, BOBA_D_DISPENSED, 0, nullptr},
    {INPUT_REG, BOBA_D_OO_RANGE_FLAG, 0, nullptr},
    {INPUT_REG, BOBA_D_TEMP, 0, nullptr},

    /*Ice Dispense*/
    {HOLDING_REG, ICE_D_RQ, 0, nullptr}, 
    {INPUT_REG, ICE_D_STATE, 0, nullptr},
    {INPUT_REG, ICE_D_DISPENSED, 0, nullptr},

    /*Syrup Dispense*/
    {HOLDING_REG, SYR_D_RQ, 0, nullptr}, 
    {INPUT_REG, SYR_D_STATE, 0, nullptr},
    {INPUT_REG, SYR_D_DISPENSED, 0, nullptr},

    /*Reset or stop*/
    {HOLDING_REG, RESET_CC_RQ, 0, nullptr},
    {HOLDING_REG, IGNORE_WEIGHT_CHECKS, 0, nullptr},

    /*tuning*/
    {HOLDING_REG, LOAD_CELL_OFFSET, 0, nullptr}
};

/*control node 1*/
typedef enum {
        IO0_NOT_USED_IN = 0,
        CUP_PREPPED_IN,
        HEATER_RELAY_OUT, 
        ICE_RELAY_OUT,
        SYRUP_RELAY_OUT,
        COMPONENTI_MOT_OUT,
        DI6_NOT_USED_IN,
        DI7_NOT_USED_IN,
        DI8_NOT_USED_IN,
        COMPONENTI_SW_IN,
        BOBA_TEMP_AIN,
        ESTOP_IN,
        LOAD_CELL_IN
} BobaCcPins;

#define NUM_CC_IO_PIN 13
static PinIO cc1_io_pins[NUM_CC_IO_PIN] = {
    PinIO(SWITCH_SENSOR_IN, IO0_NOT_USED_IN, nullptr),
    PinIO(SWITCH_SENSOR_IN, CUP_PREPPED_IN, nullptr),
    PinIO(DIGITAL_OUT, HEATER_RELAY_OUT, nullptr),
    PinIO(DIGITAL_OUT, ICE_RELAY_OUT, nullptr),
    PinIO(DIGITAL_OUT, SYRUP_RELAY_OUT, nullptr),
    PinIO(HBRIDGE_OUT, COMPONENTI_MOT_OUT, nullptr),
    PinIO(SWITCH_SENSOR_IN, DI6_NOT_USED_IN, nullptr),
    PinIO(SWITCH_SENSOR_IN, DI7_NOT_USED_IN, nullptr),
    PinIO(SWITCH_SENSOR_IN, DI8_NOT_USED_IN, nullptr),
    PinIO(SWITCH_SENSOR_IN, COMPONENTI_SW_IN, nullptr),
    PinIO(ANALOG_IN, BOBA_TEMP_AIN, nullptr),
    PinIO(SWITCH_SENSOR_IN, ESTOP_IN, nullptr),
    PinIO(ANALOG_IN, LOAD_CELL_IN, nullptr)
};

typedef enum
{
    CUP_DISP = 0,
    CUP_TRANS,
    BOBA_DISP        
} BobaCc1Motors;

#define CC1_NUM_MOTORS 3
static MotorIO cc1_motors[CC1_NUM_MOTORS] = {
    {45, 20, Connector::ConnectorModes::CPM_MODE_STEP_AND_DIR, &ConnectorM0, MotorDriver::MOVE_TARGET_REL_END_POSN},
    {800, 1000, Connector::ConnectorModes::CPM_MODE_STEP_AND_DIR, &ConnectorM1, MotorDriver::MOVE_TARGET_ABSOLUTE}, // velocity pulses per sec int accelerationLimit = 1000; // pulses per sec^2
    {800, 800, Connector::ConnectorModes::CPM_MODE_A_DIRECT_B_DIRECT, &ConnectorM3}
};

class CntrlNode1Io : public IoManagerClass {
    public:
        void update_system_mb() override;
        void cc_mb_hooks();
        
        bool ignore_weight();

    protected:
        void assign_modbus_registers() override;
        void assign_io_pins() override;
        void assign_motor_parameters() override;

        void initialize_ethernet() override;

        void read_pin_inputs() override;
        void write_pin_outputs() override;           
};

extern CntrlNode1Io CcIoManager;

#endif // CONTROL_NODE_1