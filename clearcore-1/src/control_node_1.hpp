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
        ORIENTOR_POS_CMD,
        ORIENTOR_TORQUE_CMD,
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
        DB_ORIENTOR_TIMEOUT,
        DB_ORIENTOR_MOVE_TRQ,
        DB_ORIENTOR_POS_1,
        DB_ORIENTOR_POS_2,
        DB_ORIENTOR_POS_3,
        DB_ORIENTOR_POS_4,
        DB_RELSR_MOTOR_SPEED,
        DB_RELSR_MOTOR_ACCEL,
        DB_RELSR_CLOSE_POS,
        DB_RELSR_OPEN_POS,
        DB_RELSR_TIMEOUT

} MbRegisterOffsets;

#define CC1_NUM_MODBUS_REGISTERS 41
#define CC1_NODE_NUM 1
static ModbusRegister cc1_modbus[CC1_NUM_MODBUS_REGISTERS] =
{
    {INPUT_REG, CC_NUMBER, CC1_NODE_NUM, nullptr}, 
    {INPUT_REG, WD_COUNTER, 0, nullptr},
    {INPUT_REG, SYSTEM_ERROR, 0, nullptr},
    {INPUT_REG, FW_VERSION, FIRMWARE_VER, nullptr},
    {INPUT_REG, E_STOP, 0, nullptr},

    /*Hopper: Drum*/
    {HOLDING_REG, DRUM_MOVE_CMD, 0, nullptr}, 
    {INPUT_REG, DRUM_STATE, 0, nullptr},

    /*Hopper: Flat Convey*/
    {HOLDING_REG, FLAT_CON_SPEED, 0, nullptr}, 
    {HOLDING_REG, FLAT_CON_DIR, 0, nullptr}, 
    {INPUT_REG, FLAT_CON_STATE, 0, nullptr},
    {INPUT_REG, FLAT_CON_SENSOR_1, 0, nullptr},
    {INPUT_REG, FLAT_CON_SENSOR_2, 0, nullptr},

    /*Hopper: Inclined*/
    {HOLDING_REG, INCLINE_CON_CMD, 0, nullptr}, 
    {INPUT_REG, INCLINE_CON_STATE, 0, nullptr},

    /*Hopper: Orientor*/
    {HOLDING_REG, ORIENTOR_POS_CMD, 0, nullptr}, 
    {HOLDING_REG, ORIENTOR_TORQUE_CMD, 0, nullptr}, 
    {INPUT_REG, ORIENTOR_STATE, 0, nullptr},

    /*Hopper: Release*/
    {HOLDING_REG, RELEASER_POS_CMD, 0, nullptr}, 
    {INPUT_REG, RELEASER_STATE, 0, nullptr},
    {INPUT_REG, RELEASER_SENSOR, 0, nullptr},

    /*UI*/
    {INPUT_REG, UI_AVO_SIZE, 0, nullptr},
    {HOLDING_REG, UI_BUZZER_MODE_CMD, 0, nullptr},
    {HOLDING_REG, UI_ALERT_MODE_CMD, 0, nullptr},
    {HOLDING_REG, UI_DONE_STATUS_CMD, 0, nullptr},

    /*Debug&Tuning*/
    {HOLDING_REG, RESET_CC_RQ, 0, nullptr},
    {HOLDING_REG, DB_DRUM_TIMEOUT, 0, nullptr},
    {HOLDING_REG, DB_FLAT_CON_PULLBACK, 0, nullptr},
    {HOLDING_REG, DB_FLAT_CON_TIMEOUT, 0, nullptr},
    {HOLDING_REG, DB_ORIENTOR_MOTOR_SPEED, 0, nullptr},
    {HOLDING_REG, DB_ORIENTOR_MOTOR_ACCEL, 0, nullptr},
    {HOLDING_REG, DB_ORIENTOR_TIMEOUT, 0, nullptr},
    {HOLDING_REG, DB_ORIENTOR_MOVE_TRQ, 0, nullptr},
    {HOLDING_REG, DB_ORIENTOR_POS_1, 0, nullptr},
    {HOLDING_REG, DB_ORIENTOR_POS_2, 0, nullptr},
    {HOLDING_REG, DB_ORIENTOR_POS_3, 0, nullptr},
    {HOLDING_REG, DB_ORIENTOR_POS_4, 0, nullptr},
    {HOLDING_REG, DB_RELSR_MOTOR_SPEED, 0, nullptr},
    {HOLDING_REG, DB_RELSR_MOTOR_ACCEL, 0, nullptr},
    {HOLDING_REG, DB_RELSR_CLOSE_POS, 0, nullptr},
    {HOLDING_REG, DB_RELSR_OPEN_POS, 0, nullptr},
    {HOLDING_REG, DB_RELSR_TIMEOUT, 0, nullptr}
};

/*control node 1*/
typedef enum {
        DONE_LED_OUT = 0,
        ALERT_LED_OUT,
        DRUM_RUN_STOP_OUT, 
        DRUM_DIR_OUT,
        INCLINE_RUN_STOP_OUT,
        BUZZER,
        FLAT_CON_EDGE_SEN_IN,
        DRUM_SENS_IN,
        RELEASE_SENS_IN,
        FLAT_CON_LENGTH_SEN_IN,
        AVO_SIZE_AIN,
        ESTOP_IN,
        A12_NOT_USED
} BobaCcPins;

#define NUM_CC_IO_PIN 13
static PinIO cc1_io_pins[NUM_CC_IO_PIN] = {
    PinIO(DIGITAL_OUT, DONE_LED_OUT, nullptr),
    PinIO(DIGITAL_OUT, ALERT_LED_OUT, nullptr),
    PinIO(DIGITAL_OUT, DRUM_RUN_STOP_OUT, nullptr),
    PinIO(DIGITAL_OUT, DRUM_DIR_OUT, nullptr),
    PinIO(DIGITAL_OUT, INCLINE_RUN_STOP_OUT, nullptr),
    PinIO(HBRIDGE_OUT, BUZZER, nullptr),
    PinIO(SWITCH_SENSOR_IN, FLAT_CON_EDGE_SEN_IN, nullptr),
    PinIO(SWITCH_SENSOR_IN, DRUM_SENS_IN, nullptr),
    PinIO(SWITCH_SENSOR_IN, RELEASE_SENS_IN, nullptr),
    PinIO(SWITCH_SENSOR_IN, FLAT_CON_LENGTH_SEN_IN, nullptr),
    PinIO(ANALOG_IN, AVO_SIZE_AIN, nullptr),
    PinIO(SWITCH_SENSOR_IN, ESTOP_IN, nullptr),
    PinIO(ANALOG_IN, A12_NOT_USED, nullptr)
};

typedef enum
{
    FLAT_CON_MOT = 0,
    ORIENTOR_ONE_MOT,
    ORIENTOR_TWO_MOT,
    RELEASE_MOT        
} BobaCc1Motors;

#define CC1_NUM_MOTORS 4
static MotorIO cc1_motors[CC1_NUM_MOTORS] = {
    {200, 100, Connector::ConnectorModes::CPM_MODE_STEP_AND_DIR, &ConnectorM0, MotorDriver::MOVE_TARGET_REL_END_POSN},
    {800, 1000, Connector::ConnectorModes::CPM_MODE_STEP_AND_DIR, &ConnectorM1, MotorDriver::MOVE_TARGET_ABSOLUTE}, // velocity pulses per sec int accelerationLimit = 1000; // pulses per sec^2
    {800, 800, Connector::ConnectorModes::CPM_MODE_STEP_AND_DIR, &ConnectorM2, MotorDriver::MOVE_TARGET_ABSOLUTE},
    {800, 800, Connector::ConnectorModes::CPM_MODE_STEP_AND_DIR, &ConnectorM3, MotorDriver::MOVE_TARGET_ABSOLUTE}
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