 /**
 * @file control_node_1.hpp
 * @copyright Copyright (c) 2023 Vebu Labs. All rights reserved.
 * @brief This library is used to implement control 1 specific code.
 * @author Mike Lui
*/

#ifndef CONTROL_NODE_1
#define CONTROL_NODE_1

#include <io_manager.hpp>

#define CC1_FW_VERSION 6

#define DEFAULT_FLAT_CON_ACCEL 200
#define DEFAULT_FLAT_CON_SPEED 600

#define RELEASE_DEFAULT_MOTOR_SPEED 200
#define RELEASE_DEFAULT_MOTOR_ACCEL 600
#define RELEASE_DEFAULT_TIMEOUT_MS 10000 // 10 seconds
#define RELEASE_DEFAULT_POS1 400
#define RELEASE_DEFAULT_POS2 400
#define RELEASE_DEFAULT_POS3 370
#define RELEASE_DEFAULT_POS4 370
#define RELEASE_DEFAULT_POS5 250

#define ORIENTER_DEFAULT_MOTOR_SPEED 200
#define ORIENTER_DEFAULT_MOTOR_ACCEL 600
#define ORIENTER_DEFAULT_TIMEOUT_MS 10000 // 10 seconds
#define ORIENTER_DEFAULT_POS1 250
#define ORIENTER_DEFAULT_POS2 190
#define ORIENTER_DEFAULT_POS3 150
#define ORIENTER_DEFAULT_POS4 100
#define ORIENTER_DEFAULT_POS5 70

#define AVO_SIZE_MIN_ADC 500
#define AVO_SIZE_SMALL_ADC_LIMIT 2000
#define AVO_SIZE_SMALL 1
#define AVO_SIZE_MED_ADC_LIMIT 3500
#define AVO_SIZE_MED 2
#define AVO_SIZE_LARGE_ADC_LIMIT 2500
#define AVO_SIZE_LARGE 3
#define AVO_SIZE_XLARGE_ADC_LIMIT 1000
#define AVO_SIZE_XLARGE 4
#define AVO_SIZE_ERROR 90
/*Adc values: Small, med, large, xtra large */
//1800 3000 2240 875
//4.5V 7.5V 5.6V 2.2V 

typedef enum
{
        CC_NUMBER = 0,
        WD_COUNTER,
        SYSTEM_ERROR,
        FW_VERSION,
        E_STOP,
        DRUM_MOVE_CMD,
        DRUM_STATE,
        FLAT_CON_STEPS,
        FLAT_CON_DIR,
        FLAT_CON_STATE,
        FLAT_CON_EDGE_SENSOR,
        FLAT_CON_LENGTH_SENSOR,
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
        DB_FLAT_CON_ACCEL,
        DB_FLAT_CON_SPEED,
        DB_ORIENTOR_MOTOR_SPEED,
        DB_ORIENTOR_MOTOR_ACCEL,
        DB_ORIENTOR_TIMEOUT,
        DB_ORIENTOR_MOVE_TRQ,
        DB_ORIENTOR_POS_1,
        DB_ORIENTOR_POS_2,
        DB_ORIENTOR_POS_3,
        DB_ORIENTOR_POS_4,
        DB_ORIENTOR_POS_5,
        DB_RELSR_MOTOR_SPEED,
        DB_RELSR_MOTOR_ACCEL,
        DB_RELSR_TIMEOUT,
        DB_RELSR_POS_1,
        DB_RELSR_POS_2,
        DB_RELSR_POS_3,
        DB_RELSR_POS_4,
        DB_RELSR_POS_5
        

} MbRegisterOffsets;

#define CC1_NUM_MODBUS_REGISTERS 45
#define CC1_NODE_NUM 1
static ModbusRegister cc1_modbus[CC1_NUM_MODBUS_REGISTERS] =
{
    {INPUT_REG, CC_NUMBER, CC1_NODE_NUM, nullptr}, 
    {INPUT_REG, WD_COUNTER, 0, nullptr},
    {INPUT_REG, SYSTEM_ERROR, 0, nullptr},
    {INPUT_REG, FW_VERSION, CC1_FW_VERSION, nullptr},
    {INPUT_REG, E_STOP, 0, nullptr},

    /*Hopper: Drum*/
    {HOLDING_REG, DRUM_MOVE_CMD, 0, nullptr}, 
    {INPUT_REG, DRUM_STATE, 0, nullptr},

    /*Hopper: Flat Convey*/
    {HOLDING_REG, FLAT_CON_STEPS, 0, nullptr}, 
    {HOLDING_REG, FLAT_CON_DIR, 0, nullptr}, 
    {INPUT_REG, FLAT_CON_STATE, 0, nullptr},
    {INPUT_REG, FLAT_CON_EDGE_SENSOR, 0, nullptr},
    {INPUT_REG, FLAT_CON_LENGTH_SENSOR, 0, nullptr},

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
    {HOLDING_REG, DB_FLAT_CON_ACCEL, DEFAULT_FLAT_CON_ACCEL, nullptr},
    {HOLDING_REG, DB_FLAT_CON_SPEED, DEFAULT_FLAT_CON_SPEED, nullptr},
    {HOLDING_REG, DB_ORIENTOR_MOTOR_SPEED, ORIENTER_DEFAULT_MOTOR_SPEED, nullptr},
    {HOLDING_REG, DB_ORIENTOR_MOTOR_ACCEL, ORIENTER_DEFAULT_MOTOR_ACCEL, nullptr},
    {HOLDING_REG, DB_ORIENTOR_TIMEOUT, ORIENTER_DEFAULT_TIMEOUT_MS, nullptr},
    {HOLDING_REG, DB_ORIENTOR_MOVE_TRQ, 0, nullptr},
    {HOLDING_REG, DB_ORIENTOR_POS_1, ORIENTER_DEFAULT_POS1, nullptr},
    {HOLDING_REG, DB_ORIENTOR_POS_2, ORIENTER_DEFAULT_POS2, nullptr},
    {HOLDING_REG, DB_ORIENTOR_POS_3, ORIENTER_DEFAULT_POS3, nullptr},
    {HOLDING_REG, DB_ORIENTOR_POS_4, ORIENTER_DEFAULT_POS4, nullptr},
    {HOLDING_REG, DB_ORIENTOR_POS_5, ORIENTER_DEFAULT_POS5, nullptr},
    {HOLDING_REG, DB_RELSR_MOTOR_SPEED, RELEASE_DEFAULT_MOTOR_SPEED, nullptr},
    {HOLDING_REG, DB_RELSR_MOTOR_ACCEL, RELEASE_DEFAULT_MOTOR_ACCEL, nullptr},
    {HOLDING_REG, DB_RELSR_TIMEOUT, RELEASE_DEFAULT_TIMEOUT_MS, nullptr},
    {HOLDING_REG, DB_RELSR_POS_1, RELEASE_DEFAULT_POS1, nullptr},
    {HOLDING_REG, DB_RELSR_POS_2, RELEASE_DEFAULT_POS2, nullptr},
    {HOLDING_REG, DB_RELSR_POS_3, RELEASE_DEFAULT_POS3, nullptr},
    {HOLDING_REG, DB_RELSR_POS_4, RELEASE_DEFAULT_POS4, nullptr},
    {HOLDING_REG, DB_RELSR_POS_5, RELEASE_DEFAULT_POS5, nullptr}
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
} AutocadoCcPins;

#define NUM_CC_IO_PIN 13
static PinIO cc1_io_pins[NUM_CC_IO_PIN] = {
    PinIO(DIGITAL_OUT, DONE_LED_OUT, nullptr),
    PinIO(DIGITAL_OUT, ALERT_LED_OUT, nullptr),
    PinIO(DIGITAL_OUT, DRUM_RUN_STOP_OUT, nullptr),
    PinIO(DIGITAL_OUT, DRUM_DIR_OUT, nullptr),
    PinIO(DIGITAL_OUT, INCLINE_RUN_STOP_OUT, nullptr),
    PinIO(DIGITAL_OUT, BUZZER, nullptr),
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
} AutocadoCc1Motors;

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