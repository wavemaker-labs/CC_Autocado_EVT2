 /**
 * @file control_node_1.hpp
 * @copyright Copyright (c) 2023 Vebu Labs. All rights reserved.
 * @brief This library is used to implement control 1 specific code.
 * @author Mike Lui
*/

#ifndef CONTROL_NODE_1
#define CONTROL_NODE_1

#include <io_manager.hpp>

#define CC1_FW_VERSION 1

#define DEFAULT_FLAT_CON_ACCEL 200
#define DEFAULT_FLAT_CON_SPEED 600

#define RELEASE_DEFAULT_MOTOR_SPEED 200
#define RELEASE_DEFAULT_MOTOR_ACCEL 600


typedef enum
{
        CC_NUMBER = 0,
        WD_COUNTER,
        SYSTEM_ERROR,
        FW_VERSION,
        E_STOP,
        MOTOR_1_SPEED_1,
        MOTOR_1_SPEED_2,
        MOTOR_1_STATE,
        MOTOR_1_POS_1,
        MOTOR_1_POS_2,
        MOTOR_2_POS_1,
        MOTOR_2_POS_2,
        MOTOR_3_POS_1,
        MOTOR_3_POS_2,        
        RESET_CC_RQ = 50        

} MbRegisterOffsets;

#define CC1_NUM_MODBUS_REGISTERS 15
#define CC1_NODE_NUM 1
static ModbusRegister cc1_modbus[CC1_NUM_MODBUS_REGISTERS] =
{
    {INPUT_REG, CC_NUMBER, CC1_NODE_NUM, nullptr}, 
    {INPUT_REG, WD_COUNTER, 0, nullptr},
    {INPUT_REG, SYSTEM_ERROR, 0, nullptr},
    {INPUT_REG, FW_VERSION, CC1_FW_VERSION, nullptr},
    {INPUT_REG, E_STOP, 0, nullptr},

    /*Hopper: Drum*/
    {HOLDING_REG, MOTOR_1_SPEED_1, 0, nullptr}, 
    {HOLDING_REG, MOTOR_1_SPEED_2, 0, nullptr},
    {INPUT_REG, MOTOR_1_STATE, 0, nullptr},
    {INPUT_REG, MOTOR_1_POS_1, 0, nullptr},
    {INPUT_REG, MOTOR_1_POS_2, 0, nullptr},
    {INPUT_REG, MOTOR_2_POS_1, 0, nullptr},
    {INPUT_REG, MOTOR_2_POS_2, 0, nullptr},
    {INPUT_REG, MOTOR_3_POS_1, 0, nullptr},
    {INPUT_REG, MOTOR_3_POS_2, 0, nullptr},

    /*Debug&Tuning*/
    {HOLDING_REG, RESET_CC_RQ, 0, nullptr}
};

/*
    Clearcore 
    Link: 
    Inputs: 
    IO-0 - Rail Position 0 (Digital In)
    IO-1 - Rail Position 1 (Digital In)
    DI-6 - Cutting Push Button (Digital In)
    DI-7 - Load Cutter Push Button (Digital In)
    A-9 -  Open Push Button (Digital In)
    A-10 - Receive Push Button (Digital In)
    A-11 - Clamp Push Button (Digital In)
    A-12 - Squish Push Button (Digital In)
    Outputs:
    IO2 - 
    IO3 - 
    IO4 - Solenoid
    IO5 -  
/*control node 1*/

typedef enum {
        D0_RAIL_SW_0 = 0,
        D1_RAIL_SW_1,
        D2_NOT_USED,
        D3_NOT_USED,
        D4_CUT_SOLENOID,
        D5_NOT_USED,
        D6_CUT_BUTTON,
        D7_LOAD_CUT_BUTTON,
        D8_NOT_USED,
        A9_CLP_OPEN_BUTTON,
        A10_CLP_RECIEVE_BTN,
        A11_CLP_CLAMP_BTN,
        A12_CLP_SQUISH_BTN
} AutocadoCcPins;

#define NUM_CC_IO_PIN 13
static PinIO cc1_io_pins[NUM_CC_IO_PIN] = {
    PinIO(SWITCH_SENSOR_IN, D0_RAIL_SW_0, nullptr),
    PinIO(SWITCH_SENSOR_IN, D1_RAIL_SW_1, nullptr),
    PinIO(SWITCH_SENSOR_IN, D2_NOT_USED, nullptr),
    PinIO(SWITCH_SENSOR_IN, D3_NOT_USED, nullptr),
    PinIO(DIGITAL_OUT, D4_CUT_SOLENOID, nullptr),
    PinIO(SWITCH_SENSOR_IN, D5_NOT_USED, nullptr),
    PinIO(SWITCH_SENSOR_IN, D6_CUT_BUTTON, nullptr),
    PinIO(SWITCH_SENSOR_IN, D7_LOAD_CUT_BUTTON, nullptr),
    PinIO(SWITCH_SENSOR_IN, D8_NOT_USED, nullptr),
    PinIO(SWITCH_SENSOR_IN, A9_CLP_OPEN_BUTTON, nullptr),
    PinIO(SWITCH_SENSOR_IN, A10_CLP_RECIEVE_BTN, nullptr),
    PinIO(SWITCH_SENSOR_IN, A11_CLP_CLAMP_BTN, nullptr),
    PinIO(SWITCH_SENSOR_IN, A12_CLP_SQUISH_BTN, nullptr)
};

typedef enum
{
    FLAT_CON_MOT = 0,
    ORIENTOR_ONE_MOT,
    ORIENTOR_TWO_MOT,
    RELEASE_MOT        
} AutocadoCc1Motors;

#define CC1_NUM_MOTORS 1
static MotorIO cc1_motors[CC1_NUM_MOTORS] = {
    {200, 100, Connector::ConnectorModes::CPM_MODE_STEP_AND_DIR, &ConnectorM0, MotorDriver::MOVE_TARGET_REL_END_POSN}
};


typedef enum {
        STEPPER_1 = 0,
        STEPPER_2,
        STEPPER_3
} AutocadoCcSteppers;

#define CC_NUM_DAISY_STEP_MOTORS 3
static Cc5160Stepper cc_step_mots[CC_NUM_DAISY_STEP_MOTORS] = {
    {STEPPER_1, CC_NUM_DAISY_STEP_MOTORS - 1, 0},
    {STEPPER_2, CC_NUM_DAISY_STEP_MOTORS - 1, 0},
    {STEPPER_3, CC_NUM_DAISY_STEP_MOTORS - 1, 0},
};

static const int32_t * Cc5160StepperCfg[CC_NUM_DAISY_STEP_MOTORS] = { 
    tmc5160_defaultRegisterResetState,
    tmc5160_defaultRegisterResetState,
    tmc5160_defaultRegisterResetState,
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