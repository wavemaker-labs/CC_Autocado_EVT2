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
        D8_CLP_SQUISH_BTN,
        A9_CLP_OPEN_BUTTON,
        A10_CLP_RECIEVE_BTN,
        A11_CLP_CLAMP_BTN,
        A12_NOT_USED
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
    PinIO(SWITCH_SENSOR_IN, D8_CLP_SQUISH_BTN, nullptr),
    PinIO(SWITCH_SENSOR_IN, A9_CLP_OPEN_BUTTON, nullptr),
    PinIO(SWITCH_SENSOR_IN, A10_CLP_RECIEVE_BTN, nullptr),
    PinIO(SWITCH_SENSOR_IN, A11_CLP_CLAMP_BTN, nullptr),
    PinIO(SWITCH_SENSOR_IN, A12_NOT_USED, nullptr)
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
        STEPPER_CUTTER = 0,
        STEPPER_CLAMP_LT,
        STEPPER_CLAMP_LB,
        STEPPER_CLAMP_RT,
        STEPPER_CLAMP_RB,
        STEPPER_RAIL,
} AutocadoCcSteppers;

#define CC_NUM_DAISY_STEP_MOTORS 6
static Cc5160Stepper cc_step_mots[CC_NUM_DAISY_STEP_MOTORS] = {
    {STEPPER_CUTTER, CC_NUM_DAISY_STEP_MOTORS - 1, 0},
    {STEPPER_CLAMP_LT, CC_NUM_DAISY_STEP_MOTORS - 1, 0},
    {STEPPER_CLAMP_LB, CC_NUM_DAISY_STEP_MOTORS - 1, 0},
    {STEPPER_CLAMP_RT, CC_NUM_DAISY_STEP_MOTORS - 1, 0},
    {STEPPER_CLAMP_RB, CC_NUM_DAISY_STEP_MOTORS - 1, 0},
    {STEPPER_RAIL, CC_NUM_DAISY_STEP_MOTORS - 1, 0},
};

// Default Cutter Register values
#define R00 0x00000004  // GCONF
#define R00i 0x00000014  // GCONF inverted shaft
#define R09 0x00010606  // SHORTCONF
#define R0A 0x00080400  // DRVCONF
#define R10 0x00070F02  // IHOLD_IRUN 
#define R11 0x0000000A  // TPOWERDOWN
#define R13 0x000001F4  // TPWMTHRS
#define R14 0x00001388  // TCOOLTHRS
#define R20 0x00000000  // RAMPMODE = 0 (Target position move)
#define R24 0x000007D0  // A1
#define R25 0x000061A8  // V1
#define R26 0x000003E8  // AMAX= 1000 Acceleration above V1
#define R27 0x0000C350  // VMAX= 50000
#define R28 0x000003E8  // DMAX= 1000 Deceleration above V1
#define R2A 0x000007D0  // D1= 2000 Deceleration below V1
#define R2B 0x0000001A  // VSTOP= 10 Stop velocity (Near to zero)
#define R3A 0x00010000  // ENC_CONST
#define R6C 0x000100C3  // CHOPCONF
#define R6D 0x00020000  // COOLCONF
#define R70 0xC40C001E  // PWMCONF

static const int32_t tmc5160_CutterStepperRegisterResetState[TMC5160_REGISTER_COUNT] =
{
//	0,   1,   2,   3,   4,   5,   6,   7,   8,   9,   A,   B,   C,   D,   E,   F
	R00, 0,   0,   0,   0,   0,   0,   0,   0,   R09, R0A, 0,   0,   0,   0,   0, // 0x00 - 0x0F
	R10, R11, 0,   R13, R14, 0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0, // 0x10 - 0x1F
	R20, 0,   0,   0,   R24, R25, R26, R27, R28, 0,   R2A, R2B, 0,   0,   0,   0, // 0x20 - 0x2F
	0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   R3A, 0,   0,   0,   0,   0, // 0x30 - 0x3F
	0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0, // 0x40 - 0x4F
	0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0, // 0x50 - 0x5F
	N_A, N_A, N_A, N_A, N_A, N_A, N_A, N_A, N_A, N_A, 0,   0,   R6C, R6D, 0,   0, // 0x60 - 0x6F
	R70, 0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0, // 0x70 - 0x7F
};

static const int32_t tmc5160_CutterInvStepperRegisterResetState[TMC5160_REGISTER_COUNT] =
{
//	0,   1,   2,   3,   4,   5,   6,   7,   8,   9,   A,   B,   C,   D,   E,   F
	R00i, 0,   0,   0,   0,   0,   0,   0,   0,   R09, R0A, 0,   0,   0,   0,   0, // 0x00 - 0x0F
	R10, R11, 0,   R13, R14, 0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0, // 0x10 - 0x1F
	R20, 0,   0,   0,   R24, R25, R26, R27, R28, 0,   R2A, R2B, 0,   0,   0,   0, // 0x20 - 0x2F
	0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   R3A, 0,   0,   0,   0,   0, // 0x30 - 0x3F
	0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0, // 0x40 - 0x4F
	0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0, // 0x50 - 0x5F
	N_A, N_A, N_A, N_A, N_A, N_A, N_A, N_A, N_A, N_A, 0,   0,   R6C, R6D, 0,   0, // 0x60 - 0x6F
	R70, 0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0, // 0x70 - 0x7F
};

// Undefine the default register values.
// This prevents warnings in case multiple TMC-API chip headers are included at once
#undef R00
#undef R09
#undef R0A
#undef R10
#undef R11
#undef R13
#undef R14
#undef R20
#undef R24
#undef R25
#undef R26
#undef R27
#undef R28
#undef R2A
#undef R2B
#undef R3A
#undef R6C
#undef R6D
#undef R70

// Default Register values
#define R00 0x00000004  // GCONF
#define R00i 0x00000014  // GCONF inverted shaft
#define R09 0x00010606  // SHORTCONF
#define R0A 0x00080400  // DRVCONF
#define R10 0x0006070A  // IHOLD_IRUN 
#define R11 0x0000000A  // TPOWERDOWN
#define R13 0x000001F4  // TPWMTHRS
#define R14 0x00001388  // TCOOLTHRS
#define R20 0x00000000  // RAMPMODE = 0 (Target position move)
#define R24 0x00002710  // A1  10000
#define R25 0x0000C350  // V1
#define R26 0x00004E20  // AMAX= 5000 Acceleration above V1
#define R27 0x0007A120  // VMAX= 500 000
#define R28 0x00002710  // DMAX= 5000 Deceleration above V1
#define R2A 0x00004E20   // D1= 10000 Deceleration below V1
#define R2B 0x0000000A  // VSTOP= 10 Stop velocity (Near to zero)
#define R3A 0x00010000  // ENC_CONST
#define R6C 0x000100C3  // CHOPCONF
#define R6D 0x00020000  // COOLCONF
#define R70 0xC40C001E  // PWMCONF

static const int32_t tmc5160_StepperRegisterResetState[TMC5160_REGISTER_COUNT] =
{
//	0,   1,   2,   3,   4,   5,   6,   7,   8,   9,   A,   B,   C,   D,   E,   F
	R00, 0,   0,   0,   0,   0,   0,   0,   0,   R09, R0A, 0,   0,   0,   0,   0, // 0x00 - 0x0F
	R10, R11, 0,   R13, R14, 0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0, // 0x10 - 0x1F
	R20, 0,   0,   0,   R24, R25, R26, R27, R28, 0,   R2A, R2B, 0,   0,   0,   0, // 0x20 - 0x2F
	0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   R3A, 0,   0,   0,   0,   0, // 0x30 - 0x3F
	0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0, // 0x40 - 0x4F
	0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0, // 0x50 - 0x5F
	N_A, N_A, N_A, N_A, N_A, N_A, N_A, N_A, N_A, N_A, 0,   0,   R6C, R6D, 0,   0, // 0x60 - 0x6F
	R70, 0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0, // 0x70 - 0x7F
};

static const int32_t tmc5160_StepperInvertedRegisterResetState[TMC5160_REGISTER_COUNT] =
{
//	0,   1,   2,   3,   4,   5,   6,   7,   8,   9,   A,   B,   C,   D,   E,   F
	R00i,0,   0,   0,   0,   0,   0,   0,   0,   R09, R0A, 0,   0,   0,   0,   0, // 0x00 - 0x0F
	R10, R11, 0,   R13, R14, 0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0, // 0x10 - 0x1F
	R20, 0,   0,   0,   R24, R25, R26, R27, R28, 0,   R2A, R2B, 0,   0,   0,   0, // 0x20 - 0x2F
	0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   R3A, 0,   0,   0,   0,   0, // 0x30 - 0x3F
	0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0, // 0x40 - 0x4F
	0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0, // 0x50 - 0x5F
	N_A, N_A, N_A, N_A, N_A, N_A, N_A, N_A, N_A, N_A, 0,   0,   R6C, R6D, 0,   0, // 0x60 - 0x6F
	R70, 0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0, // 0x70 - 0x7F
};

// Undefine the default register values.
// This prevents warnings in case multiple TMC-API chip headers are included at once
#undef R00
#undef R09
#undef R0A
#undef R10
#undef R11
#undef R13
#undef R14
#undef R20
#undef R24
#undef R25
#undef R26
#undef R27
#undef R28
#undef R2A
#undef R2B
#undef R3A
#undef R6C
#undef R6D
#undef R70

// Default Register values
#define R00i 0x00000014  // GCONF inverted shaft
#define R00 0x00000004  // GCONF normal shaft
#define R09 0x00010606  // SHORTCONF
#define R0A 0x00080400  // DRVCONF
#define R0B 0x0000009E  // GLOBAL SCALER
#define R10 0x00070A01  // IHOLD_IRUN IRUN = 0.7A ; IHOLD = 0.17A 
#define R11 0x0000000A  // TPOWERDOWN
#define R13 0x000001F4  // TPWMTHRS
#define R14 0x00001388  // TCOOLTHRS
#define R20 0x00000000  // RAMPMODE = 0 (Target position move)
#define R24 0x00002710  // A1 10000
#define R25 0x0003D090  // V1 250000
#define R26 0x00001388  // AMAX= 5000 Acceleration above V1
#define R27 0x0007A120  // VMAX= 500,000ppt
#define R28 0x00002388  // DMAX= 5000 Deceleration above V1
#define R2A 0x00004710  // D1= 10000 Deceleration below V1
#define R2B 0x0000005A  // VSTOP= 100 Stop velocity (Near to zero)
#define R38 0x00000200  // ENCMODE, bit 10 to set enc prescale to decimal
#define R3A 0x024707D0  // ENC_CONST 583.2
#define R3Ai 0x0FDB81F40  // ENC_CONST -583.2 for inverted
#define R3D 0x0000C350  // ENC_DEVIATION 50,000 max number of steps deviation
#define R6C 0x000100C3  // CHOPCONF
#define R6D 0x00020000  // COOLCONF
#define R70 0xC40C001E  // PWMCONF

//inverted shaft for left top and right bottom
static const int32_t tmc5160_ClampInvertStepperRegisterResetState[TMC5160_REGISTER_COUNT] =
{
//	0,   1,   2,   3,   4,   5,   6,   7,   8,   9,   A,   B,   C,   D,   E,   F
	R00i,0,   0,   0,   0,   0,   0,   0,   0,   R09, R0A, R0B, 0,   0,   0,   0, // 0x00 - 0x0F
	R10, R11, 0,   R13, R14, 0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0, // 0x10 - 0x1F
	R20, 0,   0,   0,   R24, R25, R26, R27, R28, 0,   R2A, R2B, 0,   0,   0,   0, // 0x20 - 0x2F
	0,   0,   0,   0,   0,   0,   0,   0,   R38, 0,   R3Ai,0,   0,   R3D, 0,   0, // 0x30 - 0x3F
	0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0, // 0x40 - 0x4F
	0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0, // 0x50 - 0x5F
	N_A, N_A, N_A, N_A, N_A, N_A, N_A, N_A, N_A, N_A, 0,   0,   R6C, R6D, 0,   0, // 0x60 - 0x6F
	R70, 0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0, // 0x70 - 0x7F
};

static const int32_t tmc5160_ClampStepperRegisterResetState[TMC5160_REGISTER_COUNT] =
{
//	0,   1,   2,   3,   4,   5,   6,   7,   8,   9,   A,   B,   C,   D,   E,   F
	R00, 0,   0,   0,   0,   0,   0,   0,   0,   R09, R0A, R0B, 0,   0,   0,   0, // 0x00 - 0x0F
	R10, R11, 0,   R13, R14, 0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0, // 0x10 - 0x1F
	R20, 0,   0,   0,   R24, R25, R26, R27, R28, 0,   R2A, R2B, 0,   0,   0,   0, // 0x20 - 0x2F
	0,   0,   0,   0,   0,   0,   0,   0,   R38, 0,   R3A, 0,   0,   R3D, 0,   0, // 0x30 - 0x3F
	0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0, // 0x40 - 0x4F
	0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0, // 0x50 - 0x5F
	N_A, N_A, N_A, N_A, N_A, N_A, N_A, N_A, N_A, N_A, 0,   0,   R6C, R6D, 0,   0, // 0x60 - 0x6F
	R70, 0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0, // 0x70 - 0x7F
};

// Undefine the default register values.
// This prevents warnings in case multiple TMC-API chip headers are included at once
#undef R00
#undef R00i
#undef R09
#undef R0A
#undef R10
#undef R11
#undef R13
#undef R14
#undef R20
#undef R24
#undef R25
#undef R26
#undef R27
#undef R28
#undef R2A
#undef R2B
#undef R3A
#undef R6C
#undef R6D
#undef R70



/*These are used on reset*/
static const int32_t * Cc5160StepperCfg[CC_NUM_DAISY_STEP_MOTORS] = { 
    tmc5160_CutterInvStepperRegisterResetState,
    tmc5160_ClampInvertStepperRegisterResetState,
    tmc5160_ClampStepperRegisterResetState,
    tmc5160_ClampStepperRegisterResetState,
    tmc5160_ClampInvertStepperRegisterResetState,
    tmc5160_StepperInvertedRegisterResetState
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