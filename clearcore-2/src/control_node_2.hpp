 /**
 * @file control_node_2.hpp
 * @copyright Copyright (c) 2023 Vebu Labs. All rights reserved.
 * @brief This library is used to implement control 2 specific code.
 * @author Mike Lui
*/

#ifndef CONTROL_NODE_2
#define CONTROL_NODE_2

#include <io_manager.hpp>

#define CC2_FW_VER 5

#define DB_GUTTER_MAX_MOVE_DEFAULT 7000
#define DB_GUTTER_TIMEOUT_MS_DEFAULT 9000
#define DB_GUTTER_MOT_SPEED_DEFAULT 3600  //per garrett
#define DB_GUTTER_MOT_ACCEL_DEFAULT 2000 

#define DB_TURNT_MOT_SPEED_DEFAULT 40
#define DB_TURNT_MOT_ACCEL_DEFAULT 20
#define DB_TURNT_MOT_STEP_DEFAULT 385 //per garrett

typedef enum
{
    CC_NUMBER = 0,
    WD_COUNTER,
    SYSTEM_ERROR,
    FW_VERSION,
    E_STOP,
    PEELER_M1_CMD,
    PEELER_M1_STATE,
    PEELER_CURRENT_1,
    PEELER_M2_CMD,
    PEELER_M2_STATE,
    PEELER_CURRENT_2,
    GUTTER_CMD,
    GUTTER_STATE,
    GUTTER_SENSOR_1,
    GUTTER_SENSOR_2,
    TURNTABLE_CMD,
    TURNTABLE_STATE,
    TURNTABLE_SENSOR,
    UI_START_BUTTON,
    UI_RESET_START,
    UI_START_LED,
    UI_DOOR_DRAWER_STATUS,
    RESET_CC_RQ = 50,
    DB_GUTTER_MOT_SPEED,
    DB_GUTTER_MOT_ACCEL,
    DB_GUTTER_TIMEOUT_MS,
    DB_GUTTER_MAX_MOVE,
    DB_TURNT_MOT_SPEED,
    DB_TURNT_MOT_ACCEL,
    DB_TURNT_MOVE_STEPS    

} MbRegisterOffsets;


#define CC2_NUM_MODBUS_REGISTERS 30
#define CC2_NODE_NUM 2


static ModbusRegister cc2_modbus[CC2_NUM_MODBUS_REGISTERS] =
{
    {INPUT_REG, CC_NUMBER, CC2_NODE_NUM, nullptr}, 
    {INPUT_REG, WD_COUNTER, 0, nullptr},
    {INPUT_REG, SYSTEM_ERROR, 0, nullptr},
    {INPUT_REG, FW_VERSION, CC2_FW_VER, nullptr},
    {INPUT_REG, E_STOP, 0, nullptr},

    /*Peeler*/
    {HOLDING_REG, PEELER_M1_CMD, 0, nullptr},
    {INPUT_REG, PEELER_M1_STATE, 0, nullptr},
    {INPUT_REG, PEELER_CURRENT_1, 0, nullptr},
    {HOLDING_REG, PEELER_M2_CMD, 0, nullptr},
    {INPUT_REG, PEELER_M2_STATE, 0, nullptr},
    {INPUT_REG, PEELER_CURRENT_2, 0, nullptr},

    /*Gutter*/
    {HOLDING_REG, GUTTER_CMD, 0, nullptr},
    {INPUT_REG, GUTTER_STATE, 0, nullptr},
    {INPUT_REG, GUTTER_SENSOR_1, 0, nullptr},
    {INPUT_REG, GUTTER_SENSOR_2, 0, nullptr},

    /*Turntable*/
    {HOLDING_REG, TURNTABLE_CMD, 0, nullptr},
    {INPUT_REG, TURNTABLE_STATE, 0, nullptr},
    {INPUT_REG, TURNTABLE_SENSOR, 0, nullptr},

    /*UI*/
    {INPUT_REG, UI_START_BUTTON, 0, nullptr},
    {HOLDING_REG, UI_RESET_START, 0, nullptr},
    {HOLDING_REG, UI_START_LED, 0, nullptr},
    {INPUT_REG, UI_DOOR_DRAWER_STATUS, 0, nullptr},

    /*Debug&Tune*/
    {HOLDING_REG, RESET_CC_RQ, 0, nullptr},
    {HOLDING_REG, DB_GUTTER_MOT_SPEED, DB_GUTTER_MOT_SPEED_DEFAULT, nullptr},
    {HOLDING_REG, DB_GUTTER_MOT_ACCEL, DB_GUTTER_MOT_ACCEL_DEFAULT, nullptr},
    {HOLDING_REG, DB_GUTTER_TIMEOUT_MS, DB_GUTTER_TIMEOUT_MS_DEFAULT, nullptr},
    {HOLDING_REG, DB_GUTTER_MAX_MOVE, DB_GUTTER_MAX_MOVE_DEFAULT, nullptr},
    {HOLDING_REG, DB_TURNT_MOT_SPEED, DB_TURNT_MOT_SPEED_DEFAULT, nullptr},
    {HOLDING_REG, DB_TURNT_MOT_ACCEL, DB_TURNT_MOT_ACCEL_DEFAULT, nullptr},
    {HOLDING_REG, DB_TURNT_MOVE_STEPS, DB_TURNT_MOT_STEP_DEFAULT, nullptr}

};


/*control node 2*/
typedef enum {
         BOWL_SENS_IN = 0,
        START_LED_OUT,
        DOORS_DRAWER_SEN_IN, 
        POWER_DISABLE_OUT,
        PEELER_RELAY1_OUT,
        PEELER_RELAY2_OUT,
        ESTOP_IN,
        GUTTER_SEN2_IN,
        GUTTER_SEN1_IN,
        A9_NOT_USED,
        START_BUTTON_IN,
        PEELER_I1_AIN,
        PEELER_I2_AIN
} AutocadoCcPins;


#define NUM_CC_IO_PIN 13
static PinIO cc2_io_pins[NUM_CC_IO_PIN] = {
    PinIO(SWITCH_SENSOR_IN, BOWL_SENS_IN, nullptr),
    PinIO(DIGITAL_OUT, START_LED_OUT, nullptr),
    PinIO(SWITCH_SENSOR_IN, DOORS_DRAWER_SEN_IN, nullptr),
    PinIO(DIGITAL_OUT, POWER_DISABLE_OUT, nullptr),
    PinIO(DIGITAL_OUT, PEELER_RELAY1_OUT, nullptr),
    PinIO(DIGITAL_OUT, PEELER_RELAY2_OUT, nullptr),
    PinIO(SWITCH_SENSOR_IN, ESTOP_IN, nullptr),
    PinIO(SWITCH_SENSOR_IN, GUTTER_SEN2_IN, nullptr),
    PinIO(SWITCH_SENSOR_IN, GUTTER_SEN1_IN, nullptr),
    PinIO(SWITCH_SENSOR_IN, A9_NOT_USED, nullptr),
    PinIO(SWITCH_SENSOR_IN, START_BUTTON_IN, nullptr),
    PinIO(ANALOG_IN, PEELER_I1_AIN, nullptr),
    PinIO(ANALOG_IN, PEELER_I2_AIN, nullptr)
};


typedef enum
{
    GUTTER_STEPPER = 0,
    TURNTABLE_STEPPER  
} AutocadoCc2Motors;


#define CC2_NUM_MOTORS 2
static MotorIO cc2_motors[CC2_NUM_MOTORS] = {
    {1000, 400, Connector::ConnectorModes::CPM_MODE_STEP_AND_DIR, &ConnectorM0, MotorDriver::MOVE_TARGET_REL_END_POSN},
    {1000, 400, Connector::ConnectorModes::CPM_MODE_STEP_AND_DIR, &ConnectorM1, MotorDriver::MOVE_TARGET_REL_END_POSN},
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