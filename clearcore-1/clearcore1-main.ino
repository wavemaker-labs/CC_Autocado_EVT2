/**
 * @file clearcore1-main.ino
 * @copyright Copyright (c) 2023 Vebu Labs. All rights reserved.
 * @brief This is the main executable node for the Clearcore1 - Bobacino.
 * @author Manomit Bal, Mike Lui
*/

#include "src\control_node_1.hpp"
#include "src\ice_disp_subsystem.hpp"
#include "src\syrup_disp_subsystem.hpp"
#include "src\cup_trans_subsystem.hpp"
#include "src\cup_disp_subsystem.hpp"
#include "src\boba_disp_subsystem.hpp"


#define cycleTimeMs 20
#define jitterLimitMs 10
uint32_t cycleStartTime;
uint32_t cycleLastTime;

uint16_t *ptr_system_errors;
typedef enum {
	NO_ERRORS = 0,
	CYCLE_JITTER_EXCEEDED
}SystemErrors;


void setup() 
{
	Serial.begin(9600);     // Open serial communications and wait for port to open
	uint32_t timeout = 500;
	uint32_t startTime = millis();
	while (!Serial && millis() - startTime < timeout) {
		continue;
	}
	/*Set up IO and modbus server*/
	CcIoManager.initialize_interfaces();
	CcIoManager.cc_mb_hooks();

	/*Map system and FSM to modbus and IO*/
	ptr_system_errors = CcIoManager.get_mb_data_pointer(MbRegisterOffsets::SYSTEM_ERROR);
	*ptr_system_errors = SystemErrors::NO_ERRORS;

	ice_disp.setup();
	syrup_disp.setup();
	cup_trans.setup();
	cup_disp.setup();
	boba_disp.setup();

	CcIoManager.reset_watchdog();
	cycleStartTime = millis();
}

void cycleTasks()
{
	if(cycleLastTime >= (cycleTimeMs + jitterLimitMs))
	{
		//Error state
		Serial.println("Cycle time exceeded");
		Serial.println(cycleLastTime);
		*ptr_system_errors = SystemErrors::CYCLE_JITTER_EXCEEDED;
	}
	CcIoManager.service_interfaces();
	CcIoManager.read_interfaces();
	CcIoManager.update_system_mb();
	
	ice_disp.run();
	syrup_disp.run();
	cup_trans.run();	
	cup_disp.run();
	boba_disp.run();

	CcIoManager.write_interfaces();
	CcIoManager.kick_watchdog();
}

void loop() 
{
	if (millis() - cycleStartTime >= cycleTimeMs) {
		cycleLastTime = millis() - cycleStartTime;
		cycleStartTime = millis();
        cycleTasks();
    }
}