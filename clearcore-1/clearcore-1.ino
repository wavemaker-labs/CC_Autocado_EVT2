/**
 * @file clearcore1-main.ino
 * @copyright Copyright (c) 2023 Vebu Labs. All rights reserved.
 * @brief This is the main executable node for the Clearcore1 - Autocado evt2 ccc benchtop.
 * @author Mike Lui
*/


#include <SPI.h>
#include "TMC5160.hpp"
#include "src\control_node_1.hpp"
#include "src\rail_subsystem.hpp"
#include "src\cutter_subsystem.hpp"
#include "src\clamps_subsystem.hpp"
#include "src\ui_cc1_subsystem.hpp"

#define cycleTimeMs 100
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
	uint32_t timeout = 2000;
	uint32_t startTime = millis();
	while (!Serial && millis() - startTime < timeout) {
		continue;
	}	
	Serial.println("Starting\n");


	/*Set up IO and modbus server*/
	CcIoManager.initialize_interfaces();
	CcIoManager.cc_mb_hooks();

	// /*Map system and FSM to modbus and IO*/
	ptr_system_errors = CcIoManager.get_mb_data_pointer(MbRegisterOffsets::SYSTEM_ERROR);
	*ptr_system_errors = SystemErrors::NO_ERRORS;	

    rail.setup();
    cutter.setup();
	clamps.setup();
	ui_cc1.setup();

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
	
	rail.run();
	cutter.run();
	clamps.run();
	ui_cc1.run(rail.get_subsystem_state(), cutter.get_subsystem_state());

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