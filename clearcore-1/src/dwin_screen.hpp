 /**
 * @file dwin_screen.hpp
 * @copyright Copyright (c) 2023 Vebu Labs. All rights reserved.
 * @brief This eheader file defines the common commands that will be used to command and control the screen.
 * @author Mike Lui
*/

#ifndef DWIN_SCREEN
#define DWIN_SCREEN

#define POWER_UP_SCREEN_LEN 10
const uint8_t disp_power_up [POWER_UP_SCREEN_LEN] = {0x5A, 0xA5, 0x07, 0x82, 0x00, 0x84, 0x5A, 0x01, 0x00, 0x00};

#define ERROR_SCREEN_LEN 10
const uint8_t disp_error_1 [ERROR_SCREEN_LEN] = {0x5A, 0xA5, 0x07, 0x82, 0x00, 0x84, 0x5A, 0x01, 0x00, 0x05};

#define CHANGE_SCREEN_LEN 8
const uint8_t change_screen [CHANGE_SCREEN_LEN] = {0x5A, 0xA5, 0x07, 0x82, 0x00, 0x84, 0x5A, 0x01};




#endif