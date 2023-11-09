 /**
 * @file dwin_screen.hpp
 * @copyright Copyright (c) 2023 Vebu Labs. All rights reserved.
 * @brief This eheader file defines the common commands that will be used to command and control the screen.
 * @author Mike Lui
*/

#ifndef DWIN_SCREEN
#define DWIN_SCREEN

#define CHANGE_SCREEN_LEN 10
#define SCREEN_ADDR_BYTE 9

#define POWER_UP_SCREEN_ADDRESS    0x00
#define READY_SCREEN_ADDRESS       0x03
#define PROCESSING_SCREEN_ADDRESS  0x05


#define POWER_UP_SCREEN_LEN 10
const uint8_t disp_power_up [POWER_UP_SCREEN_LEN] = {0x5A, 0xA5, 0x07, 0x82, 0x00, 0x84, 0x5A, 0x01, 0x00, 0x00};

// #define ERROR_SCREEN_LEN 10
// const uint8_t disp_error_1 [ERROR_SCREEN_LEN] = {0x5A, 0xA5, 0x07, 0x82, 0x00, 0x84, 0x5A, 0x01, 0x00, 0x05};

// #define READY_SCREEN_LEN 10
// const uint8_t disp_ready [READY_SCREEN_LEN] = {0x5A, 0xA5, 0x07, 0x82, 0x00, 0x84, 0x5A, 0x01, 0x00, 0x03};



#define PLAY_SOUND_LEN 10
const uint8_t play_sound [PLAY_SOUND_LEN] = {0x5A, 0xA5, 0x07, 0x82, 0x00, 0xA0, 0x55, 0x70, 0x90, 0x00};


#endif