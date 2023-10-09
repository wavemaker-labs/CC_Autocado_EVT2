//  /**
//  * @file flat_convey_subsystem.cpp
//  * @copyright Copyright (c) 2023 Vebu Labs. All rights reserved.
//  * @brief This library is used to control the flat conveyor subsystem
//  * @author Mike Lui
// */

// #include "flat_convey_subsystem.hpp"

// #define TMC5160_SPI_DATAGEM_BYTE_WIDTH 5

// uint8_t setupa[5] = {0xEC, 0x00, 0x01, 0x00, 0xC3};
// uint8_t setupb[5] = {0x90, 0x00, 0x00, 0x00, 0x04};//{0x90, 0x00, 0x06, 0x1F, 0x0A}; <didn't work well for this motor
// uint8_t setupc[5] = {0x91, 0x00, 0x00, 0x00, 0x0A};
// uint8_t setupd[5] = {0x80, 0x00, 0x00, 0x00, 0x04};
// uint8_t setupe[5] = {0x93, 0x00, 0x00, 0x01, 0xF4};

// uint8_t setup1[5] = {0xA4, 0x00, 0x00, 0x03, 0xE8};
// uint8_t setup2[5] = {0xA5, 0x00, 0x00, 0xC3, 0x50};
// uint8_t setup3[5] = {0xA6, 0x00, 0x00, 0x01, 0xF4};
// uint8_t setup4[5] = {0xA7, 0x00, 0x03, 0x0D, 0x40};
// uint8_t setup5[5] = {0xA8, 0x00, 0x00, 0x02, 0xBC};
// uint8_t setup6[5] = {0xAA, 0x00, 0x00, 0x05, 0x78};
// uint8_t setup7[5] = {0xAB, 0x00, 0x00, 0x00, 0x0A};
// uint8_t setup8[5] = {0x0A, 0x00, 0x00, 0x03, 0xE8};

// bool new_motor_mb_cmd = false;

// uint16_t motor_speed_byte_1_2;
// uint16_t motor_speed_byte_3_4;

// int32_t motor_vactual;

// int32_t target_position = -51200 * 2;

// SPISettings spiConfig(80000, MSBFIRST, SPI_MODE3);

// uint16_t motor_hreg_write(TRegister* reg, uint16_t val) {
//     new_motor_mb_cmd = true;
//     return val;
// }


// void setup_move(uint8_t * ptr_array);

// // void setup_move(uint8_t * ptr_array){

// // 	SPI.beginTransaction(spiConfig);
// //     SPI.transfer(ptr_array, nullptr, 5, true); 
// // 	SPI.endTransaction(); 
// // }

// // void tmc5160_readWriteArray(uint8_t channel, uint8_t *data, size_t length)
// // {
// // 	SPI.beginTransaction(spiConfig);
// //     SPI.transfer(data, nullptr, length, true); 
// // 	SPI.endTransaction(); 
// // }

// void printarray(uint8_t * byteArray, uint8_t chain_cnt)
// {
//     Serial.print("\n 2d array:\n");
//     int k = 0;
//     for (int i = 0; i < chain_cnt; i++) {
//     // Iterate through the columns
//     for (int j = 0; j < 5; j++) {
//       // Print each byte as a two-digit hex number with leading zeros
//       Serial.print("0x");
//       if (byteArray[k] < 0x10) {
//         // Add leading zero for single-digit values
//         Serial.print("0");
//       }
//       Serial.print(byteArray[k], HEX);
//       k++;
//       Serial.print(" ");
//     }
//     Serial.println(); // Move to the next line after printing a row
//   }
// }

// void print_datagem(uint8_t * data)
// {
//     Serial.print("\n Datagem:\n");
//     for (int j = 0; j < 5; j++) {
//       // Print each byte as a two-digit hex number with leading zeros
//       Serial.print("0x");
//       if (data[j] < 0x10) {
//         // Add leading zero for single-digit values
//         Serial.print("0");
//       }
//       Serial.print(data[j], HEX);
//       Serial.print(" ");
//     }
//     Serial.println(); // Move to the next line after printing a row
// }

// void tmc5160_readWriteArray(uint8_t channel, uint8_t *data, size_t length)
// {
// 	uint8_t total_chain = 3;
// 	uint8_t chain_num = 2;
//     uint16_t bufferary_size = TMC5160_SPI_DATAGEM_BYTE_WIDTH * (total_chain);
//     uint8_t tx_buffer_data[bufferary_size];
//     uint8_t rx_buffer_data[bufferary_size];

//     memset(&tx_buffer_data, 0, bufferary_size);

//     //this needs to change if only one (no daisy chain, remove -1)
//     memcpy(&tx_buffer_data[(total_chain - chain_num - 1) * TMC5160_SPI_DATAGEM_BYTE_WIDTH], data, TMC5160_SPI_DATAGEM_BYTE_WIDTH);

//     SPI.beginTransaction(spiConfig); //in the clearcore SPI API this sets CS low already

//     // delayMicroseconds(10);

//     SPI.transfer(tx_buffer_data, rx_buffer_data, bufferary_size, true);

//     SPI.endTransaction(); //in the clearcore SPI API this sets CS low already

//     // printarray(&rx_buffer_data[0], total_chain); 
//     // print_datagem(&data[0]);

//     //this needs to change if only one (no daisy chain, remove -1)
//     memcpy(data, &rx_buffer_data[(total_chain - chain_num - 1) * TMC5160_SPI_DATAGEM_BYTE_WIDTH], TMC5160_SPI_DATAGEM_BYTE_WIDTH);

//     // print_datagem(&data[0]);
// }

// void tmc5160_readWriteArray(uint8_t channel, uint8_t *data, size_t length, uint8_t chain_num, uint8_t last_num)
// {
// 	uint8_t total_chain = last_num + 1;
//     uint16_t bufferary_size = TMC5160_SPI_DATAGEM_BYTE_WIDTH * (total_chain);
//     uint8_t tx_buffer_data[bufferary_size];
//     uint8_t rx_buffer_data[bufferary_size];

//     memset(&tx_buffer_data, 0, bufferary_size);

//     memcpy(&tx_buffer_data[(last_num - chain_num) * TMC5160_SPI_DATAGEM_BYTE_WIDTH], data, TMC5160_SPI_DATAGEM_BYTE_WIDTH);

//     SPI.beginTransaction(spiConfig); //in the clearcore SPI API this sets CS low already

//     SPI.transfer(tx_buffer_data, rx_buffer_data, bufferary_size, true);

//     SPI.endTransaction(); //in the clearcore SPI API this sets CS low already

//     // printarray(&rx_buffer_data[0], total_chain); 
//     // print_datagem(&data[0]);

//     memcpy(data, &rx_buffer_data[(last_num - chain_num) * TMC5160_SPI_DATAGEM_BYTE_WIDTH], TMC5160_SPI_DATAGEM_BYTE_WIDTH);

//     // print_datagem(&data[0]);
// }

// void setup_move(uint8_t * ptr_array){

//     tmc5160_readWriteArray(0, ptr_array, 5);
// }

// void FlatConveyorFSMClass::setup()
// {
//     if(!has_setup){
//         has_setup = true;
//         motor_speed = 0;

//         // Open the SPI port on ConnectorCOM0
//         SPI.begin();

//         // send configuration
//         tmc5160_init(&motorone, 0, &motorone_cfg, tmc5160_defaultRegisterResetState);
//         tmc5160_init(&motortwo, 0, &motortwo_cfg, tmc5160_defaultRegisterResetState);
//         tmc5160_init(&motorthree, 0, &motorthree_cfg, tmc5160_defaultRegisterResetState);

//         for(int i = 0; i < 3; i++){
//             tmc5160_writeInt(&motorone, TMC5160_CHOPCONF,   0x000100C3, i, 2);
//             tmc5160_writeInt(&motorone, TMC5160_IHOLD_IRUN, 0x00000504, i, 2);
//             tmc5160_writeInt(&motorone, TMC5160_TPOWERDOWN, 0x0000000A, i, 2);
//             tmc5160_writeInt(&motorone, TMC5160_GCONF,      0x00000004, i, 2);
//             tmc5160_writeInt(&motorone, TMC5160_TPWMTHRS,   0x000001F4, i, 2);

//             tmc5160_writeInt(&motorone, TMC5160_A1,         0x000006E8, i, 2);
//             tmc5160_writeInt(&motorone, TMC5160_V1,         0x0000C350, i, 2);
//             tmc5160_writeInt(&motorone, TMC5160_AMAX,       0x000001F4, i, 2);
//             tmc5160_writeInt(&motorone, TMC5160_VMAX,       0x00030D40, i, 2);
//             tmc5160_writeInt(&motorone, TMC5160_DMAX,       0x000002BC, i, 2);
//             tmc5160_writeInt(&motorone, TMC5160_D1,         0x00000578, i, 2);
//             tmc5160_writeInt(&motorone, TMC5160_VSTOP,      0x0000000A, i, 2);
//             tmc5160_writeInt(&motorone, TMC5160_RAMPMODE,   0x00030D40, i, 2);
//         }

        

        
//         // while(1){
//         //     for(int i = 0; i < 3; i++){
//         //         tmc5160_writeInt(&motorone, TMC5160_XTARGET, target_position, i, 2);
//         //     }            
//         //     target_position = target_position * -1;
//         //     Serial.println(target_position);
//         //     delay(2000);
//         // }

//         Serial.println("Setup motor\n");


//         new_motor_mb_cmd = false;
//         CcIoManager.set_mb_w_hreg_cb(MbRegisterOffsets::MOTOR_1_SPEED_1, &motor_hreg_write);

//         state = FlatConvey::FlatConStates::SETUP;
//     }
// }

// void FlatConveyorFSMClass::read_interfaces()
// {
//     motor_speed_byte_1_2 = CcIoManager.get_mb_data(MbRegisterOffsets::MOTOR_1_SPEED_1);
//     motor_speed_byte_3_4 = CcIoManager.get_mb_data(MbRegisterOffsets::MOTOR_1_SPEED_2);
//     motor_speed = int32_t (motor_speed_byte_1_2 << 16 | motor_speed_byte_3_4);

//     tmc5160_periodicJob(&motorone, CcIoManager.getSystemTime(), 0, 2);
//     tmc5160_periodicJob(&motortwo, CcIoManager.getSystemTime(), 1, 2);
//     tmc5160_periodicJob(&motorthree, CcIoManager.getSystemTime(), 2, 2);
// }

// void FlatConveyorFSMClass::run()
// {
//     read_interfaces();

//     // if (estop_input == ESTOP_ACTIVE && state != FlatConvey::FlatConStates::ESTOP)
//     // {
//     //     state = FlatConvey::FlatConStates::ESTOP;
//     // }

//     Serial.print("\n\nVelocity: ");
//     Serial.print(motorone.velocity);
//     Serial.print("\nPosition: ");
//     Serial.print(motorone.oldX);

    

//     switch (state)
//     {
//         case FlatConvey::FlatConStates::SETUP:
//             Serial.println("Continue Setup\n");
	
//             if(motorone.config->state == CONFIG_READY && 
//             motortwo.config->state == CONFIG_READY &&
//             motorthree.config->state == CONFIG_READY)
//             {
//                 Serial.println("Finished Setup\n");
//                 state = FlatConvey::FlatConStates::CYCLE;
//                 // state = FlatConvey::FlatConStates::STOPPED;
//             }

//             break;
//         case FlatConvey::FlatConStates::STOPPED:           
//             if(new_motor_mb_cmd){
//                 new_motor_mb_cmd = false;
//                 if(motor_speed != 0){
//                     tmc5160_rotate(&motorone, motor_speed);
//                     Serial.println("Commanded speed:");
//                     Serial.println(motor_speed);
//                     Serial.println("\n");
//                 }
//                 state = FlatConvey::FlatConStates::MOVING;
//             }
//             break;

//         case FlatConvey::FlatConStates::MOVING:
//             if(new_motor_mb_cmd){
//                 new_motor_mb_cmd = false;
//                 tmc5160_rotate(&motorone, motor_speed);
//                 Serial.println("Commanded speed:");
//                 Serial.println(motor_speed);
//                 Serial.println("\n");
//                 if(motor_speed == 0){
//                     state = FlatConvey::FlatConStates::STOPPED;
//                 }
                
//             }    
//             break;

//         case FlatConvey::FlatConStates::CYCLE:
//             for(int i = 0; i < 3; i++){
//                 tmc5160_writeInt(&motorone, TMC5160_XTARGET, target_position / (i + 1), i, 2);
//             }            
//             target_position = target_position * -1;
//             move_start_time_ms = CcIoManager.getSystemTime();
//             state = FlatConvey::FlatConStates::CYCLE_WAIT;

//             break;

//         case FlatConvey::FlatConStates::CYCLE_WAIT:
//             if(CcIoManager.getSystemTime() - move_start_time_ms > 3000){
//                 state = FlatConvey::FlatConStates::CYCLE;                
//             }    
//             break;      


//         case FlatConvey::FlatConStates::ERROR_MOTOR:
//             ptr_flat_con_motor->enable = false;
//             break;
        
//         case FlatConvey::FlatConStates::ESTOP:
//         default:
//             ptr_flat_con_motor->enable = false;
//             new_motor_mb_cmd = false;
//             if (estop_input == ESTOP_RELEASED) {
//                 state = FlatConvey::STOPPED;
//             }
//             break;
//     }

//     write_interfaces();
// }

// void FlatConveyorFSMClass::write_interfaces()
// {
//     CcIoManager.set_mb_data(MbRegisterOffsets::MOTOR_1_STATE, state);
//     CcIoManager.set_mb_data(MbRegisterOffsets::MOTOR_1_POS_1, uint16_t(motorone.oldX >> 16));
//     CcIoManager.set_mb_data(MbRegisterOffsets::MOTOR_1_POS_2, uint16_t(0xFFFF & motorone.oldX));
//     CcIoManager.set_mb_data(MbRegisterOffsets::MOTOR_2_POS_1, uint16_t(motortwo.oldX >> 16));
//     CcIoManager.set_mb_data(MbRegisterOffsets::MOTOR_2_POS_2, uint16_t(0xFFFF & motortwo.oldX));;
//     CcIoManager.set_mb_data(MbRegisterOffsets::MOTOR_3_POS_1, uint16_t(motorthree.oldX >> 16));
//     CcIoManager.set_mb_data(MbRegisterOffsets::MOTOR_3_POS_2, uint16_t(0xFFFF & motorthree.oldX));
// }

// FlatConveyorFSMClass flat_con;