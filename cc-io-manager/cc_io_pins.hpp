 /**
 * @file cc_io_pins.hpp
 * @copyright Copyright (c) 2023 Vebu Labs. All rights reserved.
 * @brief This library holds io pins struct and definitions
 * @author Mike Lui
*/

#ifndef CC_IO_PINS_HPP
#define CC_IO_PINS_HPP

typedef enum {
        SWITCH_SENSOR_IN,
        ANALOG_IN,
        INTERRUPT_IN, 
        DIGITAL_OUT,
        HBRIDGE_OUT,
        ANALOG_OUT
} PinInputType;


struct PinIO
{
   PinInputType pin_mode;
   PinStatus state;
   uint16_t pin_num;
   int16_t value; // Analog or counter
   void (*interupt_cb)();

   PinIO(PinInputType pin_mode_, uint16_t pin_num_, void (*interupt_cb_)()) {
      pin_mode = pin_mode_;
      pin_num = pin_num_;
      interupt_cb = interupt_cb_;
      state = PinStatus::LOW;
      value = 0;
   }
};

#endif