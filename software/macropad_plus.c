// ===================================================================================
// Project:   MacroPad Plus for CH551, CH552 and CH554
// Version:   v1.0
// Year:      2023
// Author:    Stefan Wagner
// Github:    https://github.com/wagiminator
// EasyEDA:   https://easyeda.com/wagiminator
// License:   http://creativecommons.org/licenses/by-sa/3.0/
// ===================================================================================
//
// Description:
// ------------
// Firmware example implementation for the MacroPad Plus.
//
// References:
// -----------
// - Blinkinlabs: https://github.com/Blinkinlabs/ch554_sdcc
// - Deqing Sun: https://github.com/DeqingSun/ch55xduino
// - Ralph Doncaster: https://github.com/nerdralph/ch554_sdcc
// - WCH Nanjing Qinheng Microelectronics: http://wch.cn
//
// Compilation Instructions:
// -------------------------
// - Chip:  CH551, CH552 or CH554
// - Clock: 16 MHz internal
// - Adjust the firmware parameters in src/config.h if necessary.
// - Customize the macro functions in the corresponding section below.
// - Make sure SDCC toolchain and Python3 with PyUSB is installed.
// - Press BOOT button on the board and keep it pressed while connecting it via USB
//   with your PC.
// - Run 'make flash' immediatly afterwards.
// - To compile the firmware using the Arduino IDE, follow the instructions in the 
//   .ino file.
//
// Operating Instructions:
// -----------------------
// - Connect the board via USB to your PC. It should be detected as a HID device with
//   keyboard, mouse and joystick interface.
// - Press a macro key or turn the knob and see what happens.
// - To enter bootloader hold down rotary encoder switch while connecting the 
//   MacroPad to USB. All NeoPixels will light up white as long as the device is in 
//   bootloader mode (about 10 seconds).


// ===================================================================================
// Libraries, Definitions and Macros
// ===================================================================================

// Libraries
#include "src/config.h"                     // user configurations
#include "src/system.h"                     // system functions
#include "src/delay.h"                      // delay functions
#include "src/gpio.h"                        // GPIO pin macros (PIN_read + pin enum) — formerly pulled in via neo.h
#include "src/usb_composite.h"              // USB HID composite functions

// Prototypes for used interrupts
void USB_interrupt(void);
void USB_ISR(void) __interrupt(INT_NO_USB) {
  USB_interrupt();
}

#pragma disable_warning 110                 // Keep calm, EVELYN!

// ===================================================================================
// Macro Functions which associate Actions with Events (Customize your MacroPad here!)
// ===================================================================================
/*
// The list of available USB HID functions can be found in src/usb_composite.h
// The keys are enumerated the following way:
// +---+---+---+    -----
// | 1 | 2 | 3 |  /       \
// +---+---+---+  |encoder|
// | 4 | 5 | 6 |  \       /
// +---+---+---+    -----
*/

// Key 1 : VOL UP
inline void KEY1_PRESSED() {
  CON_press(CON_MEDIA_REWIND);   
}

inline void KEY1_RELEASED() {
  CON_release();
}

inline void KEY1_HOLD() {}

// Key 2 : PLAY/PAUSE
inline void KEY2_PRESSED() {
  CON_press(CON_MEDIA_PLAY);   
}

inline void KEY2_RELEASED() {
  CON_release();
}
inline void KEY2_HOLD() {}

// Key 3 : VOL UP
inline void KEY3_PRESSED() {
  CON_press(CON_MEDIA_FORWARD);    
}

inline void KEY3_RELEASED() {
  CON_release();
}

inline void KEY3_HOLD() {}

// Key 4 : MUTE
inline void KEY4_PRESSED() {
  KBD_press(KBD_KEY_LEFT_GUI); 
  DLY_ms(30);  
  KBD_type('d');   
}

inline void KEY4_RELEASED() {
  KBD_releaseAll();
}

inline void KEY4_HOLD() {}

// Key 5 : VIDEO OFF
inline void KEY5_PRESSED() {
  KBD_press(KBD_KEY_LEFT_GUI); 
  DLY_ms(30);  
  KBD_type('e');   
}

inline void KEY5_RELEASED() {
  KBD_releaseAll();
}

inline void KEY5_HOLD() {}

// Key 6 : HAND UP
inline void KEY6_PRESSED() {
  KBD_press(KBD_KEY_LEFT_CTRL);
  DLY_ms(30);  
  KBD_press(KBD_KEY_LEFT_GUI);
  DLY_ms(30);  
  KBD_type('h');
}

inline void KEY6_RELEASED() {
  KBD_releaseAll();
}

inline void KEY6_HOLD() {}

inline void ENC_CW_ACTION() {
  CON_press(CON_VOL_UP);  
}

inline void ENC_CW_RELEASED() {
  CON_release();
}

inline void ENC_CCW_ACTION() {
  CON_press(CON_VOL_DOWN);   

}

inline void ENC_CCW_RELEASED() {
    CON_release();
}

inline void ENC_SW_PRESSED() {
  CON_press(CON_VOL_MUTE);
}

inline void ENC_SW_RELEASED() {
  CON_release();

}

// ===================================================================================
// NeoPixel Configuration
// ===================================================================================

// Global NeoPixel brightness
// #define NEO_BRIGHT_KEYS   2         // NeoPixel brightness for keys (0..2)
// #define NEO_BRIGHT_ENC    0         // NeoPixel brightness for encoder ring (0..2)

// Key colors (hue value: 0..191)
// #define NEO_KEY1          0         // red
// #define NEO_KEY2          32        // yellow
// #define NEO_KEY3          64        // green
// #define NEO_KEY4          96        // cyan
// #define NEO_KEY5          128       // blue
// #define NEO_KEY6          160       // magenta

// ===================================================================================
// NeoPixel Functions
// ===================================================================================

// uint8_t neoencoder = 0;                           // state of NeoPixel ring rotation

// Update NeoPixel ring colors
// void NEO_encoder_update(void) {
//   uint8_t i, j;
//   j = neoencoder;
//   for(i=6; i<18; i++) {
//     NEO_writeHue(i, j, NEO_BRIGHT_ENC);
//     j += 16;
//     if(j >= 192) j -= 192;
//   }
//   NEO_update();
// }

// Rotate NeoPixel ring clockwise
// void NEO_encoder_cw(void) {
//   neoencoder += 8;
//   if(neoencoder >= 192) neoencoder -= 192;
//   NEO_encoder_update();
// }

// Rotate NeoPixel ring counter-clockwise
// void NEO_encoder_ccw(void) {
//   neoencoder -= 8;
//   if(neoencoder >= 192) neoencoder -= 64;
//   NEO_encoder_update();
// }

// ===================================================================================
// Rotary Encoder Quadrature Decoder (Ben Buxton full-step state table)
// ===================================================================================
// Row = current decoder state; column = live 2-bit pin reading (A<<1)|B, value 0..3.
// Each cell is the NEXT state. The top bits flag a completed detent:
//   ENC_DIR_CW (0x10) or ENC_DIR_CCW (0x20). Contact bounce wobbles between
//   intermediate states and never reaches a FINAL cell, so it emits nothing.
#define ENC_START      0x00
#define ENC_CW_FINAL   0x01
#define ENC_CW_BEGIN   0x02
#define ENC_CW_NEXT    0x03
#define ENC_CCW_BEGIN  0x04
#define ENC_CCW_FINAL  0x05
#define ENC_CCW_NEXT   0x06
#define ENC_DIR_CW     0x10
#define ENC_DIR_CCW    0x20

const __code uint8_t ENC_TABLE[7][4] = {
  // ENC_START
  {ENC_START,     ENC_CW_BEGIN,  ENC_CCW_BEGIN, ENC_START},
  // ENC_CW_FINAL
  {ENC_CW_NEXT,   ENC_START,     ENC_CW_FINAL,  ENC_START | ENC_DIR_CW},
  // ENC_CW_BEGIN
  {ENC_CW_NEXT,   ENC_CW_BEGIN,  ENC_START,     ENC_START},
  // ENC_CW_NEXT
  {ENC_CW_NEXT,   ENC_CW_BEGIN,  ENC_CW_FINAL,  ENC_START},
  // ENC_CCW_BEGIN
  {ENC_CCW_NEXT,  ENC_START,     ENC_CCW_BEGIN, ENC_START},
  // ENC_CCW_FINAL
  {ENC_CCW_NEXT,  ENC_CCW_FINAL, ENC_START,     ENC_START | ENC_DIR_CCW},
  // ENC_CCW_NEXT
  {ENC_CCW_NEXT,  ENC_CCW_FINAL, ENC_CCW_BEGIN, ENC_START},
};

// ===================================================================================
// Main Function
// ===================================================================================
void main(void) {
  // Variables
  __bit key1last = 0;                             // last state of key 1
  __bit key2last = 0;                             // last state of key 2
  __bit key3last = 0;                             // last state of key 3
  __bit key4last = 0;                             // last state of key 4
  __bit key5last = 0;                             // last state of key 5
  __bit key6last = 0;                             // last state of key 6
  __bit isSwitchPressed = 0;                      // state of rotary encoder switch
  uint8_t encState = ENC_START;                   // rotary encoder decoder state
  // __idata uint8_t i;                              // temp variable

  // Setup
  // NEO_init();                                     // init NeoPixels
  CLK_config();                                   // configure system clock
  DLY_ms(10);                                     // wait for clock to settle
  // NEO_clearAll();                                 // clear NeoPixels

  // Enter bootloader if rotary encoder switch is pressed
  if(!PIN_read(PIN_ENC_SW)) {                     // encoder switch pressed?
    // for(i=3*NEO_COUNT; i; i--) NEO_sendByte(127); // light up all pixels
    BOOT_now();                                   // enter bootloader
  }

  // Init USB HID device
  HID_init();                                     // init USB HID device
  DLY_ms(500);                                    // wait for Windows
  WDT_start();                                    // start watchdog timer
  // NEO_encoder_update();                           // set NeoPixel ring for encoder

  // Loop
  while(1) {

    // Handle key 1
    // ------------
    if(!PIN_read(PIN_KEY1) != key1last) {         // key state changed?
      key1last = !key1last;                       // update last state flag
      if(key1last) {                              // key was pressed?
        // NEO_writeHue(0, NEO_KEY1, NEO_BRIGHT_KEYS);    // light up corresponding NeoPixel
        // NEO_update();                             // update pixels
        KEY1_PRESSED();                           // take proper action
      }
      else {                                      // key was released?
        // NEO_clearPixel(0);                        // light up corresponding NeoPixel
        // NEO_update();                             // update pixels
        KEY1_RELEASED();                          // take proper action
      }
    }
    else if(key1last) {                           // key still being pressed?
      KEY1_HOLD();                                // take proper action
    }

    // Handle key 2
    // ------------
    if(!PIN_read(PIN_KEY2) != key2last) {         // key state changed?
      key2last = !key2last;                       // update last state flag
      if(key2last) {                              // key was pressed?
        // NEO_writeHue(1, NEO_KEY2, NEO_BRIGHT_KEYS);    // light up corresponding NeoPixel
        // NEO_update();                             // update pixels
        KEY2_PRESSED();                           // take proper action
      }
      else {                                      // key was released?
        // NEO_clearPixel(1);                        // light up corresponding NeoPixel
        // NEO_update();                             // update pixels
        KEY2_RELEASED();                          // take proper action
      }
    }
    else if(key2last) {                           // key still being pressed?
      KEY2_HOLD();                                // take proper action
    }

    // Handle key 3
    // ------------
    if(!PIN_read(PIN_KEY3) != key3last) {         // key state changed?
      key3last = !key3last;                       // update last state flag
      if(key3last) {                              // key was pressed?
        // NEO_writeHue(2, NEO_KEY3, NEO_BRIGHT_KEYS);    // light up corresponding NeoPixel
        // NEO_update();                             // update pixels
        KEY3_PRESSED();                           // take proper action
      }
      else {                                      // key was released?
        // NEO_clearPixel(2);                        // light up corresponding NeoPixel
        // NEO_update();                             // update pixels
        KEY3_RELEASED();                          // take proper action
      }
    }
    else if(key3last) {                           // key still being pressed?
      KEY3_HOLD();                                // take proper action
    }

    // Handle key 4
    // ------------
    if(!PIN_read(PIN_KEY4) != key4last) {         // key state changed?
      key4last = !key4last;                       // update last state flag
      if(key4last) {                              // key was pressed?
        // NEO_writeHue(3, NEO_KEY4, NEO_BRIGHT_KEYS);    // light up corresponding NeoPixel
        // NEO_update();                             // update pixels
        KEY4_PRESSED();                           // take proper action
      }
      else {                                      // key was released?
        // NEO_clearPixel(3);                        // light up corresponding NeoPixel
        // NEO_update();                             // update pixels
        KEY4_RELEASED();                          // take proper action
      }
    }
    else if(key4last) {                           // key still being pressed?
      KEY4_HOLD();                                // take proper action
    }

    // Handle key 5
    // ------------
    if(!PIN_read(PIN_KEY5) != key5last) {         // key state changed?
      key5last = !key5last;                       // update last state flag
      if(key5last) {                              // key was pressed?
        // NEO_writeHue(4, NEO_KEY5, NEO_BRIGHT_KEYS);    // light up corresponding NeoPixel
        // NEO_update();                             // update pixels
        KEY5_PRESSED();                           // take proper action
      }
      else {                                      // key was released?
        // NEO_clearPixel(4);                        // light up corresponding NeoPixel
        // NEO_update();                             // update pixels
        KEY5_RELEASED();                          // take proper action
      }
    }
    else if(key5last) {                           // key still being pressed?
      KEY5_HOLD();                                // take proper action
    }

    // Handle key 6
    // ------------
    if(!PIN_read(PIN_KEY6) != key6last) {         // key state changed?
      key6last = !key6last;                       // update last state flag
      if(key6last) {                              // key was pressed?
        // NEO_writeHue(5, NEO_KEY6, NEO_BRIGHT_KEYS);    // light up corresponding NeoPixel
        // NEO_update();                             // update pixels
        KEY6_PRESSED();                           // take proper action
      }
      else {                                      // key was released?
        // NEO_clearPixel(5);                        // light up corresponding NeoPixel
        // NEO_update();                             // update pixels
        KEY6_RELEASED();                          // take proper action
      }
    }
    else if(key6last) {                           // key still being pressed?
      KEY6_HOLD();                                // take proper action
    }

    // Handle rotary encoder (quadrature state-machine decoder)
    // --------------------------------------------------------
    encState = ENC_TABLE[encState & 0x0F][(PIN_read(PIN_ENC_A) << 1) | PIN_read(PIN_ENC_B)];
    if((encState & 0x30) == ENC_DIR_CW) {         // one full detent clockwise?
      ENC_CW_ACTION();                            // press + release = one clean tap;
      ENC_CW_RELEASED();                          //   USB endpoint self-paces, no delay needed
    }
    else if((encState & 0x30) == ENC_DIR_CCW) {   // one full detent counter-clockwise?
      ENC_CCW_ACTION();
      ENC_CCW_RELEASED();
    }

    // Handle rotary encoder switch
    // ----------------------------
    if(!isSwitchPressed && !PIN_read(PIN_ENC_SW)) {      // switch newly pressed?
      ENC_SW_PRESSED();                           // take proper action
      isSwitchPressed = 1;
    }
    else if(isSwitchPressed && PIN_read(PIN_ENC_SW)) {   // switch newly released?
      ENC_SW_RELEASED();                          // take proper action
      isSwitchPressed = 0;                        // update switch state
    }

    DLY_ms(1);                                    // debounce
    WDT_reset();                                  // reset watchdog
  }
}
