// ===================================================================================
// User Configurations for CH552G USB MacroPad Plus
// ===================================================================================

#pragma once

// Pin definitions
#define PIN_NEO             P10         // pin connected to NeoPixel
#define PIN_KEY1            P11 // P11 // P17         // pin connected to key 1
#define PIN_KEY2            P17 // P31         // pin connected to key 2
#define PIN_KEY3            P16 // P30         // pin connected to key 3
#define PIN_KEY4            P15         // pin connected to key 4
#define PIN_KEY5            P14         // pin connected to key 5
#define PIN_KEY6            P32         // pin connected to key 6
#define PIN_ENC_A           P31 // P34         // pin connected to rotary encoder A
#define PIN_ENC_B           P30 // P33         // pin connected to rotary encoder B
#define PIN_ENC_SW          P33 // P11         // pin connected to rotary encoder switch

// NeoPixel configuration
#define NEO_COUNT           18          // number of pixels in the string
#define NEO_GRB                         // type of pixel: NEO_GRB or NEO_RGB

// USB device descriptor
#define USB_VENDOR_ID       0x1189      // VID
#define USB_PRODUCT_ID      0x8890      // PID
#define USB_DEVICE_VERSION  0x0100      // v1.0 (BCD-format)

// USB configuration descriptor
#define USB_MAX_POWER_mA    150         // max power in mA 

// USB descriptor strings
#define MANUFACTURER_STR    'p','s','e','u','d','o','r','a','l','p','h' // 'w','a','g','i','m','i','n','a','t','o','r'
#define PRODUCT_STR         'M','a','c','r','o','p','a','d','6'
#define SERIAL_STR          '0','0','1'//'C','H','5','5','2','x','H','I','D'
#define INTERFACE_STR       'H','I','D','-','K','e','y','b','o','a','r','d'
