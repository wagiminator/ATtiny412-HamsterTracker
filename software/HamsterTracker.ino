// ===================================================================================
// Project:   Hamster Wheel Tracker with Hall Sensor
// Version:   v1.0
// Year:      2022
// Author:    Stefan Wagner
// Github:    https://github.com/wagiminator
// EasyEDA:   https://easyeda.com/wagiminator
// License:   http://creativecommons.org/licenses/by-sa/3.0/
// ===================================================================================
//
// Description:
// ------------
// The Hamster Wheel Tracker is a simple fitness monitoring device for your hamster. 
// The speed and distance traveled by the hamster in the wheel are measured using two 
// opposing neodymium magnets with complementary poles, which run past the device's 
// Hall sensor. The recorded values are displayed on an OLED. The button is used to 
// reset the stored values for the maximum speed reached and the maximum distance 
// traveled in a day. The total distance covered since the device was started up is 
// retained. The corresponding values are stored in the EEPROM so that they are not 
// lost after a power supply interruption.
//
// References:
// -----------
// The I²C OLED implementation is based on TinyOLEDdemo
// https://github.com/wagiminator/ATtiny13-TinyOLEDdemo
//
// The small OLED font was adapted from Neven Boyanov and Stephen Denne
// https://github.com/datacute/Tiny4kOLED
//
// Wiring:
// -------
//                        +-\/-+
//                  Vdd  1|°   |8  GND
//          --- TXD PA6  2|    |7  PA3 AIN3 -------- HALL SENSOR
//   BUTTON --- RXD PA7  3|    |6  PA0 AIN0 UPDI --- UPDI
// OLED SDA --- SDA PA1  4|    |5  PA2 AIN2 SCL ---- OLED SCL
//                        +----+
//
// Compilation Settings:
// ---------------------
// Core:    megaTinyCore (https://github.com/SpenceKonde/megaTinyCore)
// Board:   ATtiny412/402/212/202
// Chip:    ATtiny202 or 212 or 402 or 412
// Clock:   1 MHz internal
//
// Leave the rest on default settings. Don't forget to "Burn bootloader"!
// Compile and upload the code.
//
// No Arduino core functions or libraries are used. To compile and upload without
// Arduino IDE download AVR 8-bit toolchain at:
// https://www.microchip.com/mplab/avr-support/avr-and-arm-toolchains-c-compilers
// and extract to tools/avr-gcc. Use the makefile to compile and upload.
//
// Fuse Settings: 0:0x00 1:0x00 2:0x01 4:0x00 5:0xC5 6:0x04 7:0x00 8:0x00
//
// Operating Instructions:
// -----------------------
// Place the device on the hamster cage so that the magnets on the hamster wheel 
// pass directly in front of the Hall sensor. Connect the device to a 3.3V - 5V 
// power supply via the USB socket or the UPDI header. Press and hold the button 
// for a few seconds to reset the maximum values recorded.


// ===================================================================================
// Libraries, Definitions and Macros
// ===================================================================================

// Libraries
#include <avr/io.h>                 // for GPIO
#include <avr/eeprom.h>             // for storing user settings into EEPROM
#include <avr/interrupt.h>          // for interrupts
#include <util/delay.h>             // for delays

// Pin assignments
#define PIN_SDA       PA1           // I2C Serial Data,  connect to OLED
#define PIN_SCL       PA2           // I2C Serial Clock, connect to OLED
#define PIN_HALL      PA3           // connect to HALL sensor
#define PIN_BUTTON    PA7           // connect to button

// Firmware parameters
#define CIRCUM        40            // Wheel circumference in cm
#define RESDEL        10            // time the RESET button must be held down
#define EEPROM_IDENT  0x6FE7        // EEPROM identifier -> check if it was written

// Pin manipulation macros
enum {PA0, PA1, PA2, PA3, PA4, PA5, PA6, PA7};      // enumerate pin designators
#define pinInput(x)       VPORTA.DIR &= ~(1<<(x))   // set pin to INPUT
#define pinOutput(x)      VPORTA.DIR |=  (1<<(x))   // set pin to OUTPUT
#define pinLow(x)         VPORTA.OUT &= ~(1<<(x))   // set pin to LOW
#define pinHigh(x)        VPORTA.OUT |=  (1<<(x))   // set pin to HIGH
#define pinToggle(x)      VPORTA.IN  |=  (1<<(x))   // TOGGLE pin
#define pinRead(x)        (VPORTA.IN &   (1<<(x)))  // READ pin
#define pinPullup(x)      (&PORTA.PIN0CTRL)[x] |= PORT_PULLUPEN_bm  // enable pullup
#define pinIntEnable(x)   (&PORTA.PIN0CTRL)[x] |= PORT_ISC_BOTHEDGES_gc
#define pinIntDisable(x)  (&PORTA.PIN0CTRL)[x] &= ~PORT_ISC_gm
#define pinIntFlagClr(x)  VPORTA.INTFLAGS = (1<<(x))

// Global variables
uint16_t maxspeed = 0;              // maximum speed
uint16_t maxday   = 0;              // maximum distance per day
uint32_t distance = 0;              // total distance

// ===================================================================================
// I2C Master Implementation (Write only)
// ===================================================================================

#define I2C_FREQ  100000                          // I2C clock frequency in Hz
#define I2C_BAUD  (F_CPU / I2C_FREQ - 10) / 2;    // simplified BAUD calculation

// I2C init function
void I2C_init(void) {
  TWI0.MBAUD   = I2C_BAUD;                        // set TWI master BAUD rate
  TWI0.MCTRLA  = TWI_ENABLE_bm;                   // enable TWI master
  TWI0.MSTATUS = TWI_BUSSTATE_IDLE_gc;            // set bus state to idle
}

// I2C start transmission
void I2C_start(uint8_t addr) {
  TWI0.MADDR = addr;                              // start sending address
}

// I2C stop transmission
void I2C_stop(void) {
  while(~TWI0.MSTATUS & TWI_WIF_bm);              // wait for last transfer to complete
  TWI0.MCTRLB = TWI_MCMD_STOP_gc;                 // send stop condition
}

// I2C transmit one data byte to the slave, ignore ACK bit
void I2C_write(uint8_t data) {
  while(~TWI0.MSTATUS & TWI_WIF_bm);              // wait for last transfer to complete
  TWI0.MDATA = data;                              // start sending data byte 
}

// ===================================================================================
// OLED Implementation
// ===================================================================================

// OLED definitions
#define OLED_ADDR       0x78    // OLED write address
#define OLED_CMD_MODE   0x00    // set command mode
#define OLED_DAT_MODE   0x40    // set data mode
#define OLED_INIT_LEN   11      // 9: no screen flip, 11: screen flip

// OLED init settings
const uint8_t OLED_INIT_CMD[] = {
  0xA8, 0x1F,                   // set multiplex for 128x32
  0x20, 0x01,                   // set vertical memory addressing mode
  0xDA, 0x02,                   // set COM pins hardware configuration to sequential
  0x8D, 0x14,                   // enable charge pump
  0xAF,                         // switch on OLED
  0xA1, 0xC8                    // flip the screen
};

// OLED 5x8 font (adapted from Neven Boyanov and Stephen Denne)
const uint8_t OLED_FONT_SMALL[] = {
  0x3E, 0x51, 0x49, 0x45, 0x3E, //  0 0
  0x00, 0x42, 0x7F, 0x40, 0x00, //  1 1
  0x42, 0x61, 0x51, 0x49, 0x46, //  2 2
  0x21, 0x41, 0x45, 0x4B, 0x31, //  3 3
  0x18, 0x14, 0x12, 0x7F, 0x10, //  4 4
  0x27, 0x45, 0x45, 0x45, 0x39, //  5 5
  0x3C, 0x4A, 0x49, 0x49, 0x30, //  6 6
  0x01, 0x71, 0x09, 0x05, 0x03, //  7 7
  0x36, 0x49, 0x49, 0x49, 0x36, //  8 8
  0x06, 0x49, 0x49, 0x29, 0x1E, //  9 9
  0x00, 0x00, 0x00, 0x00, 0x00, // 10 SPACE
  0x7C, 0x04, 0x18, 0x04, 0x78, // 11 m
  0x20, 0x10, 0x08, 0x04, 0x02, // 12 /
  0x7F, 0x08, 0x04, 0x04, 0x78, // 13 h
  0x38, 0x44, 0x44, 0x48, 0x7F, // 14 d
  0x7F, 0x02, 0x0C, 0x02, 0x7F, // 15 M
  0x20, 0x54, 0x54, 0x54, 0x78, // 16 a
  0x44, 0x28, 0x10, 0x28, 0x44, // 17 x
  0x00, 0x36, 0x36, 0x00, 0x00  // 18 :
};

// OLED 6x16 font
const uint8_t OLED_FONT_BIG[] = {
  0x7C, 0x1F, 0x02, 0x20, 0x02, 0x20, 0x02, 0x20, 0x02, 0x20, 0x7C, 0x1F, //  0 0
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x7C, 0x1F, //  1 1
  0x00, 0x1F, 0x82, 0x20, 0x82, 0x20, 0x82, 0x20, 0x82, 0x20, 0x7C, 0x00, //  2 2
  0x00, 0x00, 0x82, 0x20, 0x82, 0x20, 0x82, 0x20, 0x82, 0x20, 0x7C, 0x1F, //  3 3
  0x7C, 0x00, 0x80, 0x00, 0x80, 0x00, 0x80, 0x00, 0x80, 0x00, 0x7C, 0x1F, //  4 4
  0x7C, 0x00, 0x82, 0x20, 0x82, 0x20, 0x82, 0x20, 0x82, 0x20, 0x00, 0x1F, //  5 5
  0x7C, 0x1F, 0x82, 0x20, 0x82, 0x20, 0x82, 0x20, 0x82, 0x20, 0x00, 0x1F, //  6 6
  0x7C, 0x00, 0x02, 0x00, 0x02, 0x00, 0x02, 0x00, 0x02, 0x00, 0x7C, 0x1F, //  7 7
  0x7C, 0x1F, 0x82, 0x20, 0x82, 0x20, 0x82, 0x20, 0x82, 0x20, 0x7C, 0x1F, //  8 8
  0x7C, 0x00, 0x82, 0x20, 0x82, 0x20, 0x82, 0x20, 0x82, 0x20, 0x7C, 0x1F, //  9 9
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // 10 SPACE
  0x80, 0x00, 0x80, 0x3F, 0x80, 0x00, 0x80, 0x3F, 0x80, 0x00, 0x00, 0x3F, // 11 m
  0x00, 0x30, 0x00, 0x0C, 0x00, 0x03, 0xC0, 0x00, 0x30, 0x00, 0x0C, 0x00, // 12 /
  0xFC, 0x3F, 0x00, 0x01, 0x80, 0x00, 0x80, 0x00, 0x00, 0x3F, 0x00, 0x00  // 13 h
};

// Character definitions
#define COLON   15
#define SPACE   10

// OLED BCD conversion array
const uint32_t DIVIDER[] = {1, 10, 100, 1000, 10000, 100000, 1000000};

// OLED current page
uint8_t OLED_page;

// OLED init function
void OLED_init(void) {
  _delay_ms(10);                                  // give it some time for reset
  I2C_start(OLED_ADDR);                           // start transmission to OLED
  I2C_write(OLED_CMD_MODE);                       // set command mode
  for(uint8_t i=0; i<OLED_INIT_LEN; i++) 
    I2C_write(OLED_INIT_CMD[i]);                  // send the command bytes
  I2C_stop();                                     // stop transmission
}

// OLED set the cursor
void OLED_setCursor(uint8_t xpos, uint8_t ypos) {
  I2C_start(OLED_ADDR);                           // start transmission to OLED
  I2C_write(OLED_CMD_MODE);                       // set command mode
  I2C_write(0x22);                                // command for min/max page
  I2C_write(ypos);                                // min: ypos
  (ypos > 1) ? I2C_write(3) : I2C_write(ypos);    // max: depending
  I2C_write(xpos & 0x0F);                         // set low nibble of start column
  I2C_write(0x10 | (xpos >> 4));                  // set high nibble of start column
  I2C_write(0xB0 | (ypos));                       // set start page
  I2C_stop();                                     // stop transmission
  OLED_page = ypos;
}

// OLED clear screen
void OLED_clearScreen(void) {
  uint8_t i;                                      // count variable
  OLED_setCursor(0, 0);                           // set cursor at line 0
  I2C_start(OLED_ADDR);                           // start transmission to OLED
  I2C_write(OLED_DAT_MODE);                       // set data mode
  for(i = 128; i; i--) I2C_write(0x00);           // clear line
  I2C_stop();                                     // stop transmission

  OLED_setCursor(0, 1);                           // set cursor at line 1
  I2C_start(OLED_ADDR);                           // start transmission to OLED
  I2C_write(OLED_DAT_MODE);                       // set data mode
  for(i = 128; i; i--) I2C_write(0x02);           // draw line
  I2C_stop();                                     // stop transmission

  OLED_setCursor(0, 2);                           // set cursor at lower half
  I2C_start(OLED_ADDR);                           // start transmission to OLED
  I2C_write(OLED_DAT_MODE);                       // set data mode
  do {I2C_write(0x00);} while(--i);               // clear lower half
  I2C_stop();                                     // stop transmission
}

// OLED plot a character
void OLED_plotChar(uint8_t ch) {
  if(OLED_page) {                                 // big character ?
    ch = (ch << 3) + (ch << 2);                   // calculate position of character in font array
    I2C_write(0x00); I2C_write(0x00);             // print spacing between characters
    for(uint8_t i=12; i; i--)                     // 12 bytes per character
      I2C_write(OLED_FONT_BIG[ch++]);             // print character
    I2C_write(0x00); I2C_write(0x00);             // print spacing between characters
  }
  else {                                          // small character ?
    ch += (ch << 2);                              // calculate position of character in font array
    I2C_write(0x00);                              // print spacing between characters
    for(uint8_t i=5; i; i--)                      // 5 bytes per character
      I2C_write(OLED_FONT_SMALL[ch++]);           // print character
  }
}

// OLED print a "string"; terminator: 255
void OLED_printStr(const uint8_t* p) {
  I2C_start(OLED_ADDR);                           // start transmission to OLED
  I2C_write(OLED_DAT_MODE);                       // set data mode
  while(*p < 255) OLED_plotChar(*p++);            // plot each character of the string
  I2C_stop();                                     // stop transmission
}

// OLED print decimal (BCD conversion by substraction method)
void OLED_printDec(uint32_t value, uint8_t digits) {
  uint8_t leadflag = 0;                           // flag for leading spaces
  I2C_start(OLED_ADDR);                           // start transmission to OLED
  I2C_write(OLED_DAT_MODE);                       // set data mode
  while(digits--) {                               // for all digits digits
    uint8_t digitval = 0;                         // start with digit value 0
    uint32_t divider = DIVIDER[digits];           // read current divider
    while(value >= divider) {                     // if current divider fits into the value
      leadflag = 1;                               // end of leading spaces
      digitval++;                                 // increase digit value
      value -= divider;                           // decrease value by divider
    }
    if(leadflag || (!digits)) OLED_plotChar(digitval); // print the digit
    else OLED_plotChar(SPACE);                    // or print leading space
  }
  I2C_stop();                                     // stop transmission
}

// ===================================================================================
// Speed and Distance Measurement using TCA and Pin Chane Interrupt
// ===================================================================================

volatile uint16_t s_counter = 0;                  // speed counter
volatile uint32_t d_counter = 0;                  // distance counter

// Setup peripherals for speed and distance measurement
void TACHO_init(void) {
  // Enable pin change interrupt on HALL pin
  pinIntEnable(PIN_HALL);                         // enable pin change interrupt

  // Setup timer/counter A (TCA) for speed measurement
  TCA0.SINGLE.CTRLA = TCA_SINGLE_CLKSEL_DIV64_gc; // set TCA prescaler to 64 -> 15625Hz
  TCA0.SINGLE.INTCTRL = TCA_SINGLE_OVF_bm;        // enable TCA overflow interrupt
}

// TCA overflow interrupt service routine (speed measurement timeout)
ISR(TCA0_OVF_vect) {
  TCA0.SINGLE.CTRLA    &= ~TCA_SINGLE_ENABLE_bm;  // stop timer
  TCA0.SINGLE.INTFLAGS  =  TCA_SINGLE_OVF_bm;     // clear interrupt 
  s_counter = 0;                                  // set speed to zero
}

// Pin Change Interrupt service routine for HALL pin
ISR(PORTA_PORT_vect) {
  s_counter             = TCA0.SINGLE.CNT;        // read counter value
  TCA0.SINGLE.CTRLESET  = TCA_SINGLE_CMD_RESTART_gc;  // restart counter
  TCA0.SINGLE.CTRLA    |= TCA_SINGLE_ENABLE_bm;   // start counter (if stopped)
  d_counter            += CIRCUM / 2;             // add half circumference to distance
  pinIntFlagClr(PIN_HALL);                        // clear interrupt flag
}

// ===================================================================================
// Daily Interrupt using RTC (Real Time Counter)
// ===================================================================================

volatile uint32_t lastdistance = 0;               // distance counter at day start

// Setup Real Time Counter (RTC)
void RTC_init(void) {
  RTC.CLKSEL  = RTC_CLKSEL_INT1K_gc;              // choose internal 1024Hz clock
  while(RTC.STATUS > 0);                          // wait for RTC to be ready
  RTC.CTRLA   = RTC_PRESCALER_DIV32768_gc         // prescaler 32768
              | RTC_RTCEN_bm;                     // start RTC
  RTC.PER     = 2699;                             // set period to one day
                                                  // 24 * 3600 * 1024Hz / 32678 - 1
  RTC.INTCTRL = RTC_OVF_bm;                       // enable overflow interrupt
}

// Interrupt service routine for RTC (once a day)
ISR(RTC_CNT_vect) {
  RTC.INTFLAGS = RTC_OVF_bm;                      // clear interrupt flag
  lastdistance = d_counter;                       // save current total distance
}

// ===================================================================================
// EEPROM Functions
// ===================================================================================

// Update max speed and distance stored in EEPROM
void EEPROM_update(void) {
  eeprom_update_word((uint16_t*)0, EEPROM_IDENT);
  eeprom_update_word((uint16_t*)2, maxspeed);
  eeprom_update_word((uint16_t*)4, maxday);
  eeprom_update_word((uint16_t*)6, distance);
  eeprom_update_word((uint16_t*)8, distance >> 16);
}

// Read max speed and distance stored in EEPROM
void EEPROM_get(void) {
  uint16_t identifier = eeprom_read_word((const uint16_t*)0);
  if (identifier == EEPROM_IDENT) {
    maxspeed  = eeprom_read_word((const uint16_t*)2);
    maxday    = eeprom_read_word((const uint16_t*)4);
    distance  = (uint32_t)eeprom_read_word((const uint16_t*)8) << 16;
    distance |= eeprom_read_word((const uint16_t*)6);
  }
}

// ===================================================================================
// Main Function
// ===================================================================================

// Some "strings"
const uint8_t maxStr[] = {15, 16, 17, 18, 255};   // 'Max:'
const uint8_t msStr[]  = {11, 12, 13, 255};       // 'm/s'
const uint8_t mdStr[]  = {11, 12, 14, 255};       // 'm/d'
const uint8_t mStr[]   = {11, 255};               // 'm'

int main(void) {
  // Local variables
  uint8_t  resetcounter = RESDEL;                 // counter for RESET button delay
  uint16_t vspeed;                                // speed variable
  uint32_t daily;                                 // distance on current day

  // Setup
  _PROTECTED_WRITE(CLKCTRL.MCLKCTRLB, 7);         // set clock frequency to 1 MHz
  pinPullup(PIN_BUTTON);                          // enable pullup for button pin
  EEPROM_get();                                   // get stored data from EEPROM
  I2C_init();                                     // setup I2C
  OLED_init();                                    // setup OLED
  OLED_clearScreen();                             // clear screen
  RTC_init();                                     // init RTC for daily update
  TACHO_init();                                   // setup speed and distance measurement
  d_counter = distance * 100;                     // set distance counter
  lastdistance = d_counter;                       // set day start value

  // Loop
  while(1) {
    // Read current tacho values
    cli();                                        // disable interrupts for atomic read
    distance = d_counter;                         // read current distance counter
    vspeed   = s_counter;                         // read current speed counter
    daily    = lastdistance;                      // read distance on last day change
    sei();                                        // enable interrupts

    // Calculate distances
    daily     = distance - daily;                 // calculate distance on current day
    daily    /= 100;                              // calculate in m
    distance /= 100;                              // calculate total distance in m
    if(daily > maxday) maxday = daily;            // update max distance per day

    // Calculate speed
    if(vspeed) vspeed = (uint32_t)18 * 15625 * CIRCUM / vspeed;  // speed in m/h
    if(vspeed > maxspeed) maxspeed = vspeed;      // update maximum speed

    // Print values on OLED
    OLED_setCursor(1, 0);
    OLED_printStr(maxStr);                        // print 'Max:'
    OLED_printDec(maxspeed, 5);                   // print max speed
    OLED_printStr(msStr);                         // print 'm/s'
    OLED_printDec(maxday,  6);                    // print total distance
    OLED_printStr(mdStr);                         // print 'm/d'

    OLED_setCursor(0, 2);
    OLED_printDec(vspeed, 5);                     // print current speed
    OLED_printStr(msStr);                         // print 'm/s'
    OLED_printDec(distance, 7);                   // print total distance
    OLED_printStr(mStr);                          // print 'm'

    // Update EEPROM
    if(!vspeed) EEPROM_update();                  // only after a "walk"

    // Check RESET button
    if(pinRead(PIN_BUTTON)) resetcounter = RESDEL;// button not pressed -> reset delay
    else if(!resetcounter--) {                    // button pressed -> decrease counter
      maxday = 0;                                 // if counter is zero: reset max values
      maxspeed = 0;
    }

    // A little delay
    _delay_ms(100);                               // to slow down screen update
  }
}
