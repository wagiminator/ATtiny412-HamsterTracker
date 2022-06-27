# Hamster Wheel Tracker based on ATtiny402/412
The Hamster Wheel Tracker is a simple fitness monitoring device for your hamster. The speed and distance traveled by the hamster in the wheel are measured using two opposing neodymium magnets with complementary poles, which run past the device's bipolar Hall sensor switch. The recorded values are displayed on an OLED. The button is used to reset the stored values for the maximum speed reached and the maximum distance traveled in a day. The total distance covered since the device was started up is retained. The corresponding values are stored in the EEPROM so that they are not lost after a power supply interruption.

- Design Files (EasyEDA): https://easyeda.com/wagiminator/

![pic3.jpg](https://raw.githubusercontent.com/wagiminator/ATtiny412-HamsterTracker/main/documentation/HamsterTracker_pic3.jpg)

# Hardware
## Schematic
![wiring.png](https://raw.githubusercontent.com/wagiminator/ATtiny412-HamsterTracker/main/documentation/HamsterTracker_wiring.png)

## TLE4935L Bipolar Hall Sensor Switch
The TLE4935L Hall-effect IC includes Hall generator, amplifier and Schmitt-Trigger on one chip. The internal reference provides the supply voltage for the components. A magnetic field perpendicular to the chip surface induces a voltage at the hall probe. This voltage is amplified and switches a Schmitt-Trigger with open-collector output. A protection diode against reverse power supply is integrated. The output is protected against electrical disturbances.

When a positive magnetic field is applied and the turn-on magnetic induction is exceeded, the output of the Hall-effect IC will conduct (Operate Point). The output state does not change unless a reverse magnetic field exceeding the turn-off magnetic induction is applied. In this case the output will turn off (Release Point).

![pic2.jpg](https://raw.githubusercontent.com/wagiminator/ATtiny412-HamsterTracker/main/documentation/HamsterTracker_pic2.jpg)

# Software
## Implementation
The pin change interrupt is activated on the pin that is connected to the output of the Hall sensor. The Timer/Counter A (TCA) is set up in such a way that it runs with a clock frequency of 15625Hz and triggers an interrupt in the event of an overflow.

The passing of the magnets under the Hall sensor triggers a pin change interrupt, as long as both magnets pass alternately. Passing the same magnet multiple times does not change the signal of the Hall sensor and therefore does not cause an interrupt. In this way, a "swinging" of the hamster wheel is not mistakenly recognized as a movement.

In the pin change interrupt service routine, the value of counter A is read out and buffered. The speed is later determined from this value in the main program. The counter is then reset. the variable for the distance traveled is increased by half the wheel circumference. If the wheel does not turn for a few seconds, the timer overflow interrupt is triggered, in the service routine of which the timer is stopped.

```c
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
```

At this point, the Timer/Counter B (TCB) could have been used instead of TCA and pin change interrupts, especially since the TCB is intended specifically for frequency measurements. However, this is a bit less flexible with the clock frequency and has no overflow interrupt and also does not bring any advantages for this application, since latency times are not important here.

## Compiling and Uploading the Firmware
- Open the sketch and set the cirumference of the hamster wheel in cm at the beginning of the code: `#define CIRCUM 40`.
- Connect your [programmer](https://github.com/wagiminator/AVR-Programmer) to your PC and to the UPDI header on the board.
- Use one of the following methods to compile and upload the firmware:

### If using the Arduino IDE
- Open your Arduino IDE.
- Make sure you have installed [megaTinyCore](https://github.com/SpenceKonde/megaTinyCore).
- Go to **Tools -> Board -> megaTinyCore** and select **ATtiny412/402/212/202**.
- Go to **Tools** and choose the following board options:
  - **Chip:**           ATtiny402 or ATtiny412
  - **Clock:**          1 MHz internal
  - Leave the rest at the default settings.
- Go to **Tools -> Programmer** and select your UPDI programmer.
- Go to **Tools -> Burn Bootloader** to burn the fuses.
- Open the sketch and click **Upload**.

### If using the makefile (Linux/Mac)
- Download [AVR 8-bit Toolchain](https://www.microchip.com/mplab/avr-support/avr-and-arm-toolchains-c-compilers) and extract the sub-folders (avr, bin, include, ...) to /software/tools/avr-gcc. To do this, you have to register for free with Microchip on the download site.
- Open a terminal.
- Navigate to the folder with the makefile and the sketch.
- Run `DEVICE=attiny412 PROGRMR=serialupdi PORT=/dev/ttyUSB0 make install` to compile, burn the fuses and upload the firmware (change DEVICE, PROGRMR and PORT accordingly).

# Building and Operating Instructions
1. Glue two neodymium magnets to the hamster wheel opposite each other, equidistant from the center and with opposite magnetic polarity. Make sure that the magnets are glued so tightly that they do not fall off even at high rotational speeds. 
2. Tape the back of the PCB and the Hall sensor pins with electrical tape so the hamster cage can't cause short circuits. 
3. Attach the device to the cage so that the magnets pass directly under the hall sensor.
4. Connect the device to a 3.3V - 5V power supply via the USB-C socket or the UPDI header. 
5. Press and hold the button for a few seconds to reset the maximum values recorded.

![pic4.jpg](https://raw.githubusercontent.com/wagiminator/ATtiny412-HamsterTracker/main/documentation/HamsterTracker_pic4.jpg)
![pic5.jpg](https://raw.githubusercontent.com/wagiminator/ATtiny412-HamsterTracker/main/documentation/HamsterTracker_pic5.jpg)

# References, Links and Notes
1. [ATtiny412 Datasheet](https://ww1.microchip.com/downloads/aemDocuments/documents/MCU08/ProductDocuments/DataSheets/ATtiny212-214-412-414-416-DataSheet-DS40002287A.pdf)
2. [TLE4935L Datasheet](https://datasheet.lcsc.com/lcsc/2005300119_Infineon-Technologies-TLE4935L_C539801.pdf)

# License
![license.png](https://i.creativecommons.org/l/by-sa/3.0/88x31.png)

This work is licensed under Creative Commons Attribution-ShareAlike 3.0 Unported License. 
(http://creativecommons.org/licenses/by-sa/3.0/)
