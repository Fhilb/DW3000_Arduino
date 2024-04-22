# DW3000_Arduino
A simple DW3000 library for the ESP32

## Introduction
This library is compatible with the [ESP32 UWB DW3000 PCB](https://www.makerfabs.com/esp32-uwb-dw3000.html) (WROOM version tested) chip made by makerfabs.
The project was primarely focused on giving an alternative to the [official makerfabs library](https://github.com/Makerfabs/Makerfabs-ESP32-UWB-DW3000) that is easy to understand and adapt. 

## Hardware
For the currently existing examples you will need two [ESP32 UWB DW3000 PCBs](https://www.makerfabs.com/esp32-uwb-dw3000.html).

## Limitations
The [DWM3000EVB](https://www.qorvo.com/products/p/DWM3000EVB) chip is **NOT** compatible with this library if paired with an Arduino Uno!
This is due to its limitation in cpu speed (16MHz, compared to the ESP32 with a 240MHz cpu speed).

## Installation
Just download the zip, extract it in your Arduino's "library" folder, restart your Arduino IDE and open the two examples under `file > examples > DW3000 Arduino`.

Additionally, you'll have to install the ESP32 Add-on for Arduino IDE, following [this manual](https://wiki.makerfabs.com/Installing_ESP32_Add_on_in_Arduino_IDE.html).

## Examples
Please refer to dw3000_doublesided_ranging_ping and dw3000_doublesided_ranging_pong to get the most reliable ranging results. 

### dw3000_tx and dw3000_rx
One chip sends a predefined message, the other chip receives the messages. 
Useful for checking for hardware faults as it is the easiest example.

### dw3000_ping and dw3000_pong
The ping device sends a message to the pong device, which increments the message by 1 and sends the incremented message back to the ping device.
The output on the ping device should be incremented by 2 per cycle.

### dw3000_ping_with_timestamp and dw3000_pong_with_timestamp
The ping device sends a message to the pong device, which after a fixed delay sends a message back. By then comparing the send and receive timestamps on the ping device a distance between both devices can be estimated. This is a single-sided two-way-ranging system which due to its design isn't as accurate as a double-sided two-way-ranging system for example.

### dw3000_doublesided_ranging_ping and dw3000_doublesided_ranging_pong
This is the recommended way to get more accurate ranging results.

This approach uses 3 ranging messages to evaluate the distance between both devices. This not only limits the influence of the clock offset between the chips but also averages the results between those 3 messages.

Be aware that all methods that are exclusively used in double-sided ranging have a "ds_" as a prefix.

## Method descriptions
This chapter describes the most important methods the library features.

### begin()
Has to be called before anything else is done with the library. The method configures the SPI bus to communicate with the DW3000 chip. 

### init()
Checks the connection towards the chip by reading known data from the chip and comparing it to what the data should look like. Soft resets the DW3000 chip and configures the chip.

### hardReset()
Physically resets the DW3000 chip by pulling the reset line low. This is useful if the chip didn't get disconnected from power to ensure that no registers are still set.

### setupGPIO()
Configures the GPIO pins on the DW3000 chip. These pins can be used to light some status LEDs.

### pullLEDHigh(int led)
Lights a LED. 0-2 are valid values and control a different LED each.

### pullLEDDown(int led)
Shuts off a LED. 0-2 are valid values and control a different LED each.

### ds_processRTInfo(int t_roundA, int t_replyA, int t_roundB, int t_replyB, int clk_offset)
Uses all the given parameters to calculate the distance between the chips after doing a double-sided ranging. Returns a time value that is in DW3000 custom units of ~15.65ps per unit. 

Should be processed by convertToCM() to get a usable result.


## Current problems
- [x] ~~After uploading code to your chip, you need to unplug and replug the chip for the example to work.~~ (Fixed by adding hardReset() function)
- [x] ~~Write method isn't user-friendly~~ (Updated)
