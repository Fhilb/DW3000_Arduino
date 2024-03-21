# DW3000_Arduino
A simple, not beautiful but working basic DW3000 Library

## Introduction
Hey folks!

For over a year now I programmed a library - based on makerfabs gitlab project - that is compatible with the [ESP32 UWB DW3000 PCB](https://www.makerfabs.com/esp32-uwb-dw3000.html) (WROOM version tested).
The project was primarely focussed on giving an alternative to the [official makerfabs library](https://github.com/Makerfabs/Makerfabs-ESP32-UWB-DW3000) that is easy to understand and adapt. 

## Hardware
For the currently existing examples (tx and rx) you will need two [ESP32 UWB DW3000 PCBs](https://www.makerfabs.com/esp32-uwb-dw3000.html).

## Installation
Just download the zip, extract it in your Arduino's "library" folder, restart your Arduino IDE and open the two examples under `file > examples > DW3000 Arduino`.

Additionally, you'll have to install the ESP32 Add-on for Arduino IDE, following [this example](https://wiki.makerfabs.com/Installing_ESP32_Add_on_in_Arduino_IDE.html).

## Examples
### dw3000_tx and dw3000_rx
One chip sends a predefined message, the other chip receives the messages. 
Useful for checking for hardware faults as it is the easiest example.

### dw3000_ping and dw3000_pong
The ping device sends a message to the pong device, which increments the message by 1 and sends the incremented message back to the ping device.
The output on the ping device should be incremented by 2 per cycle.

### dw3000_ping_with_timestamp and dw3000_pong_with_timestamp
The ping device sends a message to the pong device, which after a fixed delay sends a message back. By then comparing the send and receive timestamps on the ping device a distance between both devices can be estimated. This is a single-sided two-way-ranging system which due to its design isn't as accurate as a double-sided two-way-ranging system for example.

## Current problems
- [x] After uploading code to your chip, you need to unplug and replug the chip for the example to work.
