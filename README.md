# DW3000_Arduino
A simple, not beautiful but working basic DW3000 Library

## Introduction
Hey folks!

For over a year now I programmed a library - based on makerfabs gitlab project - that is compatible with the [ESP32 UWB DW3000 PCB](https://www.makerfabs.com/esp32-uwb-dw3000.html) (WROOM version tested).
The project was primarely focussed on giving an alternative to the [official makerfabs library](https://github.com/Makerfabs/Makerfabs-ESP32-UWB-DW3000) that is easy to understand and adapt. 

## Hardware
For the two currently existing examples (tx and rx) you will need two [ESP32 UWB DW3000 PCBs](https://www.makerfabs.com/esp32-uwb-dw3000.html).

## Installation
Just download the zip, extract it in your Arduino's "library" folder, restart your Arduino IDE and open the two examples under `file > examples > DW3000 Arduino`.

Additionally, you'll have to install the ESP32 Add-on for Arduino IDE, following [this example](https://wiki.makerfabs.com/Installing_ESP32_Add_on_in_Arduino_IDE.html).


## Current problems
- [x] After uploading code to your chip, you need to unplug and replug the chip for the example to work.
