# DW3000_Arduino
A simple, not beautiful but working basic DW3000 Library

## Introduction
Hey folks!
As I am currently trying out the fairly new Qorvo DW3000 modules, I wanted to share most of my work with you so far. As I am not a pro in C and C++, it was kept as simple as possible, trying to give people who don't understand every last bit of C (like myself) some insight on how it works.
This library should be used as a basic introduction in the topic, as it took me a few weeks to get through all the trouble of documentation and trial and error. It should serve as a solid foundation for whatever purpose you need it. :)

## Hardware
For the whole project, I am using an Arduino Uno Rev 3 paired with the DWM3000EVB Board. These boards are quite nice, as they fit the Arduino perfectly and can be just plugged ontop. 
But as always, it is not quite as nice as it looks like. For me, I had to add a pulldown resistor to the IRQ line, as it got pulled high randomly and you wouldn't want the Arduino getting interrupted if there is no event. I attached the interrupt on digital pin 2, so that I could add a resistor inbetween (As seen in the picture below).

![alt text](https://i.ibb.co/Bcj6gdF/arduino-DW3000-Pulldown.png)

With this little adjustment, we are ready to go.

## Software
For now, the library consists of the following functions:
# getAnchorID()
You can initialize the DW3000 library with an anchor id, which can be got back at a later time by this method. If no anchor id was given, it returns -1.

I will add examples over time, but for now, just download the zip, extract it in your Arduino's "library" folder and implement it with the "#include <DW3000.h>" tag. 





