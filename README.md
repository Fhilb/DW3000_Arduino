# DW3000_Arduino
A simple, not beautiful but working basic DW3000 Library

## Introduction
Hey folks!
As I am currently trying out the fairly new Qorvo DW3000 modules, I wanted to share most of my work with you so far. As I am not a pro in C and C++, it was kept as simple as possible, trying to give people who don't understand every last bit of C (like myself) some insight on how it works.
This library should be used as a basic introduction in the topic, as it took me a few weeks to get through all the trouble of documentation and trial and error. It should serve as a solid foundation for whatever purpose you need it. :)

## Hardware
For the whole project, I am using an Arduino Mega paired with the DWM3000EVB Board.  (!Be aware that Arduino Unos might not work due to flash capacity limitations!)
For me, I had to add a pulldown resistor (around 10k Ohm) to the IRQ line, as it got pulled high randomly and you wouldn't want the Arduino getting interrupted if there is no event. I attached the interrupt on digital pin 2, so that I could add a resistor inbetween (As seen in the picture below).

![alt text](https://i.ibb.co/Bcj6gdF/arduino-DW3000-Pulldown.png)

With this little adjustment, we are ready to go.

## Installation
Just download the zip, extract it in your Arduino's "library" folder, implement it with the "#include <DW3000.h>" tag and create an instance of the library outside your setup() method with the following code: "DW3000 dw3000;" or "DW3000 dw3000(23)" to use it with an anchor id (in this case 23 as id). Now you can use the object dw3000 to call methods, e.g. dw3000.init(); and so on.
Have fun!

```
#include <DW3000.h>

void setup() {
/* Standard configuration that has to be done in every setup: */
  attachInterrupt(digitalPinToInterrupt(2), DW3000.interruptDetect, RISING);
  pinMode(10, OUTPUT); //for now it has to be initalized manually. TODO
  Serial.begin(9600); 
  SPI.begin();
  
  
  /*Your code: */
  
  DW3000.init(); 
  int data[] = {0x03, 0xFA};
  DW3000.write(0x0, 0x4, data, 2);
  //setup code here
}

void loop() {
  //put your main code here
}
```

## Software
For now, the library consists of the following functions:

### init()
This method will write all the necessary data to the DW3000 to work properly, e.g. data rate, channel, preamble length, but will also add all the "easy to miss" things pointed out by [github user egnor](https://gist.github.com/egnor/455d510e11c22deafdec14b09da5bf54).
#### !! The init method has to be used before interacting with the DW3000 in any other way, because the Serial monitor and SPI are initialized there too !!

### getAnchorID()
You can initialize the DW3000 library with an anchor id, which can be got back at a later time by this method. If no anchor id was given, it returns -1.

### read(int base, int sub)
Returns an Integer containing the result of the first 4 octets given by the DW3000. 
Be aware, that the result cuts away leading zeros, meaning that if you are expecting the first 3 octets to be 0x0 and the last one to be 0xA, it will just return 0xA - 1010 in binary - cutting away the other 28 zeros in front.
the "base" and "sub" parameters accept any Integer value. It is recommended to just write in Hex format to avoid confusion, as the DW3000 Family Manual uses the hex format as well. 

### write(int base, int sub, data, data_len)
The write command returns 0, as it consists of the same method as the read() method with a few tweaks. This can be ignored.
In order to use the write command, you first need to create an int array, containing the data you want to transmit. 
Index 0 will be sent first, keep that in mind as the DW3000 manual often shows the register starting on the righthand side, which then would be written to first with index 0 of the array.
"data_len" defines the number of values in the data array. 
#### !! Keep in mind that you can't use your data array again after using it in the write() method as the allocated space of the array will be freed automatically after finishing the method !!

### setLED1(bool status)
Sets LED1 (TX LED) to high or low. You can use it with the "HIGH" or "LOW" parameters.

### setLED2(bool status)
Same as LED1, just with LED2 (RX LED).

### readInit()
readInit() uses the same registers that init() wrote to, but outputs its data. This is to manually check if anything isn't configured as it is supposed to and will *hopefully* be removed in future versions. 

## Examples
Soon to come :)




