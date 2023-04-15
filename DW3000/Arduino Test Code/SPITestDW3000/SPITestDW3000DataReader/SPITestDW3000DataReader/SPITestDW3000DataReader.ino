#include "SPI.h"
#include <stdlib.h>
#include <DW3000.h>

#define chipSelectPin 10
#define anchorID 0
#define led1 3
#define led2 4

volatile boolean IRQPulled = true;

long timer = 0;

int *sub;
int *base;
DW3000 dw3000;
 
void setup() {
  Serial.begin(9600);
  delay(500);
  dw3000.init();
  dw3000.readInit();
}

void loop() {

}
