#include <DW3000.h>
#include <DW3000Constants.h>

void setup() {  
  Serial.begin(9600);
  delay(20);
  DW3000.begin();
  DW3000.init();
}

void loop() {
  DW3000.standardTX();
  delay(500);
}
