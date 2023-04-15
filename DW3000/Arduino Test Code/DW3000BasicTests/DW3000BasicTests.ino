#include <DW3000.h>
#include <DW3000Constants.h>

void setup() {
  int cfg[] = {
    CHANNEL_5,
    PREAMBLE_128, // Preamble length
    PAC8, // PAC size
    9, // TX preamble code
    9, // RX preamble code
    DATARATE_6_8MB, // Data rate
    PHR_MODE_STANDARD, // PHY header mode
    PHR_RATE_6_8MB, // PHY header data rate (typically same as data rate)
    (128 + 1 + 8 - 8) // SFD timeout (preamble length + 1 + SFD length - PAC size)
  };

  DW3000.config = cfg;
  Serial.begin(9600);
  delay(20);
  DW3000.begin();

  DW3000.softReset();

  while(!DW3000.checkForIDLE()){
    Serial.println("[ERROR] IDLE FAILED! ATTEMPTING SOFT RESET...");
    DW3000.softReset();
    delay(500);
  }

  DW3000.init();
  
  Serial.println(DW3000.read(0x0,0x0), HEX);
   Serial.println(DW3000.checkForIDLE());
  //Serial.println((DW3000.read(0x0,0x44)>>16), BIN);
}

void loop() {
  // put your main code here, to run repeatedly:

}
