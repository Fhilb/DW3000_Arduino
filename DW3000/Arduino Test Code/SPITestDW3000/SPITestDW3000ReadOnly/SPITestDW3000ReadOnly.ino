#include <DW3000.h>

bool mode = 0
bool sent_rx = false

void setup() {
  Serial.begin(115200);
  DW3000.begin();
  DW3000.init();
}




void loop() {
  if(mode){ //TX
    DW3000.standardTX();
    
    delay(1000);
  }else{ //RX
    if(!sent_rx){
      DW3000.standardRX();
      sent_rx = true;
    }
  }
}
