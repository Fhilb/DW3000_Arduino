#include "SPI.h"
#include <stdlib.h>

#define chipSelectPin 10
#define anchorID 0
boolean IRQPulled = false;

void setup() {
  SPI.begin();
  Serial.begin(9600);
  
  pinMode(chipSelectPin, OUTPUT);
  pinMode(2, INPUT); //WITH EXTERNAL PULLDOWN RESISTOR!!
  digitalWrite(chipSelectPin, HIGH);

  
  attachInterrupt(digitalPinToInterrupt(2), interruptDetect, RISING);

  init_program();
  resetIRQStatusBits();
}


void interruptDetect(){ //On calling interrupt
  Serial.println("\n\nALAAAARM\n\n"); 

  int base[] = {0,0};
  int sub[] = {1,0,0,0,1,0,0};
  fullAddressRead(base, 2, sub, 7);
  resetIRQStatusBits();

  Serial.println("Finished interrupt");

  /*
   * clear bit by writing 1 to it
   * get received data
   * check if received data is anchorID-1
   * if so: instantTXThenRX()
   * else: initiateRX()
   */
}


//Working
void writeShortCommand(int cmd[], int cmd_len){
  Serial.println("\nShort Command WRITE:");

  int fill_base_len = 5;
  int fill_base[fill_base_len]; //fill leading zeros
  
  for(int i = 0; i < fill_base_len; i++){
    if (i < fill_base_len - cmd_len){
      fill_base[i] = 0;
    }else{
      fill_base[i] = cmd[i-(fill_base_len - cmd_len)];
    }
  }
  int cmd_finished[8] = {1,0};
  for(int i = 0; i < fill_base_len; i++){
    cmd_finished[i+2] = fill_base[i];
  }
  cmd_finished[7] = 1;

  int byteOne = 0;
   
  for(int i = 7; i >= 0; i--){ //8 bit iteration    
    byteOne = byteOne + cmd_finished[i] * round(pow(2,7-i)); 
  }
  Serial.println(byteOne, BIN);
  int bytes[] = {byteOne};
  sendBytes(bytes, 1, 0);
  Serial.println("Finished writing to Short Command.");
}

void initTXData(){
  //ToDo init 
}

//WORKING
void init_program(){
  Serial.println("\n+++ DecaWave DW3000 Test +++\n");
  
  int base[] = {0, 0};
  int sub[] = {1,1,1,1,0,0};
  int data[] = {0xF0, 0x20, 0x0, 0x0}; //0x80, 0x20

  fullAddressWrite(base, 2, sub, 6, data, 4); //Set IRQ for successful received data frame
}

//WORKING
void resetIRQStatusBits(){
  Serial.println("\nIRQ RESET");
  int base[] = {0, 0};
  int sub[] = {1,0,0,0,1,0,0};
  int data[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x1F};

  fullAddressWrite(base, 2, sub, 7, data, 6); //clear status register
  Serial.println("\nReading CLEARED status bits...");
  fullAddressRead(base, 2, sub, 7);
}

void getIRQBit(){
  int base[] = {0, 0};
  int sub[] = {1,0,0,0,1,0,0};
  int sub2[] = {1,0,0,1,0,0,0};
  Serial.println("Getting IRQ Bit");
  uint32_t firstSectorBit =   fullAddressRead(base, 2, sub, 7); //read status register
  uint32_t secondSectorBit =   fullAddressRead(base, 2, sub2, 7); //read status register
  Serial.println("Working on sector bits...");
  Serial.println(firstSectorBit, BIN);
  Serial.println(secondSectorBit, BIN);
  secondSectorBit = secondSectorBit >> 19;
  
  Serial.println(secondSectorBit, BIN);
}

//Working
void instantTXThenRX(){
  /*
   * write to TX_BUFFER for data
   * write preamble length, frame length, data rate and prf in TX_FCTRLb  
   * write channel in CHAN_CTRL
   */
   int base[] = {1,0,1,0,0};
   int sub[] = {0};
   int data[] = {anchorID};
   fullAddressWrite(base, 5, sub, 1, data, 1); //Writing to TX Buffer
   Serial.println("Finished writing to TX Buffer.");
   int shCmd[] = {1,1,0,0};
   writeShortCommand(shCmd, 4);
}

void initiateRX(){
  /*
   * write preamble length, frame length, data rate and prf in TX_FCTRLb  
   * write channel in CHAN_CTRL
   */
   int shCmd[] = {1,0};
   writeShortCommand(shCmd, 2);
}


/*
 * START OF IMPORTANT METHODS
 */


//Working
uint32_t sendBytes(int b[], int lenB, int recLen){ //WORKING
  digitalWrite(chipSelectPin, LOW);
  Serial.println("Sending following bytes:");
  for(int i = 0; i < lenB; i++){
    SPI.transfer(b[i]);
    Serial.print(b[i], HEX);
    Serial.print(" ");
  }
  Serial.println("");
  int rec;
  uint32_t val, tmp;
  if(recLen > 0){
    for(int i = 0; i < recLen; i++){
      tmp = SPI.transfer(0x00);
      if(i == 0){
        val = tmp; //Read first 4 octets
      }else{
        val |= (uint32_t)tmp << 8*i;
      }
    }
  }else{
    val = 0;
  }
  digitalWrite(chipSelectPin, HIGH); 
  return val;
}



//Working
uint32_t readOrWriteFullAddress(int base[], int base_len, int sub[], int sub_len, int data[], int data_len, int readWriteBit){
  int fill_base_len = 5;
  int fill_base[fill_base_len]; //fill leading zeros
  
  for(int i = 0; i < fill_base_len; i++){
    if (i < fill_base_len - base_len){
      fill_base[i] = 0;
    }else{
      fill_base[i] = base[i-(fill_base_len - base_len)];
    }
  }
 
  int fill_sub_len = 7;
  int fill_sub[fill_sub_len]; //fill leading zeros  
  for(int i = 0; i < fill_sub_len; i++){
    if (i < fill_sub_len - sub_len){
      fill_sub[i] = 0;
    }else{
      fill_sub[i] = sub[i-(fill_sub_len - sub_len)];
    }
  }

  int first_byte[8] = {readWriteBit, 1};
  for (int i = 0; i < fill_base_len; i++){
    first_byte[i+2] = fill_base[i];
  }
  first_byte[7] = fill_sub[0];

  int second_byte[8];
  second_byte[8-1] = 0; //Last two bits are set to 0 (mode selector bits)
  second_byte[8-2] = 0;

  for (int i = 0; i < fill_sub_len-1; i++){
    second_byte[i] = fill_sub[i+1];
  }

  int byteOne = 0;
  int byteTwo = 0;
 
  for(int i = 7; i >= 0; i--){ //8 bit iteration
    /*Serial.print(byteOne);
    Serial.print(" + ");
    Serial.print(first_byte2[i]);
    Serial.print(" * ");
    Serial.print(" 2^");
    Serial.print(7-i);
    Serial.print(" = ");*/
    
    byteOne = byteOne + first_byte[i] * round(pow(2,7-i)); 
    Serial.println(byteOne);

    byteTwo = byteTwo + second_byte[i] * round(pow(2,7-i));
  }

  uint32_t val;

  int bytes[data_len + 2] = {byteOne, byteTwo};

  for(int i = 0; i < data_len; i++){
    bytes[i+2] = data[i];
  }
  
  uint32_t res;
  
  if(readWriteBit == 0){
    res = (uint32_t)sendBytes(bytes, 2 + data_len, 4);
    Serial.print("Received result (HEX): ");
    Serial.print(res, HEX);
    Serial.print(" (BIN): ");
    Serial.println(res, BIN);
  }else{
    res = (uint32_t)sendBytes(bytes, 2 + data_len, 0);
  }
  return res;
}



//WORKING
uint32_t fullAddressRead(int base[], int base_len, int sub[], int sub_len){ // !! base and sub have to have MSB ... LSB already!!
  Serial.println("\n=== Full Address READ ===");
  int t[] = {0};
  return readOrWriteFullAddress(base, base_len, sub, sub_len, t, 0, 0);
}
//WORKING
uint32_t fullAddressWrite(int base[], int base_len, int sub[], int sub_len, int data[], int data_len){ // !! base and sub have to have MSB ... LSB already!!
  Serial.println("\n=== Full Address WRITE ===");
  return readOrWriteFullAddress(base, base_len, sub, sub_len, data, data_len, 1);
}



/*
 * MAIN LOOP
 */


void loop() {
  /*if(IRQPulled){
    Serial.println("IRQ Pulled");
    getIRQBit();
    resetIRQStatusBits();
    IRQPulled = false;
  }*/

  
  int base[] = {0, 0};
  int sub[] = {1,1,1,1,0,0};
  int sub2[] = {1,0,0,0,0,0,0};
  int data[] = {0xFF, 0xFF, 0xFF, 0x80, 0xFF, 0x1F}; 
  
  int bytes[] = {0x83}; //HEX Code for TX
  int bytes2[] = {0x40, 0xF0}; //HEX Code for SYS_STATUS ACCESS
  //fullAddressRead(base, 2, sub, 6);

  instantTXThenRX();
  delay(5000);
}
