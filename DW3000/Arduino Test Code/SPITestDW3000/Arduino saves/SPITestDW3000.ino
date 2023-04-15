#include "SPI.h"
#include <stdlib.h>

#define chipSelectPin 10

void setup() {
  SPI.begin();
  Serial.begin(9600);
  
  pinMode(chipSelectPin, OUTPUT);
  digitalWrite(chipSelectPin, HIGH);
  
  attachInterrupt(8, interruptDetect, RISING);
  
  init_program();
}

void interruptDetect(){ //On calling interrupt
  Serial.println("ALAAAARM");
}

void testFunctionalityDEV_ID(){
  uint32_t val;
  digitalWrite(chipSelectPin, LOW);
  SPI.transfer(0x00); //Send command [RD/WR][8/16Bit][5 Bit Base Address][LSB=0] [DW3000 Family User Manual 2.3.1.2 Figure 2]
  val = SPI.transfer(0x00); //Read first 4 octets
  val |= (uint32_t)SPI.transfer(0xff) << 8;
  val |= (uint32_t)SPI.transfer(0xff) << 16;
  val |= (uint32_t)SPI.transfer(0xff) << 24;
  digitalWrite(chipSelectPin, HIGH);
  Serial.println(val, HEX);
  delay(5000);
}

void testFunctionalityDEV_IDSingleCell(){
  uint32_t val, val2;
  digitalWrite(chipSelectPin, LOW);
  SPI.transfer(0x40);
  SPI.transfer(0x8); 
  val = SPI.transfer(0x00); //Read first 4 octets
  digitalWrite(chipSelectPin, HIGH);
  Serial.println(val, HEX);
  delay(5000);
}

void instantTXThenRX(){
  /*
   * write to TX_BUFFER for data
   * write preamble length, frame length, data rate and prf in TX_FCTRLb  
   * write channel in CHAN_CTRL
   */
  uint32_t val;
  Serial.println("\nSending, then receiving.");
  digitalWrite(chipSelectPin, LOW);
  SPI.transfer(0x99);
  val = SPI.transfer(0x00); //Read first 4 octets
  digitalWrite(chipSelectPin, HIGH);
  Serial.println("Received result:");
  Serial.println(val, HEX);
}

uint32_t getShortAddressRead(int base[], int base_len){
  Serial.println("\n");

  int fill_base_len = 5;
  int fill_base[fill_base_len]; //fill leading zeros
  
  for(int i = 0; i < fill_base_len; i++){
    if (i < fill_base_len - base_len){
      fill_base[i] = 0;
    }else{
      fill_base[i] = base[i-(fill_base_len - base_len)];
    }
  }
  int firstByte_len = 8;
  int firstByte[firstByte_len] = {0, 0};
  for(int i = 0; i < fill_base_len; i++){
    firstByte[i+2] = fill_base[i];
  }
  firstByte[firstByte_len-1] = 0;

  Serial.println("Sending following byte to DW3000:");
  for (int i = 0; i < firstByte_len; i++){
    Serial.print(firstByte[i], DEC);
  }
  Serial.println("\n");

  int byteValue = 0;
  for(int i = 7; i > 0; i--){ //8 bit iteration
    byteValue = byteValue + pow(firstByte[i],7-i); 
  }

  uint32_t val, val2;
  digitalWrite(chipSelectPin, LOW);
  SPI.transfer(byteValue); //Send command [RD/WR][8/16Bit][5 Bit Base Address][MSB-LSB] [DW3000 Family User Manual 2.3.1.2 Figure 2]
  Serial.print("Received result: ");
  for(int i = 0; i < 64; i++){
    val = SPI.transfer(0x00); //Read first 4 octets
    val |= (uint32_t)SPI.transfer(0xff) << 8;
    val |= (uint32_t)SPI.transfer(0xff) << 16;
    val |= (uint32_t)SPI.transfer(0xff) << 24;
    Serial.println(val, HEX);
  }
  digitalWrite(chipSelectPin, HIGH);
}

uint32_t getFullAddressRead(int base[], int base_len, int sub[], int sub_len){ // !! base and sub have to have MSB ... LSB already!!
  Serial.println("\n");

  
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

  int first_byte[8] = {0, 1};
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
  Serial.println("Sending following bytes to DW3000:");
  for (int i = 0; i < 8; i++){
    Serial.print(first_byte[i], DEC);
  }
  Serial.println("");
  for (int i = 0; i < 8; i++){
    Serial.print(second_byte[i], DEC);
  }
  Serial.println("\n");

  int byteOne = 0;
  int byteTwo = 0;
  for(int i = 7; i > 0; i--){ //8 bit iteration
    byteOne = byteOne + pow(first_byte[i],7-i); 
    byteTwo = byteTwo + pow(second_byte[i],7-i);
  }

  uint32_t val;
  digitalWrite(chipSelectPin, LOW);
  SPI.transfer(byteOne);
  SPI.transfer(byteTwo); 
  
  Serial.print("Received result: ");
  for(int i = 0; i < 8; i++){
    val = SPI.transfer(0x00); //Read first 4 octets
    val |= (uint32_t)SPI.transfer(0xff) << 8;
    val |= (uint32_t)SPI.transfer(0xff) << 16;
    val |= (uint32_t)SPI.transfer(0xff) << 24;
    Serial.println(val, HEX);
  }
  digitalWrite(chipSelectPin, HIGH);
}

void init_program(){
  Serial.println("\n+++ DecaWave DW3000 Test +++\n");
}

void loop() {
  instantTXThenRX();
  int base[] = {0, 0};
  int sub[] = {1,0,0};
  //getFullAddressRead(base, 2, sub, 2);
  //getShortAddressRead(base, 2);
  getFullAddressRead(base, 2, sub, 3);
  delay(5000);
}
