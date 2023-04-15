#include "SPI.h"
#include <stdlib.h>

#define chipSelectPin 10
#define anchorID 0
#define led1 3
#define led2 4

volatile boolean IRQPulled = true;

long timer = 0;

/*
 * base register arrays with leading fill zeroes to 5 bit
 */
 int base0x0[] = {0,0,0,0,0};
 int base0x1[] = {0,0,0,0,1};
 int base0x3[] = {0,0,0,1,1};
 int base0x4[] = {0,0,1,0,0};
 int base0x5[] = {0,0,1,0,1};
 int base0x6[] = {0,0,1,1,0};
 int base0x7[] = {0,0,1,1,1};
 int base0x9[] = {0,1,0,0,1};
 int base0xA[] = {0,1,0,1,0};
 int base0x12[] = {1,0,0,1,0};
 int base0x14[] = {1,0,1,0,0};
 int base0x1F[] = {1,1,1,1,1};

 /*
  * sub register arrays with leading fill zeroes to 7 bit
  */
 int sub0x0[] = {0,0,0,0,0,0,0};
 int sub0x2[] = {0,0,0,0,0,1,0};
 int sub0x4[] = {0,0,0,0,1,0,0};
 int sub0x8[] = {0,0,0,1,0,0,0};
 int sub0xC[] = {0,0,0,1,1,0,0};
 int sub0xE[] = {0,0,0,1,1,1,0};
 int sub0x14[] = {0,0,1,0,1,0,0};
 int sub0x18[] = {0,0,1,1,0,0,0};
 int sub0x1A[] = {0,0,1,1,0,1,0};
 int sub0x1C[] = {0,0,1,1,1,0,0};
 int sub0x20[] = {0,1,0,0,0,0,0};
 int sub0x24[] = {0,1,0,0,1,0,0};
 int sub0x38[] = {0,1,1,1,0,0,0};
 int sub0x3C[] = {0,1,1,1,1,0,0};
 int sub0x40[] = {1,0,0,0,0,0,0};
 int sub0x44[] = {1,0,0,0,1,0,0};
 int sub0x48[] = {1,0,0,1,0,0,0};
 int sub0x4C[] = {1,0,0,1,1,0,0};
 int sub0x50[] = {1,0,1,0,0,0,0};
 int sub0x51[] = {1,0,1,0,0,0,1};

 
void setup() {
  SPI.begin();
  Serial.begin(9600);

  delay(500);
}

//Working
uint32_t sendBytes(int b[], int lenB, int recLen){ //WORKING
  digitalWrite(chipSelectPin, LOW);
  //Serial.println("Sending following bytes:");
  for(int i = 0; i < lenB; i++){
    SPI.transfer(b[i]);
    //Serial.print(b[i], HEX);
    //Serial.print(" ");
  }
  //Serial.println("");
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
    //Serial.println(byteOne);

    byteTwo = byteTwo + second_byte[i] * round(pow(2,7-i));
  }

  uint32_t val;

  int bytes[data_len + 2] = {byteOne, byteTwo};

  for(int i = 0; i < data_len; i++){
    bytes[i+2] = data[i];
  }
  
  uint32_t res;
  
  if(readWriteBit == 0){
    Serial.print("Reading from ");
    for(int i = 0; i < fill_base_len; i++){
      Serial.print(fill_base[i]);
    }
    Serial.print(":");
    for(int i = 0; i < fill_sub_len; i++){
      Serial.print(fill_sub[i]);
    }
    Serial.println("");
    res = (uint32_t)sendBytes(bytes, 2 + data_len, 4);
    Serial.print("Received result (HEX): ");
    Serial.print(res, HEX);
    Serial.print(" (BIN): ");
    Serial.println(res, BIN);
  }else{
    Serial.print("Writing to ");
    for(int i = 0; i < fill_base_len; i++){
      Serial.print(fill_base[i]);
    }
    Serial.print(":");
    for(int i = 0; i < fill_sub_len; i++){
      Serial.print(fill_sub[i]);
    }
    Serial.println("");
    res = (uint32_t)sendBytes(bytes, 2 + data_len, 0);
  }
  return res;
}

//WORKING
uint32_t fullAddressRead(int base[], int sub[]){ // !! base and sub have to have MSB ... LSB already!!
  //Serial.println("\n=== Full Address READ ===");
  int t[] = {0};
  return readOrWriteFullAddress(base, 5, sub, 7, t, 0, 0);
}
//WORKING
uint32_t fullAddressWrite(int base[], int sub[], int data[], int data_len){ // !! base and sub have to have MSB ... LSB already!!
  //Serial.println("\n=== Full Address WRITE ===");
  return readOrWriteFullAddress(base, 5, sub, 7, data, data_len, 1);
}

void loop() {
  fullAddressRead(base0x0, sub0x0);
}
