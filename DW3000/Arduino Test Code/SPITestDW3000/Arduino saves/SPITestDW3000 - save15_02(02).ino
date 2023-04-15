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
  Serial.println("\n\nCALLED INTERRUPT\n\n"); 

  int base[] = {0,0};
  int sub[] = {1,0,0,0,1,0,0};
  int sub2[] = {1,0,0,1,0,0,0};

  int irq_reason = getIRQBit();
  switch(irq_reason){
    case 0:
      Serial.println("Interruption cause: Interruption Cause not given! Check First and Second Sector Bits.");
      break;
    case 1:
      Serial.println("Interruption cause: TX finished");
      break;
    case 2:
      Serial.println("Interruption cause: RX Data ready");
      break;
    case 3:
      Serial.println("Interruption cause: Cmd Error");
      break;
  }

  resetIRQStatusBits();

  Serial.println("Finished interrupt. Continuing...");

  /*
   * clear bit by writing 1 to it
   * get received data
   * check if received data is anchorID-1
   * if so: instantTXThenRX()
   * else: initiateRX()
   */
}

int readRXBuffer(){
  
}

int getIRQBit(){ //return values: 1=tx finish; 2=rx finish; 3=cmd error; 0=none
  int irq_status = 0;
  int mask_tx_finish_or_cmd_err =  1 << 8;
  int mask_rx_finish =  1 << 13;
  
  int base[] = {0,0};
  int sub[] = {1,0,0,0,1,0,0};
  int sub2[] = {1,0,0,1,0,0,0};
  Serial.println("Getting IRQ Bit");
  
  uint32_t firstSectorBit =   fullAddressRead(base, 2, sub, 7); //read status register
  uint32_t secondSectorBit =  fullAddressRead(base, 2, sub2, 7); //read status register

  Serial.print("First Sector Bits: ");
  Serial.println(firstSectorBit, BIN);
  int txStatus = (firstSectorBit >> 7) & 1;
  int rxStatus = (firstSectorBit >> 13) & 1;
  int cmdErrStatus = (secondSectorBit >> 8) & 1;
  Serial.print("tx, rx and err bits: ");
  Serial.print(txStatus);
  Serial.print(" ");
  Serial.print(rxStatus);
  Serial.print(" ");
  Serial.println(cmdErrStatus);
 
  Serial.print("Second Sector Bits: ");
  Serial.println(secondSectorBit, BIN);

  if(txStatus) return 1;
  if(rxStatus) return 2;
  if(cmdErrStatus) return 3;
  return 0;
}

//Working
void writeShortCommand(int cmd[], int cmd_len){
  Serial.println("Short Command WRITE:");

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
  //Serial.println(byteOne, BIN);
  int bytes[] = {byteOne};
  sendBytes(bytes, 1, 0);
  Serial.println("Finished writing to Short Command.");
}

void initTXData(){
   int base[] = {1,0,1,0,0};
   int sub[] = {0};
   int data[] = {anchorID};

   int base2[] = {0,0}; //edit frame length
   int sub2[] = {1,0,0,1,0,0};
   int data2[] = {0x03,0x1C};
   
   fullAddressWrite(base, 5, sub, 1, data, 1); //Writing to TX Buffer
   fullAddressRead(base2, 2, sub2, 6);
   fullAddressWrite(base2, 2, sub2, 6, data2, 2); //Writing to TX Buffer
   fullAddressRead(base2, 2, sub2, 6);
   Serial.println("Finished writing to TX Buffer.");
   int shCmd[] = {1,1,0,0};
   writeShortCommand(shCmd, 1);
}

//WORKING
void init_program(){
  Serial.println("\n+++ DecaWave DW3000 Test +++\n");
  
  int base[] = {0, 0};
  int sub[] = {1,1,1,1,0,0};
  int data[] = {0xF0, 0x20, 0x0, 0x0}; //0x80, 0x20
  int sub2[] = {1,0,0,0,0,0,0};

  fullAddressWrite(base, 2, sub, 6, data, 4); //Set IRQ for successful received data frame
  Serial.println("IRQ octet 5 & 6 read:");
  fullAddressRead(base, 2, sub2, 7);
}

//WORKING
void resetIRQStatusBits(){
  Serial.println("Resetting IRQ Bits... ");
  int base[] = {0, 0};
  int sub[] = {1,0,0,0,1,0,0};
  int sub2[] = {1,0,0,1,0,0,0};
  int data[] = {0xFF, 0xFF, 0xFF, 0xFF};
  int data2[] = {0xFF, 0x1F};
  fullAddressWrite(base, 2, sub, 7, data, 4); //clear status register
  fullAddressWrite(base, 2, sub2, 7, data2, 2); //clear status register
  
  Serial.print("Reading CLEARED status bits... ");
  fullAddressRead(base, 2, sub, 7);
  fullAddressRead(base, 2, sub2, 7);
}

//Working
void instantTXThenRX(){
  Serial.print("Writing data to TX buffer... ");
  /*
   * write to TX_BUFFER for data
   * write preamble length, frame length, data rate and prf in TX_FCTRLb  
   * write channel in CHAN_CTRL
   */
   int base[] = {1,0,1,0,0};
   int sub[] = {0};
   int data[] = {anchorID};
   fullAddressWrite(base, 5, sub, 1, data, 1); //Writing to TX Buffer
   Serial.println("Done");
   Serial.print("Sending short command... ");
   int shCmd[] = {1,1,0,0};
   writeShortCommand(shCmd, 4);
   Serial.println("Done");
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
uint32_t fullAddressRead(int base[], int base_len, int sub[], int sub_len){ // !! base and sub have to have MSB ... LSB already!!
  //Serial.println("\n=== Full Address READ ===");
  int t[] = {0};
  return readOrWriteFullAddress(base, base_len, sub, sub_len, t, 0, 0);
}
//WORKING
uint32_t fullAddressWrite(int base[], int base_len, int sub[], int sub_len, int data[], int data_len){ // !! base and sub have to have MSB ... LSB already!!
  //Serial.println("\n=== Full Address WRITE ===");
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

  int shCmd[] = {0}; //put dw in idle
  writeShortCommand(shCmd, 1);
  
  delay(50);
  Serial.println("Now going for TX");
  //instantTXThenRX();
  initTXData();
  delay(5000);
}
