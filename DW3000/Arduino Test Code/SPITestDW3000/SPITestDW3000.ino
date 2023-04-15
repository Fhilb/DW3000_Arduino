#include "SPI.h"
#include <stdlib.h>

#define chipSelectPin 10
#define anchorID 0
#define job 0 //0 = transmit, 1 = receive
boolean IRQPulled = false;

void setup() {
  SPI.begin();
  Serial.begin(9600);
  
  pinMode(chipSelectPin, OUTPUT);
  pinMode(2, INPUT); //WITH EXTERNAL PULLDOWN RESISTOR!!
  digitalWrite(chipSelectPin, HIGH);

  
  attachInterrupt(digitalPinToInterrupt(2), interruptDetect, RISING);

  init_program();
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
      readRXBuffer();
      break;
    case 3:
      Serial.println("Interruption cause: Cmd Error");
      break;
    default:
      Serial.print("irq_reason got wrong value. Value: ");
      Serial.println(irq_reason);
      break;
  }

  resetIRQStatusBits();
  
  Serial.println("Finished interrupt. Continuing...");

  
  initiateRX();
  /*
   * clear bit by writing 1 to it
   * get received data
   * check if received data is anchorID-1
   * if so: instantTXThenRX()
   * else: initiateRX()
   */
}

int readRXBuffer(){
  Serial.print("Reading RX Buffer0... ");
  int base[] = {1,0,0,1,0};
  int sub[] = {0};
  uint32_t buf0 = fullAddressRead(base, 5, sub, 0);
  Serial.println(buf0);
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
  int data[] = {0x80,0x3E}; //0xF0,0x2F
  
  int sub2[] = {1,0,0,1,0,0};
  int data2[] = {0x03, 0x1C};
  
  int base3[] = {1,1};
  int sub3[] = {1,1,0,0,0};
  int data3[] = {0xE5,0xE5};

  int base4[] = {1,1,0};
  int sub4[] = {0};
  int data4[] = {0x2};

  int base5[] = {1,0,0};
  int sub5[] = {1,1,1,0};
  int data5[] = {0x2};

  int base6[] = {1,1,1};
  int sub6[] = {1,0,1,0,0,0,1};
  int data6[] = {0x14};

  int base7[] = {1,1,1};
  int sub7[] = {1,1,0,1,0};
  int data7[] = {0x0E};

  int base8[] = {1,1,1};
  int sub_8[] = {1,1,1,0,0};
  int data8[] = {0x34,0x11,0x07,0x1C};

  int base9[] = {1,0,0,1};
  int sub9[] = {0};
  int data9[] = {0x3C,0x1F};

  int base10[] = {1,0,0,1};
  int sub10[] = {1,0,0,0};
  int data10[] = {0x21};

  int base11[] = {1,0,1,0};
  int sub11[] = {0};
  int data11[] = {0x00,0x08,0x01};

  int base12[] = {1,0,1,0};
  int sub12[] = {0};
  int data12[] = {0x0, 0x8, 0x1};
  
  fullAddressWrite(base, 2, sub, 6, data, 4); //Set IRQ for successful received data frame
  fullAddressWrite(base, 2, sub2, 6, data2, 2); //Frame length setup for Transmission  
  fullAddressWrite(base11, 4, sub11, 1, data11, 3); //Frame length setup for Transmission  
  
  resetIRQStatusBits();


  /*
   * Things to do documented in https://gist.github.com/egnor/455d510e11c22deafdec14b09da5bf54
   */
  fullAddressWrite(base3, 2, sub3, 5, data3, 2); //THR_64 value set to 0x32
  fullAddressWrite(base4, 3, sub4, 1, data4, 1); //DT0B4 to 0
  fullAddressWrite(base5, 3, sub5, 4, data5, 1); //COMP_DLY to 0x2
  fullAddressWrite(base6, 3, sub6, 7, data6, 1); //LDO_RLOAD to 0x14
  fullAddressWrite(base7, 3, sub7, 5, data7, 1); //RF_TX_CTRL_1 to 0x0E
  fullAddressWrite(base8, 3, sub_8, 5, data8, 4); //RF_TX_CTRL_2 to 0x1C071134 (due to channel 5, else (9) to 0x1C010034)
  fullAddressWrite(base9, 4, sub9, 1, data9, 2); //PLL_CFG to 0x1F3C (due to channel 5, else (9) to 0x0F3C)
  fullAddressWrite(base10, 4, sub10, 4, data10, 1); //PLL_CFG_LD to 0x8 (Documenation says 0x81, doesn't fit the 6bit register tho)

  fullAddressWrite(base12, 4, sub12, 1, data12, 3); //Auto antenna calibration on startup enable (ONW_PGFCAL)
  
  resetIRQStatusBits();


  /*
   * Things to do documented in https://gist.github.com/egnor/455d510e11c22deafdec14b09da5bf54
   */

   
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
  /*if(!job){
    initTXData();
  }else{
    initRXData();
  }
  delay(50);*/
  int base[] = {1,0,1,0};
  int sub[] = {0};
  fullAddressRead(base, 4, sub, 1);
  delay(5000);
}
