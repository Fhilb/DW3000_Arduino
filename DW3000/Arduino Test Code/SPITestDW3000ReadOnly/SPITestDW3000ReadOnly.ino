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

  pinMode(led1, OUTPUT);
  pinMode(led2, OUTPUT);
  pinMode(chipSelectPin, OUTPUT);
  pinMode(2, INPUT); //WITH EXTERNAL PULLDOWN RESISTOR!!
  digitalWrite(chipSelectPin, HIGH);

  delay(500);
  
  attachInterrupt(digitalPinToInterrupt(2), interruptDetect, RISING);

  init_program();
  resetIRQStatusBits();

  initiateRX();

}


void interruptDetect(){ //On calling interrupt
  Serial.println("\n\nCALLED INTERRUPT\n\n"); 

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
      digitalWrite(led1, HIGH);
      delay(100);
      digitalWrite(led1, LOW);
      readRXBuffer();
      break;
    case 3:
      Serial.println("Interruption cause: Preamble detected");  
      initiateRX();
      digitalWrite(led1, HIGH);
      digitalWrite(led2, HIGH);
      delay(50);
      digitalWrite(led1, LOW);
      digitalWrite(led2, LOW);
      break;
    case 4:
      Serial.println("Interruption cause: Cmd Error");
      break;
    default:
      Serial.print("irq_reason got wrong value. Value: ");
      Serial.println(irq_reason);
      break;
  }
  IRQPulled = true;
  resetIRQStatusBits();
  initiateRX();
  Serial.println("Finished interrupt. Continuing...");
}

int readRXBuffer(){
  Serial.print("Reading RX Buffer0... ");
  uint32_t buf0 = fullAddressRead(base0x12, sub0x0);
  Serial.println(buf0);
}

int getIRQBit(){ //return values: 1=tx finish; 2=rx finish; 3=cmd error; 0=none
  int irq_status = 0;
  int mask_tx_finish_or_cmd_err =  1 << 8;
  int mask_rx_finish =  1 << 13;

  Serial.println("Getting IRQ Bit");
  
  uint32_t firstSectorBit =   fullAddressRead(base0x0, sub0x44); //read status register
  uint32_t secondSectorBit =  fullAddressRead(base0x0, sub0x48); //read status register

  Serial.print("First Sector Bits: ");
  Serial.println(firstSectorBit, BIN);
  int txStatus = (firstSectorBit >> 7) & 1;
  int rxStatus = (firstSectorBit >> 13) & 1;
  int prDetStatus = (firstSectorBit >> 8) & 1; //preamble detect status
  int cmdErrStatus = (secondSectorBit >> 8) & 1;
  Serial.print("tx, rx, preamble_detect and err bits: ");
  Serial.print(txStatus);
  Serial.print(" ");
  Serial.print(rxStatus);
  Serial.print(" ");
  Serial.print(prDetStatus);
  Serial.print(" ");
  Serial.println(cmdErrStatus);
 
  Serial.print("Second Sector Bits: ");
  Serial.println(secondSectorBit, BIN);

  if(txStatus) return 1;
  if(rxStatus) return 2;
  if(prDetStatus) return 3;
  if(cmdErrStatus) return 4;
  return 0;
}

//Working
void writeShortCommand(int cmd[], int cmd_len){
  Serial.print("Short Command WRITE: ");

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
  Serial.println(byteOne, HEX);
  int bytes[] = {byteOne};
  sendBytes(bytes, 1, 0);
  Serial.println("Finished writing to Short Command.");
}

void initTXData(){ 
   int data[] = {anchorID};
   int data2[] = {0xF,0x1C};
   
   fullAddressWrite(base0x14, sub0x0, data, 1); //Writing to TX Buffer
   //fullAddressRead(base0x0, sub0x24);
   fullAddressWrite(base0x0, sub0x24, data2, 2); //Writing to TX Config
   //fullAddressRead(base0x0, sub0x24);
   Serial.println("Finished writing to TX Buffer.");
   int shCmd[] = {1,1,0,0};
   writeShortCommand(shCmd, 4);
}

//WORKING
void init_program(){
  int shCmd[] = {0};
  writeShortCommand(shCmd, 1);
  Serial.println("\n+++ DecaWave DW3000 Test +++\n");

  int data[] = {0xFF,0xFF,0xFF,0xFF,0xF2,0x1F}; //0xF0,0x2F //0x80,0x3E,0x0,0x0,0x0,0xF
  fullAddressWrite(base0x0, sub0x3C, data, 6); //Set IRQ for successful received data frame
  
  int data2[] = {0x03, 0x1C};
  fullAddressWrite(base0x0, sub0x24, data2, 2); //Frame length setup for Transmission  
  
  int data11[] = {0x00,0x9};
  fullAddressWrite(base0xA, sub0x0, data11, 3); //AON_DIG_CFG register setup; sets up auto-rx calibration and on-wakeup GO2IDLE
  
  /*
   * Set RX and TX config
   */
   int data15[] = {0x10,0x00,0x02,0x40}; //DGC_CFG0
   fullAddressWrite(base0x3, sub0x1C, data15, 4);
   
   int data16[] = {0x1b,0x6d,0xa4,0x89}; //DGC_CFG1
   fullAddressWrite(base0x3, sub0x20, data16, 4);
   
   int data17[] = {0x00,0x01,0xC0,0xFD}; //DGC_LUT_0
   fullAddressWrite(base0x3, sub0x38, data17, 4);
   
   int data18[] = {0x00,0x01,0xC4,0x3E}; //DGC_LUT_1
   fullAddressWrite(base0x3, sub0x3C, data18, 4);
   
   int data19[] = {0x00,0x01,0xC6,0xBE}; //DGC_LUT_2
   fullAddressWrite(base0x3, sub0x40, data19, 4);
   
   int data20[] = {0x00,0x01,0xC7,0x7E}; //DGC_LUT_3
   fullAddressWrite(base0x3, sub0x44, data20, 4);
   
   int data21[] = {0x00,0x01,0xCF,0x36}; //DGC_LUT_4
   fullAddressWrite(base0x3, sub0x48, data21, 4);
   
   int data22[] = {0x00,0x01,0xCF,0xB5}; //DGC_LUT_5
   fullAddressWrite(base0x3, sub0x4C, data22, 4);
   
   int data23[] = {0x00,0x01,0xCF,0xF5}; //DGC_LUT_6
   fullAddressWrite(base0x3, sub0x50, data23, 4);

   //SET PAC TO 32 (0x00) reg:06:00 bits:1-0, bit 4 to 0 (00001100) (0xC)
   int data25[] = {0xE};
   fullAddressWrite(base0x6, sub0x0, data25, 1);
   
   //SET PREAMBLE CODE (RX_PCODE, TX_PCODE) TO 10 (reg:01:14) //Standard SFD Type is 11 (data: 0x56, 0x5), trying 00 (0x50, 0x5)
   int data26[] = {0xBE, 0x3};
   fullAddressWrite(base0x1, sub0x14, data26, 2);

   // write preamble length, frame length, data rate and prf in TX_FCTRL  //PSR = 1024, TXFLEN = 3 Byte (1 data, 2 CRC) TXBR = 6.81Mb/s, TR Bit enabled, FINE_PLEN = 0x0
   // reg:00:24 bits 0 - 25
   int data27[] = {0x03, 0x2C};
   fullAddressWrite(base0x0, sub0x24, data27, 2);

   //set SFD Detection timeout count to 1057 (0x21, 0x4); 1018 old: (0xFA, 0x3)
   int data28[] = {0x21, 0x4};
   fullAddressWrite(base0x6, sub0x2, data28, 2);
  
   
  
  resetIRQStatusBits();


  /*
   * Things to do documented in https://gist.github.com/egnor/455d510e11c22deafdec14b09da5bf54
   */ 
  int data3[] = {0xF5,0xE4};
  fullAddressWrite(base0x3, sub0x18, data3, 2); //THR_64 value set to 0x32

  int data5[] = {0x2};
  fullAddressWrite(base0x4, sub0xC, data5, 1); //COMP_DLY to 0x2

  int data6[] = {0x14};
  fullAddressWrite(base0x7, sub0x48, data6, 1); //LDO_RLOAD to 0x14

  int data7[] = {0x0E};
  fullAddressWrite(base0x7, sub0x1A, data7, 1); //RF_TX_CTRL_1 to 0x0E

  int data8[] = {0x34,0x11,0x07,0x1C};
  fullAddressWrite(base0x7, sub0x1C, data8, 4); //RF_TX_CTRL_2 to 0x1C071134 (due to channel 5, else (9) to 0x1C010034)

  int data9[] = {0x3C,0x1F};
  fullAddressWrite(base0x9, sub0x0, data9, 2); //PLL_CFG to 0x1F3C (due to channel 5, else (9) to 0x0F3C)

  int data10[] = {0x21};
  fullAddressWrite(base0x9, sub0x8, data10, 1); //PLL_CFG_LD to 0x8 (Documenation says 0x81, doesn't fit the 6bit register tho)
  
  int data12[] = {0x0, 0x8, 0x1};
  fullAddressWrite(base0xA, sub0x0, data12, 3); //Auto antenna calibration on startup enable (ONW_PGFCAL)

  int data13[] = {0x11};
  int data14[] = {0x0}; //if finished with calibration go back in cal_mode
  
  delay(50);
  for(int i = 0; i < 5; i++){
    uint32_t h = fullAddressRead(base0x4, sub0x20); //Read antenna calibration //RX_CAL_STS => Status bit, if high antenna cal was successful
    if(h > 0){
      break;
    }
    if(i<4){
      Serial.println("Failed auto calibration of antenna. Trying again in 50ms.");
      fullAddressWrite(base0x4, sub0xC, data13, 1);
      delay(50);
    }else{
      Serial.println("[ERROR] Antenna auto calibration failed too often. Stopping to calibrate now.");
    }
  }
  fullAddressWrite(base0x4, sub0xC, data14, 1); //reset to normal operation
  
  resetIRQStatusBits();

  fullAddressRead(base0x0, sub0x3C);
  fullAddressRead(base0x0, sub0x40);
  fullAddressRead(base0x6, sub0x4);

  /*
   * Things to do documented in https://gist.github.com/egnor/455d510e11c22deafdec14b09da5bf54
   */

   
}

//WORKING
void resetIRQStatusBits(){
  int data[] = {0xFF, 0xFF, 0xFF, 0xFF};
  int data2[] = {0xFF, 0x1F};
  fullAddressWrite(base0x0, sub0x44, data, 4); //clear status register (octets 0 to 3)
  fullAddressWrite(base0x0, sub0x48, data2, 2); //clear status register (octets 4 and 5)
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



/*
 * MAIN LOOP
 */


void loop() {
  /*if(IRQPulled){
    IRQPulled = false;
    if(timer != 0){
        Serial.print("Time from last preDetIRQ: ");
        Serial.println(millis() - timer);
    }
    timer = millis();
  }*/
}
