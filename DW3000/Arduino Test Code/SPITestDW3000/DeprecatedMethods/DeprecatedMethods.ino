void setup() {
  // put your setup code here, to run once:

}


//Working
uint32_t getShortAddressRead(int base[], int base_len){
  Serial.println("\nShort Address READ:");

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
  for(int i = 0; i < 1; i++){
    val = SPI.transfer(0x00); //Read first 4 octets
    val |= (uint32_t)SPI.transfer(0xff) << 8;
    val |= (uint32_t)SPI.transfer(0xff) << 16;
    val |= (uint32_t)SPI.transfer(0xff) << 24;
    Serial.println(val, HEX);
  }
  digitalWrite(chipSelectPin, HIGH);
}



//WORKING
void testFunctionalityDEV_ID(){
  uint32_t val;
  digitalWrite(chipSelectPin, LOW);
  SPI.transfer(0x00); //Send command [RD/WR][8/16Bit][5 Bit Base Address][LSB=0] [DW3000 Family User Manual 2.3.1.2 Figure 2]
  val = SPI.transfer(0x00); //Read first 4 octets
  val |= ((uint32_t)SPI.transfer(0xff) << 8);
  val |= ((uint32_t)SPI.transfer(0xff) << 16);
  val |= ((uint32_t)SPI.transfer(0xff) << 24);
  digitalWrite(chipSelectPin, HIGH);
  Serial.println(val, HEX);
  delay(5000);
}

//Working
void instantTXThenRX(){
  Serial.print("Writing data to TX buffer... ");
  /*
   * write to TX_BUFFER for data
   * 
   */
   int data[] = {anchorID};
   fullAddressWrite(base0x14, sub0x0, data, 1); //Writing to TX Buffer
   Serial.println("Done");
   Serial.print("Sending short command... ");
   int shCmd[] = {1,1,0,0};
   writeShortCommand(shCmd, 4);
   Serial.println("Done");
}

void loop() {
  // put your main code here, to run repeatedly:

}
