#include "DW3000.h"

static int frame_buffer = 0; // Variable to store the transmitted message
static int rx_status; // Variable to store the current status of the receiver operation
static int tx_status; // Variable to store the current status of the receiver operation

void setup()
{
  Serial.begin(115200); // Init Serial
  DW3000.begin(); // Init SPI
  DW3000.hardReset(); // hard reset in case that the chip wasn't disconnected from power
  delay(200); // Wait for DW3000 chip to wake up

  if(!DW3000.checkSPI())
  {
    Serial.println("[ERROR] Could not establish SPI Connection to DW3000! Please make sure that all pins are set correctly.");
    while(100);
  }

  while (!DW3000.checkForIDLE()) // Make sure that chip is in IDLE before continuing 
  {
    Serial.println("[ERROR] IDLE1 FAILED\r");
    delay(1000);
  }

  DW3000.softReset(); 
  delay(200); // Wait for DW3000 chip to wake up


  if (!DW3000.checkForIDLE())
  {
    Serial.println("[ERROR] IDLE2 FAILED\r");
    while (100);
  }

  
  DW3000.init(); // Initialize chip (write default values, calibration, etc.)
  DW3000.setupGPIO(); //Setup the DW3000s GPIO pins for use of LEDs
  Serial.println("[INFO] Setup is finished.");

  DW3000.configureAsTX(); // Configure basic settings for frame transmitting
}

void loop() 
{ 
  /**       === Send PING ===        **/
  if(frame_buffer > 1019) frame_buffer = 0;
  
  DW3000.setTXFrame(frame_buffer + 1); // Set content of frame
  DW3000.setFrameLength(9); // Set Length of frame in bits
  DW3000.pullLEDHigh(2);
  DW3000.standardTX(); // Send fast command for transmitting
  delay(10); // Wait for frame to be sent
  DW3000.pullLEDLow(2);
  while (!(tx_status = DW3000.sentFrameSucc()))
  {
    Serial.println("[ERROR] PING could not be sent succesfully!");
  };
  
  DW3000.clearSystemStatus(); // Clear event status
  
  Serial.println("[INFO] Sent PING successfully.");  


  /**       === Await PONG ===        **/
  
  DW3000.standardRX(); // Send command to DW3000 to start the reception of frames

  while (!(rx_status = DW3000.receivedFrameSucc())) 
  {}; // Wait until frame was received
  
  if (rx_status == 1) { // If frame reception was successful
    DW3000.pullLEDHigh(1);

    frame_buffer = DW3000.read(0x12, 0x00); // Read RX_FRAME buffer0
    
    DW3000.clearSystemStatus();
    
    Serial.println("[INFO] Received PONG successfully.");  
    Serial.print("Frame data (DEC): ");
    Serial.println(frame_buffer, DEC);  //in this example the frame_buffer output should always be +2
    delay(50); // Short delay to give time for PONG-node to go into receive mode
    DW3000.pullLEDLow(1);
  }
  else // if rx_status returns error (2)
  {
    Serial.println("[ERROR] Receiver Error occured! Aborting event.");
    DW3000.clearSystemStatus();
  }

  delay(1000); // delay between pings
}
