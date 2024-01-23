#include "DW3000.h"

#define TX_SENT_DELAY 500

static int frame_buffer; // Variable to store the transmitted message
static int rx_status; // Variable to store the current status of the receiver operation
static int tx_status; // Variable to store the current status of the receiver operation

void setup()
{
  delay(5000);
  Serial.begin(115200); // Init Serial
  DW3000.begin(); // Init SPI 
  delay(200); // Wait for DW3000 chip to wake up

  while (!DW3000.checkForIDLE()) // Make sure that chip is in IDLE before continuing 
  {
    Serial.println("[ERROR] IDLE1 FAILED\r");
    while (100);
  }

  DW3000.softReset(); // Reset in case that the chip wasn't disconnected from power
  delay(200); // Wait for DW3000 chip to wake up


  if (!DW3000.checkForIDLE())
  {
    Serial.println("[ERROR] IDLE2 FAILED\r");
    while (100);
  }

  
  DW3000.init(); // Initialize chip (write default values, calibration, etc.)
  Serial.println("[INFO] Setup is finished.");
  
  DW3000.configureAsTX(); // Configure basic settings for frame transmitting
}

void loop()
{
  /**       === Await PING ===        **/

  DW3000.standardRX(); // Send command to DW3000 to start the reception of frames

  while (!(rx_status = DW3000.receivedFrameSucc())) 
  {}; // Wait until frame was received
  
  if (rx_status == 1) { // If frame reception was successful
    frame_buffer = DW3000.read(0x12, 0x00); // Read RX_FRAME buffer0

    DW3000.clearSystemStatus();
    
    Serial.println("[INFO] Received PING successfully.");  

      /**       === Send PONG ===        **/
    delay(50); // Short delay to give time for PING-node to go into receive mode

    DW3000.setTXFrame(frame_buffer + 1); // Set content of frame
    DW3000.setFrameLength(9); // Set Length of frame in bits

    DW3000.standardTX(); // Send fast command for transmitting
    delay(10); // Wait for frame to be sent
  
    while (!(tx_status = DW3000.sentFrameSucc()))
    {
      Serial.println("[ERROR] PONG could not be sent succesfully!");
    };
  
    DW3000.clearSystemStatus(); // Clear event status
  
    Serial.println("[INFO] Sent PONG successfully.");
    
  }
  else // if rx_status returns error (2)
  {
    Serial.println("[ERROR] Receiver Error occured! Aborting event.");
    DW3000.clearSystemStatus();
  }
}
