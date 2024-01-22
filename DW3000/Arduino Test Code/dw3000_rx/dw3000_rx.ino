#include "DW3000.h"

static int frame_buffer; // Variable to store the transmitted message
static int rx_status; // Variable to store the current status of the receiver operation

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
}

void loop() 
{
  DW3000.standardRX(); // Send command to DW3000 to start the reception of frames

  while (!(rx_status = DW3000.receivedFrameSucc())) 
  {}; // Wait until frame was received
  
  if (rx_status == 1) { // If frame reception was successful
    frame_buffer = DW3000.read(0x12, 0x00); // Read RX_FRAME buffer0

    DW3000.clearSystemStatus();
    
    Serial.println("[INFO] Received frame successfully.");  
    Serial.print("Frame data (DEC): ");
    Serial.println(frame_buffer, DEC);  
  }
  else // if rx_status returns error (2)
  {
    Serial.println("[ERROR] Receiver Error occured! Aborting event.");
    DW3000.clearSystemStatus();
  }
}
