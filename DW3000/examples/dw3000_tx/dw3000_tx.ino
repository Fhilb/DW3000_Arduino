#include "DW3000.h"

#define TX_SENT_DELAY 500

static int tx_status; // Variable to store the current status of the receiver operation

void setup()
{
  delay(5000);
  Serial.begin(115200); // Init Serial
  DW3000.begin(); // Init SPI
  DW3000.hardReset(); // hard reset in case that the chip wasn't disconnected from power
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
  DW3000.setTXFrame(507); // Set content of frame
  DW3000.setFrameLength(9); // Set Length of frame in bits

  DW3000.standardTX(); // Send fast command for transmitting
  delay(10); // Wait for frame to be sent

  while (!(tx_status = DW3000.sentFrameSucc()))
  {
    Serial.println("[ERROR] Frame could not be sent succesfully!");
  };

  DW3000.clearSystemStatus(); // Clear event status

  Serial.println("[INFO] Sent frame successfully.");  

  delay(TX_SENT_DELAY);
}
