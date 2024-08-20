#include "DW3000.h"

/*
   BE AWARE: Baud Rate got changed to 2.000.000!

   Approach based on the application note APS011 ("SOURCES OF ERROR IN DW1000 BASED
   TWO-WAY RANGING (TWR) SCHEMES")

   see chapter 2.4 figure 6 and the corresponding description for more information

   This approach tackles the problem of a big clock offset between the ping and pong side
   by reducing the clock offset to a minimum.

   This approach is a more advanced version of the classical ping and pong with timestamp examples.
*/

#define ROUND_DELAY 500 // Delay in milliseconds that the chip waits between PING requests

static int frame_buffer = 0; // Variable to store the transmitted message
static int rx_status; // Variable to store the current status of the receiver operation
static int tx_status; // Variable to store the current status of the receiver operation

/*
   valid stages:
   0 - default stage; starts ranging
   1 - ranging sent; awaiting response
   2 - response received; sending second range
   3 - second ranging sent; awaiting final answer
   4 - final answer received
*/
static int curr_stage = 0;

static int t_roundA = 0;
static int t_replyA = 0;

static long long rx = 0;
static long long tx = 0;

static int clock_offset = 0;

static int ranging_time = 0;

static float distance = 0;

void setup()
{
  Serial.begin(2000000); // Init Serial
  DW3000.begin(); // Init SPI
  DW3000.hardReset(); // hard reset in case that the chip wasn't disconnected from power
  delay(200); // Wait for DW3000 chip to wake up
  while (!DW3000.checkForIDLE()) // Make sure that chip is in IDLE before continuing
  {
    Serial.println("[ERROR] IDLE1 FAILED\r");
    while (100);
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
  Serial.println("> double-sided PING with timestamp example <\n");
  Serial.println("[INFO] Setup is finished.");

  DW3000.configureAsTX(); // Configure basic settings for frame transmitting

  DW3000.clearSystemStatus();
}

void loop()
{
  switch (curr_stage) {
    case 0:  // Start ranging.
      t_roundA = 0;
      t_replyA = 0;

      DW3000.ds_sendFrame(1);
      tx = DW3000.readTXTimestamp();

      curr_stage = 1;
      break;
    case 1:  // Await first response.
      if (rx_status = DW3000.receivedFrameSucc()) {
        DW3000.clearSystemStatus();
        if (rx_status == 1) { // If frame reception was successful
          if (DW3000.ds_isErrorFrame()) {
            Serial.println("[WARNING] Error frame detected! Reverting back to stage 0.");
            curr_stage = 0;
          } else if (DW3000.ds_getStage() != 2) {
            DW3000.ds_sendErrorFrame();
            curr_stage = 0;
          } else {
            curr_stage = 2;
          }
        } else // if rx_status returns error (2)
        {
          Serial.println("[ERROR] Receiver Error occured! Aborting event.");
          DW3000.clearSystemStatus();
        }
      }
      break;
    case 2:  // Response received. Send second ranging.
      rx = DW3000.readRXTimestamp();

      DW3000.ds_sendFrame(3);
      t_roundA = rx - tx;

      tx = DW3000.readTXTimestamp();
      t_replyA = tx - rx;

      curr_stage = 3;
      break;
    case 3:  // Await second response.
      if (rx_status = DW3000.receivedFrameSucc()) {
        DW3000.clearSystemStatus();
        if (rx_status == 1) { // If frame reception was successful
          if (DW3000.ds_isErrorFrame()) {
            Serial.println("[WARNING] Error frame detected! Reverting back to stage 0.");
            curr_stage = 0;
          } else {
            clock_offset = DW3000.getRawClockOffset();
            curr_stage = 4;
          }
        } else // if rx_status returns error (2)
        {
          Serial.println("[ERROR] Receiver Error occured! Aborting event.");
          DW3000.clearSystemStatus();
        }
      }
      break;
    case 4:  // Response received. Calculating results.
      ranging_time = DW3000.ds_processRTInfo(t_roundA, t_replyA, DW3000.read(0x12, 0x04), DW3000.read(0x12, 0x08), clock_offset);
      distance = DW3000.convertToCM(ranging_time);

      DW3000.printDouble(distance, 100, false); // 100 equals to 2 decimal places, 1000 to 3, 10000 to 4...
      Serial.println("cm");

      curr_stage = 0;

      delay(ROUND_DELAY);
      break;
    default:
      Serial.print("[ERROR] Entered unknown stage (");
      Serial.print(curr_stage);
      Serial.println("). Reverting back to stage 0");

      curr_stage = 0;
      break;
  }
}
