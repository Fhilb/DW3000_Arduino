/*
 * Copyright (c) 2023 by Philipp Hafkemeyer
 * Qorvo DW3000 library for Arduino
 *
 * This project is licensed under the GNU GPL v3.0 License.
 * you may not use this file except in compliance with the License.
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
*/

#include "Arduino.h"
#include "SPI.h"
//#include "stdlib.h"
#include "DW3000.h"

DW3000Class DW3000;

#define DEBUG_OUTPUT 0 // Turn to 1 to get all reads, writes, etc. as info in the console

int antenna_delay = 0x3FCA; // For calibration purposes; the smaller the number, the longer the ranging results

int led_status = 0;

int destination = 0x0;  // Default Values for Destination and Sender IDs
int sender = 0x0;

// Initial Radio Configuration
int DW3000Class::config[] = {
    CHANNEL_5,           // Channel
    PREAMBLE_128,        // Preamble Length
    9,                   // Preamble Code (Same for RX and TX!)
    PAC8,                // PAC
    DATARATE_6_8MB,      // Datarate
    PHR_MODE_STANDARD,   // PHR Mode
    PHR_RATE_850KB       // PHR Rate
};



/*
 #####  Chip Setup  #####
*/


/*
 Selects a SPI device through its Chip Select pin
 @param cs The pin number of the selected SPI device
*/
void DW3000Class::spiSelect(uint8_t cs) {
    pinMode(cs, OUTPUT);
    digitalWrite(cs, HIGH);

    delay(5);
}

/*
 Initializes the SPI Interface
*/
void DW3000Class::begin() {
    delay(5);
    pinMode(CHIP_SELECT_PIN, OUTPUT);
    SPI.begin();

    delay(5);

    spiSelect(CHIP_SELECT_PIN);

    Serial.println("[INFO] SPI ready");
}

/*
 Initializes the chip, checks for a connection and sets up a initial configuration
*/
void DW3000Class::init() {
    Serial.println("\n+++ DecaWave DW3000 Test +++\n");

    if (!checkForDevID()) {
        Serial.println("[ERROR] Dev ID is wrong! Aborting!");
        return;
    }

    setBitHigh(GEN_CFG_AES_LOW_REG, 0x10, 4);


    while (!checkForIDLE()) {
        Serial.println("[WARNING] IDLE FAILED (stage 1)");
        delay(100);
    }

    softReset();

    delay(200);

    while (!checkForIDLE()) {
        Serial.println("[WARNING] IDLE FAILED (stage 2)");
        delay(100);
    }

    uint32_t ldo_low = readOTP(0x04);
    uint32_t ldo_high = readOTP(0x05);
    uint32_t bias_tune = readOTP(0xA);
    bias_tune = (bias_tune >> 16) & BIAS_CTRL_BIAS_MASK;

    if (ldo_low != 0 && ldo_high != 0 && bias_tune != 0) {
        write(0x11, 0x1F, bias_tune);

        write(0x0B, 0x08, 0x0100);
    }

    int xtrim_value = readOTP(0x1E);

    xtrim_value = xtrim_value == 0 ? 0x2E : xtrim_value; //if xtrim_value from OTP memory is 0, choose 0x2E as default value

    write(FS_CTRL_REG, 0x14, xtrim_value);
    if (DEBUG_OUTPUT) Serial.print("xtrim: ");
    if (DEBUG_OUTPUT) Serial.println(xtrim_value);

    writeSysConfig();

    write(0x00, 0x3C, 0xFFFFFFFF); //Set Status Enable
    write(0x00, 0x40, 0xFFFF);

    write(0x0A, 0x00, 0x000900, 3); //AON_DIG_CFG register setup; sets up auto-rx calibration and on-wakeup GO2IDLE  //0xA

    /*
     * Set RX and TX config
     */
    write(0x3, 0x1C, 0x10000240); //DGC_CFG0

    write(0x3, 0x20, 0x1B6DA489); //DGC_CFG1

    write(0x3, 0x38, 0x0001C0FD); //DGC_LUT_0

    write(0x3, 0x3C, 0x0001C43E); //DGC_LUT_1

    write(0x3, 0x40, 0x0001C6BE); //DGC_LUT_2

    write(0x3, 0x44, 0x0001C77E); //DGC_LUT_3

    write(0x3, 0x48, 0x0001CF36); //DGC_LUT_4

    write(0x3, 0x4C, 0x0001CFB5); //DGC_LUT_5

    write(0x3, 0x50, 0x0001CFF5); //DGC_LUT_6

    write(0x3, 0x18, 0xE5E5); //THR_64 value set to 0x32
    int f = read(0x4, 0x20);

    //SET PAC TO 32 (0x00) reg:06:00 bits:1-0, bit 4 to 0 (00001100) (0xC)
    write(0x6, 0x0, 0x81101C);

    write(0x07, 0x34, 0x4); // Enable temp sensor readings


    /*
     * Things to do as documented in https://gist.github.com/egnor/455d510e11c22deafdec14b09da5bf54
     */
    write(0x07, 0x48, 0x14); //LDO_RLOAD to 0x14 //0x7
    write(0x07, 0x1A, 0x0E); //RF_TX_CTRL_1 to 0x0E
    write(0x07, 0x1C, 0x1C071134); //RF_TX_CTRL_2 to 0x1C071134 (due to channel 5, else (9) to 0x1C010034)
    write(0x09, 0x00, 0x1F3C); //PLL_CFG to 0x1F3C (due to channel 5, else (9) to 0x0F3C)  //0x9
    write(0x09, 0x80, 0x81); //PLL_CAL config to 0x81

    write(0x11, 0x04, 0xB40200);

    write(0x11, 0x08, 0x80030738);
    Serial.println("[INFO] Initialization finished.\n");
}

/*
 Writes the initial configuration to the chip
*/
void DW3000Class::writeSysConfig() {
    int usr_cfg = (STDRD_SYS_CONFIG & 0xFFF) | (config[5] << 3) | (config[6] << 4);

    write(GEN_CFG_AES_LOW_REG, 0x10, usr_cfg);

    if (config[2] > 24) {
        Serial.println("[ERROR] SCP ERROR! TX & RX Preamble Code higher than 24!");
    }

    int otp_write = 0x1400;

    if (config[1] >= 256) {
        otp_write |= 0x04;
    }


    write(OTP_IF_REG, 0x08, otp_write); //set OTP config
    write(DRX_REG, 0x00, 0x00, 1); //reset DTUNE0_CONFIG

    write(DRX_REG, 0x0, config[3]);

    //64 = STS length
    write(STS_CFG_REG, 0x0, 64 / 8 - 1);

    write(GEN_CFG_AES_LOW_REG, 0x29, 0x00, 1);

    write(DRX_REG, 0x0C, 0xAF5F584C);

    int chan_ctrl_val = read(GEN_CFG_AES_HIGH_REG, 0x14);  //Fetch and adjust CHAN_CTRL data
    chan_ctrl_val &= (~0x1FFF);

    chan_ctrl_val |= config[0]; //Write RF_CHAN

    chan_ctrl_val |= 0x1F00 & (config[2] << 8);
    chan_ctrl_val |= 0xF8 & (config[2] << 3);
    chan_ctrl_val |= 0x06 & (0x01 << 1);

    write(GEN_CFG_AES_HIGH_REG, 0x14, chan_ctrl_val);  //Write new CHAN_CTRL data with updated values

    int tx_fctrl_val = read(GEN_CFG_AES_LOW_REG, 0x24);

    tx_fctrl_val |= (config[1] << 12); //Add preamble length
    tx_fctrl_val |= (config[4] << 10); //Add data rate

    write(GEN_CFG_AES_LOW_REG, 0x24, tx_fctrl_val);

    write(DRX_REG, 0x02, 0x81);

    int rf_tx_ctrl_2 = 0x1C071134;
    int pll_conf = 0x0F3C;

    if (config[0]) {
        rf_tx_ctrl_2 &= ~0x00FFFF;
        rf_tx_ctrl_2 |= 0x000001;
        pll_conf &= 0x00FF;
        pll_conf |= 0x001F;
    }

    write(RF_CONF_REG, 0x1C, rf_tx_ctrl_2);
    write(FS_CTRL_REG, 0x00, pll_conf);

    write(RF_CONF_REG, 0x51, 0x14);

    write(RF_CONF_REG, 0x1A, 0x0E);

    write(FS_CTRL_REG, 0x08, 0x81);

    write(GEN_CFG_AES_LOW_REG, 0x44, 0x02);

    write(PMSC_REG, 0x04, 0x300200); //Set clock to auto mode

    write(PMSC_REG, 0x08, 0x0138);

    int success = 0;
    for (int i = 0; i < 100; i++) {
        if (read(GEN_CFG_AES_LOW_REG, 0x0) & 0x2) {
            success = 1;
            break;
        }
    }

    if (!success) {
        Serial.println("[ERROR] Couldn't lock PLL Clock!");
    }
    else {
        Serial.println("[INFO] PLL is now locked.");
    }

    int otp_val = read(OTP_IF_REG, 0x08);
    otp_val |= 0x40;
    if (config[0]) otp_val |= 0x2000;


    write(OTP_IF_REG, 0x08, otp_val);

    write(RX_TUNE_REG, 0x19, 0xF0);

    int ldo_ctrl_val = read(RF_CONF_REG, 0x48); //Save original LDO_CTRL data
    int tmp_ldo = (0x105 |
        0x100 |
        0x4 |
        0x1);

    write(RF_CONF_REG, 0x48, tmp_ldo);

    write(EXT_SYNC_REG, 0x0C, 0x020000); //Calibrate RX

    int l = read(0x04, 0x0C);

    delay(20);

    write(EXT_SYNC_REG, 0x0C, 0x11); //Enable calibration

    int succ = 0;
    for (int i = 0; i < 100; i++) {
        if (read(EXT_SYNC_REG, 0x20)) {
            succ = 1;
            break;
        }
        delay(10);
    }

    if (succ) {
        Serial.println("[INFO] PGF calibration complete.");
    }
    else {
        Serial.println("[ERROR] PGF calibration failed!");
    }

    write(EXT_SYNC_REG, 0x0C, 0x00);
    write(EXT_SYNC_REG, 0x20, 0x01);

    int rx_cal_res = read(EXT_SYNC_REG, 0x14);
    if (rx_cal_res == 0x1fffffff) {
        Serial.println("[ERROR] PGF_CAL failed in stage I!");
    }
    rx_cal_res = read(EXT_SYNC_REG, 0x1C);
    if (rx_cal_res == 0x1fffffff) {
        Serial.println("[ERROR] PGF_CAL failed in stage Q!");
    }

    write(RF_CONF_REG, 0x48, ldo_ctrl_val); //Restore original LDO_CTRL data

    write(0x0E, 0x02, 0x01); //Enable full CIA diagnostics to get signal strength information

    setTXAntennaDelay(antenna_delay); //set default antenna delay
}

/*
 Configures the chip for usage as a Transfer Device
*/
void DW3000Class::configureAsTX() {
    write(RF_CONF_REG, 0x1C, 0x34); //write pg_delay
    write(GEN_CFG_AES_HIGH_REG, 0x0C, 0xFDFDFDFD);
}

/*
 Sets the first 4 GPIO pins as output for external measurements and LED usage
*/
void DW3000Class::setupGPIO() {
    write(0x05, 0x08, 0xF0); //Set GPIO0 - GPIO3 as OUTPUT on DW3000
}



/*
 #####  Double-Sided Ranging  #####
*/


/*
 Sends a Frame that is supposed to work in double sided ranging (See DW3000 User Manual 12.3 for more)
 @param stage Double-sided Ranging is more complicated than regular single-sided Ranging. Therefore,
              stages were introduced to make sure that the right frames get received at the right time. stage is a 3 bit int.
*/
void DW3000Class::ds_sendFrame(int stage) {
    setMode(1);
    write(0x14, 0x01, sender & 0xFF);
    write(0x14, 0x02, destination & 0xFF);
    write(0x14, 0x03, stage & 0x7);
    setFrameLength(4);

    TXInstantRX(); //Await response

    bool error = true;
    for (int i = 0; i < 50; i++) {
        if (sentFrameSucc()) {
            error = false;
            break;
        }
    };
    if (error) {
        Serial.println("[ERROR] Could not send frame successfully!");
    }
}

/*
 Send the information that chip B collected to chip A for final time calculations
 @param t_roundB The time that it took between chip B (this chip) sending an answer and getting a response (rx2 - tx1)
 @param t_replyB The time that the chip took to process the received frame (tx1 - rx1)
*/
void DW3000Class::ds_sendRTInfo(int t_roundB, int t_replyB) {
    setMode(1);
    write(0x14, 0x01, destination & 0xFF);
    write(0x14, 0x02, sender & 0xFF);
    write(0x14, 0x03, 4);
    write(0x14, 0x04, t_roundB);
    write(0x14, 0x08, t_replyB);

    setFrameLength(12);

    TXInstantRX();
}

/*
 Process all Round Trip Time info
 @param t_roundA The time it took between chip A sending a frame and getting a response
 @param t_replyA The time that chip A took to process the received frame
 @param t_roundB The time that it took between chip B sending an answer and getting a response
 @param t_replyB The time that chip B took to process the received frame
 @param clk_offset The calculated clock offset between both chips (See DW3000 User Manual 10.1 for more)
 @return returns the time in units of 15.65ps that the frames were in the air on average (only one direction)
*/
int DW3000Class::ds_processRTInfo(int t_roundA, int t_replyA, int t_roundB, int t_replyB, int clk_offset) { //returns ranging time in DW3000 ps units (~15.65ps per unit)
    if (DEBUG_OUTPUT) {
        Serial.println("\nProcessing Information:");
        Serial.print("t_roundA: ");
        Serial.println(t_roundA);
        Serial.print("t_replyA: ");
        Serial.println(t_replyA);
        Serial.print("t_roundB: ");
        Serial.println(t_roundB);
        Serial.print("t_replyB: ");
        Serial.println(t_replyB);
    }

    int reply_diff = t_replyA - t_replyB;

    long double clock_offset = t_replyA > t_replyB ? 1.0 + getClockOffset(clk_offset) : 1.0 - getClockOffset(clk_offset);

    int first_rt = t_roundA - t_replyB;
    int second_rt = t_roundB - t_replyA;

    int combined_rt = (first_rt + second_rt - (reply_diff - (reply_diff * clock_offset))) / 2;
    int combined_rt_raw = (first_rt + second_rt) / 2;

    return combined_rt / 2; // divided by 2 to get just one range
}

/*
 Returns the stage that the frame was sent in
 @return The stage that the frame was sent in (read from the TX_Buffer)
*/
int DW3000Class::ds_getStage() {
    return read(0x12, 0x03) & 0b111;
}

/*
 Checks if frame is error frame by checking its mode bits
 @return True if mode == 7; False if anything else
*/
bool DW3000Class::ds_isErrorFrame() {
    return ((read(0x12, 0x00) & 0x7) == 7);
}

/*
 Sends a frame that has its mode set to 7 (Error Frame). Instantly switches to receive mode (RX)
*/
void DW3000Class::ds_sendErrorFrame() {
    Serial.println("[WARNING] Error Frame sent. Reverting back to stage 0.");
    setMode(7);
    setFrameLength(3);
    standardTX();
}



/*
 #####  Radio Settings  #####
*/


/*
 Set the channel that the chip should operate on
 @param data CHANNEL_5 or CHANNEL_9
*/
void DW3000Class::setChannel(uint8_t data) {
    if (data == CHANNEL_5 || data == CHANNEL_9) config[0] = data;
}

/*
 Set the preamble length for frame sending
 @param data See all options below or in DW3000Constants.h
*/
void DW3000Class::setPreambleLength(uint8_t data) {
    if (data == PREAMBLE_32 || data == PREAMBLE_64 || data == PREAMBLE_1024 ||
        data == PREAMBLE_256 || data == PREAMBLE_512 || data == PREAMBLE_1024 ||
        data == PREAMBLE_1536 || data == PREAMBLE_2048 || data == PREAMBLE_4096) config[1] = data;
}

/*
 Set the preamble code
 @param data Should be between 9 and 12
*/
void DW3000Class::setPreambleCode(uint8_t data) {
    if (data <= 12 && data >= 9) config[2] = data;
}

/*
 Set the PAC size
 @param data PAC4, PAC8, PAC16 or PAC32
*/
void DW3000Class::setPACSize(uint8_t data) {
    if (data == PAC4 || data == PAC8 || data == PAC16 || data == PAC32) config[3] = data;
}

/*
 Set the datarate the chip sends and receives on
 @param data DATARATE_6_8_MB or DATARATE_850KB
*/
void DW3000Class::setDatarate(uint8_t data) {
    if (data == DATARATE_6_8MB || data == DATARATE_850KB) config[4] = data;
}

/*
 Set the PHR mode for the chip
 @param data PHR_MODE_STANDARD or PHR_MODE_LONG
*/
void DW3000Class::setPHRMode(uint8_t data) {
    if (data == PHR_MODE_STANDARD || data == PHR_MODE_LONG) config[5] = data;
}

/*
 Set the PHR rate for the chip
 @param data PHR_RATE_6_8MB or PHR_RATE_850KB
*/
void DW3000Class::setPHRRate(uint8_t data) {
    if (data == PHR_RATE_6_8MB || data == PHR_RATE_850KB) config[6] = data;
}



/*
 #####  Protocol Settings  #####
*/


/*
 Sets the frame type/mode to determine between double-sided and error frames.
 @param mode The mode that should be used:
    * 0 - Standard
    * 1 - Double-Sided Ranging
    * 2-6 - Reserved
    * 7 - Error
*/
void DW3000Class::setMode(int mode) {
    write(0x14, 0x00, mode & 0x7);
}

/*
 Writes the given data to the chips TX Frame buffer
 @param frame_data The data that should be written onto the chip
*/
void DW3000Class::setTXFrame(unsigned long long frame_data) {  // deprecated! use write(TX_BUFFER_REG, [...]);
    if (frame_data > ((pow(2, 8 * 8) - FCS_LEN))) {
        Serial.println("[ERROR] Frame is too long (> 1023 Bytes - FCS_LEN)!");
        return;
    }

    write(TX_BUFFER_REG, 0x00, frame_data);
}

/*
 Sets the frames data length in bytes
 @param frameLen The length of the data in bytes
*/
void DW3000Class::setFrameLength(int frameLen) { // set Frame length in Bytes
    frameLen = frameLen + FCS_LEN;
    int curr_cfg = read(0x00, 0x24);
    if (frameLen > 1023) {
        Serial.println("[ERROR] Frame length + FCS_LEN (2) is longer than 1023. Aborting!");
        return;
    }
    int tmp_cfg = (curr_cfg & 0xFFFFFC00) | frameLen;

    write(GEN_CFG_AES_LOW_REG, 0x24, tmp_cfg);
}

/*
 Set the Antenna Delay for delayedTX operations
 @param data Can be anything between 0 and 0xFFFF
*/
void DW3000Class::setTXAntennaDelay(int delay) {
    antenna_delay = delay;
    write(0x01, 0x04, delay);
}

/*
 Set the chips sender ID. As long as the value is not changed, it won't be changed by the program.
 @param senderID The ID that should be set. Can be between 0 and 255. Default is 0
*/
void DW3000Class::setSenderID(int senderID) {
    sender = senderID;
}

/*
 Set the destination ID. As long as the value is not changed, it won't be changed by the program. If you want to send a second frame, it will still hold the same destination ID!
 @param destID The ID that the frame should be sent to. Can be between 0 and 255. Default is 0
*/
void DW3000Class::setDestinationID(int destID) {
    destination = destID;
}



/*
 #####  Status Checks  #####
*/


/*
 Checks if a frame got received successfully
 @return 1 if successfully received; 2 if RX Status Error occured; 0 if no frame got received
*/
int DW3000Class::receivedFrameSucc() {
    int sys_stat = read(GEN_CFG_AES_LOW_REG, 0x44);
    if ((sys_stat & SYS_STATUS_FRAME_RX_SUCC) > 0) {
        return 1;
    }
    else if ((sys_stat & SYS_STATUS_RX_ERR) > 0) {
        return 2;
    }
    return 0;
}

/*
 Checks if a frame got sent successfully
 @return 1 if successfully sent; 2 if TX Status Error occured; 0 if no frame got sent
*/
int DW3000Class::sentFrameSucc() { //No frame sent: 0; frame sent: 1; error while sending: 2
    int sys_stat = read(GEN_CFG_AES_LOW_REG, 0x44);
    if ((sys_stat & SYS_STATUS_FRAME_TX_SUCC) == SYS_STATUS_FRAME_TX_SUCC) {
        return 1;
    }
    return 0;
}

/*
 Returns the senderID of the received frame.
 @return senderID of the received frame by reading out the frames data
*/
int DW3000Class::getSenderID() {
    return read(0x12, 0x01) & 0xFF;
}

/*
 Returns the destinationID of the received frame.
 @return destinationID of the received frame by reading out the frames data
*/
int DW3000Class::getDestinationID() {
    return read(0x12, 0x02) & 0xFF;
}

/*
 Checks if the chip has its Power Management System Control (PMSC) module in IDLE mode
 @return True if in IDLE, False if not
 */
bool DW3000Class::checkForIDLE() {
    return (read(0x0F, 0x30) >> 16 & PMSC_STATE_IDLE) == PMSC_STATE_IDLE || (read(0x00, 0x44) >> 16 & (SPIRDY_MASK | RCINIT_MASK)) == (SPIRDY_MASK | RCINIT_MASK) ? 1 : 0;
}

/*
 Checks if SPI can communicate with the chip
 @return 1 if True, 0 if False
*/
bool DW3000Class::checkSPI() {
    return checkForDevID();
}



/*
 #####  Radio Analytics  #####
*/


/*
 Calculates the Signal Strength of the received frame in dBm
 NOTE: If not using 64MHz PRF: See user manual capter 4.7.2 for an alternative calculation method
 @return The Signal Strength of the received frame in dBm
*/
double DW3000Class::getSignalStrength() {
    int CIRpower = read(0x0C, 0x2C) & 0x1FF;
    int PAC_val = read(0x0C, 0x58) & 0xFFF;
    unsigned int DGC_decision = (read(0x03, 0x60) >> 28) & 0x7;
    double PRF_const = 121.7;

    /*Serial.println("Signal Strength Data:");
    Serial.print("CIR Power: ");
    Serial.println(CIRpower);
    Serial.print("PAC val: ");
    Serial.println(PAC_val);
    Serial.print("DGC decision: ");
    Serial.println(DGC_decision);*/

    return 10 * log10((CIRpower * (1 << 21)) / pow(PAC_val, 2)) + (6 * DGC_decision) - PRF_const;
}

/*
 Calculates the First Path Signal Strength of the received frame in dBm. Useful to check if a ranging was NLOS or LOS by comparing it to the overall Signal Strength.
 @return The First Path Signal Strength of the received frame in dBm
*/
double DW3000Class::getFirstPathSignalStrength() {
    float f1 = (read(0x0C, 0x30) & 0x3FFFFF) >> 2;
    float f2 = (read(0x0C, 0x34) & 0x3FFFFF) >> 2;
    float f3 = (read(0x0C, 0x38) & 0x3FFFFF) >> 2;

    int PAC_val = read(0x0C, 0x58) & 0xFFF;
    unsigned int DGC_decision = (read(0x03, 0x60) >> 28) & 0x7;
    double PRF_const = 121.7;

    return 10 * log10((pow(f1, 2) + pow(f2, 2) + pow(f3, 2)) / pow(PAC_val, 2)) + (6 * DGC_decision) - PRF_const;
}

/*
 Get the currently set Antenna Delay for delayedTX operations
 .@return Antenna Delay
*/
int DW3000Class::getTXAntennaDelay() { //DEPRECATED use antenna_delay variable instead!
    int delay = read(0x01, 0x04) & 0xFFFF;
    return delay;
}

/*
 Get the calculated clock offset between this chip and the chip that sent a frame
 @return Calculated clock offset of the other chip
*/
long double DW3000Class::getClockOffset() {
    if (config[0] == CHANNEL_5) {
        return getRawClockOffset() * CLOCK_OFFSET_CHAN_5_CONSTANT / 1000000;
    }
    else {
        return getRawClockOffset() * CLOCK_OFFSET_CHAN_9_CONSTANT / 1000000;
    }
}

/*
 Get the calculated clock offset from the second chips perspective
 @return Calculated clock offset of this chip from the other chips perspective
*/
long double DW3000Class::getClockOffset(int32_t sec_clock_offset) {
    if (config[0] == CHANNEL_5) {
        return sec_clock_offset * CLOCK_OFFSET_CHAN_5_CONSTANT / 1000000;
    }
    else {
        return sec_clock_offset * CLOCK_OFFSET_CHAN_9_CONSTANT / 1000000;
    }
}

/*
 Get the raw clockset offset from the register of the chip
 @return Raw clock offset
*/
int DW3000Class::getRawClockOffset() {
    int raw_offset = read(0x06, 0x29) & 0x1FFFFF;

    if (raw_offset & (1 << 20)) {
        raw_offset |= ~((1 << 21) - 1);
    }

    if (DEBUG_OUTPUT) {
        Serial.print("Raw offset: ");
        Serial.println(raw_offset);
    }
    return raw_offset;
}

/*
 Activates the chips internal temperature sensor and read its temperature
 @return the chips current temperature in °C
*/
float DW3000Class::getTempInC() {
    write(0x07, 0x34, 0x04); //enable temp sensor readings

    write(0x08, 0x00, 0x01); //enable poll

    while (!(read(0x08, 0x04) & 0x01))
    {
    };

    int res = read(0x08, 0x08);
    res = (res & 0xFF00) >> 8;
    int otp_temp = readOTP(0x09) & 0xFF;
    float tmp = (float)((res - otp_temp) * 1.05f) + 22.0f;

    write(0x08, 0x00, 0x00, 1); //Reset poll enable

    return tmp;
}

/*
 Reads the internal RX Timestamp. The timestamp is a relative timestamp to the chips internal clock. Units of ~15.65ps. (See DW3000 User Manual 4.1.7 for more)
 @return The RX Timestamp in units of ~15.65ps
*/
unsigned long long DW3000Class::readRXTimestamp() {
    uint32_t ts_low = read(0x0C, 0x00);
    unsigned long long ts_high = read(0x0C, 0x04) & 0xFF;

    unsigned long long rx_timestamp = (ts_high << 32) | ts_low;

    return rx_timestamp;
}

/*
 Reads the internal TX Timestamp. The timestamp is a relative timestamp to the chips internal clock. Units of ~15.65ps. (See DW3000 User Manual 3.2 for more)
 @return The TX Timestamp in units of ~15.65ps
*/
unsigned long long DW3000Class::readTXTimestamp() {
    unsigned long long ts_low = read(0x00, 0x74);
    unsigned long long ts_high = read(0x00, 0x78) & 0xFF;

    unsigned long long tx_timestamp = (ts_high << 32) + ts_low;

    return tx_timestamp;
}



/*
 #####  Chip Interaction  #####
*/


/*
 Writes to a specific chip register address
 @param base The chips base register address
 @param sub The chips sub register address
 @param data The data that should be written
 @param dataLen The length of the data that should be written
 @return The result of the write operation (typically 0)
*/
uint32_t DW3000Class::write(int base, int sub, uint32_t data, int dataLen) {
    return readOrWriteFullAddress(base, sub, data, dataLen, 1);
}

/*
 Writes to a specific chip register address and automatically determines the dataLen parameter
 @param base The chips base register address
 @param sub The chips sub register address
 @param data The data that should be written
 @return The result of the write operation (typically 0)
*/
uint32_t DW3000Class::write(int base, int sub, uint32_t data) {
    return readOrWriteFullAddress(base, sub, data, 0, 1);
}

/*
 Reads from a specific chip register address
 @param base The chips base register address
 @param sub The chips sub register address
 @return The result of the read operation
*/
uint32_t DW3000Class::read(int base, int sub) {
    uint32_t tmp;
    tmp = readOrWriteFullAddress(base, sub, 0, 0, 0);
    if (DEBUG_OUTPUT) Serial.println("");

    return tmp;
}

/*
 Reads just 1 Byte from the chip
 @param base The chips base register address
 @param sub The chips sub register address
 @return The result of the read operation
*/
uint8_t DW3000Class::read8bit(int base, int sub) {
    return (uint8_t)(read(base, sub) >> 24);
}



/*
 Reads a specific OTP (One Time Programmable) Memory address inside the chips register
 @param addr The OTP Memory address
 @return The result of the read operation
 */
uint32_t DW3000Class::readOTP(uint8_t addr) {
    write(OTP_IF_REG, 0x04, addr);
    write(OTP_IF_REG, 0x08, 0x02);

    return read(OTP_IF_REG, 0x10);
}



/*
 #####  Delayed Sending Settings  #####
*/


/*
 Sets a delay for a future TX operation
 @param delay The delay in units of ~4ns (see DW3000 User Manual 8.2.2.9 for more info)
*/
void DW3000Class::writeTXDelay(uint32_t delay) {
    write(0x00, 0x2C, delay);
}

/*
 A delayed TX is typically performed to have a known delay between sender and receiver.
 As the chips delayedTX function has a lower timestamp resolution, the frame gets sent
 always a little bit earlier than its supposed to be. For example:
 Delay: 10ms. Receive Time of Frame: 2010us (2.01ms). It should send at 12.01ms, but will be
 sent at 12ms as the last digits get cut off. These last digits can be precalculated and
 sent inside the frame to be added to the RX Timestamp of the receiver. In the above case, 0.01ms
 The DX_Time resolution is ~4ns, the chips internal clock resolution is ~15.65ps.

 This function calculates the missing delay time, adds it to the frames payload and sets the fixed delay (TRANSMIT_DELAY).
*/
void DW3000Class::prepareDelayedTX() {
    long long rx_ts = readRXTimestamp();

    uint32_t exact_tx_timestamp = (long long)(rx_ts + TRANSMIT_DELAY) >> 8;

    long long calc_tx_timestamp = ((rx_ts + TRANSMIT_DELAY) & ~TRANSMIT_DIFF) + antenna_delay;

    uint32_t reply_delay = calc_tx_timestamp - rx_ts;

    /*
    * PAYLOAD DESIGN:
    +------+-----------------------------------------------------------------------+-------------------------------+-------------------------------+------+------+------+-----+
    | Byte |                                 1 (0x00)                              |           2 (0x01)            |           3 (0x02)            |     4 - 6 (0x03-0x05)    |
    +------+-----+-----+----+----------+----------+----------+----------+----------+---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+------+------+------+-----+
    | Bits |  1  |  2  |  3 |     4    |     5    |     6    |     7    |     8    | 1 | 2 | 3 | 4 | 5 | 6 | 7 | 8 | 1 | 2 | 3 | 4 | 5 | 6 | 7 | 8 |                          |
    +------+-----+-----+----+----------+----------+----------+----------+----------+---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+--------------------------+
    |      | Mode bits:     | Reserved | Reserved | Reserved | Reserved | Reserved |           Sender ID           |         Destination ID        | Internal Delay / Payload |
    |      | 0 - Standard   |          |          |          |          |          |                               |                               |                          |
    |      |1-7 - See below |          |          |          |          |          |                               |                               |                          |
    +------+----------------+----------+----------+----------+----------+----------+-------------------------------+-------------------------------+--------------------------+
    *
    * Mode bits:
    * 0 - Standard
    * 1 - Double Sided Ranging
    * 2-6 - Reserved
    * 7 - Error
    */

    write(0x14, 0x01, sender & 0xFF);
    write(0x14, 0x02, destination & 0xFF);
    write(0x14, 0x03, reply_delay); //set frame content

    setFrameLength(7); // Control Byte (1 Byte) + Sender ID (1 Byte) + Dest. ID (1 Byte) + Reply Delay (4 Bytes) = 7 Bytes

    //Write delay to register 
    writeTXDelay(exact_tx_timestamp);
}



/*
 #####  Radio Stage Settings / Transfer and Receive Modes  #####
*/


/*
 Activates delayed message transfer and switches to receive mode after TX is finished
*/
void DW3000Class::delayedTXThenRX() {
    writeFastCommand(0x0F);
}

/*
 Activates delayed message transfer
*/
void DW3000Class::delayedTX() {
    writeFastCommand(0x3);
}

/*
 Performs a standard TX command
*/
void DW3000Class::standardTX() {
    DW3000Class::writeFastCommand(0x01);
}

/*
 Performs a standard RX command
*/
void DW3000Class::standardRX() {
    DW3000Class::writeFastCommand(0x02);
}

/*
 Performs a TX operation and instantly switches to Receiver mode
*/
void DW3000Class::TXInstantRX() {
    DW3000Class::writeFastCommand(0x0C);
}



/*
 #####  DW3000 Firmware Interaction  #####
*/


/*
 Soft resets the chip via software
*/
void DW3000Class::softReset() {
    clearAONConfig();

    write(PMSC_REG, 0x04, 0x1); //force clock to FAST_RC/4 clock

    write(PMSC_REG, 0x00, 0x00, 2); //init reset

    delay(100);

    write(PMSC_REG, 0x00, 0xFFFF); //return back

    write(PMSC_REG, 0x04, 0x00, 1); //set clock back to Auto mode
}

/*
 Resets the Chip by physically pulling the RST_PIN to LOW
*/
void DW3000Class::hardReset() {
    pinMode(RST_PIN, OUTPUT);
    digitalWrite(RST_PIN, LOW); // set reset pin active low to hard-reset DW3000 chip
    delay(10);
    pinMode(RST_PIN, INPUT); // get pin back in floating state
}

/*
 Clears all System Status flags
*/
void DW3000Class::clearSystemStatus() {
    write(GEN_CFG_AES_LOW_REG, 0x44, 0x3F7FFFFF);
}



/*
 #####  Hardware Status Information  #####
*/


/*
 Pulls a specific external LED to HIGH (tested on Makerfabs DW3000 board)
 @param led the index of the LED (0 - 2 possible)
 */
void DW3000Class::pullLEDHigh(int led) {
    if (led > 2) return;
    led_status = led_status + (1 << led);
    write(0x05, 0x0C, led_status);
}

/*
 Pulls a specific external LED to LOW (tested on Makerfabs DW3000 board)
 @param led the index of the LED (0 - 2 possible)
 */
void DW3000Class::pullLEDLow(int led) {
    if (led > 2) return;
    led_status = led_status & ~((int)1 << led); //https://stackoverflow.com/questions/47981/how-to-set-clear-and-toggle-a-single-bit
    write(0x05, 0x0C, led_status);
}



/*
 #####  Calculation and Conversion  #####
*/


/*
 Convert DW3000 internal picosecond units (~15.65ps per unit) to cm
 @param dw3000_ps_units DW3000 internal picosecond units. Gets returned from timestamps for example.
 @return The distance in cm
*/
double DW3000Class::convertToCM(int dw3000_ps_units) {
    return (double)dw3000_ps_units * PS_UNIT * SPEED_OF_LIGHT;
}

/*
 Calculate the Round Trip Time (RTT) for a ping operation and print out the distance in cm
*/
void DW3000Class::calculateTXRXdiff() {
    unsigned long long ping_tx = readTXTimestamp();
    unsigned long long ping_rx = readRXTimestamp();

    long double clk_offset = getClockOffset();
    long double clock_offset = 1.0 + clk_offset;

    /*
       * PAYLOAD DESIGN:
       +------+-----------------------------------------------------------------------+-------------------------------+-------------------------------+------+------+------+-----+
       | Byte |                                 1 (0x00)                              |           2 (0x01)            |           3 (0x02)            |     4 - 6 (0x03-0x...)   |
       +------+-----+-----+----+----------+----------+----------+----------+----------+---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+------+------+------+-----+
       | Bits |  1  |  2  |  3 |     4    |     5    |     6    |     7    |     8    | 1 | 2 | 3 | 4 | 5 | 6 | 7 | 8 | 1 | 2 | 3 | 4 | 5 | 6 | 7 | 8 |                          |
       +------+-----+-----+----+----------+----------+----------+----------+----------+---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+--------------------------+
       |      | Mode bits:     | Reserved | Reserved | Reserved | Reserved | Reserved |           Sender ID           |         Destination ID        | Internal Delay / Payload |
       |      | 0 - Standard   |          |          |          |          |          |                               |                               |                          |
       |      |1-7 - See below |          |          |          |          |          |                               |                               |                          |
       +------+----------------+----------+----------+----------+----------+----------+-------------------------------+-------------------------------+--------------------------+
    *
    * Mode bits:
    * 0 - Standard
    * 1 - Double Sided Ranging
    * 2-6 - Reserved
    * 7 - Error
    */

    long long t_reply = read(RX_BUFFER_0_REG, 0x03);

    /*
    * Calculate round trip time (see DW3000 User Manual page 248 for more)
    */

    if (t_reply == 0) { //t_reply is 0 when the calculation could not be done on the PONG side
        return;
    }

    long long t_round = ping_rx - ping_tx;
    long long t_prop = lround((t_round - lround(t_reply * clock_offset)) / 2);

    long double t_prop_ps = t_prop * PS_UNIT;

    long double t_prop_cm = t_prop_ps * SPEED_OF_LIGHT;
    if (t_prop_cm >= 0) {
        printDouble(t_prop_cm, 100, false); // second value sets the decimal places. 100 = 2 decimal places, 1000 = 3, 10000 = 4, ...
        Serial.println("cm");
    }
}



/*
 #####  Printing  #####
*/


/*
 Debug Output to print the Round Trip Time (RTT) and essential additional information
*/
void DW3000Class::printRoundTripInformation() {
    Serial.println("\nRound Trip Information:");
    long long tx_ts = readTXTimestamp();
    long long rx_ts = readRXTimestamp();

    Serial.print("TX Timestamp: ");
    Serial.println(tx_ts);
    Serial.print("RX Timestamp: ");
    Serial.println(rx_ts);
}

/*
 Helper function to print Doubles in a Arduino Console. Prints values with a specific number of decimal places.
 @param val The value that should be printed
 @param precision Precision is 1 followed by the number of zeros for the desired number of decimal places. Example: printDouble (3.14159, 1000); prints 3.141 (three decimal places).
 @param linebreak If True, a linebreak will be added after the print (equal to Serial.println()). If not, no linebreak (equal to Serial.print())
*/
void DW3000Class::printDouble(double val, unsigned int precision, bool linebreak) { //https://forum.arduino.cc/t/printing-a-double-variable/44327/2
    Serial.print(int(val));  // print the integer part
    Serial.print("."); // print the decimal point
    unsigned int frac;
    if (val >= 0) {
        frac = (val - int(val)) * precision;
    }
    else {
        frac = (int(val) - val) * precision;
    }
    if (linebreak) {
        Serial.println(frac, DEC); // print the fraction with linebreak
    }
    else {
        Serial.print(frac, DEC); // print the fraction without linebreak
    }
}





/*
 ==========  Private Functions  ==========
*/





/*
 #####  Single Bit Settings  #####
*/


/*
 Set bit in a defined register address
 @param reg_addr The registers base address
 @param sub_addr The registers sub address
 @param shift The bit that should be modified, relative to the base and sub address (0 for bit 0, 1 for bit 1, etc.)
 @param b The state that the bit should be set to. True if should be set to 1, False if 0
*/
void DW3000Class::setBit(int reg_addr, int sub_addr, int shift, bool b) {
    uint8_t tmpByte = read8bit(reg_addr, sub_addr);
    if (b) {
        bitSet(tmpByte, shift);
    }
    else {
        bitClear(tmpByte, shift);
    }
    write(reg_addr, sub_addr, tmpByte);
}

/*
 Sets bit to High (1) in a defined register
 @param reg_addr The registers base address
 @param sub_addr The registers sub address
 @param shift The bit that should be modified, relative to the base and sub address (0 for bit 0, 1 for bit 1, etc.)
*/
void DW3000Class::setBitHigh(int reg_addr, int sub_addr, int shift) {
    setBit(reg_addr, sub_addr, shift, 1);
}

/*
 Sets bit to Low (0) in a defined register
 @param reg_addr The registers base address
 @param sub_addr The registers sub address
 @param shift The bit that should be modified, relative to the base and sub address (0 for bit 0, 1 for bit 1, etc.)
*/
void DW3000Class::setBitLow(int reg_addr, int sub_addr, int shift) {
    setBit(reg_addr, sub_addr, shift, 0);
}



/*
 #####  Fast Commands  #####
*/


/*
 Writes a Fast Command to the chip (See DW3000 User Manual chapter 9 for more)
 @param cmd The command that should be sent
*/
void DW3000Class::writeFastCommand(int cmd) {
    if (DEBUG_OUTPUT) Serial.print("[INFO] Executing short command: ");

    int header = 0;

    header = header | 0x1;
    header = header | (cmd & 0x1F) << 1;
    header = header | 0x80;

    if (DEBUG_OUTPUT) Serial.println(header, BIN);

    int header_arr[] = { header };

    sendBytes(header_arr, 1, 0);
}



/*
 #####  SPI Interaction  #####
*/


/*
 Helper function to read or write to a specific chip register address
 @param base The base register address
 @param sub  The sub register address
 @param data The data that should be written (if anything should be written, else left at 0)
 @param dataLen The length of the data that should be sent in bits. Can be left at zero to automatically determine the data length,
        but can be very useful if e.g. 32 bits of 0x00 should be written
 @param readWriteBit Defines if a read or write operation will be performed. 0 if data should be read back, 1 if only a write should be performed.
 @return Returns 0 or the result of the read operation
*/
uint32_t DW3000Class::readOrWriteFullAddress(uint32_t base, uint32_t sub, uint32_t data, uint32_t dataLen, uint32_t readWriteBit) {
    uint32_t header = 0x00;

    if (readWriteBit) header = header | 0x80;

    header = header | ((base & 0x1F) << 1);

    if (sub > 0) {
        header = header | 0x40;
        header = header << 8;
        header = header | ((sub & 0x7F) << 2);
    }

    uint32_t header_size = header > 0xFF ? 2 : 1;
    uint32_t res = 0;

    if (!readWriteBit) {
        int headerArr[header_size];

        if (header_size == 1) {
            headerArr[0] = header;
        }
        else {
            headerArr[0] = (header & 0xFF00) >> 8;
            headerArr[1] = header & 0xFF;
        }

        res = (uint32_t)sendBytes(headerArr, header_size, 4);
        return res;
    }
    else {
        uint32_t payload_bytes = 0;
        if (dataLen == 0) {
            if (data > 0) {
                uint32_t payload_bits = countBits(data);
                payload_bytes = (payload_bits - (payload_bits % 8)) / 8; //calc the used bytes for transaction
                if ((payload_bits % 8) > 0) {
                    payload_bytes++;
                }
            }
            else {
                payload_bytes = 1;
            }
        }
        else {
            payload_bytes = dataLen;
        }
        int payload[header_size + payload_bytes];

        if (header_size == 1) {
            payload[0] = header;
        }
        else {
            payload[0] = (header & 0xFF00) >> 8;
            payload[1] = header & 0xFF;
        }

        for (int i = 0; i < payload_bytes; i++) {
            payload[header_size + i] = (data >> i * 8) & 0xFF;
        }

        res = (uint32_t)sendBytes(payload, 2 + payload_bytes, 0);  // "2 +" because the first 2 bytes are the header part
        return res;
    }
}

/*
 Internal helper function to send and receive Bytes through the SPI Interface
 @param b The bytes to be sent as an array
 @param lenB The length of the array b
 @param recLen The length of bytes that should be received back after sending the data b
 @return Returns 0 or the received bytes from the chip
 */
uint32_t DW3000Class::sendBytes(int b[], int lenB, int recLen) {
    digitalWrite(CHIP_SELECT_PIN, LOW);
    for (int i = 0; i < lenB; i++) {
        SPI.transfer(b[i]);
    }
    int rec;
    uint32_t val, tmp;
    if (recLen > 0) {
        for (int i = 0; i < recLen; i++) {
            tmp = SPI.transfer(0x00);
            if (i == 0) {
                val = tmp; //Read first 4 octets
            }
            else {
                val |= (uint32_t)tmp << 8 * i;
            }
        }
    }
    else {
        val = 0;
    }
    digitalWrite(CHIP_SELECT_PIN, HIGH);
    return val;
}



/*
 #####  Soft Reset Helper Method  #####
*/


/*
 Clears the Always On register. This register stores information as long as power is supplied to the chip.
*/
void DW3000Class::clearAONConfig() {
    write(AON_REG, NO_OFFSET, 0x00, 2);
    write(AON_REG, 0x14, 0x00, 1);

    write(AON_REG, 0x04, 0x00, 1); //clear control of aon reg

    write(AON_REG, 0x04, 0x02);

    delay(1);
}



/*
 #####  Other Helper Methods  #####
*/


/*
 Helper function to count the bits of a number
 return length of a number in bits
*/
unsigned int DW3000Class::countBits(unsigned int number) {
    return (int)log2(number) + 1;
}

/*
 Checks if a DeviceID can be read from the device (if not, SPI can not connect to the chip). Acts as a sanity check.
 @return 1 if DeviceID could be read; 0 if not.
*/
int DW3000Class::checkForDevID() {
    int res = read(GEN_CFG_AES_LOW_REG, NO_OFFSET);
    if (res != 0xDECA0302 && res != 0xDECA0312) {
        Serial.println("[ERROR] DEV_ID IS WRONG!");
        return 0;
    }
    return 1;
}
