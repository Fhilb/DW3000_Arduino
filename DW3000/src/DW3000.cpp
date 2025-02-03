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

#ifdef ESP32 //Definition for the Makerfabs DW3000 solution
    #define CHIP_SELECT_PIN 4
#else //Definition for any other chip, e.g. the DWM3000EVB shield with the Arduino Uno
    #define CHIP_SELECT_PIN 10
#endif

#define RST_PIN 27

#define DEBUG_OUTPUT 0 //Turn to 1 to get all reads, writes, etc. as info in the console

bool debug_seven_segment = 0;
bool seven_segment_used = 0;

int led_status = 0;

int device_id = 0;

int antenna_delay = 0x3FCA; //for calibration purposes; the tinier the number, the longer the ranging results

int DW3000Class::config[] = { //CHAN; PREAMBLE LENGTH; PREAMBLE CODE; PAC; DATARATE; PHR_MODE; PHR_RATE;
    CHANNEL_5,
    PREAMBLE_128,
    9, //same for tx and rx
    PAC8,
    DATARATE_6_8MB,
    PHR_MODE_STANDARD,
    PHR_RATE_850KB
};

bool DW3000Class::cmd_error = false;

void DW3000Class::begin() {
    delay(5);
    pinMode(CHIP_SELECT_PIN, OUTPUT);
    SPI.begin();

    delay(5);

    if (seven_segment_used) {
        pinMode(15, INPUT); //INIT IRQ for display
        attachInterrupt(digitalPinToInterrupt(15), DW3000Class::detectedIRQ, CHANGE);
    }

    spiSelect(CHIP_SELECT_PIN);

    Serial.println("[INFO] SPI ready");
}

void DW3000Class::hardReset() {
    pinMode(RST_PIN, OUTPUT);
    digitalWrite(RST_PIN, LOW); // set reset pin active low to hard-reset DW3000 chip
    delay(10);
    pinMode(RST_PIN, INPUT); // get pin back in floating state
}

void DW3000Class::detectedIRQ() {
    debug_seven_segment = digitalRead(15);
}

bool DW3000Class::getSevenSegmentStatus() {
    return debug_seven_segment;
}

void DW3000Class::updateDisplay() {
    if (DW3000.getSevenSegmentStatus()) {
        Serial.println("Seven segment on");
    }
    //TODO insert TM1637 code to display deviceID to seven segment display
}

/*
* This function uses 8 DIP switches connected to the pins that are declared in the below array.
* The device ID is set as a binary value through these switches.
* The LSB starts with the lowest defined pin in the pins array.
*/
void DW3000Class::updateDeviceID() { // All pins are labeled as "IO[pin number]" on the board itself
    int pins_used = 8;
    int pins[] = { 25,26,32,36,39,22, 21, 17 };
    for (int i = 0; i < pins_used; i++) { //Set all used pins as input
        pinMode(pins[i], INPUT);
    }

    int tmp_device_id = 0;

    for (int i = 0; i < pins_used; i++) {
        Serial.print("Pin ");
        Serial.print(pins[i]);
        Serial.print(": ");
        Serial.println(digitalRead(pins[i]));
        tmp_device_id += (digitalRead(pins[i]) << i);
    }

    device_id = tmp_device_id;

    Serial.print("[INFO] DeviceID updated to ");
    Serial.println(device_id);
}

void DW3000Class::setSevenSegmentActivated(bool status) {
    seven_segment_used = status;
}

void DW3000Class::spiSelect(uint8_t cs) {
    pinMode(cs, OUTPUT);
    digitalWrite(cs, HIGH);

    delay(5);
}

void DW3000Class::printFullConfig() {
    int tmp_size = 20;
    int tmp[tmp_size];
    Serial.println("\n\n#####     READING FULL CONFIG     #####\n");
    tmp[0] = read(0x00, 0x10);
    tmp[1] = read(0x00, 0x24);
    tmp[2] = read(0x00, 0x28);
    tmp[3] = read(0x00, 0x44);
    tmp[4] = read(0x01, 0x14);
    tmp[5] = read(0x02, 0x00);
    tmp[6] = read(0x03, 0x18);
    tmp[7] = read(0x04, 0x0C);
    tmp[8] = read(0x04, 0x20);
    tmp[9] = read(0x06, 0x00);
    tmp[10] = read(0x06, 0x0C);
    tmp[11] = read(0x07, 0x10);
    tmp[12] = read(0x07, 0x18);
    tmp[13] = read(0x07, 0x1C);
    tmp[14] = read(0x07, 0x50);
    tmp[15] = read(0x09, 0x00);
    tmp[16] = read(0x09, 0x08);
    tmp[17] = read(0x11, 0x04);
    tmp[18] = read(0x11, 0x08);
    tmp[19] = read(0x0E, 0x12);
    Serial.println("\n\n#####     PRINTING FULL CONFIG     #####\n");

    for (int i = 0; i < tmp_size; i++) {
        Serial.println(tmp[i], BIN);
    }
}

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
    
    //int dtune3_val[] = { 0x4C, 0x58, 0x5F, 0xAF }; //TODO if not working: change value back (p.147)
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
        rf_tx_ctrl_2 |=  0x000001;
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

void DW3000Class::configureAsTX() {
    write(RF_CONF_REG, 0x1C, 0x34); //write pg_delay
    write(GEN_CFG_AES_HIGH_REG, 0x0C, 0xFDFDFDFD);
}


void DW3000Class::setTXFrame(unsigned long long frame_data) {  // deprecated! use write(TX_BUFFER_REG, [...]);
    if (frame_data > ((pow(2, 8 * 8) - FCS_LEN))) {
        Serial.println("[ERROR] Frame is too long (> 1023 Bytes - FCS_LEN)!");
        return;
    }

    write(TX_BUFFER_REG, 0x00, frame_data);
}

void DW3000Class::printRoundTripInformation() {
    Serial.println("\nRound Trip Information:");
    long long tx_ts = readTXTimestamp();
    long long rx_ts = readRXTimestamp();

    Serial.print("TX Timestamp: ");
    Serial.println(tx_ts);
    Serial.print("RX Timestamp: ");
    Serial.println(rx_ts);
}

int DW3000Class::getAnchorID() {
	return anchor_id;
}

uint32_t DW3000Class::sendBytes(int b[], int lenB, int recLen) { //WORKING
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

bool DW3000Class::checkForIDLE() {
   return (read(0x0F, 0x30) >> 16 & PMSC_STATE_IDLE) == PMSC_STATE_IDLE || (read(0x00, 0x44) >> 16 & (SPIRDY_MASK | RCINIT_MASK)) == (SPIRDY_MASK | RCINIT_MASK) ? 1 : 0;
}

void DW3000Class::softReset() {
    clearAONConfig();

    write(PMSC_REG, 0x04, 0x1); //force clock to FAST_RC/4 clock
    
    write(PMSC_REG, 0x00, 0x00, 2); //init reset

    delay(100);

    write(PMSC_REG, 0x00, 0xFFFF); //return back

    write(PMSC_REG, 0x04, 0x00, 1); //set clock back to Auto mode
}

void DW3000Class::clearAONConfig() {
    write(AON_REG, NO_OFFSET, 0x00, 2);
    write(AON_REG, 0x14, 0x00, 1);

    write(AON_REG, 0x04, 0x00, 1); //clear control of aon reg

    write(AON_REG, 0x04, 0x02);

    delay(1);
}

uint32_t DW3000Class::readOrWriteFullAddress(uint32_t base, uint32_t sub, uint32_t data, uint32_t data_len, uint32_t readWriteBit) {
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
        if (data_len == 0) {
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
            payload_bytes = data_len;
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

        res = (uint32_t)sendBytes(payload, 2 + payload_bytes, 0);
        return res;
    }
}


uint32_t DW3000Class::read(int base, int sub) {
    uint32_t tmp;
    tmp = readOrWriteFullAddress(base, sub, 0, 0, 0);
    if (DEBUG_OUTPUT) Serial.println("");

    return tmp;
}

uint8_t DW3000Class::read8bit(int base, int sub) {
    return (uint8_t)(read(base, sub) >> 24);
}

uint32_t DW3000Class::write(int base, int sub, uint32_t data, int data_len) {
    return readOrWriteFullAddress(base, sub, data, data_len, 1);
}

uint32_t DW3000Class::write(int base, int sub, uint32_t data) {
    return readOrWriteFullAddress(base, sub, data, 0, 1);
}

uint32_t DW3000Class::readOTP(uint8_t addr) {
    write(OTP_IF_REG, 0x04, addr);
    write(OTP_IF_REG, 0x08, 0x02);

    return read(OTP_IF_REG, 0x10);
}


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

    write(0x07, 0x34, 0x4); //enable temp sensor readings


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

void DW3000Class::setupGPIO() {
    write(0x05, 0x08, 0xF0); //Set GPIO0 - GPIO3 as OUTPUT on DW3000
}

void DW3000Class::pullLEDHigh(int led) { //led 0 - 2 possible
    if (led > 2) return;
    led_status = led_status + (1 << led);
    write(0x05, 0x0C, led_status);
}

void DW3000Class::pullLEDLow(int led) { //led 0 - 2 possible
    if (led > 2) return;
    led_status = led_status & ~((int)1 << led); //https://stackoverflow.com/questions/47981/how-to-set-clear-and-toggle-a-single-bit
    write(0x05, 0x0C, led_status);
}

void DW3000Class::writeTXDelay(uint32_t delay) {
    write(0x00, 0x2C, delay);
}

void DW3000Class::delayedTXThenRX() { //Activates Transmission with delay programmed through writeTXDelay() and goes to receiver immediately
    writeFastCommand(0x0F);
}

void DW3000Class::delayedTX() {
    writeFastCommand(0x3);
}

unsigned long long DW3000Class::readRXTimestamp() {
    uint32_t ts_low = read(0x0C, 0x00);
    unsigned long long ts_high = read(0x0C, 0x04) & 0xFF;

    unsigned long long rx_timestamp = (ts_high << 32) | ts_low;

    return rx_timestamp;
}


unsigned long long DW3000Class::readTXTimestamp() {
    unsigned long long ts_low = read(0x00, 0x74);
    unsigned long long ts_high = read(0x00, 0x78) & 0xFF;

    unsigned long long tx_timestamp = (ts_high << 32) + ts_low;

    return tx_timestamp;
}


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

    write(0x14, 0x03, reply_delay); //set frame content

    setFrameLength(7); // Control Byte (1 Byte) + Sender ID (1 Byte) + Dest. ID (1 Byte) + Reply Delay (4 Bytes) = 7 Bytes

    //Write delay to register 
    writeTXDelay(exact_tx_timestamp);
}


void DW3000Class::calculateTXRXdiff() { //calc diff on PING side
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

void DW3000Class::ds_sendFrame(int stage) {
    setMode(1); 
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

void DW3000Class::ds_sendRTInfo(int t_roundB, int t_replyB) {
    setMode(1);
    write(0x14, 0x03, 4);
    write(0x14, 0x04, t_roundB);
    write(0x14, 0x08, t_replyB);

    setFrameLength(12);

    TXInstantRX();
}

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
    
    int first_rt  = t_roundA - t_replyB;
    int second_rt = t_roundB - t_replyA;
    
    int combined_rt = (first_rt + second_rt - (reply_diff - (reply_diff * clock_offset))) / 2;
    int combined_rt_raw = (first_rt + second_rt) / 2;

    return combined_rt / 2; // divided by 2 to get just one range
}

/*
* NOTE: If not using 64MHz PRF: See user manual capter 4.7.2 for an alternative calculation
*/
double DW3000Class::getSignalStrength() { //Returns the signal strength of the received frame in dBm
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

double DW3000Class::getFirstPathSignalStrength() {
    float f1 = (read(0x0C, 0x30) & 0x3FFFFF) >> 2;
    float f2 = (read(0x0C, 0x34) & 0x3FFFFF) >> 2;
    float f3 = (read(0x0C, 0x38) & 0x3FFFFF) >> 2;

    int PAC_val = read(0x0C, 0x58) & 0xFFF;
    unsigned int DGC_decision = (read(0x03, 0x60) >> 28) & 0x7;
    double PRF_const = 121.7;

    return 10 * log10((pow(f1, 2) + pow(f2, 2) + pow(f3, 2)) / pow(PAC_val, 2)) + (6 * DGC_decision) - PRF_const;
}

void DW3000Class::setReceiverAddress(unsigned int addr) { //Write receiver address to frame
    write(0x14, 0x02, addr & 0xFF); //set receiver address
}

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

int DW3000Class::receivedFrameSucc() { //No frame received: 0; frame received: 1; error while receiving: 2
    int sys_stat = read(GEN_CFG_AES_LOW_REG, 0x44);
    if ((sys_stat & SYS_STATUS_FRAME_RX_SUCC) > 0) {
        return 1;
    }
    else if ((sys_stat & SYS_STATUS_RX_ERR) > 0) {
        return 2;
    }
    return 0;
}

int DW3000Class::sentFrameSucc() { //No frame sent: 0; frame sent: 1; error while sending: 2
    int sys_stat = read(GEN_CFG_AES_LOW_REG, 0x44);
    if ((sys_stat & SYS_STATUS_FRAME_TX_SUCC) == SYS_STATUS_FRAME_TX_SUCC) {
        return 1;
    }
    return 0;
}

unsigned long long DW3000Class::readRXBuffer() { // deprecated! Use read(RX_BUFFER_0_REG, [...]); instead
    unsigned long long buf0 = read(RX_BUFFER_0_REG, 0x0);
    buf0 = buf0 + (read(0x12, 0x20) << 32);
    
    return buf0;
}

void DW3000Class::standardTX() {
    DW3000Class::writeFastCommand(0x01);
}

void DW3000Class::standardRX() {
    DW3000Class::writeFastCommand(0x02);
}

void DW3000Class::TXInstantRX() { // Execute tx, then set receiver to rx instantly
    DW3000Class::writeFastCommand(0x0C);
}

void DW3000Class::clearSystemStatus() {
    write(GEN_CFG_AES_LOW_REG, 0x44, 0x3F7FFFFF);
}

int DW3000Class::checkForDevID() {
    if (read(GEN_CFG_AES_LOW_REG, NO_OFFSET) != 0xDECA0302) {
        Serial.println("[ERROR] DEV_ID IS WRONG!");
        return 0;
    }
    return 1;
}

bool DW3000Class::checkSPI() {
    int res = read(0x00, 0x00);
    return (res > 0);

void DW3000Class::setMode(int mode) {
    write(0x14, 0x00, mode & 0x7);
}

void DW3000Class::setFrameLength(int frame_len) { // set Frame length in Bytes
    frame_len = frame_len + FCS_LEN;
    int curr_cfg = read(0x00, 0x24);
    if (frame_len > 1023) {
        Serial.println("[ERROR] Frame length + FCS_LEN (2) is longer than 1023. Aborting!");
        return;
    }
    int tmp_cfg = (curr_cfg & 0xFFFFFC00) | frame_len;
    
    write(GEN_CFG_AES_LOW_REG, 0x24, tmp_cfg);
}

double DW3000Class::convertToCM(int dw3000_ps_units) {
    return (double)dw3000_ps_units * PS_UNIT * SPEED_OF_LIGHT; 
}

int DW3000Class::ds_getStage() {
    return read(0x12, 0x03) & 0b111;
}

bool DW3000Class::ds_isErrorFrame() {
    return ((read(0x12, 0x00) & 0x7) == 7);
}

void DW3000Class::ds_sendErrorFrame() {
    Serial.println("[WARNING] Error Frame sent. Reverting back to stage 0.");
    setMode(7);
    setFrameLength(3);
    TXInstantRX();
}

/*
* Set bit in a defined register address
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

void DW3000Class::setBitHigh(int reg_addr, int sub_addr, int shift) {
    setBit(reg_addr, sub_addr, shift, 1);
}
void DW3000Class::setBitLow(int reg_addr, int sub_addr, int shift) {
    setBit(reg_addr, sub_addr, shift, 0);
}

void DW3000Class::setDeviceID(unsigned int addr) {
    device_id = addr;
}


/*
    ==== CONFIGURATION FUNCTIONS ====
*/

void DW3000Class::setChannel(uint8_t data) {
    if(data == CHANNEL_5 || data == CHANNEL_9) config[0] = data;
}

void DW3000Class::setPreambleLength(uint8_t data) {
    if (data == PREAMBLE_32 || data == PREAMBLE_64 || data == PREAMBLE_1024 ||
        data == PREAMBLE_256 || data == PREAMBLE_512 || data == PREAMBLE_1024 ||
        data == PREAMBLE_1536 || data == PREAMBLE_2048 || data == PREAMBLE_4096) config[1] = data;
}

void DW3000Class::setPreambleCode(uint8_t data) {
    if (data <= 12 && data >= 9) config[2] = data;
}

void DW3000Class::setPACSize(uint8_t data) {
    if (data == PAC4 || data == PAC8 || data == PAC16 || data == PAC32) config[3] = data;
}

void DW3000Class::setDatarate(uint8_t data) {
    if (data == DATARATE_6_8MB || data == DATARATE_850KB) config[4] = data;
}

void DW3000Class::setPHRMode(uint8_t data) {
    if (data == PHR_MODE_STANDARD || data == PHR_MODE_LONG) config[5] = data;
}

void DW3000Class::setPHRRate(uint8_t data) {
    if (data == PHR_RATE_6_8MB || data == PHR_RATE_850KB) config[6] = data;
}

void DW3000Class::setTXAntennaDelay(int delay) {
    antenna_delay = delay;
    write(0x01, 0x04, delay);
}

int DW3000Class::getTXAntennaDelay() { //DEPRECATED use antenna_delay variable instead!
    int delay = read(0x01, 0x04) & 0xFFFF;
    return delay;
}

long double DW3000Class::getClockOffset() {     
    if (config[0] == CHANNEL_5) {
        return getRawClockOffset() * CLOCK_OFFSET_CHAN_5_CONSTANT / 1000000;
    }
    else {
        return getRawClockOffset() * CLOCK_OFFSET_CHAN_9_CONSTANT / 1000000;
    }
}

long double DW3000Class::getClockOffset(int32_t sec_clock_offset) {
    if (config[0] == CHANNEL_5) {
        return sec_clock_offset * CLOCK_OFFSET_CHAN_5_CONSTANT / 1000000;
    }
    else {
        return sec_clock_offset * CLOCK_OFFSET_CHAN_9_CONSTANT / 1000000;
    }
}

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


float DW3000Class::getTempInC() {
        write(0x07, 0x34, 0x04); //enable temp sensor readings

        write(0x08, 0x00, 0x01); //enable poll

        while (!(read(0x08, 0x04) & 0x01)) 
        { };

        int res = read(0x08, 0x08);
        res = (res & 0xFF00) >> 8;
        int otp_temp = readOTP(0x09) & 0xFF;
        float tmp = (float)((res - otp_temp) * 1.05f) + 22.0f;

        write(0x08, 0x00, 0x00, 1); //Reset poll enable

        return tmp;
}


unsigned int DW3000Class::countBits(unsigned int number) {
    return (int)log2(number) + 1;
}


void DW3000Class::printDouble(double val, unsigned int precision, bool linebreak) { //https://forum.arduino.cc/t/printing-a-double-variable/44327/2
    // prints val with number of decimal places determine by precision
    // NOTE: precision is 1 followed by the number of zeros for the desired number of decimial places
    // example: printDouble( 3.1415, 100); // prints 3.14 (two decimal places)
    Serial.print(int(val));  //prints the int part
    Serial.print(","); // print the decimal point
    unsigned int frac;
    if (val >= 0) {
        frac = (val - int(val)) * precision;
    }
    else {
        frac = (int(val) - val) * precision;
    }
    if (linebreak) {
        Serial.println(frac, DEC);
    }
    else {
        Serial.print(frac, DEC);
    }
    
}
