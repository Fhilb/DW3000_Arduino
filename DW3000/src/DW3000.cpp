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
#include <stdlib.h>
#include "DW3000.h"

DW3000Class DW3000;

#define CHIP_SELECT_PIN 4
#define RST_PIN 27

#define DEBUG_OUTPUT 0 //Turn to 1 to get all reads, writes, etc. as info in the console

int no_data[] = { 0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0}; //32 bit array for clearing zeroes
int led_status = 0;

int antenna_delay = 0x4015;

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
    //attachInterrupt(digitalPinToInterrupt(2), DW3000Class::interruptDetect, RISING

    spiSelect(CHIP_SELECT_PIN);

    Serial.println("[INFO] SPI ready");
    
}

void DW3000Class::hardReset() {
    pinMode(RST_PIN, OUTPUT);
    digitalWrite(RST_PIN, LOW); // set reset pin active low to hard-reset DW3000 chip
    delay(10);
    pinMode(RST_PIN, INPUT); // get pin back in floating state
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
    //printSystemConfig();
    int usr_cfg = (STDRD_SYS_CONFIG & 0xFFF) | (config[5] << 3) | (config[6] << 4);
    /*Serial.println("config:");
    Serial.println(config[5] << 5, BIN);
    Serial.println(config[6] << 6, BIN);
    Serial.println(usr_cfg & 0xFFF, BIN);*/
    int usr_data[] = { usr_cfg & 0xFF, (usr_cfg & 0xF00)>>8};
    /*Serial.print("usr_data: ");
    Serial.print(usr_data[0]);
    Serial.print(" ; ");
    Serial.println(usr_data[1]);*/
    write(GEN_CFG_AES_LOW_REG, 0x10, usr_data, 2); //write user config
   
    if (config[2] > 24) {
        Serial.println("[ERROR] SCP ERROR! TX & RX Preamble Code higher than 24!");
    }
    
    int otp_write[] = { 0x0, 0x14 };

    if (config[1] >= 256) {
        otp_write[1] = 0x04;
    }
    

    write(OTP_IF_REG, 0x08, otp_write, 2); //set OTP config
    write(DRX_REG, 0x00, no_data, 1); //reset DTUNE0_CONFIG
    
    int usr_dtune0_cfg[] = { config[3] };
    write(DRX_REG, 0x0, usr_dtune0_cfg, 1);
    
    //64 = STS length
    int sts_cfg[] = { 64 / 8 - 1};
    write(STS_CFG_REG, 0x0, sts_cfg, 1);
    
    write(GEN_CFG_AES_LOW_REG, 0x29, no_data, 1);
    
    int dtune3_val[] = { 0x4C, 0x58, 0x5F, 0xAF }; //TODO if not working: change value back (p.147)
    write(DRX_REG, 0x0C, dtune3_val, 4);
    
    int chan_ctrl_val = read(GEN_CFG_AES_HIGH_REG, 0x14);  //Fetch and adjust CHAN_CTRL data
    chan_ctrl_val &= (~0x1FFF);

    chan_ctrl_val |= config[0]; //Write RF_CHAN

    chan_ctrl_val |= 0x1F00 & (config[2] << 8);
    chan_ctrl_val |= 0xF8 & (config[2] << 3);
    chan_ctrl_val |= 0x06 & (0x01 << 1);

    int chan_ctrl_data[] = {
        (chan_ctrl_val & 0xFF),
        (chan_ctrl_val & 0xFF00) >> 8,
        (chan_ctrl_val & 0xFF0000) >> 16,
        (chan_ctrl_val & 0xFF000000) >> 24
    };

    write(GEN_CFG_AES_HIGH_REG, 0x14, chan_ctrl_data, 4);  //Write new CHAN_CTRL data with updated values

    int tx_fctrl_val = read(GEN_CFG_AES_LOW_REG, 0x24); 

    tx_fctrl_val |= (config[1] << 12); //Add preamble length
    tx_fctrl_val |= (config[4] << 10); //Add data rate

    int tx_fctrl_data[] = {
        (tx_fctrl_val & 0xFF),
        (tx_fctrl_val & 0xFF00) >> 8,
        (tx_fctrl_val & 0xFF0000) >> 16,
        (tx_fctrl_val & 0xFF000000) >> 24
    };
    write(GEN_CFG_AES_LOW_REG, 0x24, tx_fctrl_data, 4);

    int drx_data[] = { 0x81 };
    write(DRX_REG, 0x02, drx_data, 1);

    int rf_tx_ctrl_2_data[] = { 0x34, 0x11, 0x07, 0x1C };
    int pll_conf[] = { 0x3C, 0x0F };

    if (config[0]) {
        rf_tx_ctrl_2_data[1] = 0x00;
        rf_tx_ctrl_2_data[2] = 0x01;
        pll_conf[1] = 0x1F;
    }

    write(RF_CONF_REG, 0x1C, rf_tx_ctrl_2_data, 4);
    write(FS_CTRL_REG, 0x00, pll_conf, 2);

    int ldo_rload_val[] = { 0x14 }; 
    write(RF_CONF_REG, 0x51, ldo_rload_val, 1);

    int rf_tx_ctrl_1_val[] = { 0x0E };
    write(RF_CONF_REG, 0x1A, rf_tx_ctrl_1_val, 1);

    int rf_pll_cal_val[] = { 0x81 };
    write(FS_CTRL_REG, 0x08, rf_pll_cal_val, 1);

    int sys_status_clear[] = { 0x02 };
    write(GEN_CFG_AES_LOW_REG, 0x44, sys_status_clear, 1);
    
    int auto_clock_val[] = { 0x00, 0x02, 0x30 };
    write(PMSC_REG, 0x04, auto_clock_val, 4); //Set clock to auto mode

    int ainit2idle_val[] = { 0x38, 0x01 };
    write(PMSC_REG, 0x08, ainit2idle_val, 2);

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

    int otp_data[] = {
        (otp_val & 0xFF),
        (otp_val & 0xFF00) >> 8
    };
    write(OTP_IF_REG, 0x08, otp_data, 2);

    int dgc_cfg[] = { 0xF0 };
    write(RX_TUNE_REG, 0x19, dgc_cfg, 1);

    int ldo_ctrl_val = read(RF_CONF_REG, 0x48); //Save original LDO_CTRL data
    int tmp_ldo = (0x105 |
        0x100 |
        0x4 |
        0x1);
    int tmp_ldo_data[] = {
        (tmp_ldo & 0xFF),
        (tmp_ldo & 0xFF00) >> 8
    };
    write(RF_CONF_REG, 0x48, tmp_ldo_data, 2); 

    int rx_cal_data[] = { 0x00, 0x00, 0x02 };
    write(EXT_SYNC_REG, 0x0C, rx_cal_data, 3); //Calibrate RX

    int l = read(0x04, 0x0C);

    delay(20);

    int rx_cal_data2[] = { 0x10 | 0x01 };
    write(EXT_SYNC_REG, 0x0C, rx_cal_data2, 1); //Enable calibration

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

    write(EXT_SYNC_REG, 0x0C, no_data, 2);
    int rx_cal_sts_data[] = { 0x01 };
    write(EXT_SYNC_REG, 0x20, rx_cal_sts_data, 1);

    int rx_cal_res = read(EXT_SYNC_REG, 0x14);
    if (rx_cal_res == 0x1fffffff) {
        Serial.println("[ERROR] PGF_CAL failed in stage I!");
    }
    rx_cal_res = read(EXT_SYNC_REG, 0x1C);
    if (rx_cal_res == 0x1fffffff) {
        Serial.println("[ERROR] PGF_CAL failed in stage Q!");
    }


    int ldo_rload_data[] = {
        (ldo_ctrl_val & 0xFF),
        (ldo_ctrl_val & 0xFF00) >> 8,
        (ldo_ctrl_val & 0xFF0000) >> 16,
        (ldo_ctrl_val & 0xFF000000) >> 24
    };
    write(RF_CONF_REG, 0x48, ldo_rload_data, 4); //Restore original LDO_CTRL data


    setTXAntennaDelay(0x4015); //set default antenna delay
}

void DW3000Class::configureAsTX() {
    int pg_delay_data[] = { 0x34 };
    write(RF_CONF_REG, 0x1C, pg_delay_data, 1); //write pg_delay
    int power_data[] = { 0xFD, 0xFD, 0xFD, 0xFD };
    write(GEN_CFG_AES_HIGH_REG, 0x0C, power_data, 4);
}


void DW3000Class::setTXFrame(unsigned long long frame_data) {  // deprecated! use write(TX_BUFFER_REG, [...]);
    if (frame_data > ((pow(2, 8 * 8) - FCS_LEN))) {
        Serial.println("[ERROR] Frame is too long (> 1023 Bytes - FCS_LEN)!");
        return;
    }

    int data[8]; //8 due to size of int of 8 Bytes
    for (int i = 0; i < 8; i++) {
        data[i] = (frame_data >> i * 8) & 0xFF;
    }

    write(TX_BUFFER_REG, 0x00, data, 8);
    //int h = read(TX_BUFFER_REG, 0x00);
    //Serial.println("Frame content: ");
    //Serial.println(h, BIN);
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


int* DW3000Class::getBase(int hex_num)
{
    return hexToBin(hex_num, 5);
}

int* DW3000Class::getSub(int hex_num)
{
    return hexToBin(hex_num, 7);
}

int* DW3000Class::hexToBin(int hex_num, int bit_size)
{
    int bit_count = bit_size;
    int* binary_num = (int*)malloc(bit_count * sizeof(int));
    int i;
    for (i = bit_count - 1; i >= 0; i--) {
        binary_num[i] = hex_num % 2;
        hex_num >>= 1;
    }
    return binary_num;
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

/*uint32_t DW3000Class::readOrWriteFullAddress(int* base, int base_len, int* sub, int sub_len, int* data, int data_len, int readWriteBit) {
    return DW3000Class::readOrWriteFullAddress(base, base_len, sub, sub_len, data, data_len, readWriteBit, false);
}*/

void DW3000Class::softReset() {
    clearAONConfig();

    int sys_clk_data[] = { 0x1 };
    write(PMSC_REG, 0x04, sys_clk_data, 1); //force clock to FAST_RC/4 clock
    
    write(PMSC_REG, 0x00, no_data, 2); //init reset

    delay(100);

    int reset_data[] = { 0xFF, 0xFF };
    write(PMSC_REG, 0x00, reset_data, 2); //return back

    write(PMSC_REG, 0x04, no_data, 1); //set clock back to Auto mode
}

void DW3000Class::clearAONConfig() {
    write(AON_REG, NO_OFFSET, no_data, 2);
    write(AON_REG, 0x14, no_data, 1);

    write(AON_REG, 0x04, no_data, 1); //clear control of aon reg

    int save_conf[] = {0x02};
    write(AON_REG, 0x04, save_conf, 1);

    delay(1);
}

uint32_t DW3000Class::readOrWriteFullAddress(int *base, int base_len, int *sub, int sub_len, int *data, int data_len, int readWriteBit) {
    int fill_base_len = 5;
    int num_zeros = fill_base_len - base_len;
    if (num_zeros < 0) {
        num_zeros = 0;
    }
    int fill_base[fill_base_len]; //fill leading zeros

    for (int i = 0; i < fill_base_len; i++) {
        fill_base[num_zeros + i] = base[i];
    }


    int fill_sub_len = 7;
    int fill_sub[fill_sub_len]; //fill leading zeros  
    num_zeros = fill_sub_len - sub_len;
    if (num_zeros < 0) {
        num_zeros = 0;
    }
    for (int i = 0; i < fill_sub_len; i++) {
        fill_sub[num_zeros + i] = sub[i];
    }

    int first_byte[8] = { readWriteBit, 1 };
    for (int i = 0; i < fill_base_len; i++) {
        first_byte[i + 2] = fill_base[i];
    }
    first_byte[7] = fill_sub[0];

    int second_byte[8];
    second_byte[8 - 1] = 0; //Last two bits are set to 0 (mode selector bits)
    second_byte[8 - 2] = 0;

    for (int i = 0; i < fill_sub_len - 1; i++) {
        second_byte[i] = fill_sub[i + 1];
    }

    int byteOne = 0;
    int byteTwo = 0;

    for (int i = 7; i >= 0; i--) {
        byteOne = byteOne + first_byte[i] * round(pow(2, 7 - i));
        byteTwo = byteTwo + second_byte[i] * round(pow(2, 7 - i));
    }

    uint32_t val;

    int bytes[data_len + 2] = { byteOne, byteTwo };

    for (int i = 0; i < data_len; i++) {
        bytes[i + 2] = data[i];
    }

    uint32_t res;

    if (readWriteBit == 0) {
        if (DEBUG_OUTPUT) {
            Serial.print("Reading from ");
            for (int i = 0; i < fill_base_len; i++) {
                Serial.print(fill_base[i]);
            }
            Serial.print(":");
            for (int i = 0; i < fill_sub_len; i++) {
                Serial.print(fill_sub[i]);
            }
            Serial.println("");
        }
        
        res = (uint32_t)sendBytes(bytes, 2 + data_len, 4);

        if (DEBUG_OUTPUT) {
            Serial.print("Received result (HEX): ");
            Serial.print(res, HEX);
            Serial.print(" (BIN): ");
            Serial.println(res, BIN);
        }
        return res;
    }
    else {
        if (DEBUG_OUTPUT) {
            Serial.print("Writing to ");
            for (int i = 0; i < fill_base_len; i++) {
                Serial.print(fill_base[i]);
            }
            Serial.print(":");
            for (int i = 0; i < fill_sub_len; i++) {
                Serial.print(fill_sub[i]);
            }
            Serial.println("");
        }
        res = (uint32_t)sendBytes(bytes, 2 + data_len, 0);
        return res;
    }
}

uint32_t DW3000Class::read(int base, int sub) {
    int* _base = DW3000Class::getBase(base);
    int* _sub = DW3000Class::getSub(sub);
    
    int t[] = {0};
    uint32_t tmp;
    tmp = readOrWriteFullAddress(_base, 5, _sub, 7, t, 0, 0);
    if (DEBUG_OUTPUT) Serial.println("");
    free(_base);
    free(_sub);

    return tmp;
}

uint8_t DW3000Class::read8bit(int base, int sub) {
    return (uint8_t)(read(base, sub) >> 24);
}

uint32_t DW3000Class::write(int base, int sub, int *data, int data_len) {
    int* _base = DW3000Class::getBase(base);
    int* _sub = DW3000Class::getSub(sub);
    readOrWriteFullAddress(_base, 5, _sub, 7, data, data_len, 1);
    free(_base);
    free(_sub);

    return 0;
}

uint32_t DW3000Class::readOTP(uint8_t addr) {
    int otp_man[] = { 0x1 };

    int otp_addr[] = { addr };
    write(OTP_IF_REG, 0x04, otp_addr, 1);


    int read_otp[] = { 0x02 };
    write(OTP_IF_REG, 0x08, read_otp, 1);

    return read(OTP_IF_REG, 0x10);
}


void DW3000Class::init() {
    Serial.println("\n+++ DecaWave DW3000 Test +++\n");

    if (!checkForDevID()) {
        Serial.println("[ERROR] Dev ID is wrong! Aborting!");
        return;
    }
    
    setBitHigh(GEN_CFG_AES_LOW_REG, 0x10, 4); //MOD2
    
    
    while (!checkForIDLE()) {
        Serial.println("[WARNING] IDLE FAILED (stage 1)");
        delay(100);
    }
    
    softReset(); //MOD2

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
        int bias_data[] = {
            bias_tune & 0xFF,
            (bias_tune >> 8) & 0xFF
        };
        write(0x11, 0x1F, bias_data, 2);

        int ldo_kick_data[] = { 0x00, 0x01 };
        write(0x0B, 0x08, ldo_kick_data, 2);
    }

    int xtrim_value[1];
    xtrim_value[0] = readOTP(0x1E);
    
    xtrim_value[0] = (xtrim_value[0] == 0 ? 0x2E : xtrim_value[0]);

    write(FS_CTRL_REG, 0x14, xtrim_value, 1);
    if (DEBUG_OUTPUT) Serial.print("xtrim: ");
    if (DEBUG_OUTPUT) Serial.println(xtrim_value[0]);
    

    
    writeSysConfig();
    
    int data1[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF}; //0xF2,0x2B,0x0,0x0,0x0,0x2
    DW3000Class::write(0x0, 0x3C, data1, 6); //Set Status Enable

    int data3[] = {0x00,0x9, 0x0};
    DW3000Class::write(0xA, 0x0, data3, 3); //AON_DIG_CFG register setup; sets up auto-rx calibration and on-wakeup GO2IDLE  //0xA
    
    /*
     * Set RX and TX config
     */
    int data4[] = {0x40,0x02,0x00,0x10}; //DGC_CFG0  //0x3 //MOD
    DW3000Class::write(0x3, 0x1C, data4, 4);

    int data5[] = { 0x89,0xa4,0x6d,0x1b }; //DGC_CFG1
    DW3000Class::write(0x3, 0x20, data5, 4);

    int data6[] = { 0xFD,0xC0,0x01,0x00 }; //DGC_LUT_0
    DW3000Class::write(0x3, 0x38, data6, 4);
   
    int data7[] = { 0x3E,0xC4,0x01,0x00 }; //DGC_LUT_1
    DW3000Class::write(0x3, 0x3C, data7, 4);
    
    int data8[] = { 0xBE,0xC6,0x01,0x00 }; //DGC_LUT_2
    DW3000Class::write(0x3, 0x40, data8, 4);
    
    int data9[] = { 0x7E,0xC7,0x01,0x00 }; //DGC_LUT_3
    DW3000Class::write(0x3, 0x44, data9, 4);
    
    int data10[] = { 0x36,0xCF,0x01,0x00 }; //DGC_LUT_4
    DW3000Class::write(0x3, 0x48, data10, 4);
    
    int data11[] = { 0xB5,0xCF,0x01,0x00 }; //DGC_LUT_5
    DW3000Class::write(0x3, 0x4C, data11, 4);
    
    int data12[] = { 0xF5,0xCF,0x01,0x00 }; //DGC_LUT_6
    DW3000Class::write(0x3, 0x50, data12, 4);

    int data13[] = { 0xE5,0xE5 };
    DW3000Class::write(0x3, 0x18, data13, 2); //THR_64 value set to 0x32
    int f = DW3000Class::read(0x4, 0x20);

    //SET PAC TO 32 (0x00) reg:06:00 bits:1-0, bit 4 to 0 (00001100) (0xC)
    int data14[] = { 0x1C, 0x10, 0x81 };
    DW3000Class::write(0x6, 0x0, data14, 3); 

    int data15[]{ 0x01, 0x40 };
    write(0x01, 0x04, data15, 2); //change tx antenna delay to 0x4001

    int data16[] = { 0x4 }; //enable temp sensor readings
    write(0x07, 0x34, data16, 1);

    //SET PREAMBLE CODE (RX_PCODE, TX_PCODE) TO 10 (reg:01:14) //Standard SFD Type is 11 (data: 0x56, 0x5), trying 00 (0x50, 0x5)
    //int data16[] = { 0x4A, 0x09 };
    //DW3000Class::write(0x1, 0x14, data16, 2);  //0x1

    // write preamble length, frame length, data rate and prf in TX_FCTRL  //PSR = 1024, TXFLEN = 3 Byte (1 data, 2 CRC) TXBR = 6.81Mb/s, TR Bit enabled, FINE_PLEN = 0x0
    // reg:00:24 bits 0 - 25
    /*int data27[] = {0x03, 0x2C};
    DW3000Class::write(0x0, 0x24, data27, 2);*/


    


    /*
     * Things to do as documented in https://gist.github.com/egnor/455d510e11c22deafdec14b09da5bf54
     */


    int data17[] = {0x14};
    DW3000Class::write(0x7, 0x48, data17, 1); //LDO_RLOAD to 0x14 //0x7
    int data18[] = { 0xE };
    DW3000Class::write(0x7, 0x1A, data18, 1); //RF_TX_CTRL_1 to 0x0E
    int data19[] = { 0x34,0x11,0x07,0x1C };
    DW3000Class::write(0x7, 0x1C, data19, 4); //RF_TX_CTRL_2 to 0x1C071134 (due to channel 5, else (9) to 0x1C010034)

    int data20[] = { 0x3C,0x1F };
    DW3000Class::write(0x9, 0x0, data20, 2); //PLL_CFG to 0x1F3C (due to channel 5, else (9) to 0x0F3C)  //0x9

    int data21[] = { 0x81 };
    DW3000Class::write(0x9, 0x8, data21, 1); //PLL_CAL config to 0x81

    int data22[] = { 0x11 };
    int data23[] = { 0x0 }; //if finished with calibration go back in cal_mode //also used to reset LDO_CTRL to 0x0
 
    
    delay(200);
    
    /*for (int i = 0; i < 5; i++) { //MOD2 //siehe dwt_run_pgfcal in makerfabs device_api
        delay(50);
        
        uint32_t h = DW3000Class::read(0x4, 0x20); //Read antenna calibration //RX_CAL_STS => Status bit, if high antenna cal was successful
        if (h > 0) {
            Serial.println("Antenna calibration completed.");
            break;
        }
        else {
            if (i < 4) {
                Serial.println("[WARNING] Antenna auto calibration failed! Retrying...");
                DW3000Class::write(0x4, 0x0C, data22, 1);
            }
            else {
                Serial.println("[ERROR] Antenna auto calibration failed! Aborting!");
            }
        }
    }*/
    
    if (DW3000Class::read(0x4, 0x20)) {
        int data23[] = {0x1};
        DW3000Class::write(0x4, 0x20, data23, 1);
        Serial.println("[INFO] RX Calibration was successful.");
    }
    f = DW3000Class::read(0x4, 0x20);
    
    DW3000Class::write(0x4, 0x0C, data23, 1); //Reset antenna calibration to standard mode
    
    int data24[] = {0x00, 0x02, 0xB4};
    DW3000Class::write(0x11, 0x04, data24, 3); 

    int data25[] = { 0x38, 0x07, 0x03, 0x80 };
    DW3000Class::write(0x11, 0x08, data25, 4);
    Serial.println("[INFO] Initialization finished.\n");
}

void DW3000Class::setupGPIO() {
    int data1[] = { 0xF0 };
    write(0x05, 0x08, data1, 1); //Set GPIO0 - GPIO3 as OUTPUT on DW3000
}

void DW3000Class::pullLEDHigh(int led) { //led 0 - 2 possible
    if (led > 7) return;
    led_status = led_status + (1 << led);
    int data1[] = { led_status };
    write(0x05, 0x0C, data1, 1);
}

void DW3000Class::pullLEDLow(int led) { //led 0 - 2 possible
    if (led > 7) return;
    led_status = led_status & ~((int)1 << led); //https://stackoverflow.com/questions/47981/how-to-set-clear-and-toggle-a-single-bit
    int data1[] = { led_status };
    write(0x05, 0x0C, data1, 1);
}

void DW3000Class::writeTXDelay(int delay) {
    int data[] = {
        delay & 0xFF,
        (delay & 0xFF00) >> 8,
        (delay & 0xFF0000) >> 16,
        (delay & 0xFF000000) >> 24,
    };
    write(0x00, 0x2C, data, 4);
    //Serial.print("TX Delay: ");
    //Serial.println(read(0x00, 0x2C));
}

void DW3000Class::delayedTXThenRX() { //Activates Transmission with delay programmed through writeTXDelay() and goes to receiver immediately
    int data[] = { 1, 1, 1, 1 };
    writeShortCommand(data, 4);
}

void DW3000Class::delayedTX() {
    int data[] = { 1, 1 };
    writeShortCommand(data, 2);
}

unsigned long long DW3000Class::readRXTimestamp() {
    unsigned long long ts_low = read(0x0C, 0x00);
    unsigned long long ts_high = read(0x0C, 0x04) & 0xFF;

    unsigned long long rx_timestamp = (ts_high << 32) | ts_low;
    /*Serial.print("RX: ");
    Serial.println(ts_low);
    Serial.println(ts_high); */

    return rx_timestamp;
}


unsigned long long DW3000Class::readTXTimestamp() {
    unsigned long long ts_low = read(0x00, 0x74);
    unsigned long long ts_high = read(0x00, 0x78) & 0xFF;

    unsigned long long tx_timestamp = (ts_high << 32) + ts_low;
    /*Serial.print("TX: ");
    Serial.println(ts_low);
    Serial.println(ts_high);*/

    return tx_timestamp;
}


void DW3000Class::prepareDelayedTX() {
    unsigned long long rx_ts = readRXTimestamp();

    long long exact_tx_timestamp = (long long)(rx_ts + TRANSMIT_DELAY) >> 8;

    long long calc_tx_timestamp = ((rx_ts + TRANSMIT_DELAY) & ~TRANSMIT_DIFF) + antenna_delay;

    int calc_tx_stamp_data[5]; // Size of 5 for 40 Bit Timestamp (40 bit / 8 = 5 Bytes)
    for (int i = 0; i < 5; i++){
        calc_tx_stamp_data[i] = (calc_tx_timestamp >> i * 8) & 0xFF;
    }

    int rx_stamp_data[5];
    for (int i = 0; i < 5; i++){
        rx_stamp_data[i] = (rx_ts >> i * 8) & 0xFF;
    }

    /* FRAME CONTENTS:
    * 0x00 - 0x01: [Reserved] 16 Bit for sender-ID     //TODO add functionality to first 16 bit
    * 0x02 - 0x06: [Used] Calculated Transmit Timestamp
    * 0x07 - 0x0B: [Used] RX Timestamp Time
    */

    write(0x14, 0x02, calc_tx_stamp_data, 5);
    write(0x14, 0x07, rx_stamp_data, 5);

    setFrameLength(12); // (id(16 bits) + tx(40 bits) + rx(40 bits)) / 8 = 12 Bytes

    //Write delay to register 
    writeTXDelay(exact_tx_timestamp);
}


void DW3000Class::calculateTXRXdiff() { //calc diff on PING side
    unsigned long long ping_tx = readTXTimestamp();
    unsigned long long ping_rx = readRXTimestamp();

    long double clk_offset = getClockOffset();
    long double clock_offset = 1.0 + clk_offset;

    /*
    * Get RX and TX times from PONG side
    */

    unsigned int tx_low = read(RX_BUFFER_0_REG, 0x02);
    unsigned int tx_high = read(RX_BUFFER_0_REG, 0x06) & 0xFF;

    long long pong_tx = (long long)tx_high << 32 | tx_low;

    unsigned int rx_low = read(RX_BUFFER_0_REG, 0x07);
    unsigned int rx_high = read(RX_BUFFER_0_REG, 0x0B) & 0xFF;

    long long pong_rx = (long long)rx_high << 32 | rx_low;

    /*
    * Calculate round trip time (see DW3000 User Manual page 248 for more)
    */

    if (pong_tx < pong_rx || ping_rx < ping_tx) { // return if the data would result in incorrect/negative results
        return;
    }

    long long t_reply = pong_tx - pong_rx;

    long long t_round = ping_rx - ping_tx;

    long long t_prop = (t_round - lround(t_reply * clock_offset)) / 2;

    long double t_prop_ps = t_prop * PS_UNIT; 

    long double t_prop_mm = t_prop_ps * SPEED_OF_LIGHT;

    printDouble((t_prop_mm / 1000), 100, false); // second value sets the decimal places. 100 = 2 decimal places, 1000 = 3, 10000 = 4, ...
    Serial.println("cm");
}

/*void DW3000Class::interruptDetect() { //On calling interrupt
    Serial.println("\n\nCALLED INTERRUPT\n\n");

    int irq_reason = getIRQBit();
    switch (irq_reason) {
    case 0:
        Serial.println("Interruption cause: Interruption Cause not given! Check First and Second Sector Bits.");
        break;
    case 1:
        Serial.println("Interruption cause: TX finished");
        break;
    case 2:
        Serial.println("Interruption cause: RX Data ready");
        DW3000Class::rx_rec = true;
        readRXBuffer();
        break;
    case 3:
        Serial.println("Interruption cause: Preamble detected");
        //initiateRX();
        digitalWrite(TX_LED, HIGH);
        digitalWrite(RX_LED, HIGH);
        delay(50);
        digitalWrite(TX_LED, LOW);
        digitalWrite(RX_LED, LOW);
        break;
    case 4:
        Serial.println("Interruption cause: Cmd Error");
        cmd_error = true;
        SPI.endTransaction();
        break;
    default:
        Serial.print("irq_reason got wrong value. Value: ");
        Serial.println(irq_reason);
        break;
    }
    //IRQPulled = true;
    resetIRQStatusBits();
    //initiateRX();
    Serial.println("Finished interrupt. Continuing...");
}*/


void DW3000Class::writeShortCommand(int cmd[], int cmd_len) {
    if (DEBUG_OUTPUT) Serial.print("Short Command WRITE: ");

    int fill_base_len = 5;
    int fill_base[fill_base_len]; //fill leading zeros

    for (int i = 0; i < fill_base_len; i++) {
        if (i < fill_base_len - cmd_len) {
            fill_base[i] = 0;
        }
        else {
            fill_base[i] = cmd[i - (fill_base_len - cmd_len)];
        }
    }
    int cmd_finished[8] = { 1,0 };
    for (int i = 0; i < fill_base_len; i++) {
        cmd_finished[i + 2] = fill_base[i];
    }
    cmd_finished[7] = 1;

    int byteOne = 0;

    for (int i = 7; i >= 0; i--) { //8 bit iteration    
        byteOne = byteOne + cmd_finished[i] * round(pow(2, 7 - i));
    }
    if (DEBUG_OUTPUT) Serial.println(byteOne, HEX);
    int bytes[] = { byteOne };
    sendBytes(bytes, 1, 0);
    if (DEBUG_OUTPUT) Serial.println("Finished writing to Short Command.");
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
    int cmd[] = { 0x01 }; //0x1
    DW3000Class::writeShortCommand(cmd, 1);
}

void DW3000Class::standardRX() {
    int cmd[] = { 1,0 }; //0x2
    DW3000Class::writeShortCommand(cmd, 2);
}

void DW3000Class::TXInstantRX() { // Execute tx, then set receiver to rx instantly
    int cmd[] = { 1, 1, 0, 0 }; //0xC
    DW3000Class::writeShortCommand(cmd, 4);
}

void DW3000Class::clearSystemStatus() {
    int data[] = { 0xFF, 0xFF, 0x7F, 0x3F, 0xF2, 0x1F };
    write(GEN_CFG_AES_LOW_REG, 0x44, data, 4);
}

int DW3000Class::checkForDevID() {
    if (read(GEN_CFG_AES_LOW_REG, NO_OFFSET) != 0xDECA0302) {
        Serial.println("[ERROR] DEV_ID IS WRONG!");
        return 0;
    }
    return 1;
}

void DW3000Class::setFrameLength(int frame_len) { // set Frame length in Bytes
    frame_len = frame_len + FCS_LEN;
    int curr_cfg = read(0x00, 0x24);
    if (frame_len > 1023) {
        Serial.println("[ERROR] Frame length + FCS_LEN (2) is longer than 1023. Aborting!");
        return;
    }
    int tmp_cfg = (curr_cfg & 0xFFFFFC00) | frame_len;
    int data[] = {
        tmp_cfg & 0xFF,
        (tmp_cfg & 0xFF00) >> 8
    };
    int old = read(GEN_CFG_AES_LOW_REG, 0x24);
    write(GEN_CFG_AES_LOW_REG, 0x24, data, 2);
    int h = read(GEN_CFG_AES_LOW_REG, 0x24);
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
    int data[] = { tmpByte };
    write(reg_addr, sub_addr, data, 1);
}

void DW3000Class::setBitHigh(int reg_addr, int sub_addr, int shift) {
    setBit(reg_addr, sub_addr, shift, 1);
}
void DW3000Class::setBitLow(int reg_addr, int sub_addr, int shift) {
    setBit(reg_addr, sub_addr, shift, 0);
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
    int d[] = {
        delay & 0xFF,
        (delay & 0xFF00) >> 8
    };
    antenna_delay = delay;
    write(0x01, 0x04, d, 2);
}

int DW3000Class::getTXAntennaDelay() { //DEPRECATED use antenna_delay variable instead!
    int delay = read(0x01, 0x04) & 0xFFFF;
    return delay;
}

float DW3000Class::getClockOffset() {     
    if (config[0] == CHANNEL_5) {
        return getRawClockOffset() * CLOCK_OFFSET_CHAN_5_CONSTANT / 1000000;
    }
    else {
        return getRawClockOffset() * CLOCK_OFFSET_CHAN_9_CONSTANT / 1000000;
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
        int data1[] = { 0x4 };
        write(0x07, 0x34, data1, 1); //enable temp sensor readings

        int data2[] = { 0x1 };
        write(0x08, 0x00, data2, 1); //enable poll

        while (!(read(0x08, 0x04) & 0x01)) {
        };
        int res = read(0x08, 0x08);
        res = (res & 0xFF00) >> 8;
        int otp_temp = readOTP(0x09) & 0xFF;
        float tmp = (float)((res - otp_temp) * 1.05f) + 22.0f;

        int data3[] = { 0x00 };
        write(0x08, 0x00, data3, 1); //Reset poll enable

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
    Serial.print("."); // print the decimal point
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
