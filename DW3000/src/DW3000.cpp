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

#define CHIP_SELECT_PIN 10
#define TX_LED 3 //RED
#define RX_LED 4 //GREEN

int no_data[] = { 0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0}; //32 bit array for clearing zeroes

int DW3000Class::config[] = { //CHAN; PREAMBLE LENGTH; PREAMBLE CODE; PAC; DATARATE; PHR_MODE; PHR_RATE;
    CHANNEL_5,
    PREAMBLE_128,
    9, //same for tx and rx
    PAC8,
    DATARATE_6_8MB,
    PHR_MODE_STANDARD,
    PHR_RATE_6_8MB
};

bool DW3000Class::leds_init = false;
bool DW3000Class::cmd_error = false;
bool DW3000Class::rx_rec = false;

void DW3000Class::begin() {
    delay(5);
    pinMode(CHIP_SELECT_PIN, OUTPUT);
    SPI.begin();

    attachInterrupt(digitalPinToInterrupt(2), DW3000Class::interruptDetect, RISING);
}

void DW3000Class::writeSysConfig() {
    int usr_cfg = (STDRD_SYS_CONFIG & 0xFF) | (config[5] << 3) | (config[6] << 4);
    int usr_data[] = { usr_cfg & 0xFF, usr_cfg & 0xF00 };
    write(GEN_CFG_AES_LOW_REG, 0x10, usr_data, 1); //write user config

    Serial.print("SYS_CFG looks as follows: ");
    Serial.println(read(0x0, 0x10), BIN);

    if (config[2] > 24) {
        Serial.println("[ERROR] SCP ERROR! TX & RX Preamble Code higher than 24!");
    }

    int otp_write[] = config[1] >= 256 ? { 0x00, 0x04 } : { 0x00, 0x14 };

    write(OTP_IF_REG, 0x08, otp_write, 2); //set OTP config

    write(DRX_REG, 0x00, 0x00, 1); //reset DTUNE0_CONFIG

    int usr_dtune0_cfg[] = { config[3] };
    write(DRX_REG, 0x0, usr_dtune0_cfg, 1);

    //64 = STS length
    int sts_cfg[] = { 64 / 8 - 1};
    write(STS_CFG_REG, 0x0, sts_cfg, 1);

    write(GEN_CFG_AES_LOW_REG, 0x29, no_data, 1);

    int dtune3_val[] = { 0xCC, 0x35, 0x5F, 0xAF }; //TODO if not working: change value back (p.147)
    write(0x06, 0x0C, dtune3_val, 4);

    chan_ctrl_val = read(GEN_CFG_AES_HIGH_REG, 0x14);  //Fetch and adjust CHAN_CTRL data
    chan_ctrl_val &= (~0x1FFF);

    chan_ctrl_val |= config[0]; //Write RF_CHAN

    chan_ctrl_val |= 0x1F00 & (config[2] << 8);
    chan_ctrl_val |= 0xF8 & (config[2] << 3);
    chan_ctrl_val |= 0x06 & (0x0 << 1);

    int chan_ctrl_data[] = {
        (chan_ctrl_val & 0xFF),
        (chan_ctrl_val & 0xFF00) >> 8,
        (chan_ctrl_val & 0xFF0000) >> 16,
        (chan_ctrl_val & 0xFF000000) >> 24
    };

    write(GEN_CFG_AES_HIGH_REG, 0x14, chan_ctrl_data, 4);  //Write new CHAN_CTRL data with updated values

    int tx_fctrl_val = read(GEN_CFG_AES_LOW_REG, 0x24); 

    tx_fctrl_val |= (0x3FF & config[1]); //Add preamble length
    tx_fctrl_val |= (0x400 & config[4]); //Add data rate

    int tx_fctrl_data[] = {
        (tx_fctrl_val & 0xFF),
        (tx_fctrl_val & 0xFF00) >> 8,
        (tx_fctrl_val & 0xFF0000) >> 16,
        (tx_fctrl_val & 0xFF000000) >> 24
    };

    write(GEN_CFG_AES_LOW_REG, 0x24, tx_fctrl_data, 4);

    int drx_data[] = { 0x81 };
    write(DRX_REG, 0x02, drx_data, 1);

    if (config[0]) {
        int rf_tx_ctrl_2_data[] = { 0x34, 0x00, 0x01, 0x1C };
        int pll_conf[] = { 0x3C, 0x1F };
    }
    else {
        int rf_tx_ctrl_2_data[] = { 0x34, 0x11, 0x07, 0x1C };
        int pll_conf[] = { 0x3C, 0x0F };
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

    int ainit2idle_val[] = { 0x00, 0x01 };
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

    int dgc_cfg[] = { 0x64 };
    write(RX_TUNE_REG, 0x19, dgc_cfg, 1);

    int ldo_ctrl_val = read(RF_CONF_REG, 0x48); //Save original LDO_CTRL data
    int tmp_ldo = (0x105
        0x100 |
        0x4 |
        0x1);
    int tmp_ldo_data[] = {
        (tmp_ldo & 0xFF),
        (tmp_ldo & 0xFF00) >> 8
    };
    write(RF_CONF_REG, 0x48, tmp_ldo_data, 2); 

    int rx_cal_data[] = { 0x01, 0x00, 0x02 }; 
    write(EXT_SYNC_REG, 0x0C, rx_cal_data, 3); //Calibrate RX

    delay(20);

    int rx_cal_data2[] = { 0x10 | 0x01}; 
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
    if (rx_cal_res & 0x1fffffff) {
        Serial.println("[ERROR] PGF_CAL failed in stage I!");
    }
    rx_cal_res = read(EXT_SYNC_REG, 0x1C);
    if (rx_cal_res & 0x1fffffff) {
        Serial.println("[ERROR] PGF_CAL failed in stage Q!");
    }

    int ldo_rload_data[] = {
        (ldo_ctrl_val & 0xFF),
        (ldo_ctrl_val & 0xFF00) >> 8,
        (ldo_ctrl_val & 0xFF0000) >> 16,
        (ldo_ctrl_val & 0xFF000000) >> 24
    };
    write(RF_CONF_REG, 0x48, ldo_rload_data, 4); //Restore original LDO_CTRL data

    
}

void DW3000Class::configureAsTX() {
    int pg_delay_data[] = { 0x34 };
    write(RF_CONF_REG, 0x1C, pg_delay_data, 1); //write pg_delay
    int power_data[] = { 0xFD, 0xFD, 0xFD, 0xFD };
    write(GEN_CFG_AES_HIGH_REG, 0x0C, power_data, 4);
}


void DW3000Class::setupTXSettings(int *frame_data, int frame_length) {
    write(TX_BUFFER_REG, 0x00, frame_data, frame_length);
    int tx_fctrl_val = read(GEN_CFG_AES_LOW_REG, 0x24);
    tx_fctrl_val &= 0xFC00; //Mask just the values that would be overwritten by a 2 byte write
    tx_fctrl_val |= frame_length;

    int tx_fctrl_data[] = {
        (tx_fctrl_val & 0xFF),
        (tx_fctrl_val & 0xFF00) >> 8
    }
    write(GEN_CFG_AES_LOW_REG, 0x24, tx_fctrl_data, 2);
}

//Test for memory overflow
void DW3000Class::getMemInfo()
{
    extern unsigned int __data_start;
    extern unsigned int __data_end;
    extern unsigned int __bss_start;
    extern unsigned int __bss_end;
    extern unsigned int __heap_start;
    extern void* __brkval;

    Serial.println("--------------------------------------------");

    // ---------------- Print memory profile -----------------
    int16_t ramSize = 0;   // total amount of ram available for partitioning
    int16_t dataSize = 0;  // partition size for .data section
    int16_t bssSize = 0;   // partition size for .bss section
    int16_t heapSize = 0;  // partition size for current snapshot of the heap section
    int16_t stackSize = 0; // partition size for current snapshot of the stack section
    int16_t freeMem1 = 0;  // available ram calculation #1
    int16_t freeMem2 = 0;  // available ram calculation #2
    // summaries:
    ramSize = (int)RAMEND - (int)&__data_start;
    dataSize = (int)&__data_end - (int)&__data_start;
    bssSize = (int)&__bss_end - (int)&__bss_start;
    heapSize = (int)__brkval - (int)&__heap_start;
    stackSize = (int)RAMEND - (int)SP;
    freeMem1 = (int)SP - (int)__brkval;
    freeMem2 = ramSize - stackSize - heapSize - bssSize - dataSize;
    Serial.println("----- GET MEMORY INFORMATIONS -----");
    Serial.println("");
    Serial.println("--- max. available memory by hardware ---");
    Serial.print("ram total  = "); Serial.print(ramSize, DEC); Serial.println(" bytes");
    Serial.println("--- DATA ---");
    Serial.print("data_start = 0x"); Serial.print((int)&__data_start, HEX); Serial.print(" / "); Serial.println((int)&__data_start, DEC);
    Serial.print("data_end   = 0x"); Serial.print((int)&__data_end, HEX); Serial.print(" / "); Serial.println((int)&__data_end, DEC);
    Serial.print("data size  = "); Serial.println(dataSize, DEC);
    Serial.println("--- BSS ---");
    Serial.print("bss_start  = 0x"); Serial.print((int)&__bss_start, HEX); Serial.print(" / "); Serial.println((int)&__bss_start, DEC);
    Serial.print("bss_end    = 0x"); Serial.print((int)&__bss_end, HEX); Serial.print(" / "); Serial.println((int)&__bss_end, DEC);
    Serial.print("bss size   = "); Serial.println(bssSize, DEC);
    Serial.println("--- HEAP ---");
    Serial.print("heap_start =0x"); Serial.print((int)&__heap_start, HEX); Serial.print(" / "); Serial.println((int)&__heap_start, DEC);
    Serial.println("heap_margin = ? ? ?");
    //  Serial.print("\n__malloc_margin=[0x"); Serial.print( (int) &__malloc_margin, HEX ); Serial.print("] which is ["); Serial.print( (int) &__malloc_margin, DEC); Serial.print("] bytes decimal");
    Serial.print("heap end    = 0x"); Serial.print((int)__brkval, HEX); Serial.print(" / "); Serial.println((int)__brkval, DEC);
    Serial.print("heap size   = "); Serial.println(heapSize, DEC);
    Serial.println("--- STACK ---");
    Serial.print("stack start = 0x"); Serial.print((int)SP, HEX); Serial.print(" / "); Serial.println((int)SP, DEC);
    Serial.print("stack end   = 0x"); Serial.print((int)RAMEND, HEX); Serial.print(" / "); Serial.println((int)RAMEND, DEC);
    Serial.print("stack size= "); Serial.println(stackSize, DEC);
    Serial.println("--- FREE MEMORY ---");
    Serial.print("free size1 = "); Serial.println(freeMem1, DEC);
    Serial.print("free size2 = "); Serial.println(freeMem2, DEC);
    // --- free_memory ---
    Serial.println("");
    Serial.println("--- calculated with free_memory:");
    int free_memory;
    if ((int)__brkval == 0)
        free_memory = ((int)&free_memory) - ((int)&__bss_end);
    else
        free_memory = ((int)&free_memory) - ((int)__brkval);
    Serial.print("free size3 = "); Serial.println(free_memory);

    // --- check-memory ---
    Serial.println("");
    Serial.println("--- calculated with check_memory:");
    uint8_t* heapptr, * stackptr;
    uint16_t diff = 0;
    stackptr = (uint8_t*)malloc(4);          // use stackptr temporarily
    heapptr = stackptr;                     // save value of heap pointer
    free(stackptr);      // free up the memory again (sets stackptr to 0)
    stackptr = (uint8_t*)(SP);           // save value of stack pointer
    Serial.print("heap strat = 0x"); Serial.print((int)heapptr, HEX); Serial.print(" / "); Serial.println((int)heapptr, DEC);
    Serial.print("stackptr   = 0x"); Serial.print((int)stackptr, HEX); Serial.print(" / "); Serial.println((int)stackptr, DEC);
    diff = stackptr - heapptr;
    Serial.print("free size4 = "); Serial.println((int)diff, DEC);
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
    return read(GEN_CFG_AES_LOW_REG, 0x44) >> 16 == (SPIRDY_MASK | RCINIT_MASK) ? 1 : 0;
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
        Serial.print("Reading from ");
        for (int i = 0; i < fill_base_len; i++) {
            Serial.print(fill_base[i]);
        }
        Serial.print(":");
        for (int i = 0; i < fill_sub_len; i++) {
            Serial.print(fill_sub[i]);
        }
        Serial.println("");
        res = (uint32_t)sendBytes(bytes, 2 + data_len, 4);
        Serial.print("Received result (HEX): ");
        Serial.print(res, HEX);
        Serial.print(" (BIN): ");
        Serial.println(res, BIN);
        return res;
    } .
    else {
        Serial.print("Writing to ");
        for (int i = 0; i < fill_base_len; i++) {
            Serial.print(fill_base[i]);
        }
        Serial.print(":");
        for (int i = 0; i < fill_sub_len; i++) {
            Serial.print(fill_sub[i]);
        }
        Serial.println("");
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
    Serial.println("");
    free(_base);
    free(_sub);

    return tmp;
}

uint16_t DW3000Class::read16bit(int base, int sub) {
    return (uint16_t)(read(base, sub) >> 16);
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

uint32_t DW3000Class::readOTP(uint16_t addr) {
    int manual_acc[] = { 0x01, 0x00 };
    write(OTP_IF_REG, 0x08, manual_acc, 2);
    
    int otp_addr[2];
    otp_addr[0] = { (uint8_t)(addr >> 16) };
    otp_addr[1] = { (uint8_t)(addr & 0x00FF) };
    write(OTP_IF_REG, 0x04, otp_addr, 2);

    int read_strobe[] = { 0x02, 0x00 };
    write(OTP_IF_REG, 0x08, read_strobe, 2);

    return read(OTP_IF_REG, 0x10);
}


void DW3000Class::init() {
    Serial.println("\n+++ DecaWave DW3000 Test +++\n");

    if (read(GEN_CFG_AES_LOW_REG, NO_OFFSET) != 0xDECA0302) {
        Serial.println("[ERROR] DEV_ID IS WRONG!");
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

    if (ldo_low != 0 && ldo_high != 0 && (bias_tune >> 16 & BIAS_CTRL_BIAS_MASK) != 0) {
        Serial.println("[ERROR] LDO or BIAS_TUNE not programmed! Aborting!");
        return;
    }

    int xtrim_value[1];
    xtrim_value[0] = readOTP(0x1E);

    xtrim_value[0] = (xtrim_value[0] == 0 ? 0x7F : xtrim_value[0]);

    write(FS_CTRL_REG, 0x14, xtrim_value, 1);


    writeSysConfig();

    int data1[] = {0x80,0xEB,0x7,0x0,0x0,0x1F}; //0xF0,0x2F //0x80,0x3E,0x0,0x0,0x0,0x1F  //0x0
    DW3000Class::write(0x0, 0x3C, data1, 6); //Set IRQ for successful received data frame
  
    int data2[] = { 0x03, 0x3C };
    DW3000Class::write(0x0, 0x24, data2, 2); //Frame length setup for Transmission  

    int data3[] = {0x00,0x9, 0x0};
    DW3000Class::write(0xA, 0x0, data3, 3); //AON_DIG_CFG register setup; sets up auto-rx calibration and on-wakeup GO2IDLE  //0xA

    /*
     * Set RX and TX config
     */
    int data4[] = { 0x40,0x02,0x00,0x10 }; //DGC_CFG0  //0x3
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

    int data13[] = { 0xF5,0xE4 };
    DW3000Class::write(0x3, 0x18, data13, 2); //THR_64 value set to 0x32

    //SET PAC TO 32 (0x00) reg:06:00 bits:1-0, bit 4 to 0 (00001100) (0xC)
    int data14[] = { 0xE };
    DW3000Class::write(0x6, 0x0, data14, 1);  //0x6

    //set SFD Detection timeout count to 1057 (0x21, 0x4); 1018 old: (0xFA, 0x3)
    int data15[] = { 0x21, 0x4 };
    DW3000Class::write(0x6, 0x2, data15, 2);

    //SET PREAMBLE CODE (RX_PCODE, TX_PCODE) TO 10 (reg:01:14) //Standard SFD Type is 11 (data: 0x56, 0x5), trying 00 (0x50, 0x5)
    int data16[] = { 0x56, 0x5 };
    DW3000Class::write(0x1, 0x14, data16, 2);  //0x1

    // write preamble length, frame length, data rate and prf in TX_FCTRL  //PSR = 1024, TXFLEN = 3 Byte (1 data, 2 CRC) TXBR = 6.81Mb/s, TR Bit enabled, FINE_PLEN = 0x0
    // reg:00:24 bits 0 - 25
    /*int data27[] = {0x03, 0x2C};
    DW3000Class::write(0x0, 0x24, data27, 2);*/





    /*
     * Things to do as documented in https://gist.github.com/egnor/455d510e11c22deafdec14b09da5bf54
     */


    int data17[] = { 0x14 };
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
    for (int i = 0; i < 5; i++) {
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
    }
    DW3000Class::write(0x4, 0x0C, data23, 1); //Reset antenna calibration to standard mode

    DW3000Class::resetIRQStatusBits();

    Serial.println("\nInitialization finished.\n");

}

void DW3000Class::readInit() {
    Serial.println("\nIRQ:");
    DW3000Class::read(0x0, 0x3C); //Set IRQ for successful received data frame
    Serial.println("Frame length:");
    DW3000Class::read(0x0, 0x24); //Frame length setup for Transmission  
    Serial.println("AON_DIG_CFG Register:");
    DW3000Class::read(0xA, 0x0); //AON_DIG_CFG register setup; sets up auto-rx calibration and on-wakeup GO2IDLE

    /*
     * Set RX and TX config
     */
    Serial.println("\nRX AND TX CONFIG\n");
    Serial.println("DGC_CFG0:");
    DW3000Class::read(0x3, 0x1C);//DGC_CFG0
    Serial.println("DGC_CFG1:");
    DW3000Class::read(0x3, 0x20);//DGC_CFG1
    Serial.println("DGC_LUT_0:");
    DW3000Class::read(0x3, 0x38);//DGC_LUT_0
    Serial.println("DGC_LUT_1:");
    DW3000Class::read(0x3, 0x3C);//DGC_LUT_1
    Serial.println("DGC_LUT_2:");
    DW3000Class::read(0x3, 0x40);//DGC_LUT_2
    Serial.println("DGC_LUT_3:");
    DW3000Class::read(0x3, 0x44);//DGC_LUT_3
    Serial.println("DGC_LUT_4:");
    DW3000Class::read(0x3, 0x48);//DGC_LUT_4
    Serial.println("DGC_LUT_5:");
    DW3000Class::read(0x3, 0x4C);//DGC_LUT_5
    Serial.println("DGC_LUT_6:");
    DW3000Class::read(0x3, 0x50);//DGC_LUT_6
    Serial.println("PAC:");
    //SET PAC TO 32 (0x00) reg:06:00 bits:1-0, bit 4 to 0 (00001100) (0xC)
    DW3000Class::read(0x6, 0x0);
    Serial.println("Preamble Code:");
    //SET PREAMBLE CODE (RX_PCODE, TX_PCODE) TO 10 (reg:01:14) //Standard SFD Type is 11 (data: 0x56, 0x5), trying 00 (0x50, 0x5)
    DW3000Class::read(0x1, 0x14);
    Serial.println("Preamble length:");
    // read preamble length, frame length, data rate and prf in TX_FCTRL  //PSR = 1024, TXFLEN = 3 Byte (1 data, 2 CRC) TXBR = 6.81Mb/s, TR Bit enabled, FINE_PLEN = 0x0
    // reg:00:24 bits 0 - 25
    DW3000Class::read(0x0, 0x24);
    Serial.println("SFD Detection timout count:");
    //set SFD Detection timeout count to 1057 (0x21, 0x4); 1018 old: (0xFA, 0x3)
    DW3000Class::read(0x6, 0x2);



    //resetIRQStatusBits();


    /*
     * Things to do documented in https://gist.github.com/egnor/455d510e11c22deafdec14b09da5bf54
     */
    Serial.println("\nUNDOCUMENTED VALUES\n");
    Serial.println("THR_64:");
    DW3000Class::read(0x3, 0x18); //THR_64 value set to 0x32
    Serial.println("COMP_DLY:");
    DW3000Class::read(0x4, 0xC); //COMP_DLY to 0x2
    Serial.println("LDO_RLOAD:");
    DW3000Class::read(0x7, 0x48); //LDO_RLOAD to 0x14
    Serial.println("RF_TX_CTRL_1:");
    DW3000Class::read(0x7, 0x1A); //RF_TX_CTRL_1 to 0x0E
    Serial.println("RF_TX_CTRL_2:");
    DW3000Class::read(0x7, 0x1C); //RF_TX_CTRL_2 to 0x1C071134 (due to channel 5, else (9) to 0x1C010034)
    Serial.println("PLL_CFG:");
    DW3000Class::read(0x9, 0x0); //PLL_CFG to 0x1F3C (due to channel 5, else (9) to 0x0F3C)
    Serial.println("PFF_CFG_LD:");
    DW3000Class::read(0x9, 0x8); //PLL_CFG_LD to 0x8 (Documenation says 0x81, doesn't fit the 6bit register tho)
    Serial.println("Auto antenna config enable:");
    DW3000Class::read(0xA, 0x0); //Auto antenna calibration on startup enable (ONW_PGFCAL)
    Serial.println("antenna calibration result:");
    uint32_t h = DW3000Class::read(0x4, 0x20); //Read antenna calibration //RX_CAL_STS => Status bit, if high antenna cal was successful

}

void DW3000Class::interruptDetect() { //On calling interrupt
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
}

void DW3000Class::resetIRQStatusBits() { //clear event status by writing 1 to it
    int data[] = { 0xFF, 0xFF, 0xFF, 0xFF }; //overwrite all 4 octets with 1
    int data2[] = { 0xFF, 0x1F }; //overwrite first 3 octets with 1 + first bit in octet 4
    write(0x0, 0x44, data, 4); //clear status register (octets 0 to 3)
    write(0x0, 0x48, data2, 2); //clear status register (octets 4 and 5)
}

void DW3000Class::writeShortCommand(int cmd[], int cmd_len) {
    Serial.print("Short Command WRITE: ");

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
    Serial.println(byteOne, HEX);
    int bytes[] = { byteOne };
    sendBytes(bytes, 1, 0);
    Serial.println("Finished writing to Short Command.");
}

int DW3000Class::getIRQBit() { //return values: 1=tx finish; 2=rx finish; 3=cmd error; 0=none
    int irq_status = 0;
    int mask_tx_finish_or_cmd_err = 1 << 8;
    int mask_rx_finish = 1 << 13;

    Serial.println("Getting IRQ Bit");

    uint32_t firstSectorBit = read(0x0, 0x44); //read first 4 octet status registers
    uint32_t secondSectorBit = read(0x0, 0x48); //read  octet 5 and 6 status registers (index 4 + 5)

    Serial.print("First Sector Bits: ");
    Serial.println(firstSectorBit, BIN);
    int txStatus = (firstSectorBit >> 7) & 1; //good packet send status
    int rxStatus = (firstSectorBit >> 13) & 1; //good receive status
    int prDetStatus = (firstSectorBit >> 8) & 1; //preamble detect status
    int cmdErrStatus = (secondSectorBit >> 8) & 1; //cmd error status
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

    if (txStatus) return 1;
    if (rxStatus) return 2;
    if (prDetStatus) return 3;
    if (cmdErrStatus) return 4;
    return 0;
}

int DW3000Class::readRXBuffer() {
    Serial.print("Reading RX Buffer0... ");
    uint32_t buf0 = read(0x12, 0x0);
    Serial.println(buf0);
}

void DW3000Class::setLED1(uint8_t status) {
    if (!leds_init) initLEDs();
    if (status > 0) {
        Serial.println("Writing to led high");
        digitalWrite(TX_LED, HIGH);
    }
    else {
        Serial.println("Writing to led low");
        digitalWrite(TX_LED, LOW);
    }
}

void DW3000Class::setLED2(uint8_t status) {
    if (!leds_init) initLEDs();
    if (status > 0) {
        Serial.println("Writing to led high");
        digitalWrite(RX_LED, HIGH);
    }
    else {
        Serial.println("Writing to led low");
        digitalWrite(RX_LED, LOW);
    }
}

void DW3000Class::initLEDs() {
    pinMode(RX_LED, OUTPUT);
    pinMode(TX_LED, OUTPUT);
    leds_init = true;
}

void DW3000Class::standardTX() {
    if (cmd_error) {
        cmd_error = false;
        int cmd[] = { 0 };
        writeShortCommand(cmd, 1);
        delay(10);
    }
    int cmd[] = { 1 };
    DW3000Class::writeShortCommand(cmd, 1);

    DW3000Class::setLED1(HIGH);
    DW3000Class::setLED2(HIGH);
    delay(100);
    DW3000Class::setLED1(LOW);
    DW3000Class::setLED2(LOW);
}

void DW3000Class::standardRX() {
    int cmd[] = { 1,0 };
    DW3000Class::writeShortCommand(cmd, 2);
}

/*
* Set bit in a defined register address
*/
void DW3000Class::setBit(int reg_addr, int sub_addr, int shift, bool b) {
    int* tmpByte = read8bit(reg_addr, sub_addr);
    if (b) {
        bitSet(*tmpByte, shift);
    }
    else {
        bitClear(*tmpByte, shift);
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

void DW3000Class::initTX_FCTRL() {
    setBitHigh(tx_fctrl_conf, 10);
    setBitHigh(tx_fctrl_conf, 11);
    setBitHigh(tx_fctrl_conf, 12);
}

void DW3000Class::initAONWakeUp() {
    setBitHigh(aon_dig_cfg_conf, 8);
    setBitHigh(aon_dig_cfg_conf, 11);
    //0x00, 0x9, 0x0
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
