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
#define TX_LED 3
#define RX_LED 4

/*DW3000Class::DW3000(int _anchor_id) {
    attachInterrupt(digitalPinToInterrupt(2), interruptDetect, RISING);
	anchor_id = _anchor_id;
    is_anchor = true;
}

DW3000Class::DW3000() {
    anchor_id = -1;
    is_anchor = false;
    SPI.begin();
    Serial.begin(9600);
}*/

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

uint32_t DW3000Class::readOrWriteFullAddress(int base[], int base_len, int sub[], int sub_len, int data[], int data_len, int readWriteBit) {
    DW3000Class::readOrWriteFullAddress(base, base_len, sub, sub_len, data, data_len, readWriteBit, false);
}

uint32_t DW3000Class::readOrWriteFullAddress(int base[], int base_len, int sub[], int sub_len, int data[], int data_len, int readWriteBit, bool quiet) {
    if (quiet) Serial.end();
    int fill_base_len = 5;
    int fill_base[fill_base_len]; //fill leading zeros

    for (int i = 0; i < fill_base_len; i++) {
        if (i < fill_base_len - base_len) {
            fill_base[i] = 0;
        }
        else {
            fill_base[i] = base[i - (fill_base_len - base_len)];
        }
    }

    int fill_sub_len = 7;
    int fill_sub[fill_sub_len]; //fill leading zeros  
    for (int i = 0; i < fill_sub_len; i++) {
        if (i < fill_sub_len - sub_len) {
            fill_sub[i] = 0;
        }
        else {
            fill_sub[i] = sub[i - (fill_sub_len - sub_len)];
        }
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
    }
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
    }
    if (quiet) Serial.begin(9600);
    return res;

}

uint32_t DW3000Class::read(int base, int sub) {
    int* _base = DW3000Class::getBase(base);
    int* _sub = DW3000Class::getSub(sub);
    int t[] = {0};
    return readOrWriteFullAddress(_base, 5, _sub, 7, t, 0, 0);
    free(_base);
    free(_sub);
}

uint32_t DW3000Class::write(int base, int sub, int *data, int data_len) {
    int* _base = DW3000Class::getBase(base);
    int* _sub = DW3000Class::getSub(sub);
    return readOrWriteFullAddress(_base, 5, _sub, 7, *data, data_len, 1);
    free(data);
    free(_base);
    free(_sub);
}

void DW3000Class::init() {
    SPI.begin();
    Serial.begin(9600);
    attachInterrupt(digitalPinToInterrupt(2), interruptDetect, RISING);
    int shCmd[] = { 0 };
    writeShortCommand(shCmd, 1);
    Serial.println("\n+++ DecaWave DW3000 Test +++\n");

    int data[] = { 0xFF,0xFF,0xFF,0xFF,0xF2,0x1F }; //0xF0,0x2F //0x80,0x3E,0x0,0x0,0x0,0xF
    DW3000Class::write(0x0, 0x3C, data, 6); //Set IRQ for successful received data frame

    int data2[] = { 0x03, 0x1C };
    DW3000Class::write(0x0, 0x24, data2, 2); //Frame length setup for Transmission  

    int data11[] = { 0x00,0x9 };
    DW3000Class::write(0xA, 0x0, data11, 3); //AON_DIG_CFG register setup; sets up auto-rx calibration and on-wakeup GO2IDLE

    /*
     * Set RX and TX config
     */
    int data15[] = { 0x10,0x00,0x02,0x40 }; //DGC_CFG0
    DW3000Class::write(0x3, 0x1C, data15, 4);

    int data16[] = { 0x1b,0x6d,0xa4,0x89 }; //DGC_CFG1
    DW3000Class::write(0x3, 0x20, data16, 4);

    int data17[] = { 0x00,0x01,0xC0,0xFD }; //DGC_LUT_0
    DW3000Class::write(0x3, 0x38, data17, 4);

    int data18[] = { 0x00,0x01,0xC4,0x3E }; //DGC_LUT_1
    DW3000Class::write(0x3, 0x3C, data18, 4);

    int data19[] = { 0x00,0x01,0xC6,0xBE }; //DGC_LUT_2
    DW3000Class::write(0x3, 0x40, data19, 4);

    int data20[] = { 0x00,0x01,0xC7,0x7E }; //DGC_LUT_3
    DW3000Class::write(0x3, 0x44, data20, 4);

    int data21[] = { 0x00,0x01,0xCF,0x36 }; //DGC_LUT_4
    DW3000Class::write(0x3, 0x48, data21, 4);

    int data22[] = { 0x00,0x01,0xCF,0xB5 }; //DGC_LUT_5
    DW3000Class::write(0x3, 0x4C, data22, 4);

    int data23[] = { 0x00,0x01,0xCF,0xF5 }; //DGC_LUT_6
    DW3000Class::write(0x3, 0x50, data23, 4);

    //SET PAC TO 32 (0x00) reg:06:00 bits:1-0, bit 4 to 0 (00001100) (0xC)
    int data25[] = { 0xE };
    DW3000Class::write(0x6, 0x0, data25, 1);

    //SET PREAMBLE CODE (RX_PCODE, TX_PCODE) TO 10 (reg:01:14) //Standard SFD Type is 11 (data: 0x56, 0x5), trying 00 (0x50, 0x5)
    int data26[] = { 0xBE, 0x3 };
    DW3000Class::write(0x1, 0x14, data26, 2);

    // write preamble length, frame length, data rate and prf in TX_FCTRL  //PSR = 1024, TXFLEN = 3 Byte (1 data, 2 CRC) TXBR = 6.81Mb/s, TR Bit enabled, FINE_PLEN = 0x0
    // reg:00:24 bits 0 - 25
    int data27[] = { 0x03, 0x2C };
    DW3000Class::write(0x0, 0x24, data27, 2);

    //set SFD Detection timeout count to 1057 (0x21, 0x4); 1018 old: (0xFA, 0x3)
    int data28[] = { 0x21, 0x4 };
    DW3000Class::write(0x6, 0x2, data28, 2);



    //resetIRQStatusBits();


    /*
     * Things to do documented in https://gist.github.com/egnor/455d510e11c22deafdec14b09da5bf54
     */
    int data3[] = { 0xF5,0xE4 };
    DW3000Class::write(0x3, 0x18, data3, 2); //THR_64 value set to 0x32

    int data5[] = { 0x2 };
    DW3000Class::write(0x4, 0xC, data5, 1); //COMP_DLY to 0x2

    int data6[] = { 0x14 };
    DW3000Class::write(0x7, 0x48, data6, 1); //LDO_RLOAD to 0x14

    int data7[] = { 0x0E };
    DW3000Class::write(0x7, 0x1A, data7, 1); //RF_TX_CTRL_1 to 0x0E

    int data8[] = { 0x34,0x11,0x07,0x1C };
    DW3000Class::write(0x7, 0x1C, data8, 4); //RF_TX_CTRL_2 to 0x1C071134 (due to channel 5, else (9) to 0x1C010034)

    int data9[] = { 0x3C,0x1F };
    DW3000Class::write(0x9, 0x0, data9, 2); //PLL_CFG to 0x1F3C (due to channel 5, else (9) to 0x0F3C)

    int data10[] = { 0x21 };
    DW3000Class::write(0x9, 0x8, data10, 1); //PLL_CFG_LD to 0x8 (Documenation says 0x81, doesn't fit the 6bit register tho)

    int data12[] = { 0x0, 0x8, 0x1 };
    DW3000Class::write(0xA, 0x0, data12, 3); //Auto antenna calibration on startup enable (ONW_PGFCAL)

    int data13[] = { 0x11 };
    int data14[] = { 0x0 }; //if finished with calibration go back in cal_mode

    delay(50);
    for (int i = 0; i < 5; i++) {
        uint32_t h = DW3000Class::read(0x4, 0x20); //Read antenna calibration //RX_CAL_STS => Status bit, if high antenna cal was successful
        if (h > 0) {
            break;
        }
        int data29[] = { 0x5, 0x10 };
        DW3000Class::write(0x7, 0x48, data29, 2); //Old values in register: 10000000000000011100
        int data30[] = { 0x1 };
        DW3000Class::write(0x4, 0xE, data30, 1);
        if (i < 4) {
            Serial.println("Failed auto calibration of antenna. Trying again in 50ms.");
            DW3000Class::write(0x4, 0xC, data13, 1);
            delay(50);
        }
        else {
            Serial.println("[ERROR] Antenna auto calibration failed too often. Stopping to calibrate now.");
        }
        int data31[] = {0x1C};
        write(0x7, 0x48, data31, 1);
    }
    DW3000Class::write(0x4, 0xC, data14, 1); //reset to normal operation

    //resetIRQStatusBits();
    /*
     * Things to do documented in https://gist.github.com/egnor/455d510e11c22deafdec14b09da5bf54
     */


}

void DW3000Class::readInit() {
    int shCmd[] = { 0 };
    writeShortCommand(shCmd, 1);
    Serial.println("\n+++ DecaWave DW3000 Test +++\n");
    Serial.println("IRQ:");
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
        digitalWrite(TX_LED, HIGH);
        delay(100);
        digitalWrite(TX_LED, LOW);
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
    if (status > 0) {
        digitalWrite(TX_LED, HIGH);
    }
    else {
        digitalWrite(TX_LED, LOW);
    }
}

void DW3000Class::setLED2(uint8_t status) {
    if (status > 0) {
        digitalWrite(RX_LED, HIGH);
    }
    else {
        digitalWrite(RX_LED, LOW);
    }
}