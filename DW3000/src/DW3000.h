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

#ifndef Morse_h
#define Morse_h

#include "Arduino.h"
#include "DW3000Constants.h"


class DW3000Class {
	public:
		static bool rx_rec;
		static int config[9]; 

		/* CONFIG SETTINGS */
		static void setChannel(uint8_t data);
		static void setPreambleLength(uint8_t data);
		static void setPreambleCode(uint8_t data);
		static void setPACSize(uint8_t data);
		static void setDatarate(uint8_t data);
		static void setPHRMode(uint8_t data);
		static void setPHRRate(uint8_t data);

		/* INITS */
		static void initTX_FCTRL();
		static void initAONWakeUp();

		static int getAnchorID();
		static uint32_t read(int base, int sub);
		static uint16_t read16bit(int base, int sub);
		static uint8_t read8bit(int base, int sub);
		static uint32_t write(int base, int sub, int* data, int data_len);
		static void init();
		static void readInit();
		static void setLED1(uint8_t status);
		static void setLED2(uint8_t status);
		static void interruptDetect();
		static void standardTX();
		static void standardRX();
		static void begin();
		static void getMemInfo();
		static bool checkForIDLE();
		static void softReset();
		static uint32_t readOTP(uint16_t addr);
		static void writeSysConfig();
		static void configureAsTX();

	private:
		static void setBit(int reg_addr, int sub_addr, int shift, bool b);

		static void clearAONConfig();

		static void setBitLow(int reg_addr, int sub_addr, int shift);
		static void setBitHigh(int reg_addr, int sub_addr, int shift);

		static int* getBase(int hex_num);
		static int* getSub(int hex_num);

		static void writeShortCommand(int cmd[], int cmd_len);
		static uint32_t readOrWriteFullAddress(int* base, int base_len, int* sub, int sub_len, int* data, int data_len, int readWriteBit);
		static uint32_t sendBytes(int b[], int lenB, int recLen); 
		
		static void resetIRQStatusBits();
		static int getIRQBit();
		static int readRXBuffer();

		static void initLEDs();

		//helper functions 
		static int* hexToBin(int hex_num, int bit_size);

		//private variables
		static bool is_anchor;
		static bool cmd_error;
		static bool leds_init;
		static int anchor_id;

		//config arrays
		static byte rx_cal_conf[];
		static byte tx_fctrl_conf[];
		static byte aon_dig_cfg_conf[];
};
extern DW3000Class DW3000;
#endif