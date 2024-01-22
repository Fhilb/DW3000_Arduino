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
		static int config[9]; 

		static void spiSelect(uint8_t cs);

		static void begin();
		static void init();

		static void writeSysConfig();
		static void configureAsTX();

		static void setChannel(uint8_t data);
		static void setPreambleLength(uint8_t data);
		static void setPreambleCode(uint8_t data);
		static void setPACSize(uint8_t data);
		static void setDatarate(uint8_t data);
		static void setPHRMode(uint8_t data);
		static void setPHRRate(uint8_t data);
		static void setTXFrame(int frame_data);
		static void setFrameLength(int frame_len);

		static int receivedFrameSucc();
		static int sentFrameSucc();

		static int getAnchorID();
		static void printFullConfig();
		static int readRXBuffer();
		static bool checkForIDLE();
		static int checkForDevID();

		static uint32_t read(int base, int sub);
		static uint8_t read8bit(int base, int sub);
		static uint32_t readOTP(uint16_t addr);

		static uint32_t write(int base, int sub, int* data, int data_len);

		static void standardTX();
		static void standardRX();
		static void softReset();
		static void clearSystemStatus();
		//static void interruptDetect();


	private:
		static void clearAONConfig();

		static void setBit(int reg_addr, int sub_addr, int shift, bool b);
		static void setBitLow(int reg_addr, int sub_addr, int shift);
		static void setBitHigh(int reg_addr, int sub_addr, int shift);

		static int* getBase(int hex_num);
		static int* getSub(int hex_num);

		static void writeShortCommand(int cmd[], int cmd_len);
		static uint32_t readOrWriteFullAddress(int* base, int base_len, int* sub, int sub_len, int* data, int data_len, int readWriteBit);
		static uint32_t sendBytes(int b[], int lenB, int recLen); 
		
		static int* hexToBin(int hex_num, int bit_size);

		static bool is_anchor;
		static bool cmd_error;
		static int anchor_id;
};

extern DW3000Class DW3000;
#endif