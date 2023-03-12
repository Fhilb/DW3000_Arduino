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
class DW3000Class {
	public:
		static int getAnchorID();
		static uint32_t read(int base, int sub);
		static uint32_t write(int base, int sub, int *data, int data_len);
		static void init();
		static void readInit();
		static void setLED1(uint8_t status);
		static void setLED2(uint8_t status);
		static void interruptDetect();
		static void standardTX();
		static void standardRX();
	private:
		static int* getBase(int hex_num);
		static int* getSub(int hex_num);
		static void writeShortCommand(int cmd[], int cmd_len);
		static uint32_t readOrWriteFullAddress(int *base, int base_len, int *sub, int sub_len, int *data, int data_len, int readWriteBit);
		//static uint32_t readOrWriteFullAddress(int *base, int base_len, int *sub, int sub_len, int *data, int data_len, int readWriteBit, bool quiet);
		static uint32_t sendBytes(int b[], int lenB, int recLen);
		static int* hexToBin(int hex_num, int bit_size);
		static bool is_anchor;
		static int anchor_id;
		static void resetIRQStatusBits();
		static int getIRQBit();
		static int readRXBuffer();
		static void initLEDs();
};
extern DW3000Class DW3000;
#endif