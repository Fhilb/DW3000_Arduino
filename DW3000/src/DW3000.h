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
		static void setupGPIO();

		static void setChannel(uint8_t data);
		static void setPreambleLength(uint8_t data);
		static void setPreambleCode(uint8_t data);
		static void setPACSize(uint8_t data);
		static void setDatarate(uint8_t data);
		static void setPHRMode(uint8_t data);
		static void setPHRRate(uint8_t data);
		static void setTXFrame(unsigned long long frame_data);
		static void setFrameLength(int frame_len);
		static void setTXAntennaDelay(int delay);

		static int receivedFrameSucc();
		static int sentFrameSucc();

		static int getAnchorID();
		static int getTXAntennaDelay();
		static float getClockOffset();
		static int getRawClockOffset();
		static float getTempInC();
		static void printFullConfig();
		static void printRoundTripInformation();
		static unsigned long long readRXBuffer();
		static bool checkForIDLE();
		static int checkForDevID();

		static uint32_t read(int base, int sub);
		static uint8_t read8bit(int base, int sub);
		static uint32_t readOTP(uint8_t addr);
		static unsigned long long readRXTimestamp();
		static unsigned long long readTXTimestamp();
		static void calculateTXRXdiff();

		static uint32_t write(int base, int sub, int data, int data_len);
		static uint32_t write(int base, int sub, int data);

		static void writeTXDelay(int delay);
		static void delayedTXThenRX();
		static void prepareDelayedTX();
		static void delayedTX();

		static void standardTX();
		static void standardRX();
		static void TXInstantRX();
		static void softReset();
		static void hardReset();
		static void clearSystemStatus();
		static void pullLEDHigh(int led);
		static void pullLEDLow(int led);
		//static void interruptDetect();


	private:
		static void clearAONConfig();

		static void setBit(int reg_addr, int sub_addr, int shift, bool b);
		static void setBitLow(int reg_addr, int sub_addr, int shift);
		static void setBitHigh(int reg_addr, int sub_addr, int shift);

		static void writeFastCommand(int cmd);

		static uint32_t readOrWriteFullAddress(uint32_t base, uint32_t sub, uint32_t data, uint32_t data_len, uint32_t readWriteBit);

		static uint32_t sendBytes(int b[], int lenB, int recLen); 
		
		static unsigned int countBits(unsigned int number);

		static bool is_anchor;
		static bool cmd_error;
		static int anchor_id;
		static void printDouble(double val, unsigned int precision, bool linebreak);
};

extern DW3000Class DW3000;
#endif