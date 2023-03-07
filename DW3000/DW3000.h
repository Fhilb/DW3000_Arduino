#ifndef Morse_h
#define Morse_h

#include "Arduino.h"
class DW3000
{
	public:
		DW3000(int anchor);
		DW3000();
		int getAnchorID();
		uint32_t read(int base, int sub);
		uint32_t write(int base, int sub, int *data, int data_len);
		void init();
		void readInit();
	private:
		int* getBase(int hex_num);
		int* getSub(int hex_num);
		void writeShortCommand(int cmd[], int cmd_len);
		uint32_t readOrWriteFullAddress(int base[], int base_len, int sub[], int sub_len, int data[], int data_len, int readWriteBit);
		uint32_t sendBytes(int b[], int lenB, int recLen);
		int* hexToBin(int hex_num, int bit_size);
		bool is_anchor;
		int anchor_id;
};
#endif