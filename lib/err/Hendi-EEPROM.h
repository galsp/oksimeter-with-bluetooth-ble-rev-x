#ifndef EEPROMHENDI_h
#define EEPROMHENDI_h

#include "Arduino.h"
#include <EEPROM.h>

void writeStringEEPROM(int add, String data);
String readStringEEPROM(int add);
void eepromWrite(int pos, uint8_t val, int targetSize);
void eepromWriteInt8(int pos, uint8_t val);
uint8_t eepromGetInt8(int pos);
void eepromWriteInt16(int pos, uint16_t val);
uint16_t eepromGetInt16(int pos);
void eepromWriteInt32(int pos, int val);
int eepromGetInt32(int pos);
void eepromClear(int startPos, int endPos);
int readingByte(byte theArray[], int dataLength, int startAddress);
int arrayByteLength(byte intArr[]);


#endif