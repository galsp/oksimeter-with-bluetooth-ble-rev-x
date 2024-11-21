/*
 * EEPROMHENDI.cpp - Library for managing number to eeprom.
 */

#include <EEPROM.h>
#include "Hendi-EEPROM.h"

void writeStringEEPROM(int add, String data){
  int _size = data.length();
  int _i;
  for (_i = 0; _i < _size; _i++)
  {
    EEPROM.write(add + _i, data[_i]);
  }
  EEPROM.write(add + _size, '\0'); //Add termination null character for String Data
  EEPROM.commit();
}

String readStringEEPROM(int add){
  char _data[100]; //Max 100 Bytes
  int _len = 0;
  unsigned char _k;
  _k = EEPROM.read(add);
  while (_k != '\0' && _len < 100){  //Read until null character
    _k = EEPROM.read(add + _len);
    _data[_len] = _k;
    _len++;
  }
  _data[_len] = '\0';
  return String(_data);
}

void eepromWrite(int pos, uint8_t val, int targetSize){
  byte* p = (byte*) &val;
  for(int i=0; i<targetSize;i++){
    EEPROM.write(pos + i, *(p+i));
  }
  EEPROM.commit();
}

void eepromWriteInt8(int pos, uint8_t val) { //save elapsed time to EEPROM
    byte* p = (byte*) &val;
    EEPROM.write(pos, *p);
    EEPROM.commit();
}

uint8_t eepromGetInt8(int pos) { //read elapsed time from EEPROM
  int val;
  byte* p = (byte*) &val;
  *p        = EEPROM.read(pos);
  return val;
}

void eepromWriteInt16(int pos, uint16_t val) { //save data from sensor to EEPROM
    byte* p = (byte*) &val;
    EEPROM.write(pos, *p);
    EEPROM.write(pos + 1, *(p + 1));
    EEPROM.commit();
}

uint16_t eepromGetInt16(int pos) { //read data from sensor to EEPROM
  int val;
  byte* p = (byte*) &val;
  *p        = EEPROM.read(pos);
  *(p + 1)  = EEPROM.read(pos + 1);
  return val;
}

void eepromWriteInt32(int pos, int val) {
    byte* p = (byte*) &val;
    EEPROM.write(pos, *p);
    EEPROM.write(pos + 1, *(p + 1));
    EEPROM.write(pos + 2, *(p + 2));
    EEPROM.write(pos + 3, *(p + 3));
    EEPROM.commit();
}

int eepromGetInt32(int pos) {
  int val;
  byte* p = (byte*) &val;
  *p        = EEPROM.read(pos);
  *(p + 1)  = EEPROM.read(pos + 1);
  *(p + 2)  = EEPROM.read(pos + 2);
  *(p + 3)  = EEPROM.read(pos + 3);
  return val;
}

void eepromClear(int startPos, int endPos){ //clear EEPROM
  for (int i = startPos; i < endPos; i++)
  {
    EEPROM.write(i, 0);
  }
  Serial.println("EEPROM CLEARED");
}

// int readingByte(byte theArray[], int dataLength, int startAddress){
//   int val=(theArray[startAddress] & 0xff);
//   for (int i=1; i<dataLength; i++){
//     Serial.print("i = ");
//     Serial.println(i);
//     val= ((theArray[i+startAddress] & 0xff) << ((i)*8) | val);
//   }
//   return val;
// }

// int arrayByteLength(byte intArr[]) {
//   int arrayLength=707;
//   for (int i = 0; i < sizeof(intArr); i=i+16) {
//     if (intArr[i] ==0) {
//       arrayLength = i;
//       break;
//     }
//   }
//   return arrayLength;
// }
