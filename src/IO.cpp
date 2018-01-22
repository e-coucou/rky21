#include "IO.h"

Power::Power()
{

}

float Power::getVCell() {

  byte MSB = 0;
  byte LSB = 0;
  
  readRegister(VCELL_REGISTER, MSB, LSB);
  int value = (MSB << 4) | (LSB >> 4);
  return map(value, 0x000, 0xFFF, 0, 50000) / 10000.0;
  //return value * 0.00125;
}
float Power::getSoC() {
  
  byte MSB = 0;
  byte LSB = 0;
  
  readRegister(SOC_REGISTER, MSB, LSB);
  float decimal = LSB / 256.0;
  return MSB + decimal; 
}
void Power::readRegister(byte startAddress, byte &MSB, byte &LSB) {

  Wire.beginTransmission(MAX17043_ADDRESS);
  Wire.write(startAddress);
  Wire.endTransmission();
  
  Wire.requestFrom(MAX17043_ADDRESS, 2);
  MSB = Wire.read();
  LSB = Wire.read();
}