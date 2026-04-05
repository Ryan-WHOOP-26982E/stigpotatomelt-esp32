#include "SparkFun_LIS331_ESP32.h"
#include <SPI.h>
#include <Wire.h>
#include <stdint.h>

LIS331ESP::LIS331ESP(void)
{
}

void LIS331ESP::begin(comm_mode mode)
{
  this->mode = mode;
  setPowerMode(NORMAL);
  axesEnable(true);
  uint8_t data = 0;
  for (int i = 0x21; i < 0x25; i++) LIS331_write(i,&data,1);
  for (int i = 0x30; i < 0x37; i++) LIS331_write(i,&data,1);
}

void LIS331ESP::setI2CAddr(uint8_t address)
{
  this->address = address;
}

void LIS331ESP::setSPICSPin(uint8_t pin)
{
  this->CSPin = pin;
}

void LIS331ESP::axesEnable(bool enable)
{
  uint8_t data;
  LIS331_read(CTRL_REG1, &data, 1);
  if (enable)
  {
    data |= 0x07;
  }
  else
  {
    data &= ~0x07;
  }
  LIS331_write(CTRL_REG1, &data, 1);
}

void LIS331ESP::setPowerMode(power_mode pmode)
{
  uint8_t data;
  LIS331_read(CTRL_REG1, &data, 1);

  // The power mode is the high three bits of CTRL_REG1. The mode 
  //  constants are the appropriate bit values left shifted by five, so we 
  //  need to right shift them to make them work. We also want to mask off the
  //  top three bits to zero, and leave the others untouched, so we *only*
  //  affect the power mode bits.
  data &= ~0xe0; // Clear the top three bits
  data |= pmode<<5; // set the top three bits to our pmode value
  LIS331_write(CTRL_REG1, &data, 1); // write the new value to CTRL_REG1
}

void LIS331ESP::setODR(data_rate drate)
{
  uint8_t data;
  LIS331_read(CTRL_REG1, &data, 1);

  // The data rate is bits 4:3 of CTRL_REG1. The data rate constants are the
  //  appropriate bit values; we need to right shift them by 3 to align them
  //  with the appropriate bits in the register. We also want to mask off the
  //  top three and bottom three bits, as those are unrelated to data rate and
  //  we want to only change the data rate.
  data &=~0x18;     // Clear the two data rate bits
  data |= drate<<3; // Set the two data rate bits appropriately.
  LIS331_write(CTRL_REG1, &data, 1); // write the new value to CTRL_REG1
}

void LIS331ESP::readAxes(int16_t &x, int16_t &y, int16_t &z)
{
  uint8_t data[6]; // create a buffer for our incoming data
  // Burst read all 6 axis bytes in one I2C transaction.
  // LIS331 supports auto-increment when bit 7 of the register address is set (0x28 | 0x80 = 0xA8).
  // This reads OUT_X_L, OUT_X_H, OUT_Y_L, OUT_Y_H, OUT_Z_L, OUT_Z_H consecutively.
  LIS331_read(OUT_X_L | 0x80, data, 6);
  // The data that comes out is 12-bit data, left justified, so the lower
  //  four bits of the data are always zero. We need to right shift by four,
  //  then typecase the upper data to an integer type so it does a signed
  //  right shift.
  x = data[0] | data[1] << 8;
  y = data[2] | data[3] << 8;
  z = data[4] | data[5] << 8;
  x = x >> 4;
  y = y >> 4;
  z = z >> 4;
}

uint8_t LIS331ESP::readReg(uint8_t reg_address)
{
  uint8_t data;
  LIS331_read(reg_address, &data, 1);
  return data;
}

float LIS331ESP::convertToG(int maxScale, int reading)
{
  float result = (float(maxScale) * float(reading))/2047;
  return result;
}

void LIS331ESP::setHighPassCoeff(high_pass_cutoff_freq_cfg hpcoeff)
{
  // The HPF coeff depends on the output data rate. The cutoff frequency is
  //  is approximately fs/(6*HPc) where HPc is 8, 16, 32 or 64, corresponding
  //  to the various constants available for this parameter.
  uint8_t data;
  LIS331_read(CTRL_REG2, &data, 1);
  data &= ~0xfc;  // Clear the two low bits of the CTRL_REG2
  data |= hpcoeff;
  LIS331_write(CTRL_REG2, &data, 1);
}

void LIS331ESP::enableHPF(bool enable)
{
  // Enable the high pass filter
  uint8_t data;
  LIS331_read(CTRL_REG2, &data, 1);
  if (enable)
  {
    data |= 1<<5;
  }
  else
  {
    data &= ~(1<<5);
  }
  LIS331_write(CTRL_REG2, &data, 1);
}

void LIS331ESP::HPFOnIntPin(bool enable, uint8_t pin)
{
  // Enable the hpf on signal to int pins 
  uint8_t data;
  LIS331_read(CTRL_REG2, &data, 1);
  if (enable)
  {
    if (pin == 1)
    {
      data |= 1<<3;
    }
    if (pin == 2)
    {
      data |= 1<<4;
    }
  }
  else
  {
    if (pin == 1)
    {
      data &= ~1<<3;
    }
    if (pin == 2)
    {
      data &= ~1<<4;
    }
  }
  LIS331_write(CTRL_REG2, &data, 1);
}

void LIS331ESP::intActiveHigh(bool enable)
{
  // Are the int pins active high or active low?
  uint8_t data;
  LIS331_read(CTRL_REG3, &data, 1);
  // Setting bit 7 makes int pins active low
  if (!enable)
  {
    data |= 1<<7;
  }
  else
  {
    data &= ~(1<<7);
  }
  LIS331_write(CTRL_REG3, &data, 1);
}

void LIS331ESP::intPinMode(pp_od _pinMode)
{
  uint8_t data;
  LIS331_read(CTRL_REG3, &data, 1);
  // Setting bit 6 makes int pins open drain.
  if (_pinMode == DRAIN_OPEN)
  {
    data |= 1<<6;
  }
  else
  {
    data &= ~(1<<6);
  }
  LIS331_write(CTRL_REG3, &data, 1);
}

void LIS331ESP::latchInterrupt(bool enable, uint8_t intSource)
{
  // Latch mode for interrupt. When enabled, you must read the INTx_SRC reg
  //  to clear the interrupt and make way for another.
  uint8_t data; 
  LIS331_read(CTRL_REG3, &data, 1);
  // Enable latching by setting the appropriate bit.
  if (enable)
  {
    if (intSource == 1)
    {
      data |= 1<<2;
    }
    if (intSource == 2)
    {
      data |= 1<<5;
    }
  }
  else
  {
    if (intSource == 1)
    {
      data &= ~1<<2;
    }
    if (intSource == 2)
    {
      data &= ~1<<5;
    }
  }
  LIS331_write(CTRL_REG3, &data, 1);
}

void LIS331ESP::intSrcConfig(int_sig_src src, uint8_t pin)
{

  uint8_t data; 
  LIS331_read(CTRL_REG3, &data, 1);
  // Enable latching by setting the appropriate bit.
  if (pin == 1)
  {
    data &= ~0xfc; // clear the low two bits of the register
    data |= src;
  }
  if (pin == 2)
  {
    data &= ~0xe7; // clear bits 4:3 of the register
    data |= src<<4;
  }
  LIS331_write(CTRL_REG3, &data, 1);
}

void LIS331ESP::setFullScale(fs_range range)
{
  uint8_t data; 
  LIS331_read(CTRL_REG4, &data, 1);
  data &= ~0xcf;
  data |= range<<4;
  LIS331_write(CTRL_REG4, &data, 1);
}

bool LIS331ESP::newXData()
{
  uint8_t data;
  LIS331_read(STATUS_REG, &data, 1);
  if (data & 1<<0)
  {
    return true;
  }
  else
  {
    return false;
  }
}

bool LIS331ESP::newYData()
{
  uint8_t data;
  LIS331_read(STATUS_REG, &data, 1);
  if (data & 1<<1)
  {
    return true;
  }
  else
  {
    return false;
  }
}

bool LIS331ESP::newZData()
{
  uint8_t data;
  LIS331_read(STATUS_REG, &data, 1);
  if (data & 1<<2)
  {
    return true;
  }
  else
  {
    return false;
  }
}

void LIS331ESP::enableInterrupt(int_axis axis, trig_on_level trigLevel,
                     uint8_t interrupt, bool enable)
{
  uint8_t data, reg, mask; 
  mask = 0;
  if (interrupt == 1)
  {
    reg = INT1_CFG;
  }
  else
  {
    reg = INT2_CFG;
  }
  LIS331_read(reg, &data, 1);
  if (trigLevel == TRIG_ON_HIGH)
  {
    mask = 1<<1;
  }
  else
  {
    mask = 1;
  }
  if (axis == Z_AXIS) mask = mask<<4;
  if (axis == Y_AXIS) mask = mask<<2;
  if (enable)
  {
    data |= mask;
  }
  else
  {
    data &= ~mask;
  }
  LIS331_write(reg, &data, 1);
}

void LIS331ESP::setIntDuration(uint8_t duration, uint8_t intSource)
{
  if (intSource == 1)
  {
    LIS331_write(INT1_DURATION, &duration, 1);
  }
  else
  {
    LIS331_write(INT2_DURATION, &duration, 1);
  }
}

void LIS331ESP::setIntThreshold(uint8_t threshold, uint8_t intSource)
{
  if (intSource == 1)
  {
    LIS331_write(INT1_THS, &threshold, 1);
  }
  else
  {
    LIS331_write(INT2_THS, &threshold, 1);
  }
}

void LIS331ESP::LIS331_write(uint8_t reg_address, uint8_t *data, uint8_t len)
{
  if (mode == USE_I2C)
  {
    // I2C write handling code
    Wire.beginTransmission(address);
    Wire.write(reg_address);
    for(int i = 0; i<len; i++)
    {
      Wire.write(data[i]);
    }
    Wire.endTransmission();
  }
  else
  {
    // SPI write handling code
    digitalWrite(CSPin, LOW);
    SPI.transfer(reg_address | 0x40);
    for (int i=0; i<len; i++)
    {
      SPI.transfer(data[i]);
    }
    digitalWrite(CSPin, HIGH);
  }
}

void LIS331ESP::LIS331_read(uint8_t reg_address, uint8_t *data, uint8_t len)
{
  if (mode == USE_I2C)
  {
    // I2C read handling code
    Wire.beginTransmission(address);
    Wire.write(reg_address);
    Wire.endTransmission(false);  // repeated start — keeps bus held, avoids STOP+START overhead
    Wire.requestFrom(address, len);
    for (int i = 0; i<len; i++)
    {
      data[i] = Wire.available() ? Wire.read() : 0;
    }
  }
  else
  {
    // SPI read handling code
    digitalWrite(CSPin, LOW);
    SPI.transfer(reg_address | 0xC0);
    for (int i=0; i<len; i++)
    {
      data[i] = SPI.transfer(0);
    }
    digitalWrite(CSPin, HIGH);
  }
}