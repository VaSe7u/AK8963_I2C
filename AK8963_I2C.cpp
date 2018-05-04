#include "AK8963_I2C.hpp"

AK8963_I2C::AK8963_I2C() : _i2cAddress(AK8963_ADDRESS) {}

bool AK8963_I2C::initialize() {
  bool success = true;
  softReset();
  success &= setResolution(BITS_16);
  success &= _initializeSensitivityAdjustment();
  success &= selfTest();
  success &= setResolution(BITS_16);
  success &= testConnection();
  success &= setMode(Mode::CONTINUOUS_MEASUREMENT_100HZ);
  return success;
}

uint8_t AK8963_I2C::info() {
  if (I2C::readByte(_i2cAddress, AK8963_INFO, _buffer)) {
    return _buffer[0];
  } else {
    return 0;
  }
}

uint8_t AK8963_I2C::getDeviceId() {
  I2C::readByte(_i2cAddress, AK8963_WIA, _buffer);
  return _buffer[0];
}

bool AK8963_I2C::testConnection() {
  return getDeviceId() == AK8963_ID;
}

Resolution AK8963_I2C::getResolution() {
  if (I2C::readBit(_i2cAddress, AK8963_CNTL1, (bool*)_buffer, AK8963_BIT)) {
    return (Resolution)_buffer[0];
  }
  return Resolution::BITS_INVALID;
}

bool AK8963_I2C::setResolution(Resolution resolution) {
  if (I2C::writeBit(_i2cAddress, AK8963_CNTL1, (bool)resolution,
                    AK8963_BIT)) {
    _resolution = resolution;
    return true;
  } else {
    return false;
  }
}

Mode AK8963_I2C::getMode() {
  if (I2C::readBits(_i2cAddress, AK8963_CNTL1, _buffer, AK8963_MODE_BIT, AK8963_MODE_LENGTH)) {
    return (Mode)_buffer[0];
  } else {
    return Mode::INVALID;
  }
}

bool AK8963_I2C::setMode(Mode mode) {
  if ((uint8_t)mode == 0 || (uint8_t)mode == 8
      || (uint8_t)mode == 0x0F || mode == Mode::INVALID) return false;
  powerDown();
  bool success = I2C::writeBits(_i2cAddress, AK8963_CNTL1, (uint8_t)mode,
                                AK8963_MODE_BIT, AK8963_MODE_LENGTH);
  _delay();
  return success;
}

bool AK8963_I2C::startMeasurement() {
  return setMode(Mode::SINGLE_MEASUREMENT);
}

bool AK8963_I2C::readRaw(float* x, float* y, float* z) {
  bool dataReady = false;
  if (I2C::readBit(_i2cAddress, AK8963_ST1, &dataReady, AK8963_DRDY)) {
    if (dataReady) {
      if (I2C::readBytes(_i2cAddress, AK8963_XOUT_L, _buffer, 7)) {
        if ((_buffer[6] & 0x08) == 0) { // overflow check
          *x = ( (float) ( ((int16_t)_buffer[1] << 8) | (_buffer[0]) )) * _xSensAdj;
          *y = ( (float) ( ((int16_t)_buffer[3] << 8) | (_buffer[2]) )) * _ySensAdj;
          *z = ( (float) ( ((int16_t)_buffer[5] << 8) | (_buffer[4]) )) * _zSensAdj;
          _overflow = false;
          return true;
        } // overflow (Eut > 4912uT)
        _overflow = true;
      } // couldn't read data and ST2 registers
    } // data not ready
  } // couldn't read ST register
  return false;
}

bool AK8963_I2C::read(float* x, float* y, float* z) {
  float xRaw = 0;
  float yRaw = 0;
  float zRaw = 0;
  if (readRaw(&xRaw, &yRaw, &zRaw)) {
    float multiplier = 0.14993894993894993894993894993895f;
    if (_resolution == Resolution::BITS_14) multiplier = 0.5997557997557997557997557997558f;
    *x = (float)xRaw * multiplier;
    *y = (float)yRaw * multiplier;
    *z = (float)zRaw * multiplier;
    return true;
  } else {
    return false;
  }
}

bool AK8963_I2C::overflow() {
  return _overflow;
}

bool AK8963_I2C::selfTest() {
  bool pass = true;
  powerDown();
  I2C::writeBit(_i2cAddress, AK8963_ASTC, 1,
                AK8963_SELF);
  I2C::writeBits(_i2cAddress, AK8963_CNTL1, (uint8_t)Mode::SELF_TEST,
                 AK8963_MODE_BIT, AK8963_MODE_LENGTH);
  float x = 0;
  float y = 0;
  float z = 0;
  while (readRaw(&x, &y, &z) == 0);
  I2C::writeBit(_i2cAddress, AK8963_ASTC, 0,
                AK8963_SELF);
  uint16_t range = 0;
  if (_resolution == BITS_14) {
    range = 50;
    if (z < -800 || z > -200) pass = false;
  } else if (_resolution == BITS_16) {
    range = 200;
    if (z < -3200 || z > -800) pass = false;
  } else {
    pass = false;
  }
  if (x < -range || x > range || y < -range || y > range) pass = false;
  powerDown();
  return pass;
}

bool AK8963_I2C::softReset() {
  return I2C::writeBit(_i2cAddress, AK8963_CNTL2, true,
                       AK8963_SRST);
}

bool AK8963_I2C::powerDown() {
  if (I2C::writeBits(_i2cAddress, AK8963_CNTL1, (uint8_t)Mode::POWER_DOWN,
                     AK8963_MODE_BIT, AK8963_MODE_LENGTH)) {
    _delay();
    return true;
  } else {
    return false;
  }
}

bool AK8963_I2C::powerUp() {
  bool s = softReset();
  return initialize() & s;
}

bool AK8963_I2C::disableI2C() {
  return I2C::writeByte(_i2cAddress, AK8963_I2CDIS, AK8963_DISABLE_I2C);
}

bool AK8963_I2C::enableI2C() {
  return softReset();
}

bool AK8963_I2C::_initializeSensitivityAdjustment() {
  bool s = true;
  s &= powerDown();
  s &= setMode(Mode::FUSE_ROM_ACCESS);
  if (I2C::readBytes(_i2cAddress, AK8963_ASAX, _buffer, 3)) {
    _xSensAdj = ((float)_buffer[0] - 128.0f) / 256.0f + 1.0f;
    _ySensAdj = ((float)_buffer[1] - 128.0f) / 256.0f + 1.0f;
    _zSensAdj = ((float)_buffer[2] - 128.0f) / 256.0f + 1.0f;
    powerDown();
    return s;
  }
  return false;
}

void AK8963_I2C::_delay() {
  for (volatile uint32_t i = 0; i < 3200; ++i) {
    __asm__ __volatile__ ("nop");
  }
}