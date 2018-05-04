#pragma once
#include "I2Chelper.hpp"


#define AK8963_ADDRESS      0x0C
#define AK8963_ID           0x48

#define AK8963_WIA          0x00  // Device ID (R)
#define AK8963_INFO         0x01  // Information (R)
#define AK8963_ST1          0x02  // Status 1 (R) - Data status
#define AK8963_XOUT_L       0x03  // Measurement data (R) - X
#define AK8963_XOUT_H       0x04  // Measurement data (R) - X
#define AK8963_YOUT_L       0x05  // Measurement data (R) - Y
#define AK8963_YOUT_H       0x06  // Measurement data (R) - Y
#define AK8963_ZOUT_L       0x07  // Measurement data (R) - Z
#define AK8963_ZOUT_H       0x08  // Measurement data (R) - Z
#define AK8963_ST2          0x09  // Status 2 (R) - Data status
#define AK8963_CNTL1        0x0A  // Control 1 (R/W) - Function control
#define AK8963_CNTL2        0x0B  // Control 2 (R/W) - Function control
#define AK8963_ASTC         0x0C  // Self-test (R/W)
#define AK8963_I2CDIS       0x0F  // I2C disable (R/W)
#define AK8963_ASAX         0x10  // X-axis sensitivity adjustment value (R) - Fuse ROM
#define AK8963_ASAY         0x11  // Y-axis sensitivity adjustment value (R) - Fuse ROM
#define AK8963_ASAZ         0x12  // Z-axis sensitivity adjustment value (R) - Fuse ROM


// ST1
#define AK8963_DOR          1  // Data overrun
#define AK8963_DRDY         0  // Data ready

// ST2
#define AK8963_BITM         4  // Output bit setting (mirror) | "0": 14bit, "1": 16bit
#define AK8963_HOFL         3  // Magnetic sensor overflow (if 1 - wrong data)

// CNTL1
#define AK8963_BIT          4  // Output bit setting | "0": 14bit, "1": 16bit
#define AK8963_MODE_BIT     3  // Operation mode setting [3:0]
#define AK8963_MODE_LENGTH  4  // Operation mode setting [3:0]

// CNTL2
#define AK8963_SRST         0  // Soft reset | "0": Normal, "1": Reset

// ASTC
#define AK8963_SELF         6  // Self test control | "0": Normal, "1": ST

#define AK8963_DISABLE_I2C                          0b00011011

/*
Sensitivity
0.6 µT/LSB typ. (14-bit)
0.15µT/LSB typ. (16-bit)
*/

enum Resolution : uint8_t {
  BITS_14 = 0,
  BITS_16 = 1,
  BITS_INVALID = 127,
};
enum class Mode : uint8_t {
  POWER_DOWN = 0b0000,
  SINGLE_MEASUREMENT = 0b0001,
  CONTINUOUS_MEASUREMENT_8HZ = 0b0010,
  CONTINUOUS_MEASUREMENT_SLOW = 0b0010,
  CONTINUOUS_MEASUREMENT_100HZ = 0b0110,
  CONTINUOUS_MEASUREMENT_FAST = 0b0110,
  EXTERNAL_TRIGGER_MEASUREMENT = 0b0100,
  SELF_TEST = 0b1000,
  FUSE_ROM_ACCESS = 0b1111,
  INVALID = 0b1010,
};

class AK8963_I2C {
public:
  AK8963_I2C();
  uint8_t info();
  uint8_t getDeviceId();
  bool initialize();
  bool testConnection();
  Resolution getResolution();
  bool setResolution(Resolution resolution);
  Mode getMode();
  bool setMode(Mode mode);
  bool startMeasurement();

  bool readRaw(float* x, float* y, float* z);
  bool read(float* x, float* y, float* z);

  bool overflow();

  bool selfTest();
  bool softReset();
  bool powerDown();
  bool powerUp();
  bool disableI2C();
  bool enableI2C();


private:
  uint8_t _i2cAddress;
  uint8_t _buffer[6];
  Resolution _resolution;
  float _xSensAdj;
  float _ySensAdj;
  float _zSensAdj;
  bool _overflow = false;

  bool _initializeSensitivityAdjustment();
  void _delay();
};
