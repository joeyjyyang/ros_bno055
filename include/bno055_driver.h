#ifndef BNO055_DRIVER_H
#define BNO055_DRIVER_H

// [PWR_MODE]
enum class PowerMode : unsigned char
{
  normalMode = 0x00,
  lowPowerMode = 0x01,
  suspendMode = 0x02
};

// [OPR_MODE] 
enum class OperationMode : unsigned char
{
  configMode = 0x00,
  accOnly = 0x01,
  magOnly = 0x02,
  gyroOnly = 0x03,
  imu = 0x08,
  compass = 0x09, // Requires calibration
  ndof = 0x0C// FMC is turned ON
};

namespace bno055_imu
{
class IMU
{

public:
  IMU();
  
  ~IMU();

private:

};
}

#endif // BNO055_DRIVER_H
