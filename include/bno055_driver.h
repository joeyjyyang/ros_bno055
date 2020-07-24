#ifndef BNO055_DRIVER_H
#define BNO055_DRIVER_H

enum class RegisterMap : unsigned char
{
  /* Page 0 */

  /* Page 1 */
 
};

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
  /* Non-Fusion Modes */
  accOnly = 0x01,
  magOnly = 0x02,
  gyroOnly = 0x03,
  accMag = 0x04,
  accGyro = 0x05,
  magGyro = 0x06,
  amg = 0x07,
  /* Fusion Modes */
  imu = 0x08,
  compass = 0x09, // Requires calibration
  m4g = 0x0A,
  nDofFmcOff = 0x0B,
  nDof = 0x0C// FMC is turned ON
};

namespace bno055
{
class Imu
{

public:
  Imu();
  
  ~Imu();

private:

};
}

#endif // BNO055_DRIVER_H
