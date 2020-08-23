#ifndef BNO055_DRIVER_H
#define BNO055_DRIVER_H

#include <linux/i2c-dev.h>
#include <unistd.h>
#include <sys/ioctl.h>
//#include <i2c/smbus.h>
#include <iostream>
#include <vector>
#include <cstdio>
#include <string>

extern const std::string I2C_BUS = "/dev/i2c-1";
extern const unsigned char I2C_ADDRESS;

//namespace bno055
//{
enum class RegisterMap : unsigned char
{
  // REGISTER_NAME = REGISTER_ADDRESS;

  /* Page 0 */
  CHIP_ID = 0x00,
  ACC_ID = 0x01,
  MAG_ID = 0x02,
  GYR_ID = 0x03,
  PAGE_ID = 0x07,
  ACC_DATA_X_LSB = 0x08,
  ACC_DATA_X_MSB = 0x09,
  ACC_DATA_Y_LSB = 0x0A,
  ACC_DATA_Y_MSB = 0x0B,
  ACC_DATA_Z_LSB = 0x0C,
  ACC_DATA_Z_MSB = 0x0D,
  MAG_DATA_X_LSB = 0x0E,
  MAG_DATA_X_MSB = 0x0F,
  MAG_DATA_Y_LSB = 0x10,
  MAG_DATA_Y_MSB = 0x11,
  MAG_DATA_Z_LSB = 0x12,
  MAG_DATA_Z_MSB = 0x13,
  GYR_DATA_X_LSB = 0x14,
  GYR_DATA_X_MSB = 0x15,
  GYR_DATA_Y_LSB = 0x16,
  GYR_DATA_Y_MSB = 0x17,
  GYR_DATA_Z_LSB = 0x18,
  GYR_DATA_Z_MSB = 0x19,
  EUL_HEADING_LSB = 0x1A,
  EUL_HEADING_MSB = 0x1B,
  EUL_ROLL_LSB = 0x1C,
  EUL_ROLL_MSB = 0x1D,
  EUL_PITCH_LSB = 0x1E,
  EUL_PITCH_MSB = 0x1F,
  QUA_DATA_W_LSB = 0x20,
  QUA_DATA_W_MSB = 0x21,
  QUA_DATA_X_LSB = 0x22,
  QUA_DATA_X_MSB = 0x23,
  QUA_DATA_Y_LSB = 0x24,
  QUA_DATA_Y_MSB = 0x25,
  QUA_DATA_Z_LSB = 0x26,
  QUA_DATA_Z_MSB = 0x27,
  LIA_DATA_X_LSB = 0x28,
  LIA_DATA_X_MSB = 0x29,
  LIA_DATA_Y_LSB = 0x2A,
  LIA_DATA_Y_MSB = 0x2B,
  LIA_DATA_Z_LSB = 0x2C,
  LIA_DATA_Z_MSB = 0x2D,
  GRV_DATA_X_LSB = 0x2E,
  GRV_DATA_X_MSB = 0x2F,
  GRV_DATA_Y_LSB = 0x30,
  GRV_DATA_Y_MSB = 0x31,
  GRV_DATA_Z_LSB = 0x32,
  GRV_DATA_Z_MSB = 0x33,
  CALIB_STAT = 0x35,
  ST_RESULT = 0x36,
  INT_STA = 0x37,
  SYS_CLK_STATUS = 0x38,
  SYS_STATUS = 0x39,
  SYS_ERR = 0x3A,
  UNIT_SEL = 0x3B,
  OPR_MODE = 0x3D,
  PWR_MODE = 0x3E,
  SYS_TRIGGER = 0x3F,
  AXIS_MAP_CONFIG = 0x41,
  AXIS_MAP_SIGN = 0x42,
  ACC_OFFSET_X_LSB = 0x55,
  ACC_OFFSET_X_MSB = 0x56,
  ACC_OFFSET_Y_LSB = 0x57,
  ACC_OFFSET_Y_MSB = 0x58,
  ACC_OFFSET_Z_LSB = 0x59,
  ACC_OFFSET_Z_MSB = 0x5A,
  MAG_OFFSET_X_LSB = 0x5B,
  MAG_OFFSET_X_MSB = 0x5C,
  MAG_OFFSET_Y_LSB = 0x5D,
  MAG_OFFSET_Y_MSB = 0x5E,
  MAG_OFFSET_Z_LSB = 0x5F,
  MAG_OFFSET_Z_MSB = 0x60,
  GYR_OFFSET_X_LSB = 0x61,
  GYR_OFFSET_X_MSB = 0x62,
  GYR_OFFSET_Y_LSB = 0x63,
  GYR_OFFSET_Y_MSB = 0x64,
  GYR_OFFSET_Z_LSB = 0x65,
  GYR_OFFSET_Z_MSB = 0x66,
  ACC_RADIUS_LSB = 0x67,
  ACC_RADIUS_MSB = 0x68,
  MAG_RADIUS_LSB = 0x69,
  MAG_RADIUS_MSB = 0x6A,
 
  /* Page 1 */
  ACC_CONFIG = 0x08,
  MAG_CONFIG = 0x09,
  GYR_CONFIG_0 = 0x0A,
  GYR_CONFIG_1 = 0x0B
};

// PWR_MODE Register [0x3E]
enum class PowMode : unsigned char
{
  NORMAL_MODE = 0x00,
  LOW_POWER_MODE = 0x01,
  SUSPEND_MODE = 0x02
};

// OPR_MODE Register [0x3D]
enum class OpMode : unsigned char
{
  CONFIG_MODE = 0x00,
  /* Non-Fusion Modes */
  ACC_ONLY = 0x01,
  MAG_ONLY = 0x02,
  GYRO_ONLY = 0x03,
  ACC_MAG = 0x04,
  ACC_GYRO = 0x05,
  MAG_GYRO = 0x06,
  AMG = 0x07,
  /* Fusion Modes */
  IMU = 0x08,
  COMPASS = 0x09, // Requires calibration
  M4G = 0x0A,
  NDOF_FMC_OFF = 0x0B,
  NDOF = 0x0C// FMC is turned ON
};

class Imu
{
public:
  Imu();
  int accessI2c(string i2c_bus, unsigned char i2c_address);
  void setPowMode(PowMode pow_mode);
  void setOpMode(OpMode op_mode);
  //void writeReg(unsigned char reg_addr, unsigned char val);
  //void readReg(unsigned char reg_addr);
  void enumPorts();
  int run();
  ~Imu();

private:
  PowMode pow_mode_;
  OpMode op_mode_;
};
//} // End of namespace bno055

#endif // BNO055_DRIVER_H
