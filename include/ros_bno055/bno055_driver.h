/*
  Author: Joey Yang
  Email: joeyyang.ai@gmail.com
  Description: 
    Header file of the ROS-agnostic Linux hardware driver for the BNO055 sensor.
*/

#ifndef BNO055_DRIVER_H
#define BNO055_DRIVER_H

/* Helper */
#include <iostream>
#include <vector>
#include <cstdio>
#include <string>

/* I2C */
#include <linux/i2c-dev.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <fcntl.h>

extern const char* I2C_BUS;
extern const __u8 I2C_ADDRESS;

namespace bno055
{
enum RegisterMap
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
  TEMP = 0x34,
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
enum PowMode
{
  NORMAL_MODE = 0x00,
  LOW_POWER_MODE = 0x01,
  SUSPEND_MODE = 0x02
};

// OPR_MODE Register [0x3D]
enum OprMode
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

/* Data structures to hold sensor data */
struct AccData 
{
  __s16 acc_x;
  __s16 acc_y;
  __s16 acc_z;
};

struct MagData 
{
  __s16 mag_x;
  __s16 mag_y;
  __s16 mag_z;
};

struct GyrData 
{
  __s16 gyr_x;
  __s16 gyr_y;
  __s16 gyr_z;
};

struct EulData 
{
  __s16 eul_heading;
  __s16 eul_roll;
  __s16 eul_pitch;
};

struct QuaData 
{
  __s16 qua_w;
  __s16 qua_x;
  __s16 qua_y;
  __s16 qua_z;
};

struct LiaData 
{
  __s16 lia_x;
  __s16 lia_y;
  __s16 lia_z;
};

struct GrvData 
{
  __s16 grv_x;
  __s16 grv_y;
  __s16 grv_z;
};

struct TempData 
{
  __s8 temp;
};

struct CalibStatData 
{
  __s8 calib_stat;
};

struct CalibOffsetData 
{
  __u16 acc_offset_x;
  __u16 acc_offset_y;
  __u16 acc_offset_z;
  __u16 mag_offset_x;
  __u16 mag_offset_y;
  __u16 mag_offset_z;
  __u16 gyr_offset_x;
  __u16 gyr_offset_y;
  __u16 gyr_offset_z;
};

struct CalibRadiusData
{
  __u16 acc_radius;
  __u16 mag_radius;
};

class Bno055Driver
{
public:
  Bno055Driver();
  int initI2c();
  int setConfigMode();
  int setImuMode();
  int setNdofMode();
  int getAcc();
  int getMag();
  int getGyr();
  int getEul();
  int getQua();
  int getLia();
  int getGrv();
  int getTemp();
  int getCalibStat();
  int getCalibOffset();
  int getCalibRadius();
  ~Bno055Driver();
  /* Data structure to hold sensor data */
  struct Data_ {
    double acc_x_;
    double acc_y_;
    double acc_z_;
    double mag_x_;
    double mag_y_;
    double mag_z_;
    double gyr_x_;
    double gyr_y_;
    double gyr_z_;
    double eul_heading_;
    double eul_roll_;
    double eul_pitch_;
    double qua_w_;
    double qua_x_;
    double qua_y_;
    double qua_z_;
    double lia_x_;
    double lia_y_;
    double lia_z_;
    double grv_x_;
    double grv_y_;
    double grv_z_;
    double temp_;
    int calib_stat_sys_;
    int calib_stat_acc_;
    int calib_stat_gyr_;
    int calib_stat_mag_;
    int acc_offset_x_;
    int acc_offset_y_;
    int acc_offset_z_;
    int mag_offset_x_;
    int mag_offset_y_;
    int mag_offset_z_;
    int gyr_offset_x_;
    int gyr_offset_y_;
    int gyr_offset_z_;
    int acc_radius_;
    int mag_radius_; 
  } data_;
private:
  PowMode pow_mode_;
  OprMode opr_mode_;
  int file_desc_;
};
} // End of namespace bno055

#endif // BNO055_DRIVER_H
