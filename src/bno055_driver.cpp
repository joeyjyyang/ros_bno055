/*
  Author: Joey Yang
  Email: joeyyang.ai@gmail.com
  Description: 
    Implementation file of the ROS-agnostic Linux hardware driver for the BNO055 sensor.
*/

#include "bno055_driver/bno055_driver.h"

const char* I2C_BUS = "/dev/i2c-1";
const __u8 I2C_ADDRESS = 0x28;

bno055::Bno055Driver::Bno055Driver() : pow_mode_(bno055::PowMode::NORMAL_MODE), opr_mode_(bno055::OprMode::CONFIG_MODE)
{
  printf("BNO055 IMU driver initialized.\n");
}

int bno055::Bno055Driver::initI2c()
{
  if ((file_desc_ = open(I2C_BUS, O_RDWR)) < 0)
  {
    printf("ERROR: Could not open I2C bus: %s.\n", I2C_BUS); 
    perror("ERROR: ");
    exit(-1);
  }
  else 
  {
    printf("Opened I2C bus: %s.\n", I2C_BUS);
  }
  
  if (ioctl(file_desc_, I2C_SLAVE, I2C_ADDRESS) < 0)
  {
    printf("ERROR: Could not locate BNO055 sensor at address: 0x%02X.\n", I2C_ADDRESS);
    perror("ERROR: ");
    exit(-1);
  }
  else 
  {
    printf("Located BNO055 sensor at address: 0x%02X.\n", I2C_ADDRESS);
  }

  return 1;
}

int bno055::Bno055Driver::setConfigMode()
{
  if (i2c_smbus_write_byte_data(file_desc_, bno055::RegisterMap::OPR_MODE, bno055::OprMode::CONFIG_MODE) < 0)
  {
    printf("ERROR: Could not set operation mode to CONFIG");
    perror("ERROR: ");
    exit(-1);
  }
  else 
  {
    printf("Set operation mode to CONFIG: 0x%02X.\n", bno055::OprMode::CONFIG_MODE);
  } 
  opr_mode_ = bno055::OprMode::CONFIG_MODE;
  usleep(500000);

  return 1;
}

int bno055::Bno055Driver::setImuMode()
{
  // Reset to config mode first.
  setConfigMode();

  if (i2c_smbus_write_byte_data(file_desc_, bno055::RegisterMap::OPR_MODE, bno055::OprMode::IMU) < 0)
  {
    printf("ERROR: Could not set operation mode to IMU");
    perror("ERROR: ");
    exit(-1);
  } 
  else 
  {
    printf("Set operation mode to IMU: 0x%02X.\n", bno055::OprMode::IMU);
  } 
  opr_mode_ = bno055::OprMode::IMU;
  usleep(500000);

  return 1;
}

int bno055::Bno055Driver::setNdofMode()
{
  // Reset to config mode first.
  setConfigMode();

  if (i2c_smbus_write_byte_data(file_desc_, bno055::RegisterMap::OPR_MODE, bno055::OprMode::NDOF) < 0)
  {
    printf("ERROR: Could not set operation mode to NDOF");
    perror("ERROR: ");
    exit(-1);
  } 
  else 
  {
    printf("Set operation mode to NDOF: 0x%02X.\n", bno055::OprMode::NDOF);
  } 
  opr_mode_ = bno055::OprMode::NDOF;
  usleep(500000);

  return 1;
}

int bno055::Bno055Driver::getAcc()
{
  bno055::AccData acc_data;

  if (i2c_smbus_read_i2c_block_data(file_desc_, RegisterMap::ACC_DATA_X_LSB, 0x06, (__u8*)&acc_data) != 0x06) 
  {
    printf("ERROR: Could not read accelerometer data.");
    perror("ERROR: ");
    exit(-1);
  }
 
  bno055::Bno055Driver::data_.acc_x_ = (double)acc_data.acc_x / 100.0;
  bno055::Bno055Driver::data_.acc_y_ = (double)acc_data.acc_y / 100.0;
  bno055::Bno055Driver::data_.acc_z_ = (double)acc_data.acc_z / 100.0;

  return 1;
}

int bno055::Bno055Driver::getMag()
{ 
  bno055::MagData mag_data;

  if (i2c_smbus_read_i2c_block_data(file_desc_, RegisterMap::MAG_DATA_X_LSB, 0x06, (__u8*)&mag_data) != 0x06) 
  {
    printf("ERROR: Could not read magnetometer data.");
    perror("ERROR: ");
    exit(-1);
  }
  
  bno055::Bno055Driver::data_.mag_x_ = (double)mag_data.mag_x / 16.0;
  bno055::Bno055Driver::data_.mag_y_ = (double)mag_data.mag_y / 16.0;
  bno055::Bno055Driver::data_.mag_z_ = (double)mag_data.mag_z / 16.0;

  return 1;
}

int bno055::Bno055Driver::getGyr()
{
  bno055::GyrData gyr_data;

  if (i2c_smbus_read_i2c_block_data(file_desc_, RegisterMap::GYR_DATA_X_LSB, 0x06, (__u8*)&gyr_data) != 0x06) 
  {
    printf("ERROR: Could not read gyroscope data.");
    perror("ERROR: ");
    exit(-1);
  }
  
  bno055::Bno055Driver::data_.gyr_x_ = (double)gyr_data.gyr_x / 900.0;
  bno055::Bno055Driver::data_.gyr_y_ = (double)gyr_data.gyr_y / 900.0;
  bno055::Bno055Driver::data_.gyr_z_ = (double)gyr_data.gyr_z / 900.0;

  return 1;
}

int bno055::Bno055Driver::getEul()
{
  bno055::EulData eul_data;

  if (i2c_smbus_read_i2c_block_data(file_desc_, RegisterMap::EUL_HEADING_LSB, 0x06, (__u8*)&eul_data) != 0x06)
  {
    printf("ERROR: Could not read euler angles data.");
    perror("ERROR: ");
    exit(-1);
  }
  
  bno055::Bno055Driver::data_.eul_heading_ = (double)eul_data.eul_heading / 16.0;
  bno055::Bno055Driver::data_.eul_roll_ = (double)eul_data.eul_roll / 16.0;
  bno055::Bno055Driver::data_.eul_pitch_ = (double)eul_data.eul_pitch / 16.0;

  return 1;
}

int bno055::Bno055Driver::getQua()
{
  bno055::QuaData qua_data;

  if (i2c_smbus_read_i2c_block_data(file_desc_, RegisterMap::QUA_DATA_W_LSB, 0x08, (__u8*)&qua_data) != 0x08)
  {
    printf("ERROR: Could not read quaternions data.");
    perror("ERROR: ");
    exit(-1);
  }
  
  bno055::Bno055Driver::data_.qua_w_ = (double)qua_data.qua_w / 16384.0;
  bno055::Bno055Driver::data_.qua_x_ = (double)qua_data.qua_x / 16384.0;
  bno055::Bno055Driver::data_.qua_y_ = (double)qua_data.qua_y / 16384.0;
  bno055::Bno055Driver::data_.qua_z_ = (double)qua_data.qua_z / 16384.0;

  return 1;
}

int bno055::Bno055Driver::getLia()
{
  bno055::LiaData lia_data;

  if (i2c_smbus_read_i2c_block_data(file_desc_, RegisterMap::LIA_DATA_X_LSB, 0x06, (__u8*)&lia_data) != 0x06) 
  {
    printf("ERROR: Could not read linear acceleration data.");
    perror("ERROR: ");
    exit(-1);
  }
 
  bno055::Bno055Driver::data_.lia_x_ = (double)lia_data.lia_x / 100.0;
  bno055::Bno055Driver::data_.lia_y_ = (double)lia_data.lia_y / 100.0;
  bno055::Bno055Driver::data_.lia_z_ = (double)lia_data.lia_z / 100.0;

  return 1;
}

bno055::Bno055Driver::~Bno055Driver()
{
  printf("BNO055 IMU driver destroyed.\n");  
}
