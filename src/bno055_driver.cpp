#include "../include/bno055_driver.h"

//namespace bno055
//{
Imu::Imu()
{
  cout << "BNO055 Imu initialized." << endl;
}

int Imu::initI2c()
{
  if ((file_desc_ = open(I2C_BUS, O_RDWR)) < 0)
  {
    printf("ERROR: Could not open I2C bus: %s.\n", I2C_BUS); 
    perror("ERROR: ");
    exit(-1);
  }
  
  if (ioctl(file_desc_, I2C_SLAVE, I2C_ADDRESS) < 0)
  {
    printf("ERROR: Could not locate BNO055 sensor at address: %d.\n", I2C_ADDRESS);
    perror("ERROR: ");
    exit(-1);
  }
  return 1;
}

int Imu::setConfigMode()
{
  if (i2c_smbus_write_byte_data(file_desc_, RegisterMap::OPR_MODE, OprMode::CONFIG_MODE) < 0)
  {
    printf("ERROR: Could not set operation mode to CONFIG");
    perror("ERROR: ");
    exit(-1);
  } 
  usleep(500000);
  return 1;
}

int Imu::setImuMode()
{
  // Reset to config mode first.
  setConfigMode();

  if (i2c_smbus_write_byte_data(file_desc_, RegisterMap::OPR_MODE, OprMode::IMU) < 0)
  {
    printf("ERROR: Could not set operation mode to IMU");
    perror("ERROR: ");
    exit(-1);
  } 
  usleep(500000);
  return 1;
}

int Imu::getAcc()
{
  return 1;
}

int Imu::getMag()
{
  return 1;
}

int Imu::getGyr()
{
  return 1;
}

int Imu::getEul()
{
  return 1;
}

int Imu::getQua()
{
  return 1;
}

Imu::~Imu()
{
  cout << "BNO055 Imu destroyed." << endl;
}

