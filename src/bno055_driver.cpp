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

void Imu::getAcc()
{
  char acc_buf[6] = {};
  acc_buf[0] = ACC_DATA_X_LSB;
  if (write(file_desc, acc_buf, 1) != 1)
  {
    std::cout << "ERROR: Failed to write to accelerometer register: " << ACC_DATA_X_LSB << std::endl;
    perror("ERROR: ");
  }
  
  if (read(file_desc, acc_buf, 6) != 6)
  {
    std::cout << "ERROR: Failed to read accelerometer data from register: " << ACC_DATA_X_LSB << std::endl;
    perror("ERROR: ");
  }
}

void Imu::getMag()
{
  char mag_buf[6] = {};
  mag_buf[0] = MAG_DATA_X_LSB;
  if (write(file_desc, mag_buf, 1) != 1)
  {
    std::cout << "ERROR: Failed to write to magnetometer register: " << MAG_DATA_X_LSB << std::endl;
    perror("ERROR: ");
  }
  
  if (read(file_desc, mag_buf, 6) != 6)
  {
    std::cout << "ERROR: Failed to read magnetometer data from register: " << MAG_DATA_X_LSB << std::endl;
    perror("ERROR: ");
  }
}

void Imu::getGyr()
{
  char gyr_buf[6] = {};
  gyr_buf[0] = GYR_DATA_X_LSB;
  if (write(file_desc, gyr_buf, 1) != 1)
  {
    std::cout << "ERROR: Failed to write to gyroscope register: " << GYR_DATA_X_LSB << std::endl;
    perror("ERROR: ");
  }
  
  if (read(file_desc, gyr_buf, 6) != 6)
  {
    std::cout << "ERROR: Failed to read gyroscope data from register: " << GYR_DATA_X_LSB << std::endl;
    perror("ERROR: ");
  }
}

void Imu::getEul()
{
  char eul_buf[6] = {};
  eul_buf[0] = EUL_DATA_X_LSB;
  if (write(file_desc, eul_buf, 1) != 1)
  {
    std::cout << "ERROR: Failed to write to euler register: " << EUL_DATA_X_LSB << std::endl;
    perror("ERROR: ");
  }
  
  if (read(file_desc, eul_buf, 6) != 6)
  {
    std::cout << "ERROR: Failed to read euler data from register: " << EUL_DATA_X_LSB << std::endl;
    perror("ERROR: ");
  }
}

void Imu::getQua()
{
  char qua_buf[8] = {};
  qua_buf[0] = QUA_DATA_W_LSB;
  if (write(file_desc, qua_buf, 1) != 1)
  {
    std::cout << "ERROR: Failed to write to quaternion register: " << QUA_DATA_W_LSB << std::endl;
    perror("ERROR: ");
  }
  
  if (read(file_desc, qua_buf, 6) != 6)
  {
    std::cout << "ERROR: Failed to read quaternion data from register: " << QUA_DATA_W_LSB << std::endl;
    perror("ERROR: ");
  }
}

Imu::~Imu()
{
  cout << "BNO055 Imu destroyed." << endl;
}

