#include "../include/bno055_driver.h"

//namespace bno055
//{
Imu::Imu()
{
  cout << "BNO055 Imu initialized." << endl;
}

int Imu::accessI2c()
{
  int file_desc;
  if ((file_desc = open(I2C_BUS, O_RDWR)) < 0)
  {
    std::cout << "ERROR: Could not open I2C bus: " << I2C_BUS << std::endl; 
    perror("ERROR: ");
    exit(1);
  }
  
  if (ioctl(file_desc, I2C_SLAVE, I2C_ADDRESS) < 0)
  {
    std::cout << "ERROR: Could not locate BNO055 Sensor at address: " << I2C_ADDRESS << std::endl;
    perror("ERROR: ");
    exit(1);
  }
}

void Imu::setPowMode(PowMode pow_mode)
{   
  // Set power mode.
  char write_buf[2];
  write_buf[0] = RegisterMap::PWR_MODE;
  write_buf[1] = pow_mode;
  if (write(file_desc, write_buf, 2) != 2)
  {
    std::cout << "ERROR: Failed to set power mode to: " << pow_mode << std::endl;
    perror("ERROR: ");
  }  
}

void Imu::setOprMode(OprMode opr_mode)
{
  // Set operation mode.
  char write_buf[2];
  write_buf[0] = RegisterMap::OPR_MODE;
  write_buf[1] = opr_mode;
  if (write(file_desc, write_buf, 2) != 2)
  {
    std::cout << "ERROR: Failed to set operation mode to: " << opr_mode << std::endl;
    perror("ERROR: ");
  }  
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

