#include "../include/bno055_driver.h"
using namespace std;
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

void Imu::setOpMode(OpMode op_mode)
{
  // Set operation mode.
  char write_buf[2];
  write_buf[0] = RegisterMap::OPR_MODE;
  write_buf[1] = op_mode;
  if (write(file_desc, write_buf, 2) != 2)
  {
    std::cout << "ERROR: Failed to set operation mode to: " << op_mode << std::endl;
    perror("ERROR: ");
  }  
}
/*
void writeReg(unsigned char reg_addr, unsigned char val)
{
}

void readReg(unsigned char reg_addr)
{
}
*/

Imu::~Imu()
{
  cout << "BNO055 Imu destroyed." << endl;
}

