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
}

void Imu::setOpMode(OpMode op_mode)
{
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

