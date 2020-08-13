#include "../include/bno055_driver.h"
using namespace std;
//namespace bno055
//{
Imu::Imu()
{
  cout << "BNO055 Imu initialized." << endl;
}

int Imu::accessI2c(string i2c_bus, int i2c_address, unsigned char chip_address)
{
  int file_desc;
  char buffer[5];

  if (file_desc = open(i2c_bus, O_RDWR) < 0)
  {
    cout << "ERROR: Could not open I2C bus: " << i2c_bus << endl;    
    exit(1);
  } 
  
  if (ioctl(file_desc, I2C_SLAVE, i2c_address) < 0) 
  {
    cout << "ERROR: Could not locate BNO055 Imu at address: " << i2c_address << endl;
    exit(1);
  }
  
  buffer[0] = chip_address;
  buffer[1] = 0x11; // Test value.

  if (write(file_desc, buffer, 2) != 2) 
  {
    cout << "ERROR: Could not write to register: " << chip_address << endl;
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

