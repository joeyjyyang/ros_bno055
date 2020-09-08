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
    printf("ERROR: Could not locate BNO055 sensor at address: %d.\n", I2C_ADDRESS);
    perror("ERROR: ");
    exit(-1);
  }
  else 
  {
    printf("Located BNO055 sensor at address: %d.\n", I2C_ADDRESS);
  }

  return 1;
}
/*
int bno055::Bno055Driver::setConfigMode()
{
  if (i2c_smbus_write_byte_data(file_desc_, bno055::RegisterMap::OPR_MODE, bno055::OprMode::CONFIG_MODE) < 0)
  {
    printf("ERROR: Could not set operation mode to CONFIG");
    perror("ERROR: ");
    exit(-1);
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
  opr_mode_ = bno055::OprMode::IMU;
  usleep(500000);

  return 1;
}

int bno055::Bno055Driver::getAcc()
{
  __u8 acc_x_lsb = i2c_smbus_read_byte_data(file_desc_, RegisterMap::ACC_DATA_X_LSB);
  __u8 acc_x_msb = i2c_smbus_read_byte_data(file_desc_, RegisterMap::ACC_DATA_X_MSB);
  __u8 acc_y_lsb = i2c_smbus_read_byte_data(file_desc_, RegisterMap::ACC_DATA_Y_LSB);
  __u8 acc_y_msb = i2c_smbus_read_byte_data(file_desc_, RegisterMap::ACC_DATA_Y_MSB);
  __u8 acc_z_lsb = i2c_smbus_read_byte_data(file_desc_, RegisterMap::ACC_DATA_Z_LSB);
  __u8 acc_z_msb = i2c_smbus_read_byte_data(file_desc_, RegisterMap::ACC_DATA_Z_MSB);

  if (acc_x_lsb < 0 || acc_x_msb < 0 || acc_y_lsb < 0 || acc_y_msb < 0 || acc_z_lsb < 0 || acc_z_msb < 0) {
    printf("ERROR: Could not read accelerometer data.");
    perror("ERROR: ");
    exit(-1);
  } 

  // Calculate decimal value from LSB and MSB.
  __s16 buf_acc_x = ((__s16)acc_x_msb << 8) | acc_x_lsb;
  __s16 buf_acc_y = ((__s16)acc_y_msb << 8) | acc_y_lsb;
  __s16 buf_acc_z = ((__s16)acc_z_msb << 8) | acc_z_lsb;

  AccData::acc_x_ = buf_acc_x;
  AccData::acc_y_ = buf_acc_y;
  AccData::acc_z_ = buf_acc_z;

  return 1;
}

int bno055::Bno055Driver::getMag()
{
  __u8 mag_x_lsb = i2c_smbus_read_byte_data(file_desc_, RegisterMap::MAG_DATA_X_LSB);
  __u8 mag_x_msb = i2c_smbus_read_byte_data(file_desc_, RegisterMap::MAG_DATA_X_MSB);
  __u8 mag_y_lsb = i2c_smbus_read_byte_data(file_desc_, RegisterMap::MAG_DATA_Y_LSB);
  __u8 mag_y_msb = i2c_smbus_read_byte_data(file_desc_, RegisterMap::MAG_DATA_Y_MSB);
  __u8 mag_z_lsb = i2c_smbus_read_byte_data(file_desc_, RegisterMap::MAG_DATA_Z_LSB);
  __u8 mag_z_msb = i2c_smbus_read_byte_data(file_desc_, RegisterMap::MAG_DATA_Z_MSB);

  if (mag_x_lsb < 0 || mag_x_msb < 0 || mag_y_lsb < 0 || mag_y_msb < 0 || mag_z_lsb < 0 || mag_z_msb < 0) {
    printf("ERROR: Could not read magnetometer data.");
    perror("ERROR: ");
    exit(1);
  } 

  // Calculate decimal value from LSB and MSB.
  __s16 buf_mag_x = ((__s16)mag_x_msb << 8) | mag_x_lsb;
  __s16 buf_mag_y = ((__s16)mag_y_msb << 8) | mag_y_lsb;
  __s16 buf_mag_z = ((__s16)mag_z_msb << 8) | mag_z_lsb;

  MagData::mag_x_ = buf_mag_x / 1.6;
  MagData::mag_y_ = buf_mag_y / 1.6;
  MagData::mag_z_ = buf_mag_z / 1.6;

  return 1;
}

int bno055::Bno055Driver::getGyr()
{
  __u8 gyr_x_lsb = i2c_smbus_read_byte_data(file_desc_, RegisterMap::GYR_DATA_X_LSB);
  __u8 gyr_x_msb = i2c_smbus_read_byte_data(file_desc_, RegisterMap::GYR_DATA_X_MSB);
  __u8 gyr_y_lsb = i2c_smbus_read_byte_data(file_desc_, RegisterMap::GYR_DATA_Y_LSB);
  __u8 gyr_y_msb = i2c_smbus_read_byte_data(file_desc_, RegisterMap::GYR_DATA_Y_MSB);
  __u8 gyr_z_lsb = i2c_smbus_read_byte_data(file_desc_, RegisterMap::GYR_DATA_Z_LSB);
  __u8 gyr_z_msb = i2c_smbus_read_byte_data(file_desc_, RegisterMap::GYR_DATA_Z_MSB);

  if (gyr_x_lsb < 0 || gyr_x_msb < 0 || gyr_y_lsb < 0 || gyr_y_msb < 0 || gyr_z_lsb < 0 || gyr_z_msb < 0) {
    printf("ERROR: Could not read gyroscope data.");
    perror("ERROR: ");
    exit(1);
  } 

  // Calculate decimal value from LSB and MSB.
  __s16 buf_gyr_x = ((__s16)gyr_x_msb << 8) | gyr_x_lsb;
  __s16 buf_gyr_y = ((__s16)gyr_y_msb << 8) | gyr_y_lsb;
  __s16 buf_gyr_z = ((__s16)gyr_z_msb << 8) | gyr_z_lsb;

  GyrData::gyr_x_ = buf_gyr_x / 16.0;
  GyrData::gyr_y_ = buf_gyr_y / 16.0;
  GyrData::gyr_z_ = buf_gyr_z / 16.0;

  return 1;
}

int bno055::Bno055Driver::getEul()
{
  __u8 eul_heading_lsb = i2c_smbus_read_byte_data(file_desc_, EUL_HEADING_LSB);
  __u8 eul_heading_msb = i2c_smbus_read_byte_data(file_desc_, EUL_HEADING_MSB);
  __u8 eul_roll_lsb = i2c_smbus_read_byte_data(file_desc_, EUL_ROLL_LSB);
  __u8 eul_roll_msb = i2c_smbus_read_byte_data(file_desc_, EUL_ROLL_MSB);
  __u8 eul_pitch_lsb = i2c_smbus_read_byte_data(file_desc_, EUL_PITCH_LSB);
  __u8 eul_pitch_msb = i2c_smbus_read_byte_data(file_desc_, EUL_PITCH_MSB);

  if (eul_heading_lsb < 0 || eul_heading_msb < 0 || eul_roll_lsb < 0 || eul_roll_msb < 0 || eul_pitch_lsb < 0 || eul_pitch_msb < 0) {
    printf("ERROR: Could not read euler angles data.");
    perror("ERROR: ");
    exit(1);
  } 

  // Calculate decimal value from LSB and MSB.
  __s16 buf_eul_heading = ((__s16)eul_heading_msb << 8) | eul_heading_lsb;
  __s16 buf_eul_roll = ((__s16)eul_roll_msb << 8) | eul_roll_lsb;
  __s16 buf_eul_pitch = ((__s16)eul_pitch_msb << 8) | eul_pitch_lsb;
  
  EulData::eul_heading_ = buf_eul_heading / 16.0;
  EulData::eul_roll_ = buf_eul_roll / 16.0;
  EulData::eul_pitch_ = buf_eul_pitch / 16.0;

  return 1;
}

int bno055::Bno055Driver::getQua()
{
  __u8 qua_w_lsb = i2c_smbus_read_byte_data(file_desc_, QUA_DATA_W_LSB);
  __u8 qua_w_msb = i2c_smbus_read_byte_data(file_desc_, QUA_DATA_W_MSB);
  __u8 qua_x_lsb = i2c_smbus_read_byte_data(file_desc_, QUA_DATA_X_LSB);
  __u8 qua_x_msb = i2c_smbus_read_byte_data(file_desc_, QUA_DATA_X_MSB);
  __u8 qua_y_lsb = i2c_smbus_read_byte_data(file_desc_, QUA_DATA_Y_LSB);
  __u8 qua_y_msb = i2c_smbus_read_byte_data(file_desc_, QUA_DATA_Y_MSB);
  __u8 qua_z_lsb = i2c_smbus_read_byte_data(file_desc_, QUA_DATA_Z_LSB);
  __u8 qua_z_msb = i2c_smbus_read_byte_data(file_desc_, QUA_DATA_Z_MSB);

  if (qua_w_lsb < 0 || qua_w_msb < 0 || qua_x_lsb < 0 || qua_x_msb < 0 || qua_y_lsb < 0 || qua_y_msb < 0 || qua_z_lsb < 0 || qua_z_msb < 0) {
    printf("ERROR: Could not read quaternion data.");
    perror("ERROR: ");
    exit(1);
  } 

  // Calculate decimal value from LSB and MSB.
  __s16 buf_qua_w = ((__s16)qua_w_msb << 8) | qua_w_lsb;
  __s16 buf_qua_x = ((__s16)qua_x_msb << 8) | qua_x_lsb;
  __s16 buf_qua_y = ((__s16)qua_y_msb << 8) | qua_y_lsb;
  __s16 buf_qua_z = ((__s16)qua_z_msb << 8) | qua_z_lsb;

  QuaData::qua_w_ = buf_qua_w / 16384.0;
  QuaData::qua_x_ = buf_qua_x / 16384.0;
  QuaData::qua_y_ = buf_qua_y / 16384.0;
  QuaData::qua_z_ = buf_qua_z / 16384.0;

  return 1;
}
*/
bno055::Bno055Driver::~Bno055Driver()
{
  printf("BNO055 IMU driver destroyed.\n");  
}
