#include "../include/bno055_driver.h"
using namespace std;
//namespace bno055
//{
Imu::Imu()
{
  std::cout << "BNO055 Imu initialized." << std::endl;
}

void Imu::setPowMode(PowMode pow_mode)
{
   
}

void Imu::enumPorts()
{
  vector<serial::PortInfo> devices_found = serial::list_ports();

  vector<serial::PortInfo>::iterator iter = devices_found.begin();

  while( iter != devices_found.end() )
  {
    serial::PortInfo device = *iter++;

    printf( "(%s, %s, %s)\n", device.port.c_str(), device.description.c_str(), device.hardware_id.c_str() );
  }
}

int Imu::run()
{
  std::string port("/dev/ttyS0");
  unsigned long baud = 9600;

  serial::Serial my_serial(port, baud, serial::Timeout::simpleTimeout(1000));

  cout << "Is the serial port open?";
  if(my_serial.isOpen())
    cout << " Yes." << endl;
  else
    cout << " No." << endl;

  // Get the Test string
  int count = 0;
  std::string test_string = "Test string";

  // Test the timeout, there should be 1 second between prints
  cout << "Timeout == 1000ms, asking for 1 more byte than written." << endl;
  while (count < 10) {
    size_t bytes_wrote = my_serial.write(test_string);

    string result = my_serial.read(test_string.length()+1);

    cout << "Iteration: " << count << ", Bytes written: ";
    cout << bytes_wrote << ", Bytes read: ";
    cout << result.length() << ", String read: " << result << endl;

    count += 1;
  }

  // Test the timeout at 250ms
  my_serial.setTimeout(serial::Timeout::max(), 250, 0, 250, 0);
  count = 0;
  cout << "Timeout == 250ms, asking for 1 more byte than written." << endl;
  while (count < 10) {
    size_t bytes_wrote = my_serial.write(test_string);

    string result = my_serial.read(test_string.length()+1);

    cout << "Iteration: " << count << ", Bytes written: ";
    cout << bytes_wrote << ", Bytes read: ";
    cout << result.length() << ", String read: " << result << endl;

    count += 1;
  }

  // Test the timeout at 250ms, but asking exactly for what was written
  count = 0;
  cout << "Timeout == 250ms, asking for exactly what was written." << endl;
  while (count < 10) {
    size_t bytes_wrote = my_serial.write(test_string);

    string result = my_serial.read(test_string.length());

    cout << "Iteration: " << count << ", Bytes written: ";
    cout << bytes_wrote << ", Bytes read: ";
    cout << result.length() << ", String read: " << result << endl;

    count += 1;
  }

  // Test the timeout at 250ms, but asking for 1 less than what was written
  count = 0;
  cout << "Timeout == 250ms, asking for 1 less than was written." << endl;
  while (count < 10) {
    size_t bytes_wrote = my_serial.write(test_string);

    string result = my_serial.read(test_string.length()-1);

    cout << "Iteration: " << count << ", Bytes written: ";
    cout << bytes_wrote << ", Bytes read: ";
    cout << result.length() << ", String read: " << result << endl;

    count += 1;
  }
}
