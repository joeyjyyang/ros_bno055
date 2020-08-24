#include "bno055_driver.h"

#include <ros/ros.h>

namespace bno055_driver
{
class Bno055Node
{
public:
	Bno055Node(const ros::NodeHandle& nh) : nh_(nh)
  {}
	~Bno055Node();
private:
	ros::NodeHandle nh_;
};
} // namespace bno055_driver

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "bno055_node");
  ros::NodeHandle nh("~");
  bno055_driver::Bno055Node bno055_node(nh);
  ros::spin();

  return 0;
}
