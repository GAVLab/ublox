#include <string>

#include "ros/ros.h"
#include "sensor_msgs/NavSatFix.h"

#include "ublox/ublox.h"

using std::string;

class UbloxNode
{
public:
  UbloxNode() : nh_("~"), port_("")
  {
    this->getROSParameters();
    this->configureROSCommunications();
  }
  ~UbloxNode() {

  }

  void run() {
    bool connect_result = false;
    try {
      connect_result = ublox_.Connect(port_, baudrate_);
    } catch (const std::exception &e) {
      ROS_ERROR_STREAM("Error connecting to the uBlox on port `"
                       << port_ << "`: " << e.what());
    }
  }

  void getROSParameters() {
    nh_.param<std::string>("port", port_, "/dev/ttyACM0");
    nh_.param<int>("baudrate", baudrate_, 115200);
  }

  void configureROSCommunications() {
    navsatfix_pub_ = nh_.advertise<sensor_msgs::NavSatFix>("fix", 1000);
  }

private:
  ros::NodeHandle nh_;
  string port_;
  int baudrate_;
  ros::Publisher navsatfix_pub_;

  ublox::Ublox ublox_;
};

int main (int argc, char **argv) {
  ros::init(argc, argv, "ublox_node");

  UbloxNode ublox_node;
  ublox_node.run();

  return 0;
}
