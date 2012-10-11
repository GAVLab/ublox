#include <string>

#include "ros/ros.h"
#include "sensor_msgs/NavSatFix.h"

#include "ublox/ublox.h"

using std::string;

class UbloxNode
{
public:
  UbloxNode() : nh_("~"), port_("") {
    this->getROSParameters();
    this->configureROSCommunications();
  }
  ~UbloxNode() {

  }

  void handle_NavPosLLH(NavPosLLH& nav_pos_llh, double& time_stamp) {
    sensor_msgs::NavSatFix msg;
    msg.header.stamp = ros::Time(time_stamp);
    msg.header.frame_id = "/ublox";

    // See: http://www.ros.org/doc/api/sensor_msgs/html/msg/NavSatFix.html
    msg.status.status = 0; // GPS_FIX
    msg.status.service = 0; // GPS

    msg.latitude = nav_pos_llh.latitude_scaled / 1e7f;
    msg.longitude = nav_pos_llh.longitude_scaled / 1e7f;
    msg.altitude = nav_pos_llh.height / 1000.0f;

    this->navsatfix_pub_.publish(msg);
  }

  void setup() {
    bool connect_result = false;
    try {
      connect_result = ublox_.Connect(port_, baudrate_);
    } catch (const std::exception &e) {
      ROS_ERROR_STREAM("Error connecting to the uBlox on port `"
                       << port_ << "`: " << e.what());
    }
    ublox_.set_nav_position_llh_callback(
      boost::bind(&UbloxNode::handle_NavPosLLH, this, _1, _2)
    ); 
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
  ublox_node.setup();

  ros::spin();

  return 0;
}
