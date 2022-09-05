#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Pose.h"
#include "orbit_publisher_pkg/Orbit.h"

#include <sstream>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "OrbitPositionPublisher");
  ros::NodeHandle n;
  ros::Publisher orbit_position_pub = n.advertise<geometry_msgs::Pose>("OrbitPosition", 1000);
  ros::Rate loop_rate(10);

  while (ros::ok())
  {
    geometry_msgs::Pose pos;
    
    pos.position.x = 0.0;
    pos.position.y = 0.0;
    pos.position.z = 0.0;
    pos.orientation.w = 0.50;
    pos.orientation.x = 0.50;
    pos.orientation.y = 0.50;
    pos.orientation.z = 0.50;
    orbit_position_pub.publish(pos);
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}