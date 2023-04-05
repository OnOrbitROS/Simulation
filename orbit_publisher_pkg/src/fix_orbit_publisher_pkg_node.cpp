#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Pose.h"
#include "orbit_publisher_pkg/Orbit.h"

#include <sstream>

int main(int argc, char **argv)
{
  int rate;
  ros::init(argc, argv, "OrbitPositionPublisher");
  ros::NodeHandle n;
  std::string node_name = ros::this_node::getNamespace();
  ros::Publisher orbit_position_pub = n.advertise<geometry_msgs::Pose>("OrbitPosition", 1000);
  

  if (!n.getParam("/" + node_name + "/publish_rate",rate))
  {
      rate = 10; 
  }
  ros::Rate loop_rate(rate);

  geometry_msgs::Pose pos;

  if (!n.getParam("/" + node_name + "/pos_x",pos.position.x))
  {
    pos.position.x = 0.0;
  }
  if (!n.getParam("/" + node_name + "/pos_y",pos.position.y))
  {
    pos.position.y = 0.0;
  }
  if (!n.getParam("/" + node_name + "/pos_z",pos.position.z))
  {
    pos.position.z = 0.0;
  }
  if (!n.getParam("/" + node_name + "/orientation_x",pos.orientation.x))
  {
    pos.orientation.x = 0.0;
  }
  if (!n.getParam("/" + node_name + "/orientation_y",pos.orientation.y))
  {
    pos.orientation.y = 0.0;
  }
  if (!n.getParam("/" + node_name + "/orientation_z",pos.orientation.z))
  {
    pos.orientation.z = 0.0;
  }
  if (!n.getParam("/" + node_name + "/orientation_w",pos.orientation.w))
  {
    pos.orientation.w = 1.0;
  }

  while (ros::ok())
  {  
    orbit_position_pub.publish(pos);
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}