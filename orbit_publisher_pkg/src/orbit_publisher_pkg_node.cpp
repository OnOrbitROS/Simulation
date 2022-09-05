#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Vector3.h"
#include "orbit_publisher_pkg/Orbit.h"

#include <sstream>

int main(int argc, char **argv)
{
  int rate; 
  ros::init(argc, argv, "OrbitPositionPublisher");
  ros::NodeHandle n;
  std::string node_name = ros::this_node::getName();
  ros::Publisher orbit_position_pub = n.advertise<geometry_msgs::Pose>("/"+ node_name +"/OrbitPosition", 1000);
  ros::Publisher orbit_velocity_pub = n.advertise<geometry_msgs::Vector3>("/" + node_name +"/OrbitVelocity", 1000);

  
  if (!n.getParam("/" + node_name + "/publish_rate",rate))
  {
      rate = 10; 
  }
  ros::Rate loop_rate(rate);

  Orbit orbital(n, node_name);
  orbital.SetName(node_name); 

  while (ros::ok())
  {
    geometry_msgs::Pose pos;
    geometry_msgs::Vector3 vel; 
    orbital.KeplerianToEci(ros::Time::now().toSec());
    Eigen::Quaternion<double> q; 
    q = orbital.GetLvLhRotationToEci();
    pos.position.x = orbital.GetPositionEciX();
    pos.position.y = orbital.GetPositionEciY();
    pos.position.z = orbital.GetPositionEciZ();
    pos.orientation.w = q.w();
    pos.orientation.x = q.x();
    pos.orientation.y = q.y();
    pos.orientation.z = q.z();
    orbit_position_pub.publish(pos);
    vel.x = orbital.GetVelocityEciX();
    vel.y = orbital.GetVelocityEciY();
    vel.z = orbital.GetVelocityEciZ();
    orbit_velocity_pub.publish(vel);
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}