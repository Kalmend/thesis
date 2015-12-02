#include <ros/ros.h>

#include "demo_class.h"

int main(int argc, char** argv)
{
  // An example ROS node   
  ros::init(argc, argv, "my_project_node");
  demo_namespace::DemoClass* demo = new demo_namespace::DemoClass();
  ros::Rate r(5);  // 5 Hz
  while (ros::ok())
  {
    ros::spinOnce();
    demo->demo_function();
    r.sleep();
  }
  return 0;
}
