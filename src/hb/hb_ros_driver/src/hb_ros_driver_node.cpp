/**
 * @file hb_ros_driver_node.cpp
 * @author Daniel Koch <daniel.p.koch@gmail.com>
 */

#include <ros/ros.h>
#include "hb_ros_driver/hb_ros_driver.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "hb_ros_driver");

  hb_ros_driver::HbROSDriver driver;
  ros::spin();

  return 0;
}
