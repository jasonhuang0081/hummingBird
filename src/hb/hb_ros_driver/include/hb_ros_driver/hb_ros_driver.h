/**
 * @file hb_ros_driver.h
 * @author Daniel Koch <daniel.p.koch@gmail.com>
 */

#ifndef HB_ROS_DRIVER_HB_ROS_DRIVER_H
#define HB_ROS_DRIVER_HB_ROS_DRIVER_H

#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <hb_msgs/Command.h>
#include <hb_msgs/State.h>

#include <async_comm/serial.h>

#include "hb_serial_protocol/hb_serial.h"

namespace hb_ros_driver
{

class HbROSDriver
{
public:
  HbROSDriver();
  ~HbROSDriver();

private:
  void serial_callback(const uint8_t *src, size_t len);
  void handle_encoder_msg(const hb_serial_message_t &msg);
  void handle_imu_msg(const hb_serial_message_t &msg);
  void handle_arm_status_msg(const hb_serial_message_t &msg);
  void handle_zero_msg(const hb_serial_message_t &msg);

  void command_callback(const hb_msgs::Command& msg);

  bool zero_service_callback(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res);

  async_comm::Serial *serial_;

  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;

  ros::Publisher encoder_pub_;
  ros::Publisher imu_pub_;
  ros::Publisher arm_status_pub_;

  ros::Subscriber command_sub_;

  ros::ServiceServer zero_service_;
};

} // namespace hb_ros_driver

#endif // HB_ROS_DRIVER_HB_ROS_DRIVER_H
