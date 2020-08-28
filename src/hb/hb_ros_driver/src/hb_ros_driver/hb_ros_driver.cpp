/**
 * @file hb_ros_driver.cpp
 * @author Daniel Koch <daniel.p.koch@gmail.com>
 */

#include "hb_ros_driver/hb_ros_driver.h"

#include <sensor_msgs/Imu.h>
#include <std_msgs/Bool.h>

#include <string>

namespace hb_ros_driver
{

HbROSDriver::HbROSDriver() :
  nh_private_("~")
{
  std::string port;
  int baud;

  nh_private_.param<std::string>("port", port, "/dev/ttyUSB0");
  nh_private_.param<int>("baud", baud, 115200);

  serial_ = new async_comm::Serial(port, baud);
  serial_->register_receive_callback(std::bind(&HbROSDriver::serial_callback,
                                               this,
                                               std::placeholders::_1,
                                               std::placeholders::_2));

  encoder_pub_ = nh_.advertise<hb_msgs::State>("hb_state", 1);
  imu_pub_ = nh_.advertise<sensor_msgs::Imu>("imu/data", 1);
  arm_status_pub_ = nh_.advertise<std_msgs::Bool>("hb_arm_status", 1, true);

  command_sub_ = nh_.subscribe("hb_command", 1, &HbROSDriver::command_callback, this);

  zero_service_ = nh_.advertiseService("zero", &HbROSDriver::zero_service_callback, this);

  if (!serial_->init())
  {
    ROS_FATAL("Failed to open serial port %s", port.c_str());
    ros::shutdown();
  }
}

HbROSDriver::~HbROSDriver()
{
  serial_->close();
  delete serial_;
}

void HbROSDriver::serial_callback(const uint8_t *src, size_t len)
{
  hb_serial_message_t msg;
  for (size_t i = 0; i < len; i++)
  {
    if (hb_serial_parse_byte(src[i], &msg))
    {
      switch (msg.type)
      {
      case HB_SERIAL_MSG_ENCODER:
        handle_encoder_msg(msg);
        break;
      case HB_SERIAL_MSG_IMU:
        handle_imu_msg(msg);
        break;
      case HB_SERIAL_MSG_ARM_STATUS:
        handle_arm_status_msg(msg);
        break;
      case HB_SERIAL_MSG_ZERO:
        handle_zero_msg(msg);
        break;
      }
    }
  }
}

void HbROSDriver::handle_encoder_msg(const hb_serial_message_t &msg)
{
  hb_serial_encoder_msg_t encoder;
  hb_serial_encoder_msg_unpack(&encoder, &msg);

  hb_msgs::State out_msg;
  out_msg.yaw = encoder.yaw;
  out_msg.pitch = encoder.pitch;
  out_msg.roll = encoder.roll;

  encoder_pub_.publish(out_msg);
}

void HbROSDriver::handle_imu_msg(const hb_serial_message_t &msg)
{
  hb_serial_imu_msg_t imu;
  hb_serial_imu_msg_unpack(&imu, &msg);

  sensor_msgs::Imu out_msg;

  out_msg.angular_velocity.x = imu.gyro_x;
  out_msg.angular_velocity.y = imu.gyro_y;
  out_msg.angular_velocity.z = imu.gyro_z;

  out_msg.linear_acceleration.x = imu.accel_x;
  out_msg.linear_acceleration.y = imu.accel_y;
  out_msg.linear_acceleration.z = imu.accel_z;

  imu_pub_.publish(out_msg);
}

void HbROSDriver::handle_arm_status_msg(const hb_serial_message_t &msg)
{
  hb_serial_arm_status_msg_t arm;
  hb_serial_arm_status_msg_unpack(&arm, &msg);

  std_msgs::Bool out_msg;
  out_msg.data = arm.armed;
  arm_status_pub_.publish(out_msg);

  ROS_WARN("Hummingbird is now %s", arm.armed ? "ARMED" : "DISARMED");
}

void HbROSDriver::handle_zero_msg(const hb_serial_message_t &msg)
{
  hb_serial_zero_msg_t zero;
  hb_serial_zero_msg_unpack(&zero, &msg);

  if (zero.success)
  {
    ROS_WARN("Successfully zeroed encoders. Power cycle the hummingbird for changes to take effect!");
  }
  else
  {
    ROS_ERROR("Failed to zero encoders!");
  }
}

void HbROSDriver::command_callback(const hb_msgs::Command& msg)
{
  hb_serial_setpoint_msg_t setpoint;
  setpoint.left = msg.left_motor;
  setpoint.right = msg.right_motor;

  uint8_t buf[HB_SERIAL_MAX_MESSAGE_LEN];
  size_t len = hb_serial_setpoint_msg_send_to_buffer(buf, &setpoint);
  serial_->send_bytes(buf, len);
}

bool HbROSDriver::zero_service_callback(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res)
{
  hb_serial_zero_msg_t zero;

  uint8_t buf[HB_SERIAL_MAX_MESSAGE_LEN];
  size_t len = hb_serial_zero_msg_send_to_buffer(buf, &zero);
  serial_->send_bytes(buf, len);

  return true;
}


} // namespace hb_ros_driver
