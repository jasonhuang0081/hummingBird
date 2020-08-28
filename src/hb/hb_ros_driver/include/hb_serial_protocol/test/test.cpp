#include <gtest/gtest.h>
#include <cstdint>

#include "hb_serial.h"


TEST(Core, MaxPayloadLength)
{
  size_t max_length = 0;
  for (uint8_t i = 0; i < HB_SERIAL_NUM_MSGS; i++)
  {
    if (HB_SERIAL_PAYLOAD_LEN[i] > max_length)
    {
      max_length = HB_SERIAL_PAYLOAD_LEN[i];
    }
  }

  ASSERT_EQ(max_length, HB_SERIAL_MAX_PAYLOAD_LEN);
}


bool test_parsing(hb_serial_message_t *dst, const uint8_t *src, size_t num_bytes)
{
  bool message_received = false;
  for (size_t i = 0; i < num_bytes; i++)
  {
    message_received = hb_serial_parse_byte(src[i], dst);
    if (i < num_bytes-1 && message_received)
    {
      return false;
    }
  }
  return message_received;
}


TEST(Messages, Encoders)
{
  // original message
  hb_serial_encoder_msg_t original;
  original.yaw = 1.23f;
  original.pitch = 4.56f;
  original.roll = 7.89f;

  // send to buffer
  uint8_t buf[HB_SERIAL_MAX_MESSAGE_LEN];
  size_t num_bytes = hb_serial_encoder_msg_send_to_buffer(buf, &original);

  // parse buffer
  hb_serial_message_t recv_msg;
  bool message_received = test_parsing(&recv_msg, buf, num_bytes);
  ASSERT_TRUE(message_received);

  // unpack and compare to original message
  hb_serial_encoder_msg_t received;
  hb_serial_encoder_msg_unpack(&received, &recv_msg);

  EXPECT_EQ(original.yaw, received.yaw);
  EXPECT_EQ(original.pitch, received.pitch);
  EXPECT_EQ(original.roll, received.roll);
}


TEST(Messages, Imu)
{
  // original message
  hb_serial_imu_msg_t original;
  original.accel_x = 1.23f;
  original.accel_y = 2.34f;
  original.accel_z = 3.45f;
  original.gyro_x = 4.56f;
  original.gyro_y = 5.67f;
  original.gyro_z = 6.78f;

  // send to buffer
  uint8_t buf[HB_SERIAL_MAX_MESSAGE_LEN];
  size_t num_bytes = hb_serial_imu_msg_send_to_buffer(buf, &original);

  // parse buffer
  hb_serial_message_t recv_msg;
  bool message_received = test_parsing(&recv_msg, buf, num_bytes);
  ASSERT_TRUE(message_received);

  // unpack and compare to original message
  hb_serial_imu_msg_t received;
  hb_serial_imu_msg_unpack(&received, &recv_msg);

  EXPECT_EQ(original.accel_x, received.accel_x);
  EXPECT_EQ(original.accel_y, received.accel_y);
  EXPECT_EQ(original.accel_z, received.accel_z);
  EXPECT_EQ(original.gyro_x, received.gyro_x);
  EXPECT_EQ(original.gyro_y, received.gyro_y);
  EXPECT_EQ(original.gyro_z, received.gyro_z);
}


TEST(Messages, Setpoint)
{
  // original message
  hb_serial_setpoint_msg_t original;
  original.left = -0.56f;
  original.right = 0.67f;

  // send to buffer
  uint8_t buf[HB_SERIAL_MAX_MESSAGE_LEN];
  size_t num_bytes = hb_serial_setpoint_msg_send_to_buffer(buf, &original);

  // parse buffer
  hb_serial_message_t recv_msg;
  bool message_received = test_parsing(&recv_msg, buf, num_bytes);
  ASSERT_TRUE(message_received);

  // unpack and compare to original message
  hb_serial_setpoint_msg_t received;
  hb_serial_setpoint_msg_unpack(&received, &recv_msg);

  EXPECT_EQ(original.left, received.left);
  EXPECT_EQ(original.right, received.right);
}


TEST(Messages, Arm)
{
  // original message
  hb_serial_arm_status_msg_t original;
  original.armed = false;

  // send to buffer
  uint8_t buf[HB_SERIAL_MAX_MESSAGE_LEN];
  size_t num_bytes = hb_serial_arm_status_msg_send_to_buffer(buf, &original);

  // parse buffer
  hb_serial_message_t recv_msg;
  bool message_received = test_parsing(&recv_msg, buf, num_bytes);
  ASSERT_TRUE(message_received);

  // unpack and compare to original message
  hb_serial_arm_status_msg_t received;
  hb_serial_arm_status_msg_unpack(&received, &recv_msg);

  EXPECT_EQ(original.armed, received.armed);
}


TEST(Messages, Zero)
{
  // original message
  hb_serial_zero_msg_t original;
  original.success = false;

  // send to buffer
  uint8_t buf[HB_SERIAL_MAX_MESSAGE_LEN];
  size_t num_bytes = hb_serial_zero_msg_send_to_buffer(buf, &original);

  // parse buffer
  hb_serial_message_t recv_msg;
  bool message_received = test_parsing(&recv_msg, buf, num_bytes);
  ASSERT_TRUE(message_received);

  // unpack and compare to original message
  hb_serial_zero_msg_t received;
  hb_serial_zero_msg_unpack(&received, &recv_msg);

  EXPECT_EQ(original.success, received.success);
}


int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
