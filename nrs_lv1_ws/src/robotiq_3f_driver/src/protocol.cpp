// Copyright 2012 Robotiq, Inc.
// Copyright 2026 UR5 ROS 2 maintainers
//
// The process-image mapping follows Robotiq's BSD-licensed ROS 1 3F driver
// and the Robotiq 3-Finger Adaptive Gripper instruction manual.

#include "robotiq_3f_driver/protocol.hpp"

#include <stdexcept>

namespace robotiq_3f_driver
{

ProcessImage pack_command(
  const robotiq_3f_interfaces::msg::Robotiq3FCommand & command)
{
  if (command.mode > 3U) {
    throw std::invalid_argument("Robotiq 3F mode must be in the range 0..3");
  }

  ProcessImage image{};
  image[0] = static_cast<std::uint8_t>(
    (command.activate ? 0x01U : 0x00U) |
    ((command.mode & 0x03U) << 1U) |
    (command.go_to ? 0x08U : 0x00U) |
    (command.automatic_release ? 0x10U : 0x00U));
  image[1] = static_cast<std::uint8_t>(
    (command.glove_mode ? 0x01U : 0x00U) |
    (command.individual_finger_control ? 0x04U : 0x00U) |
    (command.individual_scissor_control ? 0x08U : 0x00U));
  image[2] = 0U;
  image[3] = command.position_a;
  image[4] = command.speed_a;
  image[5] = command.force_a;
  image[6] = command.position_b;
  image[7] = command.speed_b;
  image[8] = command.force_b;
  image[9] = command.position_c;
  image[10] = command.speed_c;
  image[11] = command.force_c;
  image[12] = command.position_scissor;
  image[13] = command.speed_scissor;
  image[14] = command.force_scissor;
  image[15] = 0U;
  return image;
}

robotiq_3f_interfaces::msg::Robotiq3FStatus unpack_status(
  const ProcessImage & image)
{
  robotiq_3f_interfaces::msg::Robotiq3FStatus status;
  status.activated = (image[0] & 0x01U) != 0U;
  status.mode = static_cast<std::uint8_t>((image[0] >> 1U) & 0x03U);
  status.go_to = (image[0] & 0x08U) != 0U;
  status.initialization_status =
    static_cast<std::uint8_t>((image[0] >> 4U) & 0x03U);
  status.motion_status =
    static_cast<std::uint8_t>((image[0] >> 6U) & 0x03U);
  status.object_status_a = static_cast<std::uint8_t>(image[1] & 0x03U);
  status.object_status_b =
    static_cast<std::uint8_t>((image[1] >> 2U) & 0x03U);
  status.object_status_c =
    static_cast<std::uint8_t>((image[1] >> 4U) & 0x03U);
  status.object_status_scissor =
    static_cast<std::uint8_t>((image[1] >> 6U) & 0x03U);
  status.fault_status = image[2];
  status.requested_position_a = image[3];
  status.actual_position_a = image[4];
  status.current_a = image[5];
  status.requested_position_b = image[6];
  status.actual_position_b = image[7];
  status.current_b = image[8];
  status.requested_position_c = image[9];
  status.actual_position_c = image[10];
  status.current_c = image[11];
  status.requested_position_scissor = image[12];
  status.actual_position_scissor = image[13];
  status.current_scissor = image[14];
  return status;
}

}  // namespace robotiq_3f_driver
