// Copyright 2026 UR5 ROS 2 maintainers

#ifndef ROBOTIQ_3F_DRIVER__PROTOCOL_HPP_
#define ROBOTIQ_3F_DRIVER__PROTOCOL_HPP_

#include <array>
#include <cstdint>

#include "robotiq_3f_interfaces/msg/robotiq3_f_command.hpp"
#include "robotiq_3f_interfaces/msg/robotiq3_f_status.hpp"

namespace robotiq_3f_driver
{

constexpr std::size_t kProcessImageSize = 16;
using ProcessImage = std::array<std::uint8_t, kProcessImageSize>;

ProcessImage pack_command(
  const robotiq_3f_interfaces::msg::Robotiq3FCommand & command);

robotiq_3f_interfaces::msg::Robotiq3FStatus unpack_status(
  const ProcessImage & process_image);

}  // namespace robotiq_3f_driver

#endif  // ROBOTIQ_3F_DRIVER__PROTOCOL_HPP_
