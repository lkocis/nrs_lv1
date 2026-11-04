// Copyright 2026 UR5 ROS 2 maintainers

#include <gtest/gtest.h>

#include <stdexcept>

#include "robotiq_3f_driver/protocol.hpp"
#include "robotiq_3f_interfaces/msg/robotiq3_f_command.hpp"

namespace
{

using robotiq_3f_driver::ProcessImage;
using robotiq_3f_interfaces::msg::Robotiq3FCommand;

TEST(Protocol, PacksCompleteLegacyProcessImage)
{
  Robotiq3FCommand command;
  command.activate = true;
  command.mode = Robotiq3FCommand::MODE_SCISSOR;
  command.go_to = true;
  command.automatic_release = true;
  command.glove_mode = true;
  command.individual_finger_control = true;
  command.individual_scissor_control = true;
  command.position_a = 1U;
  command.speed_a = 2U;
  command.force_a = 3U;
  command.position_b = 4U;
  command.speed_b = 5U;
  command.force_b = 6U;
  command.position_c = 7U;
  command.speed_c = 8U;
  command.force_c = 9U;
  command.position_scissor = 10U;
  command.speed_scissor = 11U;
  command.force_scissor = 12U;

  const ProcessImage expected{
    0x1FU, 0x0DU, 0x00U, 1U, 2U, 3U, 4U, 5U,
    6U, 7U, 8U, 9U, 10U, 11U, 12U, 0U};
  EXPECT_EQ(robotiq_3f_driver::pack_command(command), expected);
}

TEST(Protocol, RejectsInvalidMode)
{
  Robotiq3FCommand command;
  command.mode = 4U;
  EXPECT_THROW(robotiq_3f_driver::pack_command(command), std::invalid_argument);
}

TEST(Protocol, UnpacksCompleteLegacyStatusImage)
{
  const ProcessImage image{
    0xFDU, 0xE4U, 0x0AU, 1U, 2U, 3U, 4U, 5U,
    6U, 7U, 8U, 9U, 10U, 11U, 12U, 0U};
  const auto status = robotiq_3f_driver::unpack_status(image);

  EXPECT_TRUE(status.activated);
  EXPECT_EQ(status.mode, 2U);
  EXPECT_TRUE(status.go_to);
  EXPECT_EQ(status.initialization_status, 3U);
  EXPECT_EQ(status.motion_status, 3U);
  EXPECT_EQ(status.object_status_a, 0U);
  EXPECT_EQ(status.object_status_b, 1U);
  EXPECT_EQ(status.object_status_c, 2U);
  EXPECT_EQ(status.object_status_scissor, 3U);
  EXPECT_EQ(status.fault_status, 0x0AU);
  EXPECT_EQ(status.requested_position_a, 1U);
  EXPECT_EQ(status.actual_position_a, 2U);
  EXPECT_EQ(status.current_a, 3U);
  EXPECT_EQ(status.requested_position_b, 4U);
  EXPECT_EQ(status.actual_position_b, 5U);
  EXPECT_EQ(status.current_b, 6U);
  EXPECT_EQ(status.requested_position_c, 7U);
  EXPECT_EQ(status.actual_position_c, 8U);
  EXPECT_EQ(status.current_c, 9U);
  EXPECT_EQ(status.requested_position_scissor, 10U);
  EXPECT_EQ(status.actual_position_scissor, 11U);
  EXPECT_EQ(status.current_scissor, 12U);
}

}  // namespace
