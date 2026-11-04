// Copyright 2026 UR5 ROS 2 maintainers

#include <cmath>
#include <stdexcept>
#include <string>
#include <vector>

#include "gtest/gtest.h"
#include "robotiq_3f_state_publisher/joint_mapping.hpp"

namespace
{

using robotiq_3f_state_publisher::FingerMaps;
using robotiq_3f_state_publisher::GripperJointMapper;
using robotiq_3f_state_publisher::PiecewiseLinearMap;

GripperJointMapper make_mapper()
{
  const std::vector<double> finger_raw{6.0, 140.0, 240.0, 255.0};
  return GripperJointMapper(
    FingerMaps{
      PiecewiseLinearMap(finger_raw, {0.0, 1.2, 1.2, 1.2}),
      PiecewiseLinearMap(finger_raw, {0.0, 0.0, 1.5, 1.5}),
      PiecewiseLinearMap(finger_raw, {0.0, -1.2, -0.9, -0.9}),
    },
    FingerMaps{
      PiecewiseLinearMap(finger_raw, {0.0, 0.8, 0.8, 0.8}),
      PiecewiseLinearMap(finger_raw, {0.0, 0.0, 0.0, 0.0}),
      PiecewiseLinearMap(finger_raw, {0.0, -0.8, -0.8, -0.8}),
    },
    PiecewiseLinearMap({0.0, 137.0, 255.0}, {0.2, 0.0, -0.18}));
}

TEST(PiecewiseLinearMap, interpolates_and_clamps)
{
  const PiecewiseLinearMap mapping({0.0, 10.0, 20.0}, {0.0, 2.0, 3.0});
  EXPECT_DOUBLE_EQ(mapping.evaluate(-1.0), 0.0);
  EXPECT_DOUBLE_EQ(mapping.evaluate(5.0), 1.0);
  EXPECT_DOUBLE_EQ(mapping.evaluate(15.0), 2.5);
  EXPECT_DOUBLE_EQ(mapping.evaluate(21.0), 3.0);
}

TEST(PiecewiseLinearMap, rejects_invalid_tables)
{
  EXPECT_THROW(PiecewiseLinearMap({0.0}, {0.0}), std::invalid_argument);
  EXPECT_THROW(PiecewiseLinearMap({0.0, 1.0}, {0.0}), std::invalid_argument);
  EXPECT_THROW(PiecewiseLinearMap({0.0, 0.0}, {0.0, 1.0}), std::invalid_argument);
}

TEST(GripperJointMapper, basic_open_reference_is_zero)
{
  const auto joints = make_mapper().map(6, 6, 6, 137);
  for (const double value : joints) {
    EXPECT_DOUBLE_EQ(value, 0.0);
  }
}

TEST(GripperJointMapper, maps_each_finger_and_opposes_scissor_joints)
{
  const auto joints = make_mapper().map(73, 140, 240, 0);
  EXPECT_DOUBLE_EQ(joints[0], 0.6);
  EXPECT_DOUBLE_EQ(joints[1], 0.0);
  EXPECT_DOUBLE_EQ(joints[2], -0.6);
  EXPECT_DOUBLE_EQ(joints[3], 0.2);
  EXPECT_DOUBLE_EQ(joints[4], 1.2);
  EXPECT_DOUBLE_EQ(joints[7], -0.2);
  EXPECT_DOUBLE_EQ(joints[8], 1.2);
  EXPECT_DOUBLE_EQ(joints[9], 1.5);
  EXPECT_DOUBLE_EQ(joints[10], -0.9);
}

TEST(GripperJointMapper, pinch_mode_selects_its_measured_finger_curve)
{
  const auto basic = make_mapper().map(0, 140, 140, 140, 137);
  const auto pinch = make_mapper().map(1, 140, 140, 140, 137);
  EXPECT_DOUBLE_EQ(basic[0], 1.2);
  EXPECT_DOUBLE_EQ(basic[2], -1.2);
  EXPECT_DOUBLE_EQ(pinch[0], 0.8);
  EXPECT_DOUBLE_EQ(pinch[1], 0.0);
  EXPECT_DOUBLE_EQ(pinch[2], -0.8);
  EXPECT_DOUBLE_EQ(pinch[4], 0.8);
  EXPECT_DOUBLE_EQ(pinch[8], 0.8);
}

TEST(GripperJointMapper, prefixes_all_joint_names)
{
  const auto names = GripperJointMapper::joint_names("cell_");
  ASSERT_EQ(names.size(), GripperJointMapper::kJointCount);
  for (const auto & name : names) {
    EXPECT_EQ(name.rfind("cell_", 0), 0U);
  }
  EXPECT_EQ(names.front(), "cell_robotiq_3f_finger_a_proximal_joint");
  EXPECT_EQ(names.back(), "cell_robotiq_3f_finger_c_distal_joint");
}

}  // namespace
