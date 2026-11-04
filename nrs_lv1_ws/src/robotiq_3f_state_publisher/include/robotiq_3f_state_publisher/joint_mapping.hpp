// Copyright 2026 UR5 ROS 2 maintainers

#ifndef ROBOTIQ_3F_STATE_PUBLISHER__JOINT_MAPPING_HPP_
#define ROBOTIQ_3F_STATE_PUBLISHER__JOINT_MAPPING_HPP_

#include <array>
#include <cstdint>
#include <string>
#include <vector>

namespace robotiq_3f_state_publisher
{

class PiecewiseLinearMap
{
public:
  PiecewiseLinearMap(std::vector<double> inputs, std::vector<double> outputs);

  [[nodiscard]] double evaluate(double input) const;

private:
  std::vector<double> inputs_;
  std::vector<double> outputs_;
};

struct FingerMaps
{
  PiecewiseLinearMap proximal;
  PiecewiseLinearMap middle;
  PiecewiseLinearMap distal;
};

class GripperJointMapper
{
public:
  static constexpr std::size_t kJointCount = 11;
  using JointPositions = std::array<double, kJointCount>;

  GripperJointMapper(FingerMaps finger_maps, PiecewiseLinearMap scissor_map);
  GripperJointMapper(
    FingerMaps finger_maps,
    FingerMaps pinch_finger_maps,
    PiecewiseLinearMap scissor_map);

  [[nodiscard]] JointPositions map(
    std::uint8_t position_a,
    std::uint8_t position_b,
    std::uint8_t position_c,
    std::uint8_t position_scissor) const;

  [[nodiscard]] JointPositions map(
    std::uint8_t mode,
    std::uint8_t position_a,
    std::uint8_t position_b,
    std::uint8_t position_c,
    std::uint8_t position_scissor) const;

  [[nodiscard]] static std::array<std::string, kJointCount> joint_names(
    const std::string & prefix);

private:
  FingerMaps finger_maps_;
  FingerMaps pinch_finger_maps_;
  PiecewiseLinearMap scissor_map_;
};

}  // namespace robotiq_3f_state_publisher

#endif  // ROBOTIQ_3F_STATE_PUBLISHER__JOINT_MAPPING_HPP_
