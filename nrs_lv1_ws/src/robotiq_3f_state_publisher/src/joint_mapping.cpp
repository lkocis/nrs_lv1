// Copyright 2026 UR5 ROS 2 maintainers

#include "robotiq_3f_state_publisher/joint_mapping.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <utility>

namespace robotiq_3f_state_publisher
{

PiecewiseLinearMap::PiecewiseLinearMap(
  std::vector<double> inputs, std::vector<double> outputs)
: inputs_(std::move(inputs)), outputs_(std::move(outputs))
{
  if (inputs_.size() < 2 || inputs_.size() != outputs_.size()) {
    throw std::invalid_argument(
            "mapping inputs and outputs must have the same size of at least two");
  }
  for (std::size_t index = 0; index < inputs_.size(); ++index) {
    if (!std::isfinite(inputs_[index]) || !std::isfinite(outputs_[index])) {
      throw std::invalid_argument("mapping values must be finite");
    }
    if (index > 0 && inputs_[index] <= inputs_[index - 1]) {
      throw std::invalid_argument("mapping inputs must be strictly increasing");
    }
  }
}

double PiecewiseLinearMap::evaluate(const double input) const
{
  if (input <= inputs_.front()) {
    return outputs_.front();
  }
  if (input >= inputs_.back()) {
    return outputs_.back();
  }

  const auto upper = std::upper_bound(inputs_.begin(), inputs_.end(), input);
  const auto upper_index = static_cast<std::size_t>(upper - inputs_.begin());
  const auto lower_index = upper_index - 1;
  const double fraction =
    (input - inputs_[lower_index]) /
    (inputs_[upper_index] - inputs_[lower_index]);
  return outputs_[lower_index] +
         fraction * (outputs_[upper_index] - outputs_[lower_index]);
}

GripperJointMapper::GripperJointMapper(
  FingerMaps finger_maps, PiecewiseLinearMap scissor_map)
: finger_maps_(finger_maps),
  pinch_finger_maps_(std::move(finger_maps)),
  scissor_map_(std::move(scissor_map))
{
}

GripperJointMapper::GripperJointMapper(
  FingerMaps finger_maps,
  FingerMaps pinch_finger_maps,
  PiecewiseLinearMap scissor_map)
: finger_maps_(std::move(finger_maps)),
  pinch_finger_maps_(std::move(pinch_finger_maps)),
  scissor_map_(std::move(scissor_map))
{
}

GripperJointMapper::JointPositions GripperJointMapper::map(
  const std::uint8_t position_a,
  const std::uint8_t position_b,
  const std::uint8_t position_c,
  const std::uint8_t position_scissor) const
{
  return map(0, position_a, position_b, position_c, position_scissor);
}

GripperJointMapper::JointPositions GripperJointMapper::map(
  const std::uint8_t mode,
  const std::uint8_t position_a,
  const std::uint8_t position_b,
  const std::uint8_t position_c,
  const std::uint8_t position_scissor) const
{
  // The gripper reports mode 1 for Pinch. Its calibrated finger curve diverges
  // from Basic/Wide only near the early physical travel limit.
  const FingerMaps & active_finger_maps = mode == 1 ? pinch_finger_maps_ : finger_maps_;
  const auto finger = [&active_finger_maps](const std::uint8_t position) {
      const double raw = static_cast<double>(position);
      return std::array<double, 3>{
      active_finger_maps.proximal.evaluate(raw),
      active_finger_maps.middle.evaluate(raw),
      active_finger_maps.distal.evaluate(raw),
      };
    };

  const auto a = finger(position_a);
  const auto b = finger(position_b);
  const auto c = finger(position_c);
  const double scissor =
    scissor_map_.evaluate(static_cast<double>(position_scissor));

  // Finger A is the fixed-base opposing finger. B and C are the two outer
  // fingers, whose scissor joints move by equal angles in opposite directions.
  return {
    a[0], a[1], a[2],
    scissor, b[0], b[1], b[2],
    -scissor, c[0], c[1], c[2],
  };
}

std::array<std::string, GripperJointMapper::kJointCount>
GripperJointMapper::joint_names(const std::string & prefix)
{
  return {
    prefix + "robotiq_3f_finger_a_proximal_joint",
    prefix + "robotiq_3f_finger_a_middle_joint",
    prefix + "robotiq_3f_finger_a_distal_joint",
    prefix + "robotiq_3f_finger_b_scissor_joint",
    prefix + "robotiq_3f_finger_b_proximal_joint",
    prefix + "robotiq_3f_finger_b_middle_joint",
    prefix + "robotiq_3f_finger_b_distal_joint",
    prefix + "robotiq_3f_finger_c_scissor_joint",
    prefix + "robotiq_3f_finger_c_proximal_joint",
    prefix + "robotiq_3f_finger_c_middle_joint",
    prefix + "robotiq_3f_finger_c_distal_joint",
  };
}

}  // namespace robotiq_3f_state_publisher
