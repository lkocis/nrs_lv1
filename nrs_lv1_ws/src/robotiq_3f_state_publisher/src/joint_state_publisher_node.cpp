// Copyright 2026 UR5 ROS 2 maintainers

#include "robotiq_3f_state_publisher/joint_mapping.hpp"

#include <chrono>
#include <cstdint>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "robotiq_3f_interfaces/msg/robotiq3_f_status.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

namespace robotiq_3f_state_publisher
{

using robotiq_3f_interfaces::msg::Robotiq3FStatus;
using sensor_msgs::msg::JointState;

std::vector<double> to_double(const std::vector<std::int64_t> & values)
{
  return {values.begin(), values.end()};
}

class JointStatePublisherNode : public rclcpp::Node
{
public:
  JointStatePublisherNode()
  : Node("joint_state_publisher")
  {
    const auto finger_breakpoints = to_double(declare_parameter<std::vector<std::int64_t>>(
      "finger.breakpoints", {0, 6, 32, 60, 96, 128, 148, 164, 192, 225, 240, 255}));
    const auto proximal = declare_parameter<std::vector<double>>(
      "finger.proximal_rad",
      {0.0, 0.0, 0.20943951023931956, 0.4886921905584123, 0.7853981633974483,
        1.0777408131064985, 1.0777408131064985, 1.0777408131064985,
        1.0777408131064985, 1.0777408131064985, 1.0777408131064985,
        1.0777408131064985});
    const auto middle = declare_parameter<std::vector<double>>(
      "finger.middle_rad",
      {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.08726646259971647, 0.4363323129985824,
        0.9337511498169663, 1.3089969389957472, 1.509709802975095, 1.509709802975095});
    const auto distal = declare_parameter<std::vector<double>>(
      "finger.distal_rad",
      {0.0, 0.0, -0.2792526803190927, -0.5148721293383273, -0.7853981633974483,
        -1.0777408131064985, -0.6894050545377601, -0.6894050545377601,
        -0.6894050545377601, -0.6894050545377601, -0.6894050545377601,
        -0.6894050545377601});

    const auto pinch_breakpoints = to_double(declare_parameter<std::vector<std::int64_t>>(
      "pinch_finger.breakpoints", {0, 6, 32, 60, 96, 112, 113, 255}));
    const auto pinch_proximal = declare_parameter<std::vector<double>>(
      "pinch_finger.proximal_rad",
      {0.0, 0.0, 0.20943951023931956, 0.4886921905584123, 0.7853981633974483,
        0.8552113334772214, 0.8552113334772214, 0.8552113334772214});
    const auto pinch_middle = declare_parameter<std::vector<double>>(
      "pinch_finger.middle_rad", {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
    const auto pinch_distal = declare_parameter<std::vector<double>>(
      "pinch_finger.distal_rad",
      {0.0, 0.0, -0.2792526803190927, -0.5148721293383273, -0.7853981633974483,
        -0.8552113334772214, -0.8552113334772214, -0.8552113334772214});

    const auto scissor_breakpoints = to_double(declare_parameter<std::vector<std::int64_t>>(
      "scissor.breakpoints", {0, 15, 24, 32, 60, 92, 128, 137, 164, 192, 220, 225, 233, 255}));
    const auto scissor = declare_parameter<std::vector<double>>(
      "scissor.angle_rad",
      {0.2399827721492203, 0.2399827721492203, 0.22252947962927702,
        0.20943951023931956, 0.14835298641951802, 0.08726646259971647,
        0.017453292519943295, 0.0, -0.05235987755982989, -0.10471975511965978,
        -0.16406094968746698, -0.1684242728174528, -0.17016960206944712,
        -0.17016960206944712});

    mapper_ = std::make_unique<GripperJointMapper>(
      FingerMaps{
        PiecewiseLinearMap(finger_breakpoints, proximal),
        PiecewiseLinearMap(finger_breakpoints, middle),
        PiecewiseLinearMap(finger_breakpoints, distal),
      },
      FingerMaps{
        PiecewiseLinearMap(pinch_breakpoints, pinch_proximal),
        PiecewiseLinearMap(pinch_breakpoints, pinch_middle),
        PiecewiseLinearMap(pinch_breakpoints, pinch_distal),
      },
      PiecewiseLinearMap(scissor_breakpoints, scissor));

    const auto names = GripperJointMapper::joint_names(
      declare_parameter<std::string>("tf_prefix", ""));
    joint_state_.name.assign(names.begin(), names.end());
    const auto reference = mapper_->map(0, 6, 6, 6, 137);
    joint_state_.position.assign(reference.begin(), reference.end());

    publish_reference_without_status_ = declare_parameter<bool>(
      "publish_reference_without_status", true);
    require_activated_status_ = declare_parameter<bool>(
      "require_activated_status", true);
    const auto status_topic = declare_parameter<std::string>(
      "status_topic", "/robotiq_3f/status");
    const auto joint_states_topic = declare_parameter<std::string>(
      "joint_states_topic", "/joint_states");
    const double publish_rate_hz = declare_parameter<double>("publish_rate_hz", 20.0);
    if (publish_rate_hz <= 0.0) {
      throw std::invalid_argument("publish_rate_hz must be greater than zero");
    }

    publisher_ = create_publisher<JointState>(joint_states_topic, rclcpp::QoS(10));
    subscription_ = create_subscription<Robotiq3FStatus>(
      status_topic, rclcpp::SensorDataQoS(),
      std::bind(&JointStatePublisherNode::status_callback, this, std::placeholders::_1));
    timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / publish_rate_hz),
      std::bind(&JointStatePublisherNode::publish, this));

    RCLCPP_WARN(
      get_logger(),
      "Using fitted Robotiq 3F AGS joint mapping from the 2026-09-04 physical sample set");
  }

private:
  void status_callback(const Robotiq3FStatus::SharedPtr status)
  {
    if (!status->connected || (require_activated_status_ && !status->activated)) {
      return;
    }
    const auto positions = mapper_->map(
      status->mode,
      status->actual_position_a,
      status->actual_position_b,
      status->actual_position_c,
      status->actual_position_scissor);
    joint_state_.position.assign(positions.begin(), positions.end());
    have_valid_status_ = true;
  }

  void publish()
  {
    if (!have_valid_status_ && !publish_reference_without_status_) {
      return;
    }
    joint_state_.header.stamp = now();
    publisher_->publish(joint_state_);
  }

  std::unique_ptr<GripperJointMapper> mapper_;
  JointState joint_state_;
  bool publish_reference_without_status_{true};
  bool require_activated_status_{true};
  bool have_valid_status_{false};
  rclcpp::Publisher<JointState>::SharedPtr publisher_;
  rclcpp::Subscription<Robotiq3FStatus>::SharedPtr subscription_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace robotiq_3f_state_publisher

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(
      std::make_shared<robotiq_3f_state_publisher::JointStatePublisherNode>());
  } catch (const std::exception & exception) {
    RCLCPP_FATAL(
      rclcpp::get_logger("robotiq_3f_joint_state_publisher"), "%s", exception.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
