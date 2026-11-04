// Copyright 2026 UR5 ROS 2 maintainers

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>

#include "control_msgs/action/gripper_command.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "robotiq_3f_interfaces/action/move.hpp"
#include "robotiq_3f_interfaces/msg/robotiq3_f_command.hpp"
#include "robotiq_3f_interfaces/msg/robotiq3_f_status.hpp"
#include "std_srvs/srv/trigger.hpp"

namespace robotiq_3f_controller
{

using Command = robotiq_3f_interfaces::msg::Robotiq3FCommand;
using Status = robotiq_3f_interfaces::msg::Robotiq3FStatus;
using Move = robotiq_3f_interfaces::action::Move;
using GripperCommand = control_msgs::action::GripperCommand;
using GoalHandleMove = rclcpp_action::ServerGoalHandle<Move>;
using GoalHandleGripperCommand = rclcpp_action::ServerGoalHandle<GripperCommand>;
using Trigger = std_srvs::srv::Trigger;
using SteadyClock = std::chrono::steady_clock;

class BusyGuard
{
public:
  explicit BusyGuard(std::atomic<bool> & busy)
  : busy_(busy) {}
  ~BusyGuard() {busy_.store(false);}

private:
  std::atomic<bool> & busy_;
};

class Robotiq3FControllerNode : public rclcpp::Node
{
public:
  Robotiq3FControllerNode()
  : Node("robotiq_3f_controller")
  {
    activation_timeout_ = std::chrono::duration_cast<SteadyClock::duration>(
      std::chrono::duration<double>(
        declare_parameter<double>("activation_timeout_s", 30.0)));
    default_move_timeout_ = std::chrono::duration_cast<SteadyClock::duration>(
      std::chrono::duration<double>(
        declare_parameter<double>("default_move_timeout_s", 15.0)));
    status_timeout_ = std::chrono::duration_cast<SteadyClock::duration>(
      std::chrono::duration<double>(
        declare_parameter<double>("status_timeout_s", 0.5)));
    activation_mode_ = declare_parameter<int>("activation_mode", Command::MODE_BASIC);
    gripper_command_closed_position_m_ = declare_parameter<double>(
      "gripper_command_closed_position_m", 0.0);
    gripper_command_max_opening_m_ = declare_parameter<double>(
      "gripper_command_max_opening_m", 0.155);
    gripper_command_closed_raw_ = declare_parameter<int>("gripper_command_closed_raw", 112);
    gripper_command_min_effort_n_ = declare_parameter<double>(
      "gripper_command_min_effort_n", 15.0);
    gripper_command_max_effort_n_ = declare_parameter<double>(
      "gripper_command_max_effort_n", 60.0);
    gripper_command_default_effort_n_ = declare_parameter<double>(
      "gripper_command_default_effort_n", 30.0);
    gripper_command_speed_ = declare_parameter<int>("gripper_command_speed", 20);
    if (activation_timeout_.count() <= 0.0 || default_move_timeout_.count() <= 0.0 ||
      status_timeout_.count() <= 0.0 || activation_mode_ < 0 || activation_mode_ > 3)
    {
      throw std::invalid_argument("invalid Robotiq 3F controller parameter");
    }
    if (!std::isfinite(gripper_command_closed_position_m_) ||
      !std::isfinite(gripper_command_max_opening_m_) ||
      gripper_command_closed_position_m_ < 0.0 ||
      gripper_command_max_opening_m_ <= gripper_command_closed_position_m_ ||
      !std::isfinite(gripper_command_min_effort_n_) ||
      !std::isfinite(gripper_command_max_effort_n_) ||
      !std::isfinite(gripper_command_default_effort_n_) ||
      gripper_command_min_effort_n_ < 0.0 ||
      gripper_command_max_effort_n_ <= gripper_command_min_effort_n_ ||
      gripper_command_default_effort_n_ < gripper_command_min_effort_n_ ||
      gripper_command_default_effort_n_ > gripper_command_max_effort_n_ ||
      gripper_command_closed_raw_ < 1 || gripper_command_closed_raw_ > 255 ||
      gripper_command_speed_ < 0 || gripper_command_speed_ > 255)
    {
      throw std::invalid_argument("invalid GripperCommand compatibility parameter");
    }

    status_group_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    operation_group_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);

    backend_command_publisher_ =
      create_publisher<Command>("backend/command", rclcpp::QoS(10).reliable());
    public_status_publisher_ =
      create_publisher<Status>("status", rclcpp::QoS(10).reliable());

    rclcpp::SubscriptionOptions subscription_options;
    subscription_options.callback_group = status_group_;
    backend_status_subscription_ = create_subscription<Status>(
      "backend/status", rclcpp::QoS(10).reliable(),
      [this](Status::ConstSharedPtr status) {receive_status(*status);},
      subscription_options);

    activate_service_ = create_service<Trigger>(
      "activate",
      std::bind(
        &Robotiq3FControllerNode::activate, this,
        std::placeholders::_1, std::placeholders::_2),
      rclcpp::ServicesQoS(), operation_group_);
    deactivate_service_ = create_service<Trigger>(
      "deactivate",
      std::bind(
        &Robotiq3FControllerNode::deactivate, this,
        std::placeholders::_1, std::placeholders::_2),
      rclcpp::ServicesQoS(), operation_group_);
    stop_service_ = create_service<Trigger>(
      "stop",
      std::bind(
        &Robotiq3FControllerNode::stop, this,
        std::placeholders::_1, std::placeholders::_2),
      rclcpp::ServicesQoS(), operation_group_);

    move_server_ = rclcpp_action::create_server<Move>(
      this, "move",
      std::bind(
        &Robotiq3FControllerNode::handle_move_goal, this,
        std::placeholders::_1, std::placeholders::_2),
      std::bind(
        &Robotiq3FControllerNode::handle_move_cancel, this,
        std::placeholders::_1),
      std::bind(
        &Robotiq3FControllerNode::handle_move_accepted, this,
        std::placeholders::_1),
      rcl_action_server_get_default_options(), operation_group_);
    gripper_command_server_ = rclcpp_action::create_server<GripperCommand>(
      this, "gripper_command",
      std::bind(
        &Robotiq3FControllerNode::handle_gripper_command_goal, this,
        std::placeholders::_1, std::placeholders::_2),
      std::bind(
        &Robotiq3FControllerNode::handle_gripper_command_cancel, this,
        std::placeholders::_1),
      std::bind(
        &Robotiq3FControllerNode::handle_gripper_command_accepted, this,
        std::placeholders::_1),
      rcl_action_server_get_default_options(), operation_group_);

    RCLCPP_INFO(get_logger(), "Controller ready. Waiting for requests");
  }

  ~Robotiq3FControllerNode() override
  {
    shutting_down_.store(true);
    status_condition_.notify_all();
    std::lock_guard<std::mutex> lock(action_thread_mutex_);
    if (action_thread_.joinable()) {
      action_thread_.join();
    }
  }

private:
  void receive_status(const Status & status)
  {
    {
      std::lock_guard<std::mutex> lock(status_mutex_);
      current_status_ = status;
      last_status_time_ = SteadyClock::now();
      have_status_ = true;
    }
    public_status_publisher_->publish(status);
    status_condition_.notify_all();
  }

  bool current_status(Status & status, std::string & reason) const
  {
    std::lock_guard<std::mutex> lock(status_mutex_);
    if (!have_status_) {
      reason = "no gripper status has been received";
      return false;
    }
    if (!current_status_.connected) {
      reason = current_status_.diagnostic.empty() ?
        "gripper backend is disconnected" : current_status_.diagnostic;
      return false;
    }
    if (SteadyClock::now() - last_status_time_ > status_timeout_) {
      reason = "gripper status is stale";
      return false;
    }
    status = current_status_;
    return true;
  }

  std::uint64_t publish_command(Command command)
  {
    command.command_id = next_command_id_.fetch_add(1U);
    {
      std::lock_guard<std::mutex> lock(command_mutex_);
      last_command_ = command;
      have_command_ = true;
    }
    backend_command_publisher_->publish(command);
    return command.command_id;
  }

  bool wait_for_status(
    const std::function<bool(const Status &)> & predicate,
    SteadyClock::time_point deadline, std::uint64_t connection_id,
    Status & final_status, std::string & reason,
    const std::function<bool()> & cancelled = [] () {return false;},
    const std::function<void(const Status &)> & progress = {})
  {
    std::unique_lock<std::mutex> lock(status_mutex_);
    while (!shutting_down_.load()) {
      if (cancelled()) {
        reason = "request cancelled";
        return false;
      }
      if (have_status_) {
        final_status = current_status_;
        if (progress) {
          progress(current_status_);
        }
        if (!current_status_.connected) {
          reason = current_status_.diagnostic.empty() ?
            "gripper disconnected" : current_status_.diagnostic;
          return false;
        }
        if (current_status_.connection_id != connection_id) {
          reason = "gripper reconnected. Request not replayed";
          return false;
        }
        if (SteadyClock::now() - last_status_time_ > status_timeout_) {
          reason = "gripper status is stale";
          return false;
        }
        if (predicate(current_status_)) {
          final_status = current_status_;
          return true;
        }
      }
      if (SteadyClock::now() >= deadline) {
        reason = "request timed out";
        return false;
      }
      status_condition_.wait_for(lock, std::chrono::milliseconds(50));
    }
    reason = "controller is shutting down";
    return false;
  }

  bool begin_operation(std::string & reason)
  {
    if (busy_.exchange(true)) {
      reason = "another gripper operation is active";
      return false;
    }
    return true;
  }

  void activate(
    const std::shared_ptr<Trigger::Request>,
    std::shared_ptr<Trigger::Response> response)
  {
    std::string reason;
    if (!begin_operation(reason)) {
      response->message = reason;
      return;
    }
    BusyGuard guard(busy_);

    Status status;
    if (!current_status(status, reason)) {
      response->message = reason;
      return;
    }
    const auto connection_id = status.connection_id;
    Command reset;
    const auto reset_command_id = publish_command(reset);
    if (!wait_for_status(
        [reset_command_id](const Status & value) {
          return value.last_command_id == reset_command_id && !value.activated &&
                 value.initialization_status == Status::INITIALIZATION_RESET;
        }, SteadyClock::now() + activation_timeout_, connection_id,
        status, reason))
    {
      response->message = "reset before activation failed: " + reason;
      return;
    }

    Command command;
    command.activate = true;
    command.mode = static_cast<std::uint8_t>(activation_mode_);
    const auto activation_command_id = publish_command(command);
    if (!wait_for_status(
        [this, activation_command_id](const Status & value) {
          return value.last_command_id == activation_command_id && value.activated &&
                 value.initialization_status == Status::INITIALIZATION_READY &&
                 value.mode == static_cast<std::uint8_t>(activation_mode_);
        }, SteadyClock::now() + activation_timeout_, connection_id,
        status, reason))
    {
      response->message = "activation failed: " + reason;
      return;
    }

    response->success = true;
    response->message = "gripper activated and ready";
  }

  void deactivate(
    const std::shared_ptr<Trigger::Request>,
    std::shared_ptr<Trigger::Response> response)
  {
    std::string reason;
    if (!begin_operation(reason)) {
      response->message = reason;
      return;
    }
    BusyGuard guard(busy_);
    Status status;
    if (!current_status(status, reason)) {
      response->message = reason;
      return;
    }
    const auto connection_id = status.connection_id;
    const auto command_id = publish_command(Command{});
    if (!wait_for_status(
        [command_id](const Status & value) {
          return value.last_command_id == command_id && !value.activated;
        },
        SteadyClock::now() + activation_timeout_, connection_id,
        status, reason))
    {
      response->message = "deactivation failed: " + reason;
      return;
    }
    response->success = true;
    response->message = "gripper deactivated";
  }

  void stop(
    const std::shared_ptr<Trigger::Request>,
    std::shared_ptr<Trigger::Response> response)
  {
    Status status;
    std::string reason;
    if (!current_status(status, reason)) {
      response->message = reason;
      return;
    }
    Command command;
    {
      std::lock_guard<std::mutex> lock(command_mutex_);
      if (have_command_) {
        command = last_command_;
      } else {
        command.activate = status.activated;
        command.mode = status.mode;
      }
    }
    command.go_to = false;
    command.automatic_release = false;
    publish_command(command);
    response->success = true;
    response->message = "stop command sent";
  }

  rclcpp_action::GoalResponse handle_move_goal(
    const rclcpp_action::GoalUUID &,
    std::shared_ptr<const Move::Goal> goal)
  {
    if (goal->mode > Command::MODE_SCISSOR) {
      RCLCPP_WARN(get_logger(), "Rejecting move with invalid mode %u", goal->mode);
      return rclcpp_action::GoalResponse::REJECT;
    }
    if (busy_.exchange(true)) {
      RCLCPP_WARN(get_logger(), "Rejecting move because another operation is active");
      return rclcpp_action::GoalResponse::REJECT;
    }
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_move_cancel(
    const std::shared_ptr<GoalHandleMove>)
  {
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_move_accepted(const std::shared_ptr<GoalHandleMove> goal_handle)
  {
    std::lock_guard<std::mutex> lock(action_thread_mutex_);
    if (action_thread_.joinable()) {
      action_thread_.join();
    }
    action_thread_ = std::thread([this, goal_handle]() {execute_move(goal_handle);});
  }

  void execute_move(const std::shared_ptr<GoalHandleMove> goal_handle)
  {
    BusyGuard guard(busy_);
    const auto goal = goal_handle->get_goal();
    auto result = std::make_shared<Move::Result>();
    Status status;
    std::string reason;
    if (!current_status(status, reason)) {
      result->message = reason;
      goal_handle->abort(result);
      return;
    }
    if (!status.activated || status.initialization_status != Status::INITIALIZATION_READY) {
      result->message = "gripper is not activated and ready";
      result->final_status = status;
      goal_handle->abort(result);
      return;
    }

    Command command;
    command.activate = true;
    command.mode = goal->mode;
    command.go_to = true;
    command.individual_finger_control = goal->individual_finger_control;
    command.individual_scissor_control = goal->individual_scissor_control;
    command.position_a = goal->position_a;
    command.speed_a = goal->speed_a;
    command.force_a = goal->force_a;
    command.position_b = goal->position_b;
    command.speed_b = goal->speed_b;
    command.force_b = goal->force_b;
    command.position_c = goal->position_c;
    command.speed_c = goal->speed_c;
    command.force_c = goal->force_c;
    command.position_scissor = goal->position_scissor;
    command.speed_scissor = goal->speed_scissor;
    command.force_scissor = goal->force_scissor;
    const auto command_id = publish_command(command);

    const double requested_timeout =
      static_cast<double>(goal->timeout.sec) +
      static_cast<double>(goal->timeout.nanosec) / 1.0e9;
    const auto timeout = requested_timeout > 0.0 ?
      std::chrono::duration_cast<SteadyClock::duration>(
        std::chrono::duration<double>(requested_timeout)) : default_move_timeout_;
    const auto connection_id = status.connection_id;
    const auto matches_request = [goal, command_id](const Status & value) {
        if (value.last_command_id != command_id ||
          value.mode != goal->mode || !value.go_to ||
          value.initialization_status != Status::INITIALIZATION_READY ||
          value.requested_position_a != goal->position_a)
        {
          return false;
        }
        if (goal->individual_finger_control &&
          (value.requested_position_b != goal->position_b ||
          value.requested_position_c != goal->position_c))
        {
          return false;
        }
        if (goal->individual_scissor_control &&
          value.requested_position_scissor != goal->position_scissor)
        {
          return false;
        }
        return value.motion_status != Status::MOTION_MOVING ||
               value.fault_status >= 0x09U;
      };
    const auto cancelled = [goal_handle]() {return goal_handle->is_canceling();};
    const auto progress = [goal_handle](const Status & value) {
        if (goal_handle->is_active()) {
          auto feedback = std::make_shared<Move::Feedback>();
          feedback->status = value;
          goal_handle->publish_feedback(feedback);
        }
      };
    const bool completed = wait_for_status(
      matches_request, SteadyClock::now() + timeout, connection_id,
      status, reason, cancelled, progress);

    result->final_status = status;
    result->fault_status = status.fault_status;
    if (!completed) {
      bool stop_sent = false;
      if (status.connected && status.connection_id == connection_id) {
        command.go_to = false;
        publish_command(command);
        stop_sent = true;
      }
      if (goal_handle->is_canceling()) {
        result->message = stop_sent ?
          "Move cancelled. Stop sent" :
          "Move cancelled. Backend unavailable";
        goal_handle->canceled(result);
      } else {
        result->message = reason + (stop_sent ?
          ". Stop sent" :
          ". Backend unavailable");
        goal_handle->abort(result);
      }
      return;
    }
    if (status.fault_status >= 0x09U) {
      result->message = "gripper reported fault " + std::to_string(status.fault_status);
      goal_handle->abort(result);
      return;
    }

    result->reached_requested_position =
      status.motion_status == Status::MOTION_REACHED;
    result->stopped_due_to_contact =
      status.motion_status == Status::MOTION_PARTIAL_CONTACT ||
      status.motion_status == Status::MOTION_ALL_CONTACT;
    result->message = result->reached_requested_position ?
      "requested position reached" : "motion stopped after object contact";
    goal_handle->succeed(result);
  }

  std::uint8_t gripper_command_position_to_raw(const double position_m) const
  {
    const double closed_fraction =
      (gripper_command_max_opening_m_ - position_m) /
      (gripper_command_max_opening_m_ - gripper_command_closed_position_m_);
    return static_cast<std::uint8_t>(
      std::lround(static_cast<double>(gripper_command_closed_raw_) * closed_fraction));
  }

  double gripper_command_raw_to_position(const std::uint8_t raw) const
  {
    const double bounded_raw = std::min(
      static_cast<double>(raw), static_cast<double>(gripper_command_closed_raw_));
    const double closed_fraction =
      bounded_raw / static_cast<double>(gripper_command_closed_raw_);
    return gripper_command_max_opening_m_ -
           closed_fraction *
           (gripper_command_max_opening_m_ - gripper_command_closed_position_m_);
  }

  std::uint8_t gripper_command_effort_to_raw(const double requested_effort_n) const
  {
    const double effort_n = requested_effort_n == 0.0 ?
      gripper_command_default_effort_n_ : requested_effort_n;
    const double fraction =
      (effort_n - gripper_command_min_effort_n_) /
      (gripper_command_max_effort_n_ - gripper_command_min_effort_n_);
    return static_cast<std::uint8_t>(std::lround(255.0 * fraction));
  }

  template<typename State>
  void fill_gripper_command_state(State & state, const Status & status) const
  {
    state.position = gripper_command_raw_to_position(status.actual_position_a);
    // The process image reports motor current as a raw byte, not contact force
    // in newtons. Reporting the requested force here would be misleading.
    state.effort = 0.0;
    state.stalled =
      status.motion_status == Status::MOTION_PARTIAL_CONTACT ||
      status.motion_status == Status::MOTION_ALL_CONTACT;
    state.reached_goal = status.motion_status == Status::MOTION_REACHED;
  }

  rclcpp_action::GoalResponse handle_gripper_command_goal(
    const rclcpp_action::GoalUUID &,
    std::shared_ptr<const GripperCommand::Goal> goal)
  {
    const double position_m = goal->command.position;
    const double effort_n = goal->command.max_effort;
    if (!std::isfinite(position_m) ||
      position_m < gripper_command_closed_position_m_ ||
      position_m > gripper_command_max_opening_m_)
    {
      RCLCPP_WARN(
        get_logger(), "Rejecting GripperCommand position %.6f outside [%.6f, %.6f] m",
        position_m, gripper_command_closed_position_m_, gripper_command_max_opening_m_);
      return rclcpp_action::GoalResponse::REJECT;
    }
    if (!std::isfinite(effort_n) || effort_n < 0.0 ||
      (effort_n > 0.0 &&
      (effort_n < gripper_command_min_effort_n_ ||
      effort_n > gripper_command_max_effort_n_)))
    {
      RCLCPP_WARN(
        get_logger(), "Rejecting GripperCommand max_effort %.3f N", effort_n);
      return rclcpp_action::GoalResponse::REJECT;
    }
    if (busy_.exchange(true)) {
      RCLCPP_WARN(get_logger(), "Rejecting GripperCommand because another operation is active");
      return rclcpp_action::GoalResponse::REJECT;
    }
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_gripper_command_cancel(
    const std::shared_ptr<GoalHandleGripperCommand>)
  {
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_gripper_command_accepted(
    const std::shared_ptr<GoalHandleGripperCommand> goal_handle)
  {
    std::lock_guard<std::mutex> lock(action_thread_mutex_);
    if (action_thread_.joinable()) {
      action_thread_.join();
    }
    action_thread_ = std::thread(
      [this, goal_handle]() {execute_gripper_command(goal_handle);});
  }

  void execute_gripper_command(
    const std::shared_ptr<GoalHandleGripperCommand> goal_handle)
  {
    BusyGuard guard(busy_);
    const auto goal = goal_handle->get_goal();
    auto result = std::make_shared<GripperCommand::Result>();
    Status status;
    std::string reason;
    if (!current_status(status, reason)) {
      RCLCPP_WARN(get_logger(), "GripperCommand aborted: %s", reason.c_str());
      goal_handle->abort(result);
      return;
    }
    if (!status.activated || status.initialization_status != Status::INITIALIZATION_READY) {
      fill_gripper_command_state(*result, status);
      RCLCPP_WARN(get_logger(), "GripperCommand aborted: gripper is not activated and ready");
      goal_handle->abort(result);
      return;
    }

    const auto position_raw = gripper_command_position_to_raw(goal->command.position);
    const auto effort_raw = gripper_command_effort_to_raw(goal->command.max_effort);
    const auto speed_raw = static_cast<std::uint8_t>(gripper_command_speed_);
    Command command;
    command.activate = true;
    // The generic endpoint intentionally selects the workcell's normal grasp
    // preset. Advanced mode selection remains available through Move.
    command.mode = Command::MODE_PINCH;
    command.go_to = true;
    command.position_a = position_raw;
    command.speed_a = speed_raw;
    command.force_a = effort_raw;
    command.position_b = position_raw;
    command.speed_b = speed_raw;
    command.force_b = effort_raw;
    command.position_c = position_raw;
    command.speed_c = speed_raw;
    command.force_c = effort_raw;
    command.speed_scissor = speed_raw;
    command.force_scissor = effort_raw;
    const auto command_id = publish_command(command);
    const auto connection_id = status.connection_id;

    const auto matches_request = [position_raw, command_id](const Status & value) {
        return value.last_command_id == command_id &&
               value.mode == Command::MODE_PINCH && value.go_to &&
               value.initialization_status == Status::INITIALIZATION_READY &&
               value.requested_position_a == position_raw &&
               (value.motion_status != Status::MOTION_MOVING ||
               value.fault_status >= 0x09U);
      };
    const auto cancelled = [goal_handle]() {return goal_handle->is_canceling();};
    const auto progress = [this, goal_handle](const Status & value) {
        if (goal_handle->is_active()) {
          auto feedback = std::make_shared<GripperCommand::Feedback>();
          fill_gripper_command_state(*feedback, value);
          goal_handle->publish_feedback(feedback);
        }
      };
    const bool completed = wait_for_status(
      matches_request, SteadyClock::now() + default_move_timeout_, connection_id,
      status, reason, cancelled, progress);
    fill_gripper_command_state(*result, status);

    if (!completed) {
      bool stop_sent = false;
      if (status.connected && status.connection_id == connection_id) {
        command.go_to = false;
        publish_command(command);
        stop_sent = true;
      }
      RCLCPP_WARN(
        get_logger(), "GripperCommand did not complete: %s%s", reason.c_str(),
        stop_sent ? ". Stop sent" : ". Backend unavailable");
      if (goal_handle->is_canceling()) {
        goal_handle->canceled(result);
      } else {
        goal_handle->abort(result);
      }
      return;
    }
    if (status.fault_status >= 0x09U) {
      RCLCPP_WARN(
        get_logger(), "GripperCommand aborted on fault %u", status.fault_status);
      goal_handle->abort(result);
      return;
    }
    goal_handle->succeed(result);
  }

  int activation_mode_{0};
  SteadyClock::duration activation_timeout_{std::chrono::seconds(30)};
  SteadyClock::duration default_move_timeout_{std::chrono::seconds(15)};
  SteadyClock::duration status_timeout_{std::chrono::milliseconds(500)};

  rclcpp::CallbackGroup::SharedPtr status_group_;
  rclcpp::CallbackGroup::SharedPtr operation_group_;
  rclcpp::Publisher<Command>::SharedPtr backend_command_publisher_;
  rclcpp::Publisher<Status>::SharedPtr public_status_publisher_;
  rclcpp::Subscription<Status>::SharedPtr backend_status_subscription_;
  rclcpp::Service<Trigger>::SharedPtr activate_service_;
  rclcpp::Service<Trigger>::SharedPtr deactivate_service_;
  rclcpp::Service<Trigger>::SharedPtr stop_service_;
  rclcpp_action::Server<Move>::SharedPtr move_server_;
  rclcpp_action::Server<GripperCommand>::SharedPtr gripper_command_server_;

  mutable std::mutex status_mutex_;
  std::condition_variable status_condition_;
  Status current_status_;
  SteadyClock::time_point last_status_time_{};
  bool have_status_{false};

  std::mutex command_mutex_;
  Command last_command_;
  bool have_command_{false};

  std::atomic<bool> busy_{false};
  std::atomic<bool> shutting_down_{false};
  std::atomic<std::uint64_t> next_command_id_{1U};
  std::mutex action_thread_mutex_;
  std::thread action_thread_;

  double gripper_command_closed_position_m_{0.0};
  double gripper_command_max_opening_m_{0.155};
  int gripper_command_closed_raw_{112};
  double gripper_command_min_effort_n_{15.0};
  double gripper_command_max_effort_n_{60.0};
  double gripper_command_default_effort_n_{30.0};
  int gripper_command_speed_{20};
};

}  // namespace robotiq_3f_controller

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<robotiq_3f_controller::Robotiq3FControllerNode>();
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 3U);
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
