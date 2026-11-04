// Copyright 2026 UR5 ROS 2 maintainers

#include <algorithm>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <memory>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <string>
#include <thread>

#include "diagnostic_updater/diagnostic_updater.hpp"
#include "rclcpp/rclcpp.hpp"
#include "robotiq_3f_driver/modbus_tcp_client.hpp"
#include "robotiq_3f_driver/protocol.hpp"
#include "robotiq_3f_interfaces/msg/robotiq3_f_command.hpp"
#include "robotiq_3f_interfaces/msg/robotiq3_f_status.hpp"

namespace robotiq_3f_driver
{

using namespace std::chrono_literals;
using Command = robotiq_3f_interfaces::msg::Robotiq3FCommand;
using Status = robotiq_3f_interfaces::msg::Robotiq3FStatus;

class Robotiq3FDriverNode : public rclcpp::Node
{
public:
  Robotiq3FDriverNode()
  : Node("robotiq_3f_driver"), updater_(this)
  {
    address_ = declare_parameter<std::string>("gripper_ip", "");
    const auto port = declare_parameter<int>("port", 502);
    const auto unit_id = declare_parameter<int>("unit_id", 0);
    const auto poll_rate_hz = declare_parameter<double>("poll_rate_hz", 10.0);
    const auto connect_timeout_ms = declare_parameter<int>("connect_timeout_ms", 1000);
    const auto response_timeout_ms = declare_parameter<int>("response_timeout_ms", 500);
    const auto reconnect_initial_ms = declare_parameter<int>("reconnect_initial_ms", 500);
    const auto reconnect_max_ms = declare_parameter<int>("reconnect_max_ms", 5000);
    allow_automatic_release_ =
      declare_parameter<bool>("allow_automatic_release", false);

    if (port < 1 || port > 65535) {
      throw std::invalid_argument("port must be in the range 1..65535");
    }
    if (unit_id < 0 || unit_id > 255) {
      throw std::invalid_argument("unit_id must be in the range 0..255");
    }
    if (poll_rate_hz <= 0.0) {
      throw std::invalid_argument("poll_rate_hz must be positive");
    }
    if (connect_timeout_ms <= 0 || response_timeout_ms <= 0 ||
      reconnect_initial_ms <= 0 || reconnect_max_ms < reconnect_initial_ms)
    {
      throw std::invalid_argument("timeout and reconnect parameters are invalid");
    }

    poll_period_ = std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::duration<double>(1.0 / poll_rate_hz));
    connect_timeout_ = std::chrono::milliseconds(connect_timeout_ms);
    response_timeout_ = std::chrono::milliseconds(response_timeout_ms);
    reconnect_initial_ = std::chrono::milliseconds(reconnect_initial_ms);
    reconnect_max_ = std::chrono::milliseconds(reconnect_max_ms);
    port_ = static_cast<std::uint16_t>(port);
    unit_id_ = static_cast<std::uint8_t>(unit_id);

    status_publisher_ = create_publisher<Status>("backend/status", rclcpp::QoS(10).reliable());
    command_subscription_ = create_subscription<Command>(
      "backend/command", rclcpp::QoS(10).reliable(),
      [this](Command::ConstSharedPtr command) {receive_command(*command);});

    updater_.setHardwareID(address_.empty() ? "robotiq_3f_unconfigured" : address_);
    updater_.add("Modbus TCP connection", this, &Robotiq3FDriverNode::diagnose);

    if (address_.empty()) {
      RCLCPP_WARN(
        get_logger(),
        "gripper_ip is empty. Driver disconnected");
    }

    worker_ = std::thread([this]() {run();});
  }

  ~Robotiq3FDriverNode() override
  {
    stop_requested_.store(true);
    sleep_condition_.notify_all();
    if (worker_.joinable()) {
      worker_.join();
    }
  }

private:
  bool sleep_interruptibly(std::chrono::milliseconds duration)
  {
    std::unique_lock<std::mutex> lock(sleep_mutex_);
    sleep_condition_.wait_for(
      lock, duration, [this]() {return stop_requested_.load();});
    return !stop_requested_.load();
  }

  void receive_command(const Command & command)
  {
    if (command.mode > Command::MODE_SCISSOR) {
      RCLCPP_ERROR(get_logger(), "Ignoring gripper command with invalid mode %u", command.mode);
      return;
    }
    if (command.command_id == 0U) {
      RCLCPP_ERROR(get_logger(), "Ignoring gripper command with reserved command_id zero");
      return;
    }
    if (command.automatic_release && !allow_automatic_release_) {
      RCLCPP_ERROR(
        get_logger(),
        "Ignoring automatic-release command because allow_automatic_release is false");
      return;
    }
    if (!ready_for_commands_.load()) {
      RCLCPP_WARN(
        get_logger(),
        "Ignoring command. No current status");
      return;
    }
    std::lock_guard<std::mutex> lock(command_mutex_);
    pending_command_ = command;
  }

  void publish_status(Status status, bool connected, const std::string & diagnostic)
  {
    status.stamp = get_clock()->now();
    status.connected = connected;
    status.connection_id = connection_id_.load();
    status.last_command_id = last_command_id_.load();
    status.diagnostic = diagnostic;
    status_publisher_->publish(status);

    std::lock_guard<std::mutex> lock(health_mutex_);
    connected_ = connected;
    last_diagnostic_ = diagnostic;
  }

  void mark_disconnected(const std::string & reason)
  {
    ready_for_commands_.store(false);
    last_command_id_.store(0U);
    {
      std::lock_guard<std::mutex> lock(command_mutex_);
      pending_command_.reset();
    }
    Status status;
    publish_status(status, false, reason);
  }

  void run()
  {
    if (address_.empty()) {
      mark_disconnected("gripper_ip is not configured");
      while (sleep_interruptibly(1s)) {
        mark_disconnected("gripper_ip is not configured");
      }
      return;
    }

    ModbusTcpClient client(
      address_, port_, unit_id_, connect_timeout_, response_timeout_);
    auto reconnect_delay = reconnect_initial_;

    while (!stop_requested_.load()) {
      if (!client.is_open()) {
        try {
          client.connect();
          connection_id_.fetch_add(1U);
          ready_for_commands_.store(false);
          last_command_id_.store(0U);
          reconnect_delay = reconnect_initial_;
          RCLCPP_INFO(
            get_logger(), "Connected to Robotiq 3F at %s:%u",
            address_.c_str(), static_cast<unsigned int>(port_));
        } catch (const std::exception & error) {
          mark_disconnected(error.what());
          RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 5000,
            "Robotiq 3F connection failed: %s", error.what());
          if (!sleep_interruptibly(reconnect_delay)) {
            break;
          }
          reconnect_delay = std::min(reconnect_delay * 2, reconnect_max_);
          continue;
        }
      }

      const auto cycle_start = std::chrono::steady_clock::now();
      try {
        std::optional<Command> command;
        if (ready_for_commands_.load()) {
          std::lock_guard<std::mutex> lock(command_mutex_);
          command.swap(pending_command_);
        }
        if (command) {
          client.write_command(pack_command(*command));
          last_command_id_.store(command->command_id);
        }

        auto status = unpack_status(client.read_status());
        publish_status(status, true, "connected");
        ready_for_commands_.store(true);
        updater_.force_update();
      } catch (const std::exception & error) {
        client.close();
        mark_disconnected(error.what());
        RCLCPP_ERROR(
          get_logger(), "Communication failed. Reconnecting: %s", error.what());
        continue;
      }

      const auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now() - cycle_start);
      if (elapsed < poll_period_ && !sleep_interruptibly(poll_period_ - elapsed)) {
        break;
      }
    }
    client.close();
    ready_for_commands_.store(false);
  }

  void diagnose(diagnostic_updater::DiagnosticStatusWrapper & status)
  {
    std::lock_guard<std::mutex> lock(health_mutex_);
    if (address_.empty()) {
      status.summary(diagnostic_msgs::msg::DiagnosticStatus::ERROR, "gripper_ip is unset");
    } else if (connected_) {
      status.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "connected and polling status");
    } else {
      status.summary(diagnostic_msgs::msg::DiagnosticStatus::WARN, last_diagnostic_);
    }
    status.add("gripper_ip", address_);
    status.add("port", static_cast<int>(port_));
    status.add("unit_id", static_cast<int>(unit_id_));
    status.add("automatic_release_allowed", allow_automatic_release_);
    status.add("connection_id", connection_id_.load());
  }

  diagnostic_updater::Updater updater_;
  rclcpp::Publisher<Status>::SharedPtr status_publisher_;
  rclcpp::Subscription<Command>::SharedPtr command_subscription_;

  std::string address_;
  std::uint16_t port_{502};
  std::uint8_t unit_id_{0};
  std::chrono::milliseconds poll_period_{100};
  std::chrono::milliseconds connect_timeout_{1000};
  std::chrono::milliseconds response_timeout_{500};
  std::chrono::milliseconds reconnect_initial_{500};
  std::chrono::milliseconds reconnect_max_{5000};
  bool allow_automatic_release_{false};

  std::atomic<bool> stop_requested_{false};
  std::atomic<bool> ready_for_commands_{false};
  std::atomic<std::uint64_t> connection_id_{0};
  std::atomic<std::uint64_t> last_command_id_{0};
  std::thread worker_;
  std::mutex sleep_mutex_;
  std::condition_variable sleep_condition_;
  std::mutex command_mutex_;
  std::optional<Command> pending_command_;

  std::mutex health_mutex_;
  bool connected_{false};
  std::string last_diagnostic_{"not connected"};
};

}  // namespace robotiq_3f_driver

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<robotiq_3f_driver::Robotiq3FDriverNode>());
  rclcpp::shutdown();
  return 0;
}
