// Copyright 2026 UR5 ROS 2 maintainers

#ifndef ROBOTIQ_3F_DRIVER__MODBUS_TCP_CLIENT_HPP_
#define ROBOTIQ_3F_DRIVER__MODBUS_TCP_CLIENT_HPP_

#include <chrono>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include <boost/asio.hpp>

#include "robotiq_3f_driver/protocol.hpp"

namespace robotiq_3f_driver
{

class ModbusTcpClient
{
public:
  ModbusTcpClient(
    std::string address, std::uint16_t port, std::uint8_t unit_id,
    std::chrono::milliseconds connect_timeout,
    std::chrono::milliseconds response_timeout);
  ~ModbusTcpClient();

  ModbusTcpClient(const ModbusTcpClient &) = delete;
  ModbusTcpClient & operator=(const ModbusTcpClient &) = delete;

  void connect();
  void close() noexcept;
  bool is_open() const noexcept;

  ProcessImage read_status();
  void write_command(const ProcessImage & command);

private:
  using Clock = std::chrono::steady_clock;

  void wait_for(std::int16_t events, Clock::time_point deadline);
  void send_all(const std::vector<std::uint8_t> & data);
  std::vector<std::uint8_t> receive_exact(std::size_t size);
  std::vector<std::uint8_t> transact(
    const std::vector<std::uint8_t> & pdu);

  std::string address_;
  std::uint16_t port_;
  std::uint8_t unit_id_;
  std::chrono::milliseconds connect_timeout_;
  std::chrono::milliseconds response_timeout_;
  std::uint16_t transaction_id_{0};
  boost::asio::io_context io_context_;
  std::unique_ptr<boost::asio::ip::tcp::socket> socket_;
};

}  // namespace robotiq_3f_driver

#endif  // ROBOTIQ_3F_DRIVER__MODBUS_TCP_CLIENT_HPP_
