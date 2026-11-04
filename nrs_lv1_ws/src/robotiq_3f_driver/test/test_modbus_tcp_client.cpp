// Copyright 2026 UR5 ROS 2 maintainers

#include <gtest/gtest.h>

#include <chrono>
#include <cstdint>
#include <future>
#include <thread>
#include <vector>

#include <boost/asio.hpp>

#include "robotiq_3f_driver/modbus_tcp_client.hpp"

namespace
{

using namespace std::chrono_literals;
using boost::asio::ip::tcp;
using robotiq_3f_driver::ProcessImage;

std::vector<std::uint8_t> read_exact(tcp::socket & socket, std::size_t size)
{
  std::vector<std::uint8_t> data(size);
  boost::asio::read(socket, boost::asio::buffer(data));
  return data;
}

std::vector<std::uint8_t> response_header(
  const std::vector<std::uint8_t> & request, std::uint16_t length)
{
  return {
    request[0], request[1], 0U, 0U,
    static_cast<std::uint8_t>((length >> 8U) & 0xFFU),
    static_cast<std::uint8_t>(length & 0xFFU), request[6]};
}

TEST(ModbusTcpClient, ReadsStatusAndWritesCommandAtAddressZero)
{
  boost::asio::io_context server_context;
  tcp::acceptor acceptor(server_context, tcp::endpoint(tcp::v4(), 0U));
  const auto port = acceptor.local_endpoint().port();
  std::promise<bool> server_result;

  std::thread server([&]() {
      try {
        tcp::socket socket(server_context);
        acceptor.accept(socket);

        const auto read_request = read_exact(socket, 12U);
        bool valid =
        read_request[6] == 7U && read_request[7] == 0x04U &&
        read_request[8] == 0U && read_request[9] == 0U &&
        read_request[10] == 0U && read_request[11] == 8U;
        auto read_response = response_header(read_request, 19U);
        read_response.push_back(0x04U);
        read_response.push_back(16U);
        for (std::uint8_t value = 0U; value < 16U; ++value) {
          read_response.push_back(value);
        }
        boost::asio::write(socket, boost::asio::buffer(read_response));

        const auto write_request = read_exact(socket, 29U);
        valid = valid && write_request[6] == 7U && write_request[7] == 0x10U &&
        write_request[8] == 0U && write_request[9] == 0U &&
        write_request[10] == 0U && write_request[11] == 8U &&
        write_request[12] == 16U;
        for (std::size_t index = 0U; index < 16U; ++index) {
          valid = valid && write_request[13U + index] == index;
        }
        auto write_response = response_header(write_request, 6U);
        write_response.insert(write_response.end(), {0x10U, 0U, 0U, 0U, 8U});
        boost::asio::write(socket, boost::asio::buffer(write_response));
        server_result.set_value(valid);
      } catch (...) {
        server_result.set_exception(std::current_exception());
      }
    });

  robotiq_3f_driver::ModbusTcpClient client(
    "127.0.0.1", port, 7U, 500ms, 500ms);
  client.connect();
  const auto status = client.read_status();
  for (std::size_t index = 0U; index < status.size(); ++index) {
    EXPECT_EQ(status[index], index);
  }
  ProcessImage command{};
  for (std::size_t index = 0U; index < command.size(); ++index) {
    command[index] = static_cast<std::uint8_t>(index);
  }
  client.write_command(command);
  client.close();

  EXPECT_TRUE(server_result.get_future().get());
  server.join();
}

}  // namespace
