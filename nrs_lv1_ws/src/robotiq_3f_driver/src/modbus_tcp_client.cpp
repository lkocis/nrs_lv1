// Copyright 2026 UR5 ROS 2 maintainers

#include "robotiq_3f_driver/modbus_tcp_client.hpp"

#include <poll.h>
#include <sys/socket.h>

#include <algorithm>
#include <array>
#include <cerrno>
#include <cstring>
#include <limits>
#include <stdexcept>
#include <utility>

#include <boost/system/system_error.hpp>

namespace robotiq_3f_driver
{
namespace
{

std::runtime_error io_error(const std::string & operation, const std::string & detail)
{
  return std::runtime_error(operation + ": " + detail);
}

std::uint16_t read_u16(const std::vector<std::uint8_t> & data, std::size_t offset)
{
  return static_cast<std::uint16_t>(
    (static_cast<std::uint16_t>(data.at(offset)) << 8U) |
    static_cast<std::uint16_t>(data.at(offset + 1U)));
}

void append_u16(std::vector<std::uint8_t> & data, std::uint16_t value)
{
  data.push_back(static_cast<std::uint8_t>((value >> 8U) & 0xFFU));
  data.push_back(static_cast<std::uint8_t>(value & 0xFFU));
}

}  // namespace

ModbusTcpClient::ModbusTcpClient(
  std::string address, std::uint16_t port, std::uint8_t unit_id,
  std::chrono::milliseconds connect_timeout,
  std::chrono::milliseconds response_timeout)
: address_(std::move(address)),
  port_(port),
  unit_id_(unit_id),
  connect_timeout_(connect_timeout),
  response_timeout_(response_timeout)
{
}

ModbusTcpClient::~ModbusTcpClient()
{
  close();
}

void ModbusTcpClient::connect()
{
  close();
  boost::system::error_code error;
  const auto address = boost::asio::ip::make_address(address_, error);
  if (error) {
    throw io_error("invalid gripper IP address", error.message());
  }

  socket_ = std::make_unique<boost::asio::ip::tcp::socket>(io_context_);
  socket_->open(address.is_v6() ? boost::asio::ip::tcp::v6() : boost::asio::ip::tcp::v4(), error);
  if (error) {
    close();
    throw io_error("open Modbus TCP socket", error.message());
  }
  socket_->non_blocking(true, error);
  if (error) {
    close();
    throw io_error("configure Modbus TCP socket", error.message());
  }

  socket_->connect(boost::asio::ip::tcp::endpoint(address, port_), error);
  if (error == boost::asio::error::would_block ||
    error == boost::asio::error::in_progress ||
    error == boost::asio::error::already_started)
  {
    try {
      wait_for(POLLOUT, Clock::now() + connect_timeout_);
    } catch (...) {
      close();
      throw;
    }

    int socket_error = 0;
    socklen_t option_size = sizeof(socket_error);
    if (::getsockopt(
        socket_->native_handle(), SOL_SOCKET, SO_ERROR,
        &socket_error, &option_size) != 0)
    {
      const auto detail = std::strerror(errno);
      close();
      throw io_error("inspect Modbus TCP connection", detail);
    }
    if (socket_error != 0) {
      const auto detail = std::strerror(socket_error);
      close();
      throw io_error("connect to Modbus TCP gripper", detail);
    }
  } else if (error) {
    const auto detail = error.message();
    close();
    throw io_error("connect to Modbus TCP gripper", detail);
  }
}

void ModbusTcpClient::close() noexcept
{
  if (!socket_) {
    return;
  }
  boost::system::error_code ignored;
  socket_->cancel(ignored);
  socket_->shutdown(boost::asio::ip::tcp::socket::shutdown_both, ignored);
  socket_->close(ignored);
  socket_.reset();
}

bool ModbusTcpClient::is_open() const noexcept
{
  return socket_ && socket_->is_open();
}

void ModbusTcpClient::wait_for(std::int16_t events, Clock::time_point deadline)
{
  if (!is_open()) {
    throw io_error("wait for Modbus TCP socket", "socket is closed");
  }

  while (true) {
    const auto remaining =
      std::chrono::duration_cast<std::chrono::milliseconds>(deadline - Clock::now());
    if (remaining <= std::chrono::milliseconds::zero()) {
      throw io_error("wait for Modbus TCP socket", "timed out");
    }
    const auto bounded = std::min<std::int64_t>(
      remaining.count(), std::numeric_limits<int>::max());
    pollfd descriptor{socket_->native_handle(), events, 0};
    const int result = ::poll(&descriptor, 1, static_cast<int>(bounded));
    if (result > 0) {
      if ((descriptor.revents & (POLLERR | POLLHUP | POLLNVAL)) != 0) {
        throw io_error("wait for Modbus TCP socket", "connection closed or failed");
      }
      if ((descriptor.revents & events) != 0) {
        return;
      }
    } else if (result == 0) {
      throw io_error("wait for Modbus TCP socket", "timed out");
    } else if (errno != EINTR) {
      throw io_error("poll Modbus TCP socket", std::strerror(errno));
    }
  }
}

void ModbusTcpClient::send_all(const std::vector<std::uint8_t> & data)
{
  const auto deadline = Clock::now() + response_timeout_;
  std::size_t offset = 0U;
  while (offset < data.size()) {
    wait_for(POLLOUT, deadline);
    boost::system::error_code error;
    const auto count = socket_->write_some(
      boost::asio::buffer(data.data() + offset, data.size() - offset), error);
    if (error == boost::asio::error::would_block || error == boost::asio::error::try_again) {
      continue;
    }
    if (error) {
      throw io_error("write Modbus TCP request", error.message());
    }
    if (count == 0U) {
      throw io_error("write Modbus TCP request", "connection closed");
    }
    offset += count;
  }
}

std::vector<std::uint8_t> ModbusTcpClient::receive_exact(std::size_t size)
{
  const auto deadline = Clock::now() + response_timeout_;
  std::vector<std::uint8_t> data(size);
  std::size_t offset = 0U;
  while (offset < size) {
    wait_for(POLLIN, deadline);
    boost::system::error_code error;
    const auto count = socket_->read_some(
      boost::asio::buffer(data.data() + offset, data.size() - offset), error);
    if (error == boost::asio::error::would_block || error == boost::asio::error::try_again) {
      continue;
    }
    if (error) {
      throw io_error("read Modbus TCP response", error.message());
    }
    if (count == 0U) {
      throw io_error("read Modbus TCP response", "connection closed");
    }
    offset += count;
  }
  return data;
}

std::vector<std::uint8_t> ModbusTcpClient::transact(
  const std::vector<std::uint8_t> & pdu)
{
  if (!is_open()) {
    throw io_error("perform Modbus TCP transaction", "socket is closed");
  }
  if (pdu.empty() || pdu.size() > 253U) {
    throw std::invalid_argument("invalid Modbus PDU size");
  }

  ++transaction_id_;
  std::vector<std::uint8_t> request;
  request.reserve(7U + pdu.size());
  append_u16(request, transaction_id_);
  append_u16(request, 0U);
  append_u16(request, static_cast<std::uint16_t>(1U + pdu.size()));
  request.push_back(unit_id_);
  request.insert(request.end(), pdu.begin(), pdu.end());
  send_all(request);

  const auto header = receive_exact(7U);
  if (read_u16(header, 0U) != transaction_id_) {
    throw io_error("validate Modbus TCP response", "transaction ID mismatch");
  }
  if (read_u16(header, 2U) != 0U) {
    throw io_error("validate Modbus TCP response", "protocol ID is not zero");
  }
  if (header[6] != unit_id_) {
    throw io_error("validate Modbus TCP response", "unit ID mismatch");
  }
  const auto length = read_u16(header, 4U);
  if (length < 2U || length > 254U) {
    throw io_error("validate Modbus TCP response", "invalid length field");
  }
  auto response_pdu = receive_exact(static_cast<std::size_t>(length - 1U));
  if ((response_pdu[0] & 0x80U) != 0U) {
    const auto exception_code = response_pdu.size() > 1U ? response_pdu[1] : 0U;
    throw io_error(
            "Modbus exception response",
            "function=" + std::to_string(response_pdu[0] & 0x7FU) +
            " code=" + std::to_string(exception_code));
  }
  if (response_pdu[0] != pdu[0]) {
    throw io_error("validate Modbus TCP response", "function code mismatch");
  }
  return response_pdu;
}

ProcessImage ModbusTcpClient::read_status()
{
  std::vector<std::uint8_t> request{0x04U, 0x00U, 0x00U, 0x00U, 0x08U};
  const auto response = transact(request);
  if (response.size() != 18U || response[1] != kProcessImageSize) {
    throw io_error("decode Robotiq status", "unexpected input-register byte count");
  }
  ProcessImage image{};
  std::copy_n(response.begin() + 2, kProcessImageSize, image.begin());
  return image;
}

void ModbusTcpClient::write_command(const ProcessImage & command)
{
  std::vector<std::uint8_t> request;
  request.reserve(22U);
  request.push_back(0x10U);
  append_u16(request, 0U);
  append_u16(request, 8U);
  request.push_back(static_cast<std::uint8_t>(kProcessImageSize));
  request.insert(request.end(), command.begin(), command.end());
  const auto response = transact(request);
  if (response.size() != 5U || read_u16(response, 1U) != 0U ||
    read_u16(response, 3U) != 8U)
  {
    throw io_error("validate Robotiq command write", "unexpected register echo");
  }
}

}  // namespace robotiq_3f_driver
