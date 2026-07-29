// Copyright 2024 IDMind
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "fake_brickd.hpp"

#include <arpa/inet.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <poll.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cerrno>
#include <chrono>
#include <cstring>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

namespace idmind_imu
{
namespace testing
{

namespace
{

/// The alphabet the TinkerForge bindings use; note the missing l, I, O and 0.
constexpr char kBase58Alphabet[] =
  "123456789abcdefghijkmnopqrstuvwxyzABCDEFGHJKLMNPQRSTUVWXYZ";

/// How long the accept/receive loops block before re-checking the stop flag.
constexpr int kPollIntervalMs = 50;

/// How long ``push_all_data`` waits for a client to appear before giving up.
constexpr auto kClientWaitTimeout = std::chrono::seconds(2);

/// Append a little-endian unsigned value of \p bytes width.
void append_le(std::vector<uint8_t> & out, uint64_t value, size_t bytes)
{
  for (size_t i = 0; i < bytes; ++i) {
    out.push_back(static_cast<uint8_t>((value >> (8 * i)) & 0xFF));
  }
}

/// Append a signed 16-bit value, little-endian, as the protocol encodes every count.
void append_i16(std::vector<uint8_t> & out, int16_t value)
{
  append_le(out, static_cast<uint16_t>(value), 2);
}

/// Append each element of an array of counts.
template<size_t N>
void append_i16_array(std::vector<uint8_t> & out, const std::array<int16_t, N> & values)
{
  for (int16_t value : values) {
    append_i16(out, value);
  }
}

/// Append a fixed-width, NUL-padded string field.
void append_fixed_string(std::vector<uint8_t> & out, const std::string & value, size_t width)
{
  for (size_t i = 0; i < width; ++i) {
    out.push_back(i < value.size() ? static_cast<uint8_t>(value[i]) : 0);
  }
}

/// Read a little-endian unsigned value of \p bytes width from \p data at \p offset.
uint64_t read_le(const std::vector<uint8_t> & data, size_t offset, size_t bytes)
{
  uint64_t value = 0;
  for (size_t i = 0; i < bytes; ++i) {
    value |= static_cast<uint64_t>(data[offset + i]) << (8 * i);
  }
  return value;
}

/// Read exactly \p size bytes from \p fd, polling so a stop request can interrupt the wait.
///
/// Returns false when the peer closed, an error occurred, or \p stopping became true.
bool recv_exact(int fd, void * buffer, size_t size, const std::atomic<bool> & stopping)
{
  auto * out = static_cast<uint8_t *>(buffer);
  size_t remaining = size;

  while (remaining > 0) {
    if (stopping.load()) {
      return false;
    }

    struct pollfd pfd {fd, POLLIN, 0};
    const int ready = ::poll(&pfd, 1, kPollIntervalMs);
    if (ready < 0) {
      if (errno == EINTR) {
        continue;
      }
      return false;
    }
    if (ready == 0) {
      continue;
    }

    const ssize_t received = ::recv(fd, out, remaining, 0);
    if (received <= 0) {
      if (received < 0 && errno == EINTR) {
        continue;
      }
      return false;
    }
    out += received;
    remaining -= static_cast<size_t>(received);
  }
  return true;
}

/// Write every byte of \p data to \p fd, tolerating short writes.
bool send_all(int fd, const std::vector<uint8_t> & data)
{
  size_t sent = 0;
  while (sent < data.size()) {
    const ssize_t written = ::send(fd, data.data() + sent, data.size() - sent, MSG_NOSIGNAL);
    if (written <= 0) {
      if (written < 0 && errno == EINTR) {
        continue;
      }
      return false;
    }
    sent += static_cast<size_t>(written);
  }
  return true;
}

}  // namespace

uint32_t base58_decode_uid(const std::string & uid)
{
  uint64_t value = 0;
  uint64_t base = 1;

  for (size_t index = uid.size(); index-- > 0; ) {
    size_t digit = 58;
    for (size_t k = 0; k < 58; ++k) {
      if (kBase58Alphabet[k] == uid[index]) {
        digit = k;
        break;
      }
    }
    if (digit == 58) {
      throw std::invalid_argument("invalid base58 character in UID: " + uid);
    }
    value += digit * base;
    if (index > 0) {
      base *= 58;
    }
  }

  // The bindings fold a >32-bit UID down to 32 bits with this exact bit shuffle; the fake
  // must reproduce it or its reply headers would not match the client's device table.
  if (value > 0xFFFFFFFFULL) {
    const uint64_t value1 = value & 0xFFFFFFFFULL;
    const uint64_t value2 = (value >> 32) & 0xFFFFFFFFULL;
    value = (value1 & 0x00000FFF);
    value |= (value1 & 0x0F000000) >> 12;
    value |= (value2 & 0x0000003F) << 16;
    value |= (value2 & 0x000F0000) << 6;
    value |= (value2 & 0x3F000000) << 2;
  }

  return static_cast<uint32_t>(value);
}

// -- lifecycle --------------------------------------------------------------------------------

FakeBrickd::FakeBrickd(std::string uid, uint16_t device_identifier)
: uid_(std::move(uid)), uid_num_(base58_decode_uid(uid_)), device_identifier_(device_identifier)
{
}

FakeBrickd::~FakeBrickd()
{
  stop();
}

void FakeBrickd::start()
{
  stopping_.store(false);

  server_fd_ = ::socket(AF_INET, SOCK_STREAM, 0);
  if (server_fd_ < 0) {
    throw std::runtime_error("FakeBrickd: socket() failed");
  }

  int reuse = 1;
  ::setsockopt(server_fd_, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));

  struct sockaddr_in address {};
  address.sin_family = AF_INET;
  address.sin_addr.s_addr = ::inet_addr("127.0.0.1");
  address.sin_port = 0;  // ephemeral

  if (::bind(server_fd_, reinterpret_cast<struct sockaddr *>(&address), sizeof(address)) < 0) {
    ::close(server_fd_);
    server_fd_ = -1;
    throw std::runtime_error("FakeBrickd: bind() failed");
  }
  if (::listen(server_fd_, 1) < 0) {
    ::close(server_fd_);
    server_fd_ = -1;
    throw std::runtime_error("FakeBrickd: listen() failed");
  }

  socklen_t length = sizeof(address);
  if (::getsockname(server_fd_, reinterpret_cast<struct sockaddr *>(&address), &length) < 0) {
    ::close(server_fd_);
    server_fd_ = -1;
    throw std::runtime_error("FakeBrickd: getsockname() failed");
  }
  port_ = ntohs(address.sin_port);

  accept_thread_ = std::thread(&FakeBrickd::accept_loop, this);
}

void FakeBrickd::stop()
{
  if (stopping_.exchange(true)) {
    return;
  }

  // Deliberately do NOT close server_fd_ here. The accept loop reads it every iteration, and
  // writing it from this thread would be a genuine data race (ThreadSanitizer catches it).
  // The loop polls with a timeout and re-checks `stopping_`, so it exits on its own within one
  // poll interval; the fd is closed below, once that thread has been joined.
  {
    std::lock_guard<std::mutex> lock(client_mutex_);
    if (client_fd_ >= 0) {
      ::shutdown(client_fd_, SHUT_RDWR);
      ::close(client_fd_);
      client_fd_ = -1;
    }
  }

  // Join the accept loop first: once it is gone, nothing can append another client thread.
  if (accept_thread_.joinable()) {
    accept_thread_.join();
  }
  for (auto & thread : client_threads_) {
    if (thread.joinable()) {
      thread.join();
    }
  }
  client_threads_.clear();

  // Now single-threaded again, so touching the listening socket is safe.
  if (server_fd_ >= 0) {
    ::close(server_fd_);
    server_fd_ = -1;
  }
}

// -- accept / receive loops -------------------------------------------------------------------

void FakeBrickd::accept_loop()
{
  while (!stopping_.load()) {
    struct pollfd pfd {server_fd_, POLLIN, 0};
    const int ready = ::poll(&pfd, 1, kPollIntervalMs);
    if (ready < 0) {
      if (errno == EINTR) {
        continue;
      }
      return;
    }
    if (ready == 0) {
      continue;
    }

    const int connection = ::accept(server_fd_, nullptr, nullptr);
    if (connection < 0) {
      return;
    }

    // Replies are tiny; Nagle would otherwise add 40ms to every getter round-trip.
    int nodelay = 1;
    ::setsockopt(connection, IPPROTO_TCP, TCP_NODELAY, &nodelay, sizeof(nodelay));

    {
      std::lock_guard<std::mutex> lock(client_mutex_);
      client_fd_ = connection;
    }
    client_threads_.emplace_back(&FakeBrickd::client_loop, this, connection);
  }
}

void FakeBrickd::client_loop(int client_fd)
{
  while (!stopping_.load()) {
    std::array<uint8_t, 8> header{};
    if (!recv_exact(client_fd, header.data(), header.size(), stopping_)) {
      break;
    }

    const uint8_t length = header[4];
    std::vector<uint8_t> payload;
    if (length > 8) {
      payload.resize(static_cast<size_t>(length) - 8);
      if (!recv_exact(client_fd, payload.data(), payload.size(), stopping_)) {
        break;
      }
    }

    dispatch(client_fd, header, payload);
  }

  std::lock_guard<std::mutex> lock(client_mutex_);
  if (client_fd_ == client_fd) {
    ::close(client_fd_);
    client_fd_ = -1;
  }
}

// -- request dispatch -------------------------------------------------------------------------

void FakeBrickd::dispatch(
  int client_fd, const std::array<uint8_t, 8> & header, const std::vector<uint8_t> & payload)
{
  const uint8_t function_id = header[5];
  const uint8_t sequence_options = header[6];
  const uint8_t sequence_number = (sequence_options >> 4) & 0x0F;
  const bool response_expected = (sequence_options & 0x08) != 0;

  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    requests_.push_back(function_id);
  }

  if (function_id == kFunctionEnumerate) {
    // One unsolicited CALLBACK_ENUMERATE packet: uid 0, sequence 0, 34 bytes total.
    std::vector<uint8_t> body = identity_payload();
    body.push_back(0);  // enumeration_type = AVAILABLE
    send_packet(client_fd, 0, kCallbackEnumerate, 0, body);
    return;
  }

  if (function_id == kFunctionDisconnectProbe) {
    // Idle-connection keepalive: the client expects no reply whatsoever. Answering it, or
    // closing the socket instead, would make the bindings declare the connection dead.
    return;
  }

  if (function_id == kFunctionGetIdentity) {
    // Answering this is what makes Device::check_validity() accept the device; without it
    // every later getter fails with WRONG_DEVICE_TYPE.
    send_packet(client_fd, uid_num_, kFunctionGetIdentity, sequence_number, identity_payload());
    return;
  }

  std::vector<uint8_t> reply;
  bool known = true;
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    switch (function_id) {
      case kFunctionSetAllDataPeriod:
        all_data_period_ = static_cast<uint32_t>(read_le(payload, 0, 4));
        break;
      case kFunctionGetAllDataPeriod:
        append_le(reply, all_data_period_, 4);
        break;
      case kFunctionSetSensorFusionMode:
        sensor_fusion_mode_ = payload.at(0);
        break;
      case kFunctionGetSensorFusionMode:
        reply.push_back(sensor_fusion_mode_);
        break;
      case kFunctionLedsOn:
        leds_on_ = true;
        break;
      case kFunctionLedsOff:
        leds_on_ = false;
        break;
      case kFunctionAreLedsOn:
        reply.push_back(leds_on_ ? 1 : 0);
        break;
      case kFunctionEnableStatusLed:
        status_led_enabled_ = true;
        break;
      case kFunctionDisableStatusLed:
        status_led_enabled_ = false;
        break;
      case kFunctionIsStatusLedEnabled:
        reply.push_back(status_led_enabled_ ? 1 : 0);
        break;
      default:
        known = false;
        break;
    }
  }

  if (known && response_expected) {
    send_packet(client_fd, uid_num_, function_id, sequence_number, reply);
  }
}

// -- reply builders ---------------------------------------------------------------------------

std::vector<uint8_t> FakeBrickd::identity_payload() const
{
  // Format: uid[8], connected_uid[8], position[1], hardware_version[3], firmware_version[3],
  // device_identifier[2] = 25 bytes.
  std::vector<uint8_t> body;
  append_fixed_string(body, uid_, 8);
  append_fixed_string(body, "0", 8);
  body.push_back(static_cast<uint8_t>('0'));
  body.insert(body.end(), {2, 0, 0});
  body.insert(body.end(), {2, 0, 0});
  append_le(body, device_identifier_, 2);
  return body;
}

void FakeBrickd::send_packet(
  int fd, uint32_t uid_num, uint8_t function_id, uint8_t sequence_number,
  const std::vector<uint8_t> & body)
{
  std::vector<uint8_t> packet;
  append_le(packet, uid_num, 4);
  packet.push_back(static_cast<uint8_t>(8 + body.size()));
  packet.push_back(function_id);
  packet.push_back(static_cast<uint8_t>((sequence_number << 4) & 0xF0));
  packet.push_back(0);  // error code / future use
  packet.insert(packet.end(), body.begin(), body.end());

  send_all(fd, packet);
}

// -- test-facing API --------------------------------------------------------------------------

std::vector<uint8_t> FakeBrickd::requests() const
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  return requests_;
}

size_t FakeBrickd::request_count() const
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  return requests_.size();
}

bool FakeBrickd::has_client() const
{
  std::lock_guard<std::mutex> lock(client_mutex_);
  return client_fd_ >= 0;
}

uint8_t FakeBrickd::sensor_fusion_mode() const
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  return sensor_fusion_mode_;
}

uint32_t FakeBrickd::all_data_period() const
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  return all_data_period_;
}

bool FakeBrickd::leds_on() const
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  return leds_on_;
}

bool FakeBrickd::status_led_enabled() const
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  return status_led_enabled_;
}

bool FakeBrickd::push_all_data(const AllData & data)
{
  // Payload: 22 int16 counts + int8 temperature + uint8 calibration = 46 bytes, so 54 with
  // the header. The bindings drop any all-data packet that is not exactly that length.
  std::vector<uint8_t> body;
  append_i16_array(body, data.acceleration);
  append_i16_array(body, data.magnetic_field);
  append_i16_array(body, data.angular_velocity);
  append_i16_array(body, data.euler_angle);
  append_i16_array(body, data.quaternion);
  append_i16_array(body, data.linear_acceleration);
  append_i16_array(body, data.gravity_vector);
  body.push_back(static_cast<uint8_t>(data.temperature));
  body.push_back(data.calibration_status);

  // ipcon_connect() returns as soon as the TCP handshake completes, which can be before the
  // accept loop here has published the client fd. Pushing immediately after connecting would
  // otherwise be silently dropped, so wait for the fd rather than no-op.
  if (!wait_for_client(kClientWaitTimeout)) {
    return false;
  }

  std::lock_guard<std::mutex> lock(client_mutex_);
  if (client_fd_ < 0) {
    return false;
  }
  send_packet(client_fd_, uid_num_, kCallbackAllData, 0, body);
  return true;
}

bool FakeBrickd::wait_for_client(std::chrono::milliseconds timeout) const
{
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  while (std::chrono::steady_clock::now() < deadline) {
    if (has_client()) {
      return true;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  }
  return has_client();
}

}  // namespace testing
}  // namespace idmind_imu
