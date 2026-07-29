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

/// \file
/// A fake BrickDaemon TCP server, for testing the TinkerForge bindings offline.
///
/// This is a test *helper*, not a test: it implements just enough of the BrickDaemon wire
/// protocol to convince the real, unmodified bindings that a genuine IMU Brick 2.0 is
/// attached, with no hardware and no ``brickd`` process. ``test_fake_brickd.cpp`` proves the
/// fake is convincing to the raw bindings; ``test_brick_v2_driver.cpp`` and
/// ``test_imu_node.cpp`` exercise the real code against it.
///
/// The wire protocol is an 8-byte little-endian header (uid, length, function_id,
/// sequence/response-expected, error/future) followed by an optional payload, framed purely
/// by the length byte. There is no handshake and no magic bytes.

#ifndef FAKE_BRICKD_HPP_
#define FAKE_BRICKD_HPP_

#include <array>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

namespace idmind_imu
{
namespace testing
{

/// Mirrors ``IPConnection`` FUNCTION_ENUMERATE.
constexpr uint8_t kFunctionEnumerate = 254;
/// Mirrors ``IPCON_CALLBACK_ENUMERATE``.
constexpr uint8_t kCallbackEnumerate = 253;
/// Mirrors FUNCTION_DISCONNECT_PROBE, sent about every 5s on an idle connection.
constexpr uint8_t kFunctionDisconnectProbe = 128;
/// Mirrors DEVICE_FUNCTION_GET_IDENTITY, common to every TinkerForge device.
constexpr uint8_t kFunctionGetIdentity = 255;
/// Mirrors ``IMU_V2_DEVICE_IDENTIFIER``.
constexpr uint16_t kDeviceIdentifierImuV2 = 18;
/// Mirrors ``IMU_V2_CALLBACK_ALL_DATA``.
constexpr uint8_t kCallbackAllData = 40;

// IMU Brick 2.0 function ids the driver actually exercises.
constexpr uint8_t kFunctionLedsOn = 10;
constexpr uint8_t kFunctionLedsOff = 11;
constexpr uint8_t kFunctionAreLedsOn = 12;
constexpr uint8_t kFunctionSetAllDataPeriod = 30;
constexpr uint8_t kFunctionGetAllDataPeriod = 31;
constexpr uint8_t kFunctionSetSensorFusionMode = 43;
constexpr uint8_t kFunctionGetSensorFusionMode = 44;
constexpr uint8_t kFunctionEnableStatusLed = 238;
constexpr uint8_t kFunctionDisableStatusLed = 239;
constexpr uint8_t kFunctionIsStatusLedEnabled = 240;

/// One all-data frame's worth of raw counts, as pushed by ``FakeBrickd::push_all_data``.
///
/// Every field defaults to zero (the quaternion to the identity rotation) so a test can
/// override only the field it cares about.
struct AllData
{
  std::array<int16_t, 3> acceleration{0, 0, 0};
  std::array<int16_t, 3> magnetic_field{0, 0, 0};
  std::array<int16_t, 3> angular_velocity{0, 0, 0};
  std::array<int16_t, 3> euler_angle{0, 0, 0};
  std::array<int16_t, 4> quaternion{16383, 0, 0, 0};
  std::array<int16_t, 3> linear_acceleration{0, 0, 0};
  std::array<int16_t, 3> gravity_vector{0, 0, 0};
  int8_t temperature{0};
  uint8_t calibration_status{0};
};

/// A minimal fake BrickDaemon that speaks just enough protocol to fool the real bindings.
///
/// Binds to an ephemeral port on 127.0.0.1 (read back via ``port()``) so tests never collide
/// on a fixed port. Handles ENUMERATE, GET_IDENTITY (the classic ``check_validity()`` trap),
/// the IMU Brick 2.0 getters/setters the driver uses, and ignores DISCONNECT_PROBE rather
/// than closing the connection. ``push_all_data()`` injects an unsolicited all-data frame.
class FakeBrickd
{
public:
  /// Create the fake; does not bind or listen until ``start()`` is called.
  explicit FakeBrickd(
    std::string uid = "6Dpwed", uint16_t device_identifier = kDeviceIdentifierImuV2);

  /// Stop the fake, closing every socket and joining every thread.
  ~FakeBrickd();

  FakeBrickd(const FakeBrickd &) = delete;
  FakeBrickd & operator=(const FakeBrickd &) = delete;

  /// Bind an ephemeral port and start accepting connections on a background thread.
  void start();

  /// Close every socket and join every thread. Idempotent: safe to call twice.
  void stop();

  /// The ephemeral port the fake is listening on; valid only after ``start()``.
  uint16_t port() const {return port_;}

  /// The device's base58 UID, as reported by enumerate and get_identity.
  const std::string & uid() const {return uid_;}

  /// Every function id received so far, in arrival order.
  std::vector<uint8_t> requests() const;

  /// How many requests have been received so far.
  size_t request_count() const;

  /// Whether a client is currently connected.
  bool has_client() const;

  /// Push one unsolicited all-data frame to the connected client.
  ///
  /// Waits briefly for a client to appear first, since ``ipcon_connect`` can return before
  /// the accept loop has published the fd. Returns whether the frame was actually sent.
  bool push_all_data(const AllData & data = AllData{});

  /// Block until a client is connected or \p timeout elapses; returns whether one is.
  bool wait_for_client(
    std::chrono::milliseconds timeout = std::chrono::seconds(2)) const;

  /// The fusion mode the fake currently reports, as changed by any setter it received.
  uint8_t sensor_fusion_mode() const;

  /// The all-data period the fake currently reports, in milliseconds.
  uint32_t all_data_period() const;

  /// Whether the fake currently reports its orientation/direction LEDs as on.
  bool leds_on() const;

  /// Whether the fake currently reports its status LED as enabled.
  bool status_led_enabled() const;

private:
  /// Accept one client connection at a time and service it until it disconnects.
  void accept_loop();
  /// Read framed packets from one client and dispatch each until it disconnects.
  void client_loop(int client_fd);
  /// Handle one request packet: enumerate, disconnect-probe, identity, or a getter/setter.
  void dispatch(
    int client_fd, const std::array<uint8_t, 8> & header,
    const std::vector<uint8_t> & payload);

  /// Build the shared (uid, connected_uid, position, hw, fw, device_identifier) body.
  std::vector<uint8_t> identity_payload() const;
  /// Assemble and write one 8-byte-header packet directly on \p fd.
  void send_packet(
    int fd, uint32_t uid_num, uint8_t function_id, uint8_t sequence_number,
    const std::vector<uint8_t> & body);

  std::string uid_;
  uint32_t uid_num_;
  uint16_t device_identifier_;
  uint16_t port_{0};

  /// Device state the getters report and the setters change.
  mutable std::mutex state_mutex_;
  uint32_t all_data_period_{0};
  uint8_t sensor_fusion_mode_{2};
  bool leds_on_{false};
  bool status_led_enabled_{true};
  std::vector<uint8_t> requests_;

  int server_fd_{-1};
  mutable std::mutex client_mutex_;
  int client_fd_{-1};

  std::atomic<bool> stopping_{true};
  std::thread accept_thread_;
  /// Client threads are only ever appended by the accept loop and joined by ``stop()``, after
  /// the accept loop itself has been joined, so no two threads ever touch this concurrently.
  std::vector<std::thread> client_threads_;
};

/// Decode a TinkerForge base58 UID the same way the bindings do, including the 64->32 fold.
uint32_t base58_decode_uid(const std::string & uid);

}  // namespace testing
}  // namespace idmind_imu

#endif  // FAKE_BRICKD_HPP_
