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
/// Self-tests proving FakeBrickd is convincing to the real TinkerForge bindings.
///
/// These use a real IPConnection and a real IMUV2 from the vendored bindings, never the
/// driver under test, so a failure here means the fake itself is wrong rather than anything
/// in idmind_imu. A short ipcon timeout keeps every mistake failing fast.

#include <gtest/gtest.h>

#include <chrono>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

#include "fake_brickd.hpp"

extern "C" {
#include "tinkerforge/brick_imu_v2.h"
#include "tinkerforge/ip_connection.h"
}

using idmind_imu::testing::AllData;
using idmind_imu::testing::FakeBrickd;

namespace
{

/// Deadline for anything that waits on a background callback.
constexpr auto kWaitTimeout = std::chrono::seconds(5);

/// A one-shot, thread-safe slot a callback can fill and a test can wait on.
template<typename T>
class Slot
{
public:
  /// Store \p value and wake any waiter.
  void set(const T & value)
  {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      value_ = value;
    }
    cv_.notify_all();
  }

  /// Wait up to kWaitTimeout for a value, returning nullopt on timeout.
  std::optional<T> get()
  {
    std::unique_lock<std::mutex> lock(mutex_);
    if (!cv_.wait_for(lock, kWaitTimeout, [this] {return value_.has_value();})) {
      return std::nullopt;
    }
    return value_;
  }

private:
  std::mutex mutex_;
  std::condition_variable cv_;
  std::optional<T> value_;
};

/// One enumeration event, as captured by the enumerate callback.
struct Enumeration
{
  std::string uid;
  uint16_t device_identifier;
};

/// One all-data frame, as captured by the all-data callback.
struct CapturedAllData
{
  std::array<int16_t, 3> acceleration;
  int8_t temperature;
  uint8_t calibration_status;
};

/// Owns a fake and a real IPConnection connected to it, torn down in the right order.
class FakeBrickdTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    fake_.start();
    ipcon_create(&ipcon_);
    // 0.5s instead of the bindings' 2.5s default, so a protocol mistake fails fast.
    ipcon_set_timeout(&ipcon_, 500);
    ASSERT_EQ(ipcon_connect(&ipcon_, "127.0.0.1", fake_.port()), E_OK);
  }

  void TearDown() override
  {
    ipcon_destroy(&ipcon_);  // disconnects internally, joining the callback thread
    fake_.stop();
  }

  /// Create a slot that is guaranteed to outlive the callback thread.
  ///
  /// A slot declared as a local in the test body would be destroyed when the body returns,
  /// while the callback registered on ipcon_ stays live until TearDown's ipcon_destroy joins
  /// the callback thread. A late packet arriving in that window would then write through a
  /// dangling pointer. Keeping the slot in a fixture member closes that window, because
  /// members are destroyed only after TearDown has run.
  template<typename T>
  Slot<T> * make_slot()
  {
    auto slot = std::make_shared<Slot<T>>();
    slots_.push_back(slot);
    return slot.get();
  }

  FakeBrickd fake_;
  IPConnection ipcon_{};

private:
  std::vector<std::shared_ptr<void>> slots_;
};

}  // namespace

TEST_F(FakeBrickdTest, EnumerateFindsTheFakeDevice)
{
  auto * slot = make_slot<Enumeration>();

  auto callback = [](
    const char * uid, const char * connected_uid, char position,
    uint8_t hardware_version[3], uint8_t firmware_version[3],
    uint16_t device_identifier, uint8_t enumeration_type, void * user_data) {
      (void)connected_uid; (void)position; (void)hardware_version;
      (void)firmware_version; (void)enumeration_type;
      static_cast<Slot<Enumeration> *>(user_data)->set(Enumeration{uid, device_identifier});
    };

  ipcon_register_callback(
    &ipcon_, IPCON_CALLBACK_ENUMERATE, reinterpret_cast<void (*)(void)>(+callback), slot);
  ASSERT_EQ(ipcon_enumerate(&ipcon_), E_OK);

  const auto result = slot->get();
  ASSERT_TRUE(result.has_value()) << "enumerate callback never fired";
  EXPECT_EQ(result->uid, fake_.uid());
  EXPECT_EQ(result->device_identifier, 18);
}

TEST_F(FakeBrickdTest, GetterRoundTripsThroughGetIdentity)
{
  // This is what exercises the hidden GET_IDENTITY request fired by check_validity() before
  // the first real getter. If the fake answered it wrong, or not at all, this would fail with
  // WRONG_DEVICE_TYPE instead of returning the fake's default fusion mode.
  IMUV2 imu;
  imu_v2_create(&imu, fake_.uid().c_str(), &ipcon_);

  uint8_t mode = 0;
  EXPECT_EQ(imu_v2_get_sensor_fusion_mode(&imu, &mode), E_OK);
  EXPECT_EQ(mode, 2);

  imu_v2_destroy(&imu);
}

TEST_F(FakeBrickdTest, SetterChangesWhatTheGetterLaterReturns)
{
  IMUV2 imu;
  imu_v2_create(&imu, fake_.uid().c_str(), &ipcon_);

  ASSERT_EQ(imu_v2_set_sensor_fusion_mode(&imu, 0), E_OK);
  uint8_t mode = 99;
  ASSERT_EQ(imu_v2_get_sensor_fusion_mode(&imu, &mode), E_OK);
  EXPECT_EQ(mode, 0);

  bool leds = false;
  ASSERT_EQ(imu_v2_leds_on(&imu), E_OK);
  ASSERT_EQ(imu_v2_are_leds_on(&imu, &leds), E_OK);
  EXPECT_TRUE(leds);

  ASSERT_EQ(imu_v2_leds_off(&imu), E_OK);
  ASSERT_EQ(imu_v2_are_leds_on(&imu, &leds), E_OK);
  EXPECT_FALSE(leds);

  imu_v2_destroy(&imu);
}

TEST_F(FakeBrickdTest, PushAllDataFiresTheRegisteredCallback)
{
  auto * slot = make_slot<CapturedAllData>();

  IMUV2 imu;
  imu_v2_create(&imu, fake_.uid().c_str(), &ipcon_);

  auto callback = [](
    int16_t acceleration[3], int16_t magnetic_field[3], int16_t angular_velocity[3],
    int16_t euler_angle[3], int16_t quaternion[4], int16_t linear_acceleration[3],
    int16_t gravity_vector[3], int8_t temperature, uint8_t calibration_status,
    void * user_data) {
      (void)magnetic_field; (void)angular_velocity; (void)euler_angle; (void)quaternion;
      (void)linear_acceleration; (void)gravity_vector;
      static_cast<Slot<CapturedAllData> *>(user_data)->set(
        CapturedAllData{
      {acceleration[0], acceleration[1], acceleration[2]},
      temperature, calibration_status});
    };

  imu_v2_register_callback(
    &imu, IMU_V2_CALLBACK_ALL_DATA, reinterpret_cast<void (*)(void)>(+callback), slot);

  AllData data;
  data.acceleration = {11, 22, 33};
  data.temperature = 42;
  data.calibration_status = 0xAB;
  fake_.push_all_data(data);

  const auto result = slot->get();
  ASSERT_TRUE(result.has_value()) << "all-data callback never fired";
  EXPECT_EQ(result->acceleration, (std::array<int16_t, 3>{11, 22, 33}));
  EXPECT_EQ(result->temperature, 42);
  EXPECT_EQ(result->calibration_status, 0xAB);

  imu_v2_destroy(&imu);
}

TEST_F(FakeBrickdTest, DisconnectProbeDoesNotCloseTheConnection)
{
  // The bindings send a disconnect probe on an idle connection and expect no reply. A fake
  // that answered it, or closed the socket, would make the connection look dead.
  IMUV2 imu;
  imu_v2_create(&imu, fake_.uid().c_str(), &ipcon_);

  uint8_t mode = 0;
  ASSERT_EQ(imu_v2_get_sensor_fusion_mode(&imu, &mode), E_OK);
  EXPECT_EQ(ipcon_get_connection_state(&ipcon_), IPCON_CONNECTION_STATE_CONNECTED);

  imu_v2_destroy(&imu);
}

TEST(FakeBrickdUid, Base58DecodeMatchesTheBindingsFold)
{
  // The fake must fold a >32-bit UID exactly like the bindings do, or its reply headers would
  // not match the client's device table and every getter would time out.
  EXPECT_EQ(
    idmind_imu::testing::base58_decode_uid("1"), 0u);
  EXPECT_NE(idmind_imu::testing::base58_decode_uid("6Dpwed"), 0u);
}
