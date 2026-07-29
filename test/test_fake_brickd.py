"""
Self-tests proving ``fake_brickd.FakeBrickd`` is convincing to the real ``tinkerforge`` bindings.

These tests use a real ``IPConnection`` and a real ``BrickIMUV2`` from the installed bindings,
never the driver under test, so a failure here means the fake itself is wrong rather than
anything in ``idmind_imu``. ``ipcon.set_timeout(0.5)`` keeps every mistake failing fast instead
of hanging for the bindings' default 2.5s.
"""

import queue

import pytest
from tinkerforge.brick_imu_v2 import BrickIMUV2
from tinkerforge.ip_connection import IPConnection

from fake_brickd import FakeBrickd

#: Deadline (seconds) for anything that waits on a background callback.
WAIT_TIMEOUT = 5.0


@pytest.fixture
def fake():
    """Start a fake BrickDaemon on an ephemeral port and stop it after the test."""
    with FakeBrickd() as server:
        yield server


@pytest.fixture
def ipcon(fake):
    """Build a real IPConnection, connected to the fake, with a short timeout so bugs fail fast."""
    connection = IPConnection()
    connection.set_timeout(0.5)
    connection.connect("127.0.0.1", fake.port)
    yield connection
    connection.disconnect()


def test_enumerate_finds_the_fake_device(fake, ipcon):
    """``enumerate()`` must yield a callback reporting the IMU Brick 2.0 identifier and uid."""
    results = queue.Queue()

    def on_enumerate(
        uid, connected_uid, position, hardware_version, firmware_version,
        device_identifier, enumeration_type
    ):
        results.put((uid, device_identifier))

    ipcon.register_callback(IPConnection.CALLBACK_ENUMERATE, on_enumerate)
    ipcon.enumerate()

    uid, device_identifier = results.get(timeout=WAIT_TIMEOUT)
    assert uid == fake.uid
    assert device_identifier == 18


def test_getter_round_trips_through_get_identity(fake, ipcon):
    """
    A getter on a freshly constructed device must succeed.

    This is exactly what exercises the hidden ``GET_IDENTITY`` request fired by
    ``Device.check_validity()`` before the first real getter; if the fake answered it wrong
    (or not at all) this would raise ``Error.WRONG_DEVICE_TYPE`` instead of returning cleanly.
    """
    imu = BrickIMUV2(fake.uid, ipcon)
    assert imu.get_sensor_fusion_mode() == 2  # the fake's default


def test_setter_changes_what_the_getter_later_returns(fake, ipcon):
    """A setter must actually change the fake's internal state, not just be acknowledged."""
    imu = BrickIMUV2(fake.uid, ipcon)

    imu.set_sensor_fusion_mode(0)
    assert imu.get_sensor_fusion_mode() == 0

    imu.leds_on()
    assert imu.are_leds_on() is True
    imu.leds_off()
    assert imu.are_leds_on() is False


def test_push_all_data_fires_the_registered_callback(fake, ipcon):
    """``push_all_data`` must cause a registered ``CALLBACK_ALL_DATA`` handler to fire."""
    imu = BrickIMUV2(fake.uid, ipcon)
    results = queue.Queue()

    def on_all_data(acceleration, magnetic_field, angular_velocity, euler_angle, quaternion,
                    linear_acceleration, gravity_vector, temperature, calibration_status):
        results.put((acceleration, temperature, calibration_status))

    imu.register_callback(BrickIMUV2.CALLBACK_ALL_DATA, on_all_data)

    fake.push_all_data(acceleration=(11, 22, 33), temperature=42, calibration_status=0xAB)

    acceleration, temperature, calibration_status = results.get(timeout=WAIT_TIMEOUT)
    assert acceleration == (11, 22, 33)
    assert temperature == 42
    assert calibration_status == 0xAB
