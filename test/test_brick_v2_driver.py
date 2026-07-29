"""
Tests of the real ``ImuBrickV2Driver`` against ``fake_brickd.FakeBrickd``.

Unlike ``test_fake_brickd.py`` (which proves the fake is convincing to the raw bindings),
these tests exercise ``idmind_imu.drivers.brick_v2`` itself: connection lifecycle, unit
conversion on the ``CALLBACK_ALL_DATA`` path, the ``acceleration_source`` switch, that
configuration converges once rather than being polled, and clean shutdown. Every wait has a
hard deadline so a regression shows up as a fast, clear failure instead of a hang.
"""

import logging
import queue
import threading
import time

import pytest

from fake_brickd import FakeBrickd
from idmind_imu.drivers.base import ImuSample
from idmind_imu.drivers.brick_v2 import ImuBrickV2Driver

#: Deadline for the driver to connect and discover the fake device.
START_DEADLINE = 5.0
#: Deadline for a pushed sample to arrive at the ``on_sample`` callback.
SAMPLE_DEADLINE = 5.0
#: Deadline for config convergence and thread teardown to complete.
SETTLE_DEADLINE = 5.0
#: Window over which config-related requests must NOT keep growing (the polling regression).
STEADY_STATE_WINDOW = 1.5

_LOGGER = logging.getLogger("test_brick_v2_driver")


def _wait_until(predicate, timeout, message):
    """Poll ``predicate`` until it is true or ``timeout`` elapses, else ``pytest.fail``."""
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return
        time.sleep(0.02)
    pytest.fail(message)


@pytest.fixture
def fake():
    """Start a fake BrickDaemon on an ephemeral port and stop it after the test."""
    with FakeBrickd() as server:
        yield server


@pytest.fixture
def samples():
    """Return a queue that ``on_sample`` pushes ``ImuSample`` instances into."""
    return queue.Queue()


def _make_driver(fake_server, on_sample, **config_overrides):
    """Build an ``ImuBrickV2Driver`` pointed at ``fake_server``, with any config overrides."""
    config = {"host": "127.0.0.1", "port": fake_server.port}
    config.update(config_overrides)
    return ImuBrickV2Driver(config, on_sample, _LOGGER)


def test_start_returns_promptly_and_reaches_connected(fake, samples):
    """``start()`` must not block, and the driver must reach connected+device_present."""
    driver = _make_driver(fake, samples.put)
    try:
        started_at = time.monotonic()
        driver.start()
        assert time.monotonic() - started_at < 1.0

        _wait_until(
            lambda: driver.state().connected and driver.state().device_present,
            START_DEADLINE,
            "driver did not reach connected+device_present in time",
        )
        state = driver.state()
        assert state.connected is True
        assert state.device_present is True
    finally:
        driver.stop()


def test_all_data_callback_converts_to_si_units(fake, samples):
    """A pushed ALL_DATA frame must arrive as an ``ImuSample`` with correct SI values."""
    driver = _make_driver(fake, samples.put)
    try:
        driver.start()
        _wait_until(
            lambda: driver.state().device_present, START_DEADLINE, "device never found"
        )

        fake.push_all_data(
            magnetic_field=(16, 0, 0),
            angular_velocity=(16, 0, 0),
            quaternion=(16383, 0, 0, 0),
            linear_acceleration=(100, 0, 0),
            temperature=25,
            calibration_status=0b11100100,  # sys=3, gyro=2, acc=1, mag=0
        )

        sample = samples.get(timeout=SAMPLE_DEADLINE)
        assert isinstance(sample, ImuSample)
        assert sample.angular_velocity[0] == pytest.approx(0.0174533, abs=1e-6)
        assert sample.linear_acceleration[0] == pytest.approx(1.0)
        assert sample.magnetic_field[0] == pytest.approx(1e-6)
        assert sample.orientation[3] == pytest.approx(1.0)  # ROS xyzw -> w
        assert sample.temperature == pytest.approx(25.0)
        # (sys, gyro, acc, mag): four distinct levels so a wrong field order cannot pass.
        assert sample.calibration == (3, 2, 1, 0)
    finally:
        driver.stop()


def test_acceleration_source_raw_selects_the_raw_field(fake, samples):
    """``acceleration_source="raw"`` must report the brick's ``acceleration`` field."""
    driver = _make_driver(fake, samples.put, acceleration_source="raw")
    try:
        driver.start()
        _wait_until(
            lambda: driver.state().device_present, START_DEADLINE, "device never found"
        )

        fake.push_all_data(acceleration=(500, 0, 0), linear_acceleration=(100, 0, 0))

        sample = samples.get(timeout=SAMPLE_DEADLINE)
        assert sample.linear_acceleration[0] == pytest.approx(5.0)
    finally:
        driver.stop()


def test_acceleration_source_default_selects_the_linear_field(fake, samples):
    """The default ``acceleration_source`` ("linear") must report ``linear_acceleration``."""
    driver = _make_driver(fake, samples.put)
    try:
        driver.start()
        _wait_until(
            lambda: driver.state().device_present, START_DEADLINE, "device never found"
        )

        fake.push_all_data(acceleration=(500, 0, 0), linear_acceleration=(100, 0, 0))

        sample = samples.get(timeout=SAMPLE_DEADLINE)
        assert sample.linear_acceleration[0] == pytest.approx(1.0)
    finally:
        driver.stop()


def test_config_is_applied_once_and_not_polled(fake, samples):
    """
    Config must converge a bounded number of times, not be re-polled forever.

    This is the regression guard for the old implementation's 1Hz config polling: once the
    driver has settled, the count of config-related requests recorded by the fake must stop
    growing.
    """
    driver = _make_driver(
        fake, samples.put, imu_freq=10.0, imu_leds=True, imu_fusion_mode=1
    )
    try:
        driver.start()
        # GET_ALL_DATA_PERIOD (31) is the last getter in the single _do_apply_config job, so
        # its presence means the job has essentially finished (bar its own conditional setter).
        _wait_until(
            lambda: 31 in fake.requests, SETTLE_DEADLINE, "config was never applied"
        )
        time.sleep(0.2)  # let any trailing setter from that same job land

        first_count = len(fake.requests)
        time.sleep(STEADY_STATE_WINDOW)
        second_count = len(fake.requests)

        assert second_count == first_count, (
            "requests kept growing after settling: {} -> {}".format(
                first_count, second_count
            )
        )
    finally:
        driver.stop()


def test_stop_is_clean_idempotent_and_leaves_no_threads(fake, samples):
    """``stop()`` must be safe to call twice and must not leave background threads running."""
    threads_before = {t.ident for t in threading.enumerate()}

    driver = _make_driver(fake, samples.put)
    driver.start()
    _wait_until(lambda: driver.state().connected, START_DEADLINE, "driver never connected")

    driver.stop()
    driver.stop()  # idempotent: must not raise or hang

    def no_leaked_threads():
        return all(
            not t.is_alive() for t in threading.enumerate() if t.ident not in threads_before
        )

    _wait_until(
        no_leaked_threads, SETTLE_DEADLINE, "driver left background threads running after stop()"
    )
