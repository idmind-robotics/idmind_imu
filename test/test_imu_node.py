"""
Integration tests of ``idmind_imu.imu_node.ImuNode`` against ``fake_brickd.FakeBrickd``.

These exercise the real node (brick_v2 driver, real ROS publishers/services) with no hardware
and no ``brickd`` process: the fake stands in for BrickDaemon on an ephemeral port. The node
runs under a ``MultiThreadedExecutor`` on a background thread; a separate subscriber node
receives its topics. Every wait has a hard deadline so a regression shows up as a fast, clear
failure instead of a hang.
"""

import threading
import time

import pytest
import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from sensor_msgs.msg import Imu
from std_msgs.msg import UInt8MultiArray

from fake_brickd import FakeBrickd
from idmind_imu.imu_node import ImuNode

#: Deadline for the node to connect and discover the fake device.
START_DEADLINE = 5.0
#: Deadline for a pushed sample to arrive at a subscriber.
MESSAGE_DEADLINE = 5.0


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


@pytest.fixture(autouse=True)
def _rclpy_context():
    """Init/shutdown rclpy once per test, isolating each test's node graph."""
    rclpy.init()
    yield
    rclpy.shutdown()


class _Harness:
    """Owns an ``ImuNode``, a subscriber node, and the executor spinning both."""

    def __init__(self, fake_server, **param_overrides):
        overrides = {"host": "127.0.0.1", "port": fake_server.port}
        overrides.update(param_overrides)
        parameter_overrides = [
            Parameter(name, value=value) for name, value in overrides.items()
        ]

        self.node = ImuNode(parameter_overrides=parameter_overrides)
        self.subscriber = Node("test_imu_node_subscriber")
        self.messages = {}
        self._msg_lock = threading.Lock()

        self.executor = MultiThreadedExecutor()
        self.executor.add_node(self.node)
        self.executor.add_node(self.subscriber)
        self._spin_thread = threading.Thread(target=self.executor.spin, daemon=True)
        self._spin_thread.start()

    def subscribe(self, topic, msg_type):
        """Subscribe to ``topic`` under the node's namespace, storing the latest message."""
        box = []

        def _cb(msg):
            with self._msg_lock:
                box.append(msg)

        self.subscriber.create_subscription(msg_type, "/idmind_imu/" + topic, _cb, 10)
        return box

    def wait_for_device(self):
        """Block until the driver reports the fake device present."""
        _wait_until(
            lambda: self.node._driver.state().device_present,
            START_DEADLINE,
            "device never found",
        )

    def shutdown(self):
        """Tear down the driver, executor, and both nodes."""
        self.node.stop_driver()
        self.executor.shutdown()
        self._spin_thread.join(timeout=5.0)
        self.subscriber.destroy_node()
        self.node.destroy_node()


@pytest.fixture
def harness(fake):
    """Build and tear down a ``_Harness`` around the fake brick daemon."""
    h = _Harness(fake)
    try:
        yield h
    finally:
        h.shutdown()


def test_imu_message_has_correct_units_frame_and_zero_off_diagonals(harness, fake):
    """A pushed ALL_DATA frame must produce an ``Imu`` with correct SI values and covariances."""
    box = harness.subscribe("imu", Imu)
    harness.wait_for_device()

    fake.push_all_data(
        angular_velocity=(16, 0, 0),
        quaternion=(16383, 0, 0, 0),
        linear_acceleration=(100, 0, 0),
        calibration_status=0b11100100,  # sys=3, gyro=2, acc=1, mag=0
    )

    _wait_until(lambda: len(box) > 0, MESSAGE_DEADLINE, "no Imu message received")
    msg = box[0]

    assert msg.header.frame_id == "imu"
    assert msg.angular_velocity.x == pytest.approx(0.0174533, abs=1e-6)
    assert msg.linear_acceleration.x == pytest.approx(1.0)
    assert msg.orientation.w == pytest.approx(1.0)

    off_diagonal_indices = (1, 2, 3, 5, 6, 7)
    for i in off_diagonal_indices:
        assert msg.angular_velocity_covariance[i] == 0.0
        assert msg.linear_acceleration_covariance[i] == 0.0


def test_calibration_topic_preserves_field_order(harness, fake):
    """``/idmind_imu/calibration`` must carry ``[sys, gyro, acc, mag]`` in that exact order."""
    box = harness.subscribe("calibration", UInt8MultiArray)
    harness.wait_for_device()

    # Four distinct levels so a wrong field order cannot pass this assertion.
    fake.push_all_data(calibration_status=0b11100100)  # sys=3, gyro=2, acc=1, mag=0

    _wait_until(lambda: len(box) > 0, MESSAGE_DEADLINE, "no calibration message received")
    assert list(box[0].data) == [3, 2, 1, 0]


def test_diagnostics_reports_calibration_warning(harness, fake):
    """``/diagnostics`` must include a WARN calibration status while a component is 0."""
    diag_box = []
    harness.subscriber.create_subscription(
        DiagnosticArray, "/diagnostics", lambda msg: diag_box.append(msg), 10
    )
    harness.wait_for_device()

    fake.push_all_data(calibration_status=0b11100100)  # mag=0 -> partially uncalibrated

    def _has_calibration_warning():
        for msg in list(diag_box):
            for status in msg.status:
                if "Calibration" in status.name and status.level == DiagnosticStatus.WARN:
                    return True
        return False

    _wait_until(_has_calibration_warning, MESSAGE_DEADLINE, "no calibration WARN diagnostic seen")


def test_orientation_covariance_sentinel_when_fusion_mode_off(fake):
    """``orientation_covariance[0]`` must be ``-1.0`` when ``imu_fusion_mode`` is 0 (OFF)."""
    h = _Harness(fake, imu_fusion_mode=0)
    try:
        box = h.subscribe("imu", Imu)
        h.wait_for_device()

        fake.push_all_data(quaternion=(16383, 0, 0, 0))

        _wait_until(lambda: len(box) > 0, MESSAGE_DEADLINE, "no Imu message received")
        assert box[0].orientation_covariance[0] == -1.0
    finally:
        h.shutdown()
