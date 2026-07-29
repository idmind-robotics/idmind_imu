#! /usr/bin/env python3
"""
Hardware-agnostic ROS 2 node for streaming IMU data.

This node contains no TinkerForge-specific code: it talks only to the ``ImuDriver``
interface obtained through ``idmind_imu.drivers.registry.get_driver``, so swapping the
``driver`` parameter is enough to point it at a different IMU backend. Unit conversion and
covariance construction are delegated to ``idmind_imu.conversions``.
"""

import collections
import math
import threading
import time

import rclpy
from diagnostic_msgs.msg import DiagnosticStatus
from diagnostic_updater import Updater
from geometry_msgs.msg import Vector3Stamped
from rcl_interfaces.msg import ParameterDescriptor, SetParametersResult
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import Imu, MagneticField, Temperature
from std_msgs.msg import Float32, UInt8MultiArray
from std_srvs.srv import Trigger

from idmind_imu import conversions
from idmind_imu.drivers.registry import get_driver

#: Gyro noise stddev (0.3 deg/s, converted to rad/s) used for angular_velocity_covariance.
GYRO_VARIANCE = math.radians(0.3) ** 2
#: Linear-acceleration variances (x, y, z), preserved from the previous implementation.
ACCEL_VARIANCE = (0.01, 0.01, 0.05)
#: Magnetic-field variance (T^2), preserved from the previous implementation.
MAG_VARIANCE = (0.6e-6) ** 2
#: Number of recent sample timestamps kept to estimate the observed data rate.
RATE_WINDOW = 50

#: Parameter names whose changes must be forwarded to the driver's ``apply_config``.
_DRIVER_CONFIG_KEYS = frozenset((
    "host", "port", "imu_freq", "imu_leds", "imu_fusion_mode",
    "auto_reconnect", "acceleration_source", "orientation_stddev",
))


class ImuNode(Node):
    """
    ROS 2 node that streams samples from an ``ImuDriver`` onto a fixed set of topics.

    All hardware access goes through the driver obtained via ``driver`` parameter; this class
    only converts ``ImuSample`` instances into ROS messages and manages parameters, services,
    and diagnostics.
    """

    def __init__(self, parameter_overrides=None):
        """Declare parameters, create publishers/service/diagnostics, and start the driver."""
        super().__init__("idmind_imu", parameter_overrides=parameter_overrides or [])
        node_prefix = self.get_name() + "/"

        self.pub_callbacks = ReentrantCallbackGroup()
        self.srv_callbacks = ReentrantCallbackGroup()
        self.main_callback_group = ReentrantCallbackGroup()

        # -- Parameters ------------------------------------------------------------------
        self.driver_name = self.declare_parameter(
            "driver", "brick_v2",
            ParameterDescriptor(description="Name of the IMU driver backend to use")
        ).get_parameter_value().string_value
        self.host = self.declare_parameter(
            "host", "localhost",
            ParameterDescriptor(description="Hostname/IP of the IMU transport (e.g. BrickDaemon)")
        ).get_parameter_value().string_value
        self.port = self.declare_parameter(
            "port", 4223,
            ParameterDescriptor(description="Port of the IMU transport")
        ).get_parameter_value().integer_value
        self.control_freq = self.declare_parameter(
            "control_freq", 20.0,
            ParameterDescriptor(description="Frequency of the watchdog loop")
        ).get_parameter_value().double_value
        self.imu_freq = self.declare_parameter(
            "imu_freq", 20.0,
            ParameterDescriptor(description="Frequency of IMU stream")
        ).get_parameter_value().double_value
        self.imu_frame = self.declare_parameter(
            "imu_frame", "imu",
            ParameterDescriptor(description="Frame name for the IMU")
        ).get_parameter_value().string_value
        self.imu_leds = self.declare_parameter(
            "imu_leds", False,
            ParameterDescriptor(description="Enable/Disable IMU Leds")
        ).get_parameter_value().bool_value
        self.imu_fusion_mode = self.declare_parameter(
            "imu_fusion_mode", 2,
            ParameterDescriptor(description="Fusion Mode of the IMU")
        ).get_parameter_value().integer_value
        self.timeout = self.declare_parameter(
            "timeout", 1.0,
            ParameterDescriptor(description="Timeout for IMU Error")
        ).get_parameter_value().double_value
        self.auto_reconnect = self.declare_parameter(
            "auto_reconnect", True,
            ParameterDescriptor(description="Enable driver-level auto-reconnect")
        ).get_parameter_value().bool_value
        self.acceleration_source = self.declare_parameter(
            "acceleration_source", "linear",
            ParameterDescriptor(description="Which brick field to report as linear_acceleration")
        ).get_parameter_value().string_value
        self.orientation_stddev = self.declare_parameter(
            "orientation_stddev", 0.01,
            ParameterDescriptor(description="Base orientation stddev at full calibration")
        ).get_parameter_value().double_value
        self.add_on_set_parameters_callback(self.update_parameters)

        # -- Shared state, guarded by _lock (touched by the driver thread and the executor) --
        self._lock = threading.Lock()
        self._last_sample_monotonic = None
        self._last_calibration = None
        self._sample_times = collections.deque(maxlen=RATE_WINDOW)
        self._last_watchdog_monotonic = time.monotonic()

        self._ready = False
        self._last_msg = ""
        self._last_msg_time = time.monotonic()

        # -- Services --------------------------------------------------------------------
        self.create_service(
            Trigger, node_prefix + "ready", self.report_ready, callback_group=self.srv_callbacks
        )

        # -- Publishers --------------------------------------------------------------------
        self.imu_pub = self.create_publisher(
            Imu, node_prefix + "imu", 10, callback_group=self.pub_callbacks
        )
        self.temp_pub = self.create_publisher(
            Temperature, node_prefix + "temperature", 10, callback_group=self.pub_callbacks
        )
        self.mag_pub = self.create_publisher(
            MagneticField, node_prefix + "magnetic_field", 10, callback_group=self.pub_callbacks
        )
        self.euler_pub = self.create_publisher(
            Float32, node_prefix + "euler", 10, callback_group=self.pub_callbacks
        )
        self.gravity_pub = self.create_publisher(
            Vector3Stamped, node_prefix + "gravity", 10, callback_group=self.pub_callbacks
        )
        self.calib_pub = self.create_publisher(
            UInt8MultiArray, node_prefix + "calibration", 10, callback_group=self.pub_callbacks
        )
        self.timer_pub = self.create_publisher(
            Float32, node_prefix + "timer", 10, callback_group=self.pub_callbacks
        )

        # -- Driver --------------------------------------------------------------------------
        driver_cls = get_driver(self.driver_name)
        self._driver = driver_cls(self._driver_config(), self._on_sample, self.get_logger())
        self._driver.start()

        # -- Diagnostics ---------------------------------------------------------------------
        self.updater = Updater(self)
        self.updater.setHardwareID(self._driver.state().hardware_id)
        self.updater.add("Connection", self._diagnose_connection)
        self.updater.add("Data flow", self._diagnose_data_flow)
        self.updater.add("Calibration", self._diagnose_calibration)

        # -- Watchdog --------------------------------------------------------------------------
        self.watchdog_timer = self.create_timer(
            1.0 / self.control_freq, self.watchdog, callback_group=self.main_callback_group
        )

        self._ready = True
        self.log("Node is initialized.")

    # -- Driver config ------------------------------------------------------------------------

    def _driver_config(self):
        """Build the hardware-relevant config subset, as read from the current parameters."""
        return {
            "host": self.host,
            "port": self.port,
            "imu_freq": self.imu_freq,
            "imu_leds": self.imu_leds,
            "imu_fusion_mode": self.imu_fusion_mode,
            "auto_reconnect": self.auto_reconnect,
            "acceleration_source": self.acceleration_source,
            "orientation_stddev": self.orientation_stddev,
        }

    # -- Services / parameters ------------------------------------------------------------------

    def report_ready(self, _req, resp):
        """Reply with whether the node has finished initializing."""
        resp.success = self._ready
        resp.message = self.get_name() + " is " + ("ready" if self._ready else "not ready")
        return resp

    def update_parameters(self, params):
        """Apply changed parameters locally and forward the hardware-relevant subset."""
        changed_driver_config = {}
        for p in params:
            if p.name == "driver":
                self.driver_name = p.value
            elif p.name == "host":
                self.host = p.value
            elif p.name == "port":
                self.port = p.value
            elif p.name == "control_freq":
                if p.value <= 0.0:
                    return SetParametersResult(
                        successful=False, reason="control_freq must be > 0"
                    )
                self.control_freq = p.value
                self.watchdog_timer.timer_period_ns = int(1e9 / self.control_freq)
            elif p.name == "imu_freq":
                self.imu_freq = p.value
            elif p.name == "imu_frame":
                self.imu_frame = p.value
            elif p.name == "imu_leds":
                self.imu_leds = p.value
            elif p.name == "imu_fusion_mode":
                self.imu_fusion_mode = p.value
            elif p.name == "timeout":
                self.timeout = p.value
            elif p.name == "auto_reconnect":
                self.auto_reconnect = p.value
            elif p.name == "acceleration_source":
                self.acceleration_source = p.value
            elif p.name == "orientation_stddev":
                self.orientation_stddev = p.value

            if p.name in _DRIVER_CONFIG_KEYS:
                changed_driver_config[p.name] = p.value

        if changed_driver_config:
            self._driver.apply_config(changed_driver_config)

        return SetParametersResult(successful=True)

    # -- Sample handling (runs on a driver thread) ---------------------------------------------

    def _on_sample(self, sample):
        """Convert one ``ImuSample`` into ROS messages and publish; never blocks on the lock."""
        now = self.get_clock().now().to_msg()
        fusion_mode = sample.fusion_mode if sample.fusion_mode is not None else 0

        with self._lock:
            self._last_sample_monotonic = time.monotonic()
            self._sample_times.append(self._last_sample_monotonic)
            if sample.calibration is not None:
                self._last_calibration = sample.calibration

        if (
            sample.orientation is not None
            or sample.angular_velocity is not None
            or sample.linear_acceleration is not None
        ):
            self._publish_imu(now, sample, fusion_mode)

        if sample.temperature is not None:
            self._publish_temperature(now, sample.temperature)

        if sample.magnetic_field is not None:
            self._publish_magnetic_field(now, sample.magnetic_field)

        if sample.euler is not None:
            euler_msg = Float32()
            euler_msg.data = float(sample.euler[2])
            self.euler_pub.publish(euler_msg)

        if sample.gravity is not None:
            self._publish_gravity(now, sample.gravity)

        if sample.calibration is not None:
            calib_msg = UInt8MultiArray()
            calib_msg.data = list(sample.calibration)
            self.calib_pub.publish(calib_msg)

    def _publish_imu(self, stamp, sample, fusion_mode):
        """Build and publish the ``imu`` message, with zero-off-diagonal covariances."""
        msg = Imu()
        msg.header.stamp = stamp
        msg.header.frame_id = self.imu_frame

        if sample.orientation is not None:
            x, y, z, w = sample.orientation
            msg.orientation.x = x
            msg.orientation.y = y
            msg.orientation.z = z
            msg.orientation.w = w
        msg.orientation_covariance = conversions.orientation_covariance(
            fusion_mode, sample.calibration, self.orientation_stddev
        )

        if sample.angular_velocity is not None:
            x, y, z = sample.angular_velocity
            msg.angular_velocity.x = x
            msg.angular_velocity.y = y
            msg.angular_velocity.z = z
        msg.angular_velocity_covariance = conversions.diagonal_covariance(
            GYRO_VARIANCE, GYRO_VARIANCE, GYRO_VARIANCE
        )

        if sample.linear_acceleration is not None:
            x, y, z = sample.linear_acceleration
            msg.linear_acceleration.x = x
            msg.linear_acceleration.y = y
            msg.linear_acceleration.z = z
        msg.linear_acceleration_covariance = conversions.diagonal_covariance(*ACCEL_VARIANCE)

        self.imu_pub.publish(msg)

    def _publish_temperature(self, stamp, temperature):
        """Build and publish the ``temperature`` message."""
        msg = Temperature()
        msg.header.stamp = stamp
        msg.header.frame_id = self.imu_frame
        msg.temperature = float(temperature)
        self.temp_pub.publish(msg)

    def _publish_magnetic_field(self, stamp, magnetic_field):
        """Build and publish the ``magnetic_field`` message, with zero-off-diagonal covariance."""
        msg = MagneticField()
        msg.header.stamp = stamp
        msg.header.frame_id = self.imu_frame
        x, y, z = magnetic_field
        msg.magnetic_field.x = x
        msg.magnetic_field.y = y
        msg.magnetic_field.z = z
        msg.magnetic_field_covariance = conversions.diagonal_covariance(
            MAG_VARIANCE, MAG_VARIANCE, MAG_VARIANCE
        )
        self.mag_pub.publish(msg)

    def _publish_gravity(self, stamp, gravity):
        """Build and publish the ``gravity`` message."""
        msg = Vector3Stamped()
        msg.header.stamp = stamp
        msg.header.frame_id = self.imu_frame
        x, y, z = gravity
        msg.vector.x = x
        msg.vector.y = y
        msg.vector.z = z
        self.gravity_pub.publish(msg)

    # -- Watchdog (runs on an executor thread, no hardware I/O) -----------------------------

    def watchdog(self):
        """
        Publish the heartbeat, check data staleness, and refresh diagnostics.

        Must never perform hardware I/O and must never treat an exception as fatal: on error
        it logs and returns, leaving the timer running for the next period.
        """
        try:
            now = time.monotonic()
            period = now - self._last_watchdog_monotonic
            self._last_watchdog_monotonic = now

            heartbeat = Float32()
            heartbeat.data = float(period)
            self.timer_pub.publish(heartbeat)

            with self._lock:
                last_sample = self._last_sample_monotonic
            if last_sample is None or (now - last_sample) > self.timeout:
                self.log("No data from IMU driver", alert="warn")

            # Diagnostics are NOT forced from here: Updater runs its own timer at
            # `diagnostic_updater.period` (1 Hz by default). Forcing an update every watchdog
            # tick would republish /diagnostics at control_freq, swamping the aggregator.
        except Exception as exc:
            # CRITICAL: log and continue. Never shut down or cancel the timer from here — a
            # single transient exception must not leave the node permanently inert.
            self.log("Exception in watchdog: {}".format(exc), alert="error")

    # -- Diagnostics tasks --------------------------------------------------------------------

    def _diagnose_connection(self, stat):
        """Report ERROR if disconnected, WARN if connected but no device, else OK."""
        state = self._driver.state()
        if not state.connected:
            stat.summary(DiagnosticStatus.ERROR, "Not connected: {}".format(state.detail))
        elif not state.device_present:
            stat.summary(DiagnosticStatus.WARN, "No device found: {}".format(state.detail))
        else:
            stat.summary(DiagnosticStatus.OK, state.detail)
        return stat

    def _diagnose_data_flow(self, stat):
        """Report ERROR when the last sample is stale (or none was ever received)."""
        now = time.monotonic()
        with self._lock:
            last_sample = self._last_sample_monotonic
            sample_times = list(self._sample_times)

        age = (now - last_sample) if last_sample is not None else None
        if len(sample_times) >= 2:
            rate = (len(sample_times) - 1) / (sample_times[-1] - sample_times[0])
        else:
            rate = 0.0

        if age is None or age > self.timeout:
            stat.summary(DiagnosticStatus.ERROR, "No data within timeout")
        else:
            stat.summary(DiagnosticStatus.OK, "Receiving data")

        stat.add("rate_hz", "{:.2f}".format(rate))
        stat.add("age_s", "n/a" if age is None else "{:.2f}".format(age))
        return stat

    def _diagnose_calibration(self, stat):
        """Report WARN while any calibration component is 0, else OK."""
        with self._lock:
            calibration = self._last_calibration

        if calibration is None:
            calibration = (0, 0, 0, 0)
            stat.summary(DiagnosticStatus.WARN, "No calibration data yet")
        elif min(calibration) == 0:
            stat.summary(DiagnosticStatus.WARN, "Partially uncalibrated")
        else:
            stat.summary(DiagnosticStatus.OK, "Fully calibrated")

        sys_c, gyro_c, acc_c, mag_c = calibration
        stat.add("sys", str(sys_c))
        stat.add("gyro", str(gyro_c))
        stat.add("acc", str(acc_c))
        stat.add("mag", str(mag_c))
        return stat

    # -- Logging + lifecycle ------------------------------------------------------------------

    def log(self, msg, alert="info"):
        """Log ``msg`` via the node logger, suppressing an identical message within 1 second."""
        now = time.monotonic()
        if msg == self._last_msg and (now - self._last_msg_time) < 1.0:
            return
        self._last_msg = msg
        self._last_msg_time = now
        full_msg = "{}: {}".format(self.get_name(), msg)
        if alert == "warn":
            self.get_logger().warning(full_msg)
        elif alert == "error":
            self.get_logger().error(full_msg)
        else:
            self.get_logger().info(full_msg)

    def stop_driver(self):
        """Stop the underlying driver. Safe to call during shutdown."""
        self._driver.stop()


def main(args=None):
    """Spin an ``ImuNode`` under a ``MultiThreadedExecutor`` until interrupted."""
    rclpy.init(args=args)
    node = ImuNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.stop_driver()
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
