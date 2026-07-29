"""
Driver for the TinkerForge IMU Brick 2.0 (BNO-055), reached over BrickDaemon TCP.

All unit conversion is delegated to :mod:`idmind_imu.conversions`; this module only speaks
raw TinkerForge counts to the hardware and hands SI-unit ``ImuSample`` instances upward.

Threading model: the ``tinkerforge`` bindings dispatch every registered callback serially on
one internal "Callback-Processor" thread, so the ``CALLBACK_ENUMERATE`` and
``CALLBACK_ALL_DATA`` handlers here do the minimum possible work and never perform a blocking
hardware round-trip. All configuration I/O (the getters/setters used to converge the device
onto the desired config) happens on a dedicated worker thread fed by a queue, so a slow config
round-trip can never stall delivery of the sensor stream. A separate connect-retry thread
handles the initial (blocking) ``ipcon.connect()``, since the library's auto-reconnect only
covers reconnection after a first successful connect.
"""

import queue
import threading

from tinkerforge.brick_imu_v2 import BrickIMUV2
from tinkerforge.ip_connection import Error, IPConnection

from idmind_imu.conversions import (
    acceleration_from_brick,
    angular_velocity_from_brick,
    decode_calibration,
    euler_from_brick,
    magnetic_field_from_brick,
    quaternion_from_brick,
)
from idmind_imu.drivers.base import DriverState, ImuDriver, ImuSample

#: Fallback config values, used only when the caller omits a key. These deliberately mirror
#: the node's parameter defaults: a mismatch here would silently reconfigure the hardware.
#: In particular ``imu_fusion_mode`` must not fall back to 0, which would turn fusion OFF.
DEFAULT_CONFIG = {
    "host": "localhost",
    "port": 4223,
    "auto_reconnect": True,
    "imu_freq": 20.0,
    "imu_leds": False,
    "imu_fusion_mode": 2,
    "acceleration_source": "linear",
    "orientation_stddev": 0.01,
}


class ImuBrickV2Driver(ImuDriver):
    """Driver for a single TinkerForge IMU Brick 2.0, connected through BrickDaemon."""

    def __init__(self, config, on_sample, logger):
        """Build the driver; does not touch the network until ``start()`` is called."""
        super().__init__(config, on_sample, logger)
        self._ipcon = IPConnection()
        self._imu = None
        self._imu_uid = None
        self._lock = threading.Lock()
        self._state = DriverState()
        self._job_queue = queue.Queue()
        self._stop_event = threading.Event()
        self._started = False
        self._worker_thread = None
        self._connect_thread = None

    def _cfg(self, key):
        """Read a config key, falling back to ``DEFAULT_CONFIG`` when the caller omitted it."""
        return self._config.get(key, DEFAULT_CONFIG[key])

    # -- ImuDriver interface -------------------------------------------------------------

    def start(self):
        """
        Register callbacks, start the config worker, and start connect-retrying.

        Returns immediately; connecting and enumeration happen in background threads.
        """
        with self._lock:
            if self._started:
                return
            self._started = True

        self._stop_event.clear()
        auto_reconnect = bool(self._cfg("auto_reconnect"))
        self._ipcon.set_auto_reconnect(auto_reconnect)
        self._ipcon.register_callback(IPConnection.CALLBACK_CONNECTED, self._connected_callback)
        self._ipcon.register_callback(
            IPConnection.CALLBACK_DISCONNECTED, self._disconnected_callback
        )
        self._ipcon.register_callback(IPConnection.CALLBACK_ENUMERATE, self._enumerate_callback)

        self._worker_thread = threading.Thread(target=self._worker_loop, daemon=True)
        self._worker_thread.start()

        self._connect_thread = threading.Thread(target=self._connect_loop, daemon=True)
        self._connect_thread.start()

    def stop(self):
        """Tear the driver down. Idempotent: safe before ``start()`` or called twice."""
        with self._lock:
            if not self._started:
                return
            self._started = False

        self._stop_event.set()

        try:
            self._ipcon.register_callback(IPConnection.CALLBACK_CONNECTED, None)
            self._ipcon.register_callback(IPConnection.CALLBACK_DISCONNECTED, None)
            self._ipcon.register_callback(IPConnection.CALLBACK_ENUMERATE, None)
        except Exception as exc:
            self._logger.warning("IMU brick: failed to remove callbacks: {}".format(exc))

        self._job_queue.put(None)

        try:
            self._ipcon.disconnect()
        except Error:
            pass
        except Exception as exc:
            self._logger.warning("IMU brick: disconnect failed: {}".format(exc))

        if self._connect_thread is not None:
            self._connect_thread.join(timeout=2.0)
        if self._worker_thread is not None:
            self._worker_thread.join(timeout=2.0)

        with self._lock:
            self._imu = None
            self._imu_uid = None
            self._state.connected = False
            self._state.device_present = False
            self._state.detail = "stopped"

    def apply_config(self, config):
        """Merge ``config`` into the current configuration and enqueue a convergence job."""
        with self._lock:
            self._config.update(config)
        self._job_queue.put(self._do_apply_config)

    def state(self):
        """Return an immutable snapshot of the current connection and device status."""
        with self._lock:
            return DriverState(
                connected=self._state.connected,
                device_present=self._state.device_present,
                hardware_id=self._state.hardware_id,
                detail=self._state.detail,
            )

    # -- Connection lifecycle -------------------------------------------------------------

    def _connect_loop(self):
        """Retry ``ipcon.connect()`` roughly once a second until it succeeds or stop fires."""
        with self._lock:
            host = self._cfg("host")
            port = int(self._cfg("port"))

        while not self._stop_event.is_set():
            try:
                self._ipcon.connect(host, port)
                return
            except Error as exc:
                self._logger.warning(
                    "IMU brick: connect to {}:{} failed: {}".format(host, port, exc)
                )
            except Exception as exc:
                self._logger.warning(
                    "IMU brick: connect to {}:{} failed: {}".format(host, port, exc)
                )
            self._stop_event.wait(1.0)

    def _connected_callback(self, reason):
        """Handle ``IPConnection.CALLBACK_CONNECTED``: mark connected, then enumerate."""
        with self._lock:
            self._state.connected = True
            self._state.detail = "connected, enumerating"
        self._logger.info("IMU brick: connected to BrickDaemon (reason={})".format(reason))
        try:
            self._ipcon.enumerate()
        except Error as exc:
            self._logger.error("IMU brick: enumerate() failed: {}".format(exc))

    def _disconnected_callback(self, reason):
        """Handle ``IPConnection.CALLBACK_DISCONNECTED``: clear device state."""
        with self._lock:
            self._state.connected = False
            self._state.device_present = False
            self._state.detail = "disconnected (reason={})".format(reason)
            self._imu = None
            self._imu_uid = None
        self._logger.warning("IMU brick: disconnected from BrickDaemon (reason={})".format(
            reason
        ))

    def _enumerate_callback(
        self,
        uid,
        connected_uid,
        position,
        hardware_version,
        firmware_version,
        device_identifier,
        enumeration_type,
    ):
        """
        Handle ``IPConnection.CALLBACK_ENUMERATE``: identify and wire up the brick.

        Does only cheap, non-blocking work: identifies the brick, constructs it, registers
        the ALL_DATA callback, updates state, and enqueues a config job. All hardware I/O for
        configuration happens later on the worker thread.
        """
        if enumeration_type == IPConnection.ENUMERATION_TYPE_DISCONNECTED:
            with self._lock:
                if self._imu_uid == uid:
                    self._imu = None
                    self._imu_uid = None
                    self._state.device_present = False
                    self._state.detail = "device disconnected"
            return

        if device_identifier != BrickIMUV2.DEVICE_IDENTIFIER:
            return

        imu = BrickIMUV2(uid, self._ipcon)
        imu.register_callback(BrickIMUV2.CALLBACK_ALL_DATA, self._all_data_callback)

        with self._lock:
            self._imu = imu
            self._imu_uid = uid
            self._state.device_present = True
            self._state.hardware_id = "IMU Brick 2.0 ({})".format(uid)
            self._state.detail = "device enumerated"

        self._logger.info("IMU brick: found device {}".format(uid))
        self._job_queue.put(self._do_apply_config)

    # -- Sensor data -----------------------------------------------------------------------

    def _all_data_callback(
        self,
        acceleration,
        magnetic_field,
        angular_velocity,
        euler_angle,
        quaternion,
        linear_acceleration,
        gravity_vector,
        temperature,
        calibration_status,
    ):
        """
        Handle ``BrickIMUV2.CALLBACK_ALL_DATA``: convert counts and publish.

        Runs on the TinkerForge callback-processor thread. Never lets an exception escape,
        so a broken consumer can never stall or kill delivery of subsequent samples.
        """
        try:
            with self._lock:
                fusion_mode = int(self._cfg("imu_fusion_mode"))
                accel_source = self._cfg("acceleration_source")

            if accel_source == "raw":
                linear = acceleration_from_brick(acceleration)
            else:
                linear = acceleration_from_brick(linear_acceleration)

            sample = ImuSample(
                orientation=quaternion_from_brick(quaternion),
                orientation_valid=(fusion_mode != 0),
                angular_velocity=angular_velocity_from_brick(angular_velocity),
                linear_acceleration=linear,
                magnetic_field=magnetic_field_from_brick(magnetic_field),
                gravity=acceleration_from_brick(gravity_vector),
                euler=euler_from_brick(euler_angle),
                temperature=float(temperature),
                calibration=decode_calibration(calibration_status),
                fusion_mode=fusion_mode,
            )
            self._on_sample(sample)
        except Exception as exc:
            self._logger.error("IMU brick: ALL_DATA callback failed: {}".format(exc))

    # -- Configuration (worker thread only) --------------------------------------------------

    def _worker_loop(self):
        """Consume config jobs from the queue until a ``None`` sentinel stops it."""
        while True:
            job = self._job_queue.get()
            if job is None:
                return
            try:
                job()
            except Exception as exc:
                self._logger.error("IMU brick: config job failed: {}".format(exc))

    def _do_apply_config(self):
        """Read the device's current config and issue only the setters actually needed."""
        with self._lock:
            imu = self._imu
            config = dict(self._config)

        if imu is None:
            return

        try:
            self._apply_leds(imu, config)
            self._apply_fusion_mode(imu, config)
            self._apply_data_period(imu, config)
        except Error as exc:
            self._logger.error("IMU brick: failed to apply config: {}".format(exc))

    def _apply_leds(self, imu, config):
        """
        Converge the two independent LED booleans onto the single ``imu_leds`` config key.

        ``are_leds_on()`` (the blue "heartbeat" LEDs) and ``is_status_led_enabled()`` are
        independent on the hardware and must be read and driven separately.
        """
        desired = bool(config.get("imu_leds", DEFAULT_CONFIG["imu_leds"]))

        if imu.are_leds_on() != desired:
            if desired:
                imu.leds_on()
            else:
                imu.leds_off()

        if imu.is_status_led_enabled() != desired:
            if desired:
                imu.enable_status_led()
            else:
                imu.disable_status_led()

    def _apply_fusion_mode(self, imu, config):
        """Set the sensor fusion mode if it differs from the desired one."""
        desired = int(config.get("imu_fusion_mode", DEFAULT_CONFIG["imu_fusion_mode"]))
        if imu.get_sensor_fusion_mode() != desired:
            imu.set_sensor_fusion_mode(desired)

    def _apply_data_period(self, imu, config):
        """Set the ALL_DATA callback period from ``imu_freq`` if it differs from the current."""
        freq = config.get("imu_freq", DEFAULT_CONFIG["imu_freq"])
        if freq <= 0:
            self._logger.warning(
                "IMU brick: invalid imu_freq {}, skipping data period update".format(freq)
            )
            return
        desired_period = int(1000.0 / freq)
        if imu.get_all_data_period() != desired_period:
            imu.set_all_data_period(desired_period)
