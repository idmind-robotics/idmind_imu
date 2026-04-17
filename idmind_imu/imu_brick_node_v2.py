#! /usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from rcl_interfaces.msg import ParameterDescriptor, SetParametersResult

from std_srvs.srv import Trigger
from std_msgs.msg import Float32, UInt8MultiArray
from sensor_msgs.msg import MagneticField, Temperature, Imu
from geometry_msgs.msg import Vector3Stamped
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus

# IMU Brick API
from tinkerforge.ip_connection import IPConnection
from tinkerforge.brick_imu_v2 import BrickIMUV2

# Other py Packages
import threading
import time
from tf_transformations import euler_from_quaternion


class IDMindImuBrick(Node):
    def __init__(self):
        super().__init__("idmind_imu")
        self.ready = False
        self.verbose = 3
        node_prefix = self.get_name() + "/"
        self.last_msg = ""
        self.last_msg_ts = self.get_clock().now().to_msg()
        self.pub_callbacks = ReentrantCallbackGroup()
        self.srv_callbacks = ReentrantCallbackGroup()
        self.main_callback_group = ReentrantCallbackGroup()

        # Parameters
        self.host = "localhost"
        self.port = 4223

        self.control_freq = self.declare_parameter(
            "control_freq", 20.0,
            ParameterDescriptor(description="Frequency of the main loop")
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
            "auto_reconnect", False,
            ParameterDescriptor(description="Enable TinkerForge library auto-reconnect")
        ).get_parameter_value().bool_value
        self.add_on_set_parameters_callback(self.update_parameters)

        # Thread safety for shared IMU references
        self._imu_lock = threading.Lock()
        # Tracks when update_config() last ran to rate-limit hardware I/O
        self._last_config_update = 0.0
        # Holds the one-shot timer used to delay config after enumeration
        self._config_timer = None
        # Flag to gracefully stop the main loop during shutdown
        self._shutdown_in_progress = False

        # Services
        self.create_service(Trigger, node_prefix + "ready", self.report_ready,
                            callback_group=self.srv_callbacks)
        # Publishers
        self.imu_pub = self.create_publisher(Imu, node_prefix + "imu", 10,
                                             callback_group=self.pub_callbacks)
        self.temp_pub = self.create_publisher(Temperature, node_prefix + "temperature", 10,
                                              callback_group=self.pub_callbacks)
        self.mag_pub = self.create_publisher(MagneticField, node_prefix + "magnetic_field", 10,
                                             callback_group=self.pub_callbacks)
        self.euler_pub = self.create_publisher(Float32, node_prefix + "euler", 10,
                                               callback_group=self.pub_callbacks)
        self.gravity_pub = self.create_publisher(Vector3Stamped, node_prefix + "gravity", 10,
                                                 callback_group=self.pub_callbacks)
        self.calib_pub = self.create_publisher(UInt8MultiArray, node_prefix + "calibration", 10,
                                               callback_group=self.pub_callbacks)
        self.diag_pub = self.create_publisher(DiagnosticArray, "/diagnostics", 10,
                                              callback_group=self.pub_callbacks)
        self.looper_pub = self.create_publisher(Float32, node_prefix + "timer", 10,
                                                callback_group=self.pub_callbacks)

        # IMU state — always access self.imu and self.imu_uid under _imu_lock
        self.t = None
        self.imu_uid = None
        self.imu = None
        self.ipcon = IPConnection()
        self.ipcon.set_auto_reconnect(self.auto_reconnect)
        self.ipcon.register_callback(IPConnection.CALLBACK_CONNECTED, self.brick_daemon_connection)
        self.ipcon.register_callback(IPConnection.CALLBACK_DISCONNECTED, self.brick_daemon_disconnection)

        self.last_imu_msg = None
        self.in_loop = False
        self.main_loop_timer = self.create_timer(
            1.0 / self.control_freq, self.main_loop,
            callback_group=self.main_callback_group
        )

        self.ready = True
        self.log("Node is initialized.", 2)

    ###################
    #    CALLBACKS    #
    ###################

    def report_ready(self, _req, resp):
        """Simple service callback to confirm the node is alive."""
        self.log("Replying to 'ready' request", 2)
        resp.success = self.ready
        resp.message = self.get_name() + " is " + ("ready" if self.ready else "not ready")
        return resp

    def update_parameters(self, params):
        msg = "\n".join(["{} to {}".format(p.name, p.value) for p in params])
        self.log("Changing parameters: {}".format(msg), 2)
        for p in params:
            if p.name == "control_freq":
                self.control_freq = p.value
                self.main_loop_timer.timer_period_ns = int(1e9 / self.control_freq)
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
                self.ipcon.set_auto_reconnect(self.auto_reconnect)
        return SetParametersResult(successful=True)

    def brick_daemon_connection(self, reason):
        """Registered for IPConnection.CALLBACK_CONNECTED."""
        self.log("Connected to BrickDaemon", 2)
        self.ipcon.register_callback(IPConnection.CALLBACK_ENUMERATE, self.enumerate_callback)
        self.ipcon.enumerate()
        return True

    def brick_daemon_disconnection(self, reason):
        """Registered for IPConnection.CALLBACK_DISCONNECTED."""
        self.log("Disconnected from BrickDaemon", 2, alert="warn")
        with self._imu_lock:
            self.imu = None
            self.imu_uid = None
        return True

    def enumerate_callback(self, uid, con_uid, pos, hd_version, firmware_version,
                           device_identifier, enumeration_type):
        """Registered for IPConnection.CALLBACK_ENUMERATE."""
        if enumeration_type == IPConnection.ENUMERATION_TYPE_DISCONNECTED:
            self.log("IMU Brick disconnected during enumeration", 2)
            return

        if device_identifier == 18:  # BrickIMUV2
            self.log("IMU Brick found (uid={})".format(uid), 2)
            with self._imu_lock:
                self.imu_uid = uid
                self.imu = BrickIMUV2(self.imu_uid, self.ipcon)
                self.imu.register_callback(self.imu.CALLBACK_ALL_DATA, self.publish_imu)
            # Delay config by 2 s without blocking this callback thread
            if self._config_timer is not None:
                self._config_timer.cancel()
            self._config_timer = self.create_timer(
                2.0, self._apply_config_once,
                callback_group=self.main_callback_group
            )

    def _apply_config_once(self):
        """One-shot ROS timer: apply IMU config after the enumeration warm-up delay."""
        self.update_config()
        if self._config_timer is not None:
            self._config_timer.cancel()
            self._config_timer = None

    def publish_imu(self, acc, mag, ang_vel, euler, quat, linear_acc, gravity, temp,
                    calibration_status):
        """Registered for BrickIMUV2.CALLBACK_ALL_DATA — runs in a TinkerForge thread."""
        try:
            now = self.get_clock().now().to_msg()

            # --- IMU message ---
            imu_msg = Imu()
            imu_msg.header.stamp = now
            imu_msg.header.frame_id = self.imu_frame

            q = [quat[1] / 16383.0, quat[2] / 16383.0, quat[3] / 16383.0, quat[0] / 16383.0]
            [r, p, y] = euler_from_quaternion(q)

            imu_msg.orientation.x = q[0]
            imu_msg.orientation.y = q[1]
            imu_msg.orientation.z = q[2]
            imu_msg.orientation.w = q[3]
            imu_msg.orientation_covariance = [
                1e-4, 0.0, 0.0,
                0.0, 1e-4, 0.0,
                0.0, 0.0, 1e-4
            ]

            # Angular velocity — raw unit: 1/16 deg/s → rad/s
            imu_msg.angular_velocity.x = (ang_vel[0] / 16.0) * 3.14159 / 180.0
            imu_msg.angular_velocity.y = (ang_vel[1] / 16.0) * 3.14159 / 180.0
            imu_msg.angular_velocity.z = (ang_vel[2] / 16.0) * 3.14159 / 180.0
            var = pow(0.3 * 3.14159 / 180.0, 2)
            imu_msg.angular_velocity_covariance = [0.03 * var] * 9
            imu_msg.angular_velocity_covariance[0] = var
            imu_msg.angular_velocity_covariance[4] = var
            imu_msg.angular_velocity_covariance[8] = var

            # Linear acceleration — raw unit: 1/100 m/s²
            imu_msg.linear_acceleration.x = linear_acc[0] / 100.0
            imu_msg.linear_acceleration.y = linear_acc[1] / 100.0
            imu_msg.linear_acceleration.z = linear_acc[2] / 100.0
            imu_msg.linear_acceleration_covariance = [0.0] * 9
            imu_msg.linear_acceleration_covariance[0] = 0.01
            imu_msg.linear_acceleration_covariance[4] = 0.01
            imu_msg.linear_acceleration_covariance[8] = 0.05

            # --- Temperature ---
            tmsg = Temperature()
            tmsg.header.stamp = now
            tmsg.header.frame_id = self.imu_frame
            tmsg.temperature = float(temp)

            # --- Magnetic field — raw unit: 1/(16 * 1e6) T ---
            mag_msg = MagneticField()
            mag_msg.header.stamp = now
            mag_msg.header.frame_id = self.imu_frame
            mag_msg.magnetic_field.x = mag[0] / (16 * 1e6)
            mag_msg.magnetic_field.y = mag[1] / (16 * 1e6)
            mag_msg.magnetic_field.z = mag[2] / (16 * 1e6)
            mag_msg.magnetic_field_covariance = [pow(0.6e-6, 2)] * 9

            # --- Gravity vector — raw unit: 1/100 m/s² ---
            gravity_msg = Vector3Stamped()
            gravity_msg.header.stamp = now
            gravity_msg.header.frame_id = self.imu_frame
            gravity_msg.vector.x = gravity[0] / 100.0
            gravity_msg.vector.y = gravity[1] / 100.0
            gravity_msg.vector.z = gravity[2] / 100.0

            # --- Calibration status [sys, gyro, acc, mag], each 0–3 ---
            # calibration_status is a uint8 bitmask: bits[7:6]=sys,[5:4]=gyro,[3:2]=acc,[1:0]=mag
            cal_sys  = (calibration_status >> 6) & 0x03
            cal_gyro = (calibration_status >> 4) & 0x03
            cal_acc  = (calibration_status >> 2) & 0x03
            cal_mag  = (calibration_status >> 0) & 0x03
            calib_list = [cal_sys, cal_gyro, cal_acc, cal_mag]
            calib_msg = UInt8MultiArray()
            calib_msg.data = calib_list

            # Publish all
            self.imu_pub.publish(imu_msg)
            self.temp_pub.publish(tmsg)
            self.mag_pub.publish(mag_msg)
            self.gravity_pub.publish(gravity_msg)
            self.calib_pub.publish(calib_msg)

            euler_msg = Float32()
            euler_msg.data = y
            self.euler_pub.publish(euler_msg)

            self.last_imu_msg = self.get_clock().now().to_msg()

            # Publish diagnostics — warn if any calibration component is 0
            if min(calib_list) == 0:
                self.publish_diagnostic(
                    DiagnosticStatus.WARN,
                    "IMU partially uncalibrated: sys={} gyro={} acc={} mag={}".format(
                        *calib_list)
                )
            else:
                self.publish_diagnostic(DiagnosticStatus.OK, "IMU is OK")

        except Exception as err:
            self.log(str(err), 2, alert="error")

    #########################
    #  AUXILIARY FUNCTIONS  #
    #########################

    def getDt(self, last):
        current_time = self.get_clock().now().to_msg()
        ct = current_time.sec + (current_time.nanosec / 1e9)
        lt = last.sec + (last.nanosec / 1e9)
        return ct - lt

    def log(self, msg, msg_level, log_level=-1, alert="info"):
        """Rate-limited logging: suppresses duplicate messages within 1 second."""
        msg = str(msg)
        if (self.last_msg == msg) and self.getDt(self.last_msg_ts) < 1:
            return False
        if self.verbose >= msg_level:
            if alert == "info":
                self.get_logger().info("{}: {}".format(self.get_name(), msg))
            elif alert == "warn":
                self.get_logger().warning("{}: {}".format(self.get_name(), msg))
            elif alert == "error":
                self.get_logger().error("{}: {}".format(self.get_name(), msg))
            self.last_msg = msg
            self.last_msg_ts = self.get_clock().now().to_msg()

    def publish_diagnostic(self, level, message):
        diag_msg = DiagnosticArray()
        diag_msg.header.frame_id = "imu"
        diag_msg.header.stamp = self.get_clock().now().to_msg()
        imu_status = DiagnosticStatus()
        imu_status.name = "IMU"
        imu_status.hardware_id = "IMU Brick V2"
        imu_status.level = level
        imu_status.message = message
        diag_msg.status.append(imu_status)
        self.diag_pub.publish(diag_msg)

    ###################
    #    MAIN LOOP    #
    ###################

    def shutdown(self):
        """Disconnect from BrickDaemon and clean up the connection thread."""
        # Signal the main_loop to exit immediately
        self._shutdown_in_progress = True

        # Cancel the main loop timer to stop callbacks from firing during shutdown
        try:
            self.main_loop_timer.cancel()
        except Exception:
            pass

        # Cancel any pending config timer
        if self._config_timer is not None:
            try:
                self._config_timer.cancel()
            except Exception:
                pass
            self._config_timer = None

        # Disconnect from BrickDaemon
        try:
            self.ipcon.disconnect()
        except Exception as err:
            self.log("Exception during shutdown: {}".format(err), 1, alert="error")

        # Wait for the connection thread to finish
        if self.t is not None and self.t.is_alive():
            self.t.join(timeout=2.0)
        return True

    def connect_brick_daemon(self):
        """Spawn a background thread for the initial (blocking) ipcon.connect() call.

        Guards against spawning multiple threads if a connection attempt is already
        in progress.
        """
        if self.t is not None and self.t.is_alive():
            return False  # Already connecting
        self.t = threading.Thread(
            target=self._connect_with_exception_handling,
            name="connect_brick_daemon",
        )
        self.t.daemon = True
        self.t.start()
        return True

    def _connect_with_exception_handling(self):
        """Wrapper for ipcon.connect() that handles exceptions gracefully."""
        try:
            self.ipcon.connect(self.host, self.port)
        except Exception:
            # Connection failed (expected if daemon isn't running).
            # The main_loop will retry on next iteration.
            pass

    def update_config(self):
        """Read current IMU config and apply any pending parameter changes.

        Acquires a local reference to self.imu under the lock so hardware I/O
        is performed outside the lock.
        """
        with self._imu_lock:
            imu = self.imu
        if imu is None:
            return False

        try:
            imu_freq = imu.get_all_data_period()
            imu_leds = imu.is_status_led_enabled() or imu.are_leds_on()
            fusion_mode = imu.get_sensor_fusion_mode()
        except Exception:
            self.log("Exception reading config from IMU", 1, alert="error")
            return False

        try:
            imu_freq_goal = int(1000.0 / self.imu_freq)
            if imu_freq != imu_freq_goal:
                self.log("Changing data period from {} to {}".format(imu_freq, imu_freq_goal), 2)
                imu.set_all_data_period(imu_freq_goal)

            if imu_leds != self.imu_leds:
                if self.imu_leds:
                    self.log("Turning LEDs ON", 2)
                    imu.leds_on()
                    imu.enable_status_led()
                else:
                    self.log("Turning LEDs OFF", 2)
                    imu.leds_off()
                    imu.disable_status_led()

            if fusion_mode != self.imu_fusion_mode:
                self.log("Changing Sensor Fusion mode to {}".format(self.imu_fusion_mode), 2)
                imu.set_sensor_fusion_mode(self.imu_fusion_mode)

            self._last_config_update = time.time()
            return True

        except Exception as err:
            self.log("Exception applying IMU config: {}".format(err), 2, alert="warn")
            return False

    def main_loop(self):
        try:
            # Exit early if shutdown has been initiated
            if self._shutdown_in_progress or not rclpy.ok():
                if self.main_loop_timer is not None:
                    self.main_loop_timer.cancel()
                return

            if self.in_loop:
                return True
            self.in_loop = True

            conn_state = self.ipcon.get_connection_state()

            if conn_state == 0:  # CONNECTION_STATE_DISCONNECTED
                self.log("Connecting to Brick Daemon", 2)
                self.connect_brick_daemon()

            elif conn_state == 2:  # CONNECTION_STATE_PENDING
                self.log("Waiting for Brick Daemon connection...", 3)

            else:  # CONNECTION_STATE_CONNECTED
                with self._imu_lock:
                    imu_uid = self.imu_uid

                if imu_uid is None:
                    self.log("Looking for IMU Brick", 2)
                    self.publish_diagnostic(DiagnosticStatus.WARN, "Looking for IMU Brick")
                    self.ipcon.enumerate()
                else:
                    if self.last_imu_msg is None or self.getDt(self.last_imu_msg) > self.timeout:
                        self.log("No data from IMU Brick", 2, alert="warn")
                        self.publish_diagnostic(DiagnosticStatus.ERROR, "No data from IMU Brick")
                    elif time.time() - self._last_config_update > 1.0:
                        # Rate-limit config polling to once per second
                        self.update_config()

            self.looper_pub.publish(Float32())
            self.in_loop = False

        except Exception as err:
            self.log("Exception in main_loop: {}".format(err), 1, alert="error")
            if self.t is not None and self.t.is_alive():
                self.t.join(timeout=2.0)
            self.shutdown()
            self.in_loop = False


def main(args=None):
    rclpy.init(args=args)
    imu_node = IDMindImuBrick()
    executor = MultiThreadedExecutor(num_threads=6)
    executor.add_node(imu_node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        print('\033[91m' + "Shutting down IMUBrick Node" + '\033[0m')
    finally:
        executor.shutdown()
        imu_node.shutdown()
        imu_node.destroy_node()

    try:
        rclpy.shutdown()
    except Exception:
        pass


if __name__ == '__main__':
    main()
