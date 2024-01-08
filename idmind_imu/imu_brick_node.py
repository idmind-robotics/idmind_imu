#! /usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from rcl_interfaces.msg import ParameterDescriptor, SetParametersResult

from std_srvs.srv import Trigger
from std_msgs.msg import Float32
from sensor_msgs.msg import MagneticField, Temperature, Imu
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus

# IMU Brick API
from tinkerforge.ip_connection import IPConnection
from tinkerforge.brick_imu_v2 import BrickIMUV2

# Other py Packages
import threading
import numpy as np


class IDMindImuBrick(Node):
    def __init__(self):
        super().__init__("idmind_imu")
        self.ready = False
        self.verbose = 3
        node_prefix = self.get_name()+"/"
        self.last_msg = ""
        self.last_msg_ts = self.get_clock().now().to_msg()
        self.pub_callbacks = ReentrantCallbackGroup()
        self.srv_callbacks = ReentrantCallbackGroup()
        self.main_callback_group = ReentrantCallbackGroup()

        # Parameters
        self.host = "localhost"
        self.port = 4223

        self.control_freq = self.declare_parameter("control_freq", 20.0, ParameterDescriptor(description="Frequency of the main loop")).get_parameter_value().double_value
        self.imu_freq = self.declare_parameter("imu_freq", 20.0, ParameterDescriptor(description="Frequency of IMU stream")).get_parameter_value().double_value
        self.imu_frame = self.declare_parameter("imu_frame", "imu", ParameterDescriptor(description="Frame name for the IMU")).get_parameter_value().string_value
        self.imu_leds = self.declare_parameter("imu_leds", False, ParameterDescriptor(description="Enable/Disable IMU Leds")).get_parameter_value().bool_value
        self.imu_fusion_mode = self.declare_parameter("imu_fusion_mode", 2, ParameterDescriptor(description="Fusion Mode of the IMU")).get_parameter_value().integer_value
        self.timeout = self.declare_parameter("timeout", 1.0, ParameterDescriptor(description="Timeout for IMU Error")).get_parameter_value().double_value
        self.add_on_set_parameters_callback(self.update_parameters)

        # Callbacks
        # Services
        self.create_service(Trigger, node_prefix+"ready", self.report_ready, callback_group=self.srv_callbacks)
        # Publishers
        self.imu_pub = self.create_publisher(Imu, node_prefix+"imu", 10, callback_group=self.pub_callbacks)
        self.temp_pub = self.create_publisher(Temperature, node_prefix+"temperature", 10, callback_group=self.pub_callbacks)
        self.mag_pub = self.create_publisher(MagneticField, node_prefix+"magnetic_field", 10, callback_group=self.pub_callbacks)
        self.euler_pub = self.create_publisher(Float32, node_prefix+"euler", 10, callback_group=self.pub_callbacks)
        self.diag_pub = self.create_publisher(DiagnosticArray, "/diagnostics", 10, callback_group=self.pub_callbacks)
        self.looper_pub = self.create_publisher(Float32, node_prefix+"timer", 10, callback_group=self.pub_callbacks)
        # Subscribers
        
        
        # Finalizing
        self.t = None
        self.imu_uid = None
        self.ipcon = IPConnection()
        self.ipcon.register_callback(IPConnection.CALLBACK_CONNECTED, self.brick_daemon_connection)
        self.ipcon.register_callback(IPConnection.CALLBACK_DISCONNECTED, self.brick_daemon_disconnection)
        
        # Timers
        self.last_imu_msg = None
        self.main_loop_timer = self.create_timer(1.0/self.control_freq, self.main_loop, callback_group=self.main_callback_group)
        
        self.ready = True
        self.log("Node is initialized.", 2)

    ###################
    #    CALLBACKS    #
    ###################
    def report_ready(self, _req, resp):
        """ Simple Service callback to show node is ready """
        self.log("Replying to 'ready' request", 2)
        resp.success = self.ready
        resp.message = self.get_name()+" is " + ("ready" if self.ready else "not ready")
        return resp

    def update_parameters(self, params):
        msg = "\n".join(["{} to {}".format(p.name, p.value) for p in params])
        self.log("Changing parameters: {}" .format(msg), 2)
        for p in params:
            if p.name == "control_freq":
                self.control_freq = p.value
                self.main_loop_timer.timer_period_ns = 1e9/self.control_freq
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
        return SetParametersResult(successful=True)
    
    # IMU Connection Callback
    def brick_daemon_connection(self, reason):
        """ Callback registered for IPConnection.CALLBACK_CONNECTED """
        # Call enumerate and reconnect to IMU Brick
        self.log("Connected to BrickDaemon", 2)
        self.ipcon.register_callback(IPConnection.CALLBACK_ENUMERATE, self.enumerate_callback)
        self.ipcon.enumerate()
        return True
    
    # IMU Disconnection Callback
    def brick_daemon_disconnection(self, reason):
        """ Callback registered for IPConnection.CALLBACK_DISCONNECTED """
        self.log("IMU Disconnected", 2, alert="warn")
        self.imu = None
        self.imu_uid = None
        return True

    # IMU Enumeration Callback
    def enumerate_callback(self, uid, con_uid, pos, hd_version, firmware_version, device_identifier, enumeration_type):
        if enumeration_type == IPConnection.ENUMERATION_TYPE_DISCONNECTED:
            self.log("Waiting for IMU Brick to connect", 2)
            return

        # Set imu_uid if any IMU is discovered, we assume that there is only one
        if device_identifier == 18:
            self.log("IMU Brick connected to BrickDaemon", 2)
            self.imu_uid = uid
            self.imu = BrickIMUV2(self.imu_uid, self.ipcon)
            self.imu.register_callback(self.imu.CALLBACK_ALL_DATA, self.publish_imu)
            self.get_clock().sleep_for(Duration(seconds=2.0))
            self.update_config()

    def publish_imu(self, acc, mag, ang_vel, euler, quat, linear_acc, gravity, temp, calibration_status):
        try:
            
            # self.log("================", 2)
            # self.log(acc, 2)
            # self.log(mag, 2)
            # self.log(ang_vel, 2)
            # self.log(euler, 2)
            # self.log(quat, 2)
            # self.log(linear_acc, 2)
            # self.log(gravity, 2)
            # self.log(temp, 2)
            # self.log(calibration_status, 2)

            imu_msg = Imu()
            imu_msg.header.stamp = self.get_clock().now().to_msg()
            imu_msg.header.frame_id = self.imu_frame
            # Quaternion Orientation
            imu_msg.orientation.w = quat[0]/16383.0
            imu_msg.orientation.x = quat[1]/16383.0
            imu_msg.orientation.y = quat[2]/16383.0
            imu_msg.orientation.z = quat[3]/16383.0

            imu_msg.orientation_covariance = [
            0.001, 0.0, 0.0,
            0.0, 0.001, 0.0,
            0.0, 0.0, 0.001
            ]

            # Angular Velocity - noise 0.3deg/s, 3% cross axis
            imu_msg.angular_velocity.x = (ang_vel[0]/16.0)*3.14/180
            imu_msg.angular_velocity.y = (ang_vel[1]/16.0)*3.14/180
            imu_msg.angular_velocity.z = (ang_vel[2]/16.0)*3.14/180
            var = pow(0.3*3.14/180., 2)
            imu_msg.angular_velocity_covariance = [0.03*var] * 9
            imu_msg.angular_velocity_covariance[0] = var
            imu_msg.angular_velocity_covariance[4] = var
            imu_msg.angular_velocity_covariance[8] = var

            # Linear Acceleration
            imu_msg.linear_acceleration.x = linear_acc[0]/100.0
            imu_msg.linear_acceleration.y = linear_acc[1]/100.0
            imu_msg.linear_acceleration.z = linear_acc[2]/100.0
            imu_msg.linear_acceleration_covariance = [0.0] * 9
            imu_msg.linear_acceleration_covariance[0] = 0.01
            imu_msg.linear_acceleration_covariance[4] = 0.01
            imu_msg.linear_acceleration_covariance[8] = 0.05
            # Temperature
            tmsg = Temperature()
            tmsg.header.stamp = self.get_clock().now().to_msg()
            tmsg.header.frame_id = self.imu_frame
            tmsg.temperature = float(temp)
            # Magnetic Field - noise 0.6
            mag_msg = MagneticField()
            mag_msg.header.stamp = self.get_clock().now().to_msg()
            mag_msg.header.frame_id = self.imu_frame
            mag_msg.magnetic_field.x = mag[0]/(16*1e6)
            mag_msg.magnetic_field.y = mag[1]/(16*1e6)
            mag_msg.magnetic_field.z = mag[2]/(16*1e6)
            mag_msg.magnetic_field_covariance = [pow(0.6e-6,2)]*9
            # Message publishing
            self.imu_pub.publish(imu_msg)
            self.temp_pub.publish(tmsg)
            self.mag_pub.publish(mag_msg)
            y = euler[0]/16.0
            r = euler[1]/16.0
            p = euler[2]/16.0
            # self.euler_pub.publish("Roll: {} | Pitch: {} | Yaw: {}".format(r, p, y))
            msg = Float32()
            msg.data = y
            self.euler_pub.publish(msg)
            self.last_imu_msg = self.get_clock().now().to_msg()
            self.publish_diagnostic(DiagnosticStatus.OK, "IMU is OK")
        except Exception as err:
            self.log(err, 2, alert="error")
        
    #########################
    #  AUXILIARY FUNCTIONS  #
    #########################
    def getDt(self, last):
        current_time = self.get_clock().now().to_msg()

        ct = current_time.sec + (current_time.nanosec/1e+9)
        lt = last.sec + (last.nanosec/1e+9)
        dt = (ct - lt)
        return dt

    def log(self, msg, msg_level, log_level=-1, alert="info"):
        """
        Log function that publish in screen and in topic
        :param msg: Message to be published
        :param msg_level: Message level (1-10, where 1 is most important)
        :param log_level: Message level for logging (1-10, optional, -1 uses the same as msg_level)
        :param alert: Alert level of message - "info", "warn" or "error"
        :return:
        """
        if (self.last_msg == msg) and self.getDt(self.last_msg_ts) < 1:
            return False
        if self.verbose >= msg_level:
            if alert == "info":
                self.get_logger().info(
                    "{}: {}".format(self.get_name(), msg))
            elif alert == "warn":
                self.get_logger().warning(
                    "{}: {}".format(self.get_name(), msg))
            elif alert == "error":
                self.get_logger().error(
                    "{}: {}".format(self.get_name(), msg))
            self.last_msg = msg
            self.last_msg_ts = self.get_clock().now().to_msg()

    def publish_diagnostic(self, level, message):
        """ Auxiliary method to publish Diagnostic messages """
        diag_msg = DiagnosticArray()
        diag_msg.header.frame_id = "imu"
        diag_msg.header.stamp = self.get_clock().now().to_msg()
        imu_msg = DiagnosticStatus()
        imu_msg.name = "IMU"
        imu_msg.hardware_id = "IMU Brick IMU"
        imu_msg.level = level
        imu_msg.message = message
        diag_msg.status.append(imu_msg)
        self.diag_pub.publish(diag_msg)
    
    ###################
    #    MAIN LOOP    #
    ###################
    def connect_brick_daemon(self):
        """ Non-blocking method to connect to BrickDaemon """
        # IPConnection.connect() is blocking, so call on different thread
        self.t = threading.Thread(target=self.ipcon.connect, name="connect_brick_daemon", args=(self.host, self.port))
        self.t.start()
        return True

    def update_config(self):
        try:
            imu_freq = self.imu.get_all_data_period()
            imu_leds = self.imu.is_status_led_enabled() or self.imu.are_leds_on()
            fusion_mode = self.imu.get_sensor_fusion_mode()
        except Exception:
            self.log("Exception reading status from IMU", 1, alert="error")
            return False
        
        try:
            imu_freq_goal = int(1000.0/self.imu_freq)
            if imu_freq != imu_freq_goal:
                self.log("Changing data period from {} to {}".format(imu_freq, imu_freq_goal), 2)
                self.imu.set_all_data_period(imu_freq_goal)           
            
            if imu_leds != self.imu_leds:
                if self.imu_leds:
                    self.log("Turning LEDs ON", 2)
                    self.imu.leds_on()
                    self.imu.enable_status_led()
                else:
                    self.log("Turning LEDs OFF", 2)
                    self.imu.leds_off()
                    self.imu.disable_status_led()
    
            if fusion_mode != self.imu_fusion_mode:
                self.log("Changing Sensor Fusion mode to {}".format(self.imu_fusion_mode), 2)
                self.imu.set_sensor_fusion_mode(self.imu_fusion_mode)
            return True    
        
        except Exception as err:
            self.log("Exception changing status of IMU: {}".format(err), 2, alert="warn")
            return False
        
                
    def main_loop(self):
        try:
            if rclpy.ok():
                # Check Daemon Connection
                conn_state = self.ipcon.get_connection_state()
                
                if conn_state != 1:
                    self.log("Connecting to Brick Daemon", 2)
                    if conn_state == 0:
                        self.connect_brick_daemon()
                # Check IMU Brick Connection
                elif self.imu_uid is None:
                    self.log("Looking for IMU Brick", 2)
                    self.publish_diagnostic(DiagnosticStatus.WARN, "Looking for IMU Brick")
                    self.ipcon.enumerate()
                # Compute IMU stuff
                else:
                    if self.last_imu_msg is None or self.getDt(self.last_imu_msg) > self.timeout:
                        msg = "No data from IMU Brick"
                        self.log(msg, 2, alert="warn")
                        self.publish_diagnostic(DiagnosticStatus.ERROR, msg)
                    else:
                        self.update_config()
                self.looper_pub.publish(Float32())

            else:
                self.log("ROS is not OK", 1, alert="error")
        except Exception:
            print("Wait for thread to close")
            self.t.join()
            print("Closed")
            

def main(args=None):
    rclpy.init(args=args)
    try:
        imu_node = IDMindImuBrick()    
        executor = MultiThreadedExecutor(num_threads=6)
        executor.add_node(imu_node)
        try:
            executor.spin()
        finally:
            executor.shutdown()
            imu_node.shutdown()
            imu_node.destroy_node()
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
