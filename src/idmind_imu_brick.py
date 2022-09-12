#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from idmind_msgs.msg import Log
from sensor_msgs.msg import Imu
from std_msgs.msg import String
from std_srvs.srv import Trigger, TriggerResponse
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus

import threading
from tinkerforge.ip_connection import IPConnection
from tinkerforge.brick_imu_v2 import BrickIMUV2


# TODO:
# - Dynamic Params (rate)
# - Connection and reconnection

class ImuBrick(object):
    def __init__(self, verbose=5):
        self.verbose = verbose
        self.verbose_logs = verbose
        self.logging = rospy.Publisher("/idmind_logging", Log, queue_size=10)
        self.imu_pub = rospy.Publisher("~imu", Imu, queue_size=10)
        self.euler_pub = rospy.Publisher("~euler", String, queue_size=10)
        self.diag_pub = rospy.Publisher("/diagnostics", DiagnosticArray, queue_size=10)

        self.ready = False
        rospy.Service("~ready", Trigger, self.report_ready)

        # Dynamic Params
        self.r = rospy.Rate(20)
        self.host = "localhost"
        self.port = 4223
        self.tf_prefix = ""
        self.imu_frame = "imu"

        #  Connection Variables and callbacks  #
        self.ipcon = IPConnection()
        self.ipcon.register_callback(IPConnection.CALLBACK_CONNECTED, self.brick_daemon_connection)
        self.ipcon.register_callback(IPConnection.CALLBACK_DISCONNECTED, self.brick_daemon_disconnection)

        self.imu = None
        self.imu_uid = None
        self.connect_thread = None

    ###############
    #  CALLBACKS  #
    ###############
    def report_ready(self, _req):
        """ Simple Service callback to show node is ready """
        self.log("Replying to 'ready' request", 7)
        return TriggerResponse(self.ready, rospy.get_name()+" is " + ("ready" if self.ready else "not ready"))

    def brick_daemon_connection(self, reason):
        """ Callback registered for IPConnection.CALLBACK_CONNECTED """
        # Call enumerate and reconnect to IMU Brick
        self.ipcon.register_callback(IPConnection.CALLBACK_ENUMERATE, self.enumerate_callback)
        self.ipcon.enumerate()
        return True

    def brick_daemon_disconnection(self, reason):
        """ Callback registered for IPConnection.CALLBACK_DISCONNECTED """
        self.imu = None
        self.imu_iud = None
        return True

    def enumerate_callback(self, uid, con_uid, pos, hd_version, firmware_version, device_identifier, enumeration_type):
        if enumeration_type == IPConnection.ENUMERATION_TYPE_DISCONNECTED:
            self.log("Waiting for IMU Brick to connect", 7)
            return

        # Set imu_uid if any IMU is discovered, we assume that there is only one
        if device_identifier == 18:
            self.imu_uid = uid
            self.imu = BrickIMUV2(self.imu_uid, self.ipcon)
            self.imu.register_callback(self.imu.CALLBACK_ALL_DATA, self.publish_imu)
            self.imu.set_all_data_period(50)

    def publish_imu(self, acc, mag, ang_vel, euler, quat, linear_acc, gravity, temp, calibration_status):
        try:
            imu_msg = Imu()
            imu_msg.header.stamp = rospy.Time.now()
            imu_msg.header.frame_id = self.tf_prefix + self.imu_frame

            # Quaternion Orientation
            imu_msg.orientation.w = quat[0]/16383.0
            imu_msg.orientation.x = quat[1]/16383.0
            imu_msg.orientation.y = quat[2]/16383.0
            imu_msg.orientation.z = quat[3]/16383.0
            imu_msg.orientation_covariance = [
            0.001, 0, 0,
            0, 0.001, 0,
            0, 0, 0.001
            ]

            # Angular Velocity
            imu_msg.angular_velocity.x = ang_vel[0]/16.0
            imu_msg.angular_velocity.y = ang_vel[1]/16.0
            imu_msg.angular_velocity.z = ang_vel[2]/16.0
            imu_msg.angular_velocity_covariance = [0.0] * 9
            imu_msg.angular_velocity_covariance[0] = 0.01
            imu_msg.angular_velocity_covariance[4] = 0.01
            imu_msg.angular_velocity_covariance[8] = 0.05

            # Linear Acceleration
            imu_msg.linear_acceleration.x = linear_acc[0]
            imu_msg.linear_acceleration.y = linear_acc[1]
            imu_msg.linear_acceleration.z = linear_acc[2]
            imu_msg.linear_acceleration_covariance = [0.0] * 9
            imu_msg.linear_acceleration_covariance[0] = 0.01
            imu_msg.linear_acceleration_covariance[4] = 0.01
            imu_msg.linear_acceleration_covariance[8] = 0.05

            # Message publishing
            self.imu_pub.publish(imu_msg)
            y = euler[0]/16.0
            r = euler[1]/16.0
            p = euler[2]/16.0
            self.euler_pub.publish("Roll: {} | Pitch: {} | Yaw: {}".format(r, p, y))
        except Exception as err:
            self.log(err, 2, alert="error")

    #########################
    #  AUXILIARY FUNCTIONS  #
    #########################
    def log(self, msg, msg_level, log_level=-1, alert="info"):
        """
        Log function that publish in screen and in topic
        :param msg: Message to be published
        :param msg_level: Message level (1-10, where 1 is most important)
        :param log_level: Message level for logging (1-10, optional, -1 uses the same as msg_level)
        :param alert: Alert level of message - "info", "warn" or "error"
        :return:
        """
        if self.verbose >= msg_level:
            if alert == "info":
                rospy.loginfo("{}: {}".format(rospy.get_name(), msg))
            elif alert == "warn":
                rospy.logwarn("{}: {}".format(rospy.get_name(), msg))
            elif alert == "error":
                rospy.logerr("{}: {}".format(rospy.get_name(), msg))
        if self.verbose_logs >= (log_level if log_level != -1 else msg_level):
            self.logging.publish(rospy.Time.now().to_sec(), rospy.get_name(), msg)

    def publish_diagnostic(self, level, message):
        """ Auxiliary method to publish Diagnostic messages """
        diag_msg = DiagnosticArray()
        diag_msg.header.frame_id = "imu"
        diag_msg.header.stamp = rospy.Time.now()
        imu_msg = DiagnosticStatus()
        imu_msg.name = "IMU"
        imu_msg.hardware_id = "IMU Brick IMU"
        imu_msg.level = level
        imu_msg.message = message
        diag_msg.status.append(imu_msg)
        self.diag_pub.publish(diag_msg)

    def connect_brick_daemon(self):
        """ Non-blocking method to connect to BrickDaemon """
        # IPConnection.connect() is blocking, so call on different thread
        t = threading.Thread(target=self.ipcon.connect, name="connect_brick_daemon", args=(self.host, self.port))
        t.start()
        return True

    def start(self):

        while not rospy.is_shutdown():
            try:
                # Check Daemon Connection
                conn_state = self.ipcon.get_connection_state()
                if conn_state != 1:
                    self.log("Connecting to Brick Daemon", 2, alert="warn")
                    if conn_state == 0:
                        self.connect_brick_daemon()
                # Check IMU Brick Connection
                elif self.imu_uid is None:
                    self.log("Looking for for IMU Brick", 5)
                    self.ipcon.enumerate()
                # Compute IMU stuff
                else:
                    self.log("Doing IMU Stuff", 5)

                self.r.sleep()
            except KeyboardInterrupt:
                self.log("Shutting down by user", 2)
                break

        
if __name__ == "__main__":
    rospy.init_node("idmind_imu")
    i = ImuBrick()
    i.start()




# HOST = "localhost"
# PORT = 4223
# UID = "6LeW45"  # Change XXYYZZ to the UID of your IMU Brick 2.0


# def cb_all_data(acceleration, magnetic_field, angular_velocity, euler_angle, quaternion,
#                 linear_acceleration, gravity_vector, temperature, calibration_status):
#     print("Acceleration [X]: " + str(acceleration[0]/100.0) + " m/s²")
#     print("Acceleration [Y]: " + str(acceleration[1]/100.0) + " m/s²")
#     print("Acceleration [Z]: " + str(acceleration[2]/100.0) + " m/s²")
#     print("Magnetic Field [X]: " + str(magnetic_field[0]/16.0) + " µT")
#     print("Magnetic Field [Y]: " + str(magnetic_field[1]/16.0) + " µT")
#     print("Magnetic Field [Z]: " + str(magnetic_field[2]/16.0) + " µT")
#     print("Angular Velocity [X]: " + str(angular_velocity[0]/16.0) + " °/s")
#     print("Angular Velocity [Y]: " + str(angular_velocity[1]/16.0) + " °/s")
#     print("Angular Velocity [Z]: " + str(angular_velocity[2]/16.0) + " °/s")
#     print("Euler Angle [Heading]: " + str(euler_angle[0]/16.0) + " °")
#     print("Euler Angle [Roll]: " + str(euler_angle[1]/16.0) + " °")
#     print("Euler Angle [Pitch]: " + str(euler_angle[2]/16.0) + " °")
#     print("Quaternion [W]: " + str(quaternion[0]/16383.0))
#     print("Quaternion [X]: " + str(quaternion[1]/16383.0))
#     print("Quaternion [Y]: " + str(quaternion[2]/16383.0))
#     print("Quaternion [Z]: " + str(quaternion[3]/16383.0))
#     print("Linear Acceleration [X]: " + str(linear_acceleration[0]/100.0) + " m/s²")
#     print("Linear Acceleration [Y]: " + str(linear_acceleration[1]/100.0) + " m/s²")
#     print("Linear Acceleration [Z]: " + str(linear_acceleration[2]/100.0) + " m/s²")
#     print("Gravity Vector [X]: " + str(gravity_vector[0]/100.0) + " m/s²")
#     print("Gravity Vector [Y]: " + str(gravity_vector[1]/100.0) + " m/s²")
#     print("Gravity Vector [Z]: " + str(gravity_vector[2]/100.0) + " m/s²")
#     print("Temperature: " + str(temperature) + " °C")
#     print("Calibration Status: " + format(calibration_status, "08b"))
#     print("")


# if __name__ == "__main__":
#     ipcon = IPConnection()          # Create IP connection
#     imu = BrickIMUV2(UID, ipcon)    # Create device object

#     ipcon.connect(HOST, PORT)       # Connect to brickd
#     # Don't use device before ipcon is connected

#     # Register all data callback to function cb_all_data
#     imu.register_callback(imu.CALLBACK_ALL_DATA, cb_all_data)

#     # Set period for all data callback to 0.1s (100ms)
#     imu.set_all_data_period(100)

#     input("Press key to exit\n")    # Use raw_input() in Python 2
#     ipcon.disconnect()
