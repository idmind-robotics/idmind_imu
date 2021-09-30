#!/usr/bin/env python

import numpy as np
from serial import SerialException

import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Quaternion
from tf_conversions import transformations
from std_srvs.srv import Trigger, TriggerResponse

from idmind_serial2.idmind_serialport import IDMindSerial
from idmind_msgs.msg import Log

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus

VERBOSE = 7
LOGS = 7


class IDMindIMU:
    """
    This class extracts data from the Sparkfun OpenLog Artemis (with ICM 20948)
    The OpenLogArtemis sketch must be uploaded to the unit. It will publish to /imu the values of orientation,
    angular velocity and linear acceleration.
    In case the connection is lost, it will try to reconnect.

    TODO: Allow for calibration of components
    """
    def __init__(self):
        # Logging
        self.ready = False
        rospy.Service("~ready", Trigger, self.report_ready)
        self.logging = rospy.Publisher("/idmind_logging", Log, queue_size=10)
        self.diag_pub = rospy.Publisher("/diagnostics", DiagnosticArray, queue_size=10)
        self.val_exc = 0

        self.imu_data = ""
        self.imu_reading = Imu()
        self.calibration = True
        self.tf_prefix = rospy.get_param("~tf_prefix", "")

        # Connect to IMU
        self.fails = 0
        self.ser = None
        self.connection()

        self.imu_pub = rospy.Publisher("~imu", Imu, queue_size=10)
        self.imu_euler_pub = rospy.Publisher("~euler_string", String, queue_size=10)

        self.ready = True
        self.log("Node is ready", 4)

    ###############
    #  CALLBACKS  #
    ###############
    def report_ready(self, _req):
        """ Simple Service callback to show node is ready """
        self.log("Replying to 'ready' request", 7)
        return TriggerResponse(self.ready, rospy.get_name()+" is " + ("ready" if self.ready else "not ready"))

    def connection(self):
        """
        Function that connects to IMU port. Repeats until found.
        :return:
        """
        connected = False
        while not connected and not rospy.is_shutdown():
            self.log("Searching for IMU", 4)
            try:
                self.ser = IDMindSerial("/dev/idmind-artemis", baudrate=115200, timeout=0.5)
                self.log("OpenLog Artemis found.", 4)
                connected = True
            except SerialException as err:
                if err.errno == 2:
                    self.log("Openlog Artemis IMU not found", 2, alert="warn")
                    rospy.sleep(1)
                else:
                    self.log("Exception connecting to IMU: {}".format(err), 2, alert="warn")
                    rospy.sleep(1)

        return connected

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
        if VERBOSE >= msg_level:
            if alert == "info":
                rospy.loginfo("{}: {}".format(rospy.get_name(), msg))
            elif alert == "warn":
                rospy.logwarn("{}: {}".format(rospy.get_name(), msg))
            elif alert == "error":
                rospy.logerr("{}: {}".format(rospy.get_name(), msg))
        if LOGS >= (log_level if log_level != -1 else msg_level):
            self.logging.publish(rospy.Time.now().to_sec(), rospy.get_name(), msg)

    def calibrate_imu(self):
        """
            This method will wait for the IMU to output values and wait for the values to get steady
            :return:
        """
        self.log("Calibrating IMU", 5)
        r = rospy.Rate(20)
        activated = False
        calibrated = False
        reads = 0
        # Wait for the IMU to start outputting values
        while not activated and not rospy.is_shutdown():
            try:
                data = self.ser.readline()
                if len(data.split(",")) == 16:
                    self.log("IMU is outputting values", 7)
                    activated = True
                else:
                    reads = reads + 1
                if reads > 50:
                    self.log("IMU is not outputting values. Restarting.", 2, alert="warn")
                    self.connection()
                    reads = 0
            except KeyboardInterrupt:
                raise KeyboardInterrupt()
            finally:
                r.sleep()

        # Wait for yaw values to stabilize
        # rtcDate,rtcTime,Q6_1,Q6_2,Q6_3,RawAX,RawAY,RawAZ,RawGX,RawGY,RawGZ,RawMX,RawMY,RawMZ,output_Hz,\r\n
        reads = 0
        th_hist = [1e5]*3
        while not calibrated and not rospy.is_shutdown():
            try:
                data = self.ser.readline().split(",")
                q = [float(data[2]), float(data[3]), float(data[4]), 0]
                if ((q[0] * q[0]) + (q[1] * q[1]) + (q[2] * q[2])) <= 1.0:
                    q[3] = np.sqrt(1.0 - ((q[0] * q[0]) + (q[1] * q[1]) + (q[2] * q[2])))
                else:
                    continue
                th_hist.pop(0)
                th_hist.append(transformations.euler_from_quaternion(q)[2])
                # Compute second derivative
                deriv = (th_hist[2]-th_hist[1]) - (th_hist[1]-th_hist[0])
                if reads > 50 and abs(deriv) < 1e-6:
                    self.log("IMU is calibrated.", 5)
                    calibrated = True
                else:
                    self.log("IMU is calibrating.", 7)
                    reads = reads + 1
            except KeyboardInterrupt:
                raise KeyboardInterrupt()
            finally:
                r.sleep()
        self.calibration = False
        return True

    def compute_imu_msg(self):
        # Get data
        # rtcDate,rtcTime,Q6_1,Q6_2,Q6_3,RawAX,RawAY,RawAZ,RawGX,RawGY,RawGZ,RawMX,RawMY,RawMZ,output_Hz,\r\n
        try:
            data = self.ser.readline().split(",")
        except SerialException:
            self.log("Error reading data from IMU", 2, alert="warn")
            self.fails = self.fails + 1
            if self.fails > 10:
                self.connection()
                return
        else:
            self.fails = 0

        # Compute Absolute Quaternion
        q = [float(data[2]), float(data[3]), float(data[4]), 0]
        if ((q[0] * q[0]) + (q[1] * q[1]) + (q[2] * q[2])) > 1.0:
            self.log("Inconsistent IMU readings", 4, alert="warn")
            #self.log("Q0: {} | Q1: {} | Q2: {}".format(q[0], q[1], q[2]), 2, alert="warn")            
            #return
            q_norm = (q[0] * q[0]) + (q[1] * q[1]) + (q[2] * q[2])
            q[0] = q[0]/q_norm
            q[1] = q[1]/q_norm
            q[2] = q[2]/q_norm

        q[3] = np.sqrt(1.0 - ((q[0] * q[0]) + (q[1] * q[1]) + (q[2] * q[2])))

        new_q = Quaternion()
        new_q.x = q[0]
        new_q.y = q[1]
        new_q.z = q[2]
        new_q.w = q[3]

        # Compute Linear Acceleration
        acc_x = float(data[5])/1000
        acc_y = float(data[6])/1000
        acc_z = float(data[7])/1000

        # Compute Angular Velocity
        w_x = float(data[8])
        w_y = float(data[9])
        w_z = float(data[10])

        # Compute IMU Msg
        imu_msg = Imu()
        imu_msg.header.frame_id = self.tf_prefix+"imu"
        imu_msg.header.stamp = rospy.Time.now()  # + rospy.Duration(0.5)
        imu_msg.orientation = new_q
        # Set the sensor covariances
        imu_msg.orientation_covariance = [
            0.0001, 0, 0,
            0, 0.0001, 0,
            0, 0, 0.0001
        ]

        # Angular Velocity
        imu_msg.angular_velocity.x = w_x
        imu_msg.angular_velocity.y = w_y
        imu_msg.angular_velocity.z = w_z
        # Datasheet says:
        # - Noise Spectral Density: 0.015dps/sqrt(Hz)
        # - Cross Axis Sensitivy: +-2%
        # diag = pow(0.015/np.sqrt(20), 2)
        # factor = 0.02
        # imu_msg.angular_velocity_covariance = [
        #    diag, w_x*factor, w_x*factor,
        #    w_y*factor, diag, w_y*factor,
        #    w_z*factor, w_z*factor, diag
        # ]
        imu_msg.angular_velocity_covariance = [0.0] * 9
        imu_msg.angular_velocity_covariance[0] = 0.0001
        imu_msg.angular_velocity_covariance[4] = 0.0001
        imu_msg.angular_velocity_covariance[8] = 0.0001
        # imu_msg.angular_velocity_covariance = [-1] * 9

        # Linear Acceleration
        imu_msg.linear_acceleration.x = acc_x
        imu_msg.linear_acceleration.y = acc_y
        imu_msg.linear_acceleration.z = acc_z
        # imu_msg.linear_acceleration.x = 0
        # imu_msg.linear_acceleration.y = 0
        # imu_msg.linear_acceleration.z = 9.82
        # imu_msg.linear_acceleration_covariance = [-1] * 9
        # Datasheet says:
        # - Noise Spectral Density: 230microg/sqrt(Hz)
        # - Cross Axis Sensitivy: +-2%
        # diag = pow(230e-6/np.sqrt(20), 2)/256.
        # factor = 0.02/256.
        # imu_msg.linear_acceleration_covariance = [
        #     diag, acc_x*factor, acc_x*factor,
        #    acc_y*factor, diag, acc_y*factor,
        #    acc_z*factor, acc_z*factor, diag
        # ]
        imu_msg.linear_acceleration_covariance = [0.0] * 9
        imu_msg.linear_acceleration_covariance[0] = 0.001
        imu_msg.linear_acceleration_covariance[4] = 0.001
        imu_msg.linear_acceleration_covariance[8] = 0.001

        # Message publishing
        self.imu_pub.publish(imu_msg)
        new_q = imu_msg.orientation
        [r, p, y] = transformations.euler_from_quaternion([new_q.x, new_q.y, new_q.z, new_q.w])
        self.imu_euler_pub.publish("Roll: {} | Pitch: {} | Yaw: {}".format(r, p, y))

    def publish_diagnostic(self, level, message):
        """ Auxiliary method to publish Diagnostic messages """
        diag_msg = DiagnosticArray()
        diag_msg.header.frame_id = "imu"
        diag_msg.header.stamp = rospy.Time.now()
        imu_msg = DiagnosticStatus()
        imu_msg.name = "IMU"
        imu_msg.hardware_id = "OpenLog Artemis IMU"
        imu_msg.level = level
        imu_msg.message = message
        diag_msg.status.append(imu_msg)
        self.diag_pub.publish(diag_msg)

    def start(self):
        r = rospy.Rate(20)

        while not rospy.is_shutdown():
            try:
                if self.calibration:
                    self.calibrate_imu()
                else:
                    self.compute_imu_msg()
                self.publish_diagnostic(0, "OK")
                r.sleep()
            except KeyboardInterrupt:
                self.log("Shutting down by user", 2)
                break
            except IOError as io_exc:
                self.publish_diagnostic(1, "Lost connection to IMU")
                self.log("Lost connection to IMU", 3)
                if not self.connection():
                    rospy.sleep(2)


if __name__ == "__main__":
    rospy.init_node("idmind_artemis")

    imu = IDMindIMU()
    imu.start()
    rospy.loginfo("{}: Node stopped".format(rospy.get_name()))
