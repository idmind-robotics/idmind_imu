#!/usr/bin/env python

import numpy as np
from scipy.signal import butter, lfilter, freqz
from serial import SerialException

import rospy
from std_msgs.msg import String, Int32
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Quaternion
from tf_conversions import transformations
from std_srvs.srv import Trigger, TriggerResponse
from idmind_imu.cfg import ImuParamsConfig
import dynamic_reconfigure.server

from idmind_serial2.idmind_serialport import IDMindSerial
from idmind_msgs.msg import Log

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus

# TESTING RESTARTING PORT
import os
import fcntl

VERBOSE = 5
LOGS = 5


# Butterwoth Low pass filter for linear acceleration
def butter_lowpass(cutoff, fs, order=5):
    nyq = 0.5 * fs
    normal_cutoff = cutoff / nyq
    b, a = butter(order, normal_cutoff, btype='low', analog=False)
    return b, a


def butter_lowpass_filter(data, cutoff, fs, order=5):
    b, a = butter_lowpass(cutoff, fs, order=order)
    y = lfilter(b, a, data)
    return y


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

        self.euler = [0.0]*3
        self.imu_data = ""
        self.calibration = True
        self.last_imu = [Imu()]*10
        self.acc_hist = [[0, 0, 9.82]] * 100
        self.acc_ratio = 1.0
        self.tf_prefix = rospy.get_param("~tf_prefix", "")

        self.imu_pub = rospy.Publisher("~imu", Imu, queue_size=10)
        self.imu_euler_pub = rospy.Publisher("~euler_string", String, queue_size=10)

        self.baudrate = rospy.get_param("~baudrate", 230400)

        # Connect to IMU
        self.fails = 0
        self.ser = None
        self.connection()

        self.imu_rate = rospy.get_param("~imu_rate", 100)
        self.control_freq = rospy.get_param("~control_freq", 100)
        self.dynamic_server = dynamic_reconfigure.server.Server(ImuParamsConfig, callback=self.update_imu_params)
        self.update_rates = True

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
                self.ser = IDMindSerial("/dev/idmind-artemis", baudrate=self.baudrate, timeout=0.5)
                self.log("OpenLog Artemis found.", 4)
                connected = True
            except SerialException as err:
                if err.errno == 2:
                    self.log("Openlog Artemis IMU not found", 2, alert="warn")
                    rospy.sleep(1)
                elif "Inappropriate ioctl" in str(err):
                    self.log("Restarting USB port", 2, alert="error")
                    fd = os.open("/dev/idmind-artemis", os.O_WRONLY)
                    # try:
                    #     fcntl.ioctl(fd, USBDEVFS_RESET, 0)
                    # finally:
                    os.close(fd)
                    rospy.sleep(1)
                else:
                    self.log("Exception connecting to IMU: {}".format(err), 2, alert="warn")
                    rospy.sleep(1)

        return connected

    def update_imu_params(self, config, level):
        """ Callback to an update in the motor dynamic parameters of the platform """
        self.log("Updating: {} to {} and {} to {}".format(self.control_freq, config.control_freq, self.imu_rate, config.imu_freq), 2)
        self.control_freq = config.control_freq
        self.rate = rospy.Rate(config.control_freq)
        if self.imu_rate != config.imu_freq:
            self.update_rates = True
            self.imu_rate = config.imu_freq
        return config

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
        activated = False
        calibrated = False
        reads = 0
        # Wait for the IMU to start outputting values
        while not activated and not rospy.is_shutdown():
            try:
                data = bytearray(self.ser.readline()).decode("latin")
                if len(data.split(",")) == 16:
                    self.log("IMU is outputting values", 7)
                    activated = True
                else:
                    reads = reads + 1
                    print(data)
                if reads > 50:
                    self.log("IMU is not outputting values. Restarting.", 2, alert="warn")
                    self.connection()
                    reads = 0
            except KeyboardInterrupt:
                raise KeyboardInterrupt()
            finally:
                self.rate.sleep()

        # Wait for yaw values to stabilize
        # rtcDate,rtcTime,Q6_1,Q6_2,Q6_3,RawAX,RawAY,RawAZ,RawGX,RawGY,RawGZ,RawMX,RawMY,RawMZ,output_Hz,\r\n
        reads = 0
        th_hist = [1e5]*3
        while not calibrated and not rospy.is_shutdown():
            try:
                data = bytearray(self.ser.readline()).decode("latin").split(",")
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
                    acc_x = float(data[5])
                    acc_y = float(data[6])
                    acc_z = float(data[7])
                    self.acc_ratio = 9.82/np.linalg.norm([acc_x, acc_y, acc_z])
                    calibrated = True
                else:
                    self.log("IMU is calibrating.", 7)
                    reads = reads + 1
            except KeyboardInterrupt:
                raise KeyboardInterrupt()
            finally:
                self.rate.sleep()
        self.calibration = False
        return True

    def set_imu_rate(self):
        """ This method must send the appropiate message to set a new rate for IMU """
        # Send character to enter menu
        serialcmd = "a\r\n"
        self.ser.write(serialcmd.encode())
        rospy.sleep(1)
        # Send character '1' to enter "Confirue Terminal Output"
        serialcmd = "1\r\n"
        self.ser.write(serialcmd.encode())
        rospy.sleep(1)
        # Send character '4' to enter "Set IMU Rate"
        serialcmd = "4\r\n"
        self.ser.write(serialcmd.encode())
        rospy.sleep(1)
        # Send rate in Hz
        serialcmd = "{}\r\n".format(self.imu_rate)
        self.ser.write(serialcmd.encode())
        rospy.sleep(1)
        serialcmd = "x\r\n"
        self.ser.write(serialcmd.encode())
        rospy.sleep(1)
        serialcmd = "x\r\n"
        self.ser.write(serialcmd.encode())
        rospy.sleep(1)

        # Wait for IMU to publish compliant messages
        self.ser.reset_input_buffer()
        self.ser.reset_output_buffer()
        return True

    def compute_imu_msg(self):
        """
            This method reads data from the Openlog Artemis output.
            We receive 3 coord Quaternion, which causes discontinuities in rotation.
            RPY coords are stored and reused to compute a new quaternion
        """
        # Get data
        # rtcDate,rtcTime,Q6_1,Q6_2,Q6_3,RawAX,RawAY,RawAZ,RawGX,RawGY,RawGZ,RawMX,RawMY,RawMZ,output_Hz,\r\n
        try:
            data = bytearray(self.ser.readline()).decode("latin").split(",")
        except SerialException:
            self.log("Error reading data from IMU", 2, alert="warn")
            self.fails = self.fails + 1
            if self.fails > 10:
                self.connection()
            return
        else:
            self.fails = 0

        if len(data) < 16:
            self.log("IMU Communication failed", 4, alert="warn")
            return

        # Compute Absolute Quaternion
        try:
            q = [float(data[2]), float(data[3]), float(data[4]), 0]
            if ((q[0] * q[0]) + (q[1] * q[1]) + (q[2] * q[2])) > 1.0:
                self.log("Inconsistent IMU readings", 4, alert="warn")
                return
            q[3] = np.sqrt(1.0 - ((q[0] * q[0]) + (q[1] * q[1]) + (q[2] * q[2])))
        except ValueError:
            self.log("Error converting IMU message - {}".format(data), 5, alert="warn")
            return

        # Compute Linear Acceleration
        acc = [round(float(data[5])*self.acc_ratio, 3), round(float(data[6])*self.acc_ratio, 3), round(float(data[7])*self.acc_ratio, 3)]
        self.acc_hist.pop(0)
        self.acc_hist.append(acc)

        # Compute Angular Velocity
        # w_x = float(data[8])*3.14/180
        # w_y = float(data[9])*3.14/180
        # w_z = float(data[10])*3.14/180
        # Compute Angular Velocity from Quat6
        lq = self.last_imu[-1].orientation
        euler1 = transformations.euler_from_quaternion([lq.x, lq.y, lq.z, lq.w])
        euler2 = transformations.euler_from_quaternion(q)
        curr_time = rospy.Time.now()
        dt = (curr_time - self.last_imu[-1].header.stamp).to_sec()
        w = []
        for i in range(0, 3):
            dth = euler2[i] - euler1[i]
            # The IMU Quaternion jumps need to be handled
            while (3.14 < dth) or (dth < -3.14):
                dth = dth - np.sign(dth)*2*np.pi
            # Keep euler angles in [-2p and 2pi]
            self.euler[i] += dth
            while (2*np.pi < self.euler[i]) or (self.euler[i] < -2*np.pi):
                self.euler[i] = self.euler[i] - np.sign(self.euler[i])*2*np.pi
            w.append(round(dth/dt, 4))

        q_est = transformations.quaternion_from_euler(self.euler[0], self.euler[1], self.euler[2])
        new_q = Quaternion()
        new_q.x = q_est[0]
        new_q.y = q_est[1]
        new_q.z = q_est[2]
        new_q.w = q_est[3]

        # Compute IMU Msg
        imu_msg = Imu()
        imu_msg.header.frame_id = self.tf_prefix+"imu"
        imu_msg.header.stamp = curr_time
        imu_msg.orientation = new_q
        # Set the sensor covariances
        imu_msg.orientation_covariance = [
           0.001, 0, 0,
           0, 0.001, 0,
           0, 0, 0.001
        ]

        # Angular Velocity
        imu_msg.angular_velocity.x = w[0]
        imu_msg.angular_velocity.y = w[1]
        imu_msg.angular_velocity.z = w[2]
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
        imu_msg.angular_velocity_covariance[0] = -1
        imu_msg.angular_velocity_covariance[0] = 0.01
        imu_msg.angular_velocity_covariance[4] = 0.01
        imu_msg.angular_velocity_covariance[8] = 0.05
        # imu_msg.angular_velocity_covariance = [-1] * 9

        # Linear Acceleration
        acc = [0, 0, 0]
        for idx in range(0, 3):
            data = [a[idx] for a in self.acc_hist]
            res = butter_lowpass_filter(data, cutoff=0.5, fs=10.0, order=1)
            acc[idx] = res[-1]
        imu_msg.linear_acceleration.x = acc[0]
        imu_msg.linear_acceleration.y = acc[1]
        imu_msg.linear_acceleration.z = acc[2]
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
        imu_msg.linear_acceleration_covariance[0] = 0.01
        imu_msg.linear_acceleration_covariance[4] = 0.01
        imu_msg.linear_acceleration_covariance[8] = 0.05

        # Message publishing
        self.imu_pub.publish(imu_msg)
        new_q = imu_msg.orientation
        [r, p, y] = transformations.euler_from_quaternion([new_q.x, new_q.y, new_q.z, new_q.w])
        self.imu_euler_pub.publish("Roll: {} | Pitch: {} | Yaw: {}".format(r, p, y))
        self.last_imu.pop(0)
        self.last_imu.append(imu_msg)

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
        self.rate = rospy.Rate(self.control_freq)

        while not rospy.is_shutdown():
            try:
                if self.calibration:
                    self.calibrate_imu()
                elif self.update_rates:
                    self.set_imu_rate()
                    self.update_rates = False
                else:
                    self.compute_imu_msg()
                self.publish_diagnostic(0, "OK")
                self.rate.sleep()
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
