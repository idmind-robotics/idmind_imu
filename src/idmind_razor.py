#!/usr/bin/env python

import sys
import subprocess
import serial.tools.list_ports
from serial import SerialException
from math import degrees

import rospy
import copy
import tf2_ros
from std_msgs.msg import String
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Quaternion
from tf_conversions import transformations
from std_srvs.srv import Trigger, TriggerResponse

from idmind_messages.msg import Log
from idmind_serial2.idmind_serialport import IDMindSerial
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from tf2_geometry_msgs import do_transform_pose, do_transform_vector3
from geometry_msgs.msg import Vector3Stamped, PoseStamped, TransformStamped, QuaternionStamped, Quaternion

VERBOSE = 4
LOGS = 4


def do_transform_quaternion(quaternion_msg, transform):
    # Use PoseStamped to transform the Quaternion
    src_ps = PoseStamped()
    src_ps.header = quaternion_msg.header
    src_ps.pose.orientation = quaternion_msg.quaternion
    tgt_ps = do_transform_pose(src_ps, transform)
    tgt_o = QuaternionStamped()
    tgt_o.header = tgt_ps.header
    tgt_o.quaternion = tgt_ps.pose.orientation
    return tgt_o


def do_transform_imu(imu_msg, transform):
    """
    Transforms the given Imu message into the given target frame
    :param imu_msg:
    :type imu_msg: Imu
    :param transform:
    :type transform: TransformStamped
    :return: Imu message transformed into the target frame
    :rtype: Imu
    """
    src_l = Vector3Stamped()
    src_l.header = imu_msg.header
    src_l.vector = imu_msg.linear_acceleration
    src_a = Vector3Stamped()
    src_a.header = imu_msg.header
    src_a.vector = imu_msg.angular_velocity
    src_o = QuaternionStamped()
    src_o.header = imu_msg.header
    src_o.quaternion = imu_msg.orientation

    tgt_l = do_transform_vector3(src_l, transform)
    tgt_a = do_transform_vector3(src_a, transform)
    tgt_o = do_transform_quaternion(src_o, transform)

    tgt_imu = copy.deepcopy(imu_msg)
    tgt_imu.header.frame_id = transform.child_frame_id
    tgt_imu.linear_acceleration = tgt_l.vector
    tgt_imu.angular_velocity = tgt_a.vector
    tgt_imu.orientation = tgt_o.quaternion
    return tgt_imu


tf2_ros.TransformRegistration().add(Imu, do_transform_quaternion)
tf2_ros.TransformRegistration().add(Imu, do_transform_imu)


class IMUException(Exception):
    """ Exception raised by IMU """
    def __init__(self, device, message):
        self.device = device
        self.message = message
        super(IMUException, self).__init__(self.message)


class IDMindIMU:
    """
    This class extracts data from the Sparkfun Razor M0 9DoF IMU.
    The .ino file must be uploaded to the unit. It will publish to /imu the values of orientation, angular velocity
    and linear acceleration.
    In case the connection is lost, it will try to reconnect.

    TODO: Allow for calibration of components
    """
    def __init__(self):

        # Logging
        self.logging = rospy.Publisher("/idmind_logging", Log, queue_size=10)
        self.diag_pub = rospy.Publisher("diagnostics", DiagnosticArray, queue_size=10)
        self.val_exc = 0

        self.imu_data = ""
        self.last_imu = Imu()
        self.imu_reading = Imu()
        self.calibration = True
        self.imu_offset = Quaternion()
        self.imu_offset.w = -1
        self.tf_prefix = rospy.get_param("~tf_prefix", "")
        self.target_frame = rospy.get_param("~target_frame", "imu")

        # Connect to IMU
        self.ser = None
        self.connection()

        self.tf_buffer = tf2_ros.Buffer()
        self.transform_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.imu_pub = rospy.Publisher("{}/imu".format(rospy.get_name()), Imu, queue_size=10)
        self.imu_euler_pub = rospy.Publisher("{}/euler_string".format(rospy.get_name()), String, queue_size=10)

        rospy.Service("{}/calibration".format(rospy.get_name()), Trigger, self.request_calibration)

    def connection(self):
        """
            Function that connects to IMU port.
            Tries /dev/idmind-imu (created by udev rules) and then tries all available ports
            Repeats until found. Flags for calibration.
            :return:
        """
        connected = False
        while not connected and not rospy.is_shutdown():
            try:
                self.ser = IDMindSerial("/dev/idmind-imu", baudrate=115200, timeout=1)
                connected = True
            except SerialException:
                self.log("Unable to connect to /dev/idmind-imu.", 2)
            except Exception as serial_exc:
                self.log("Exception caught: {}".format(serial_exc), 2)
            if not connected:
                self.log("Searching on other ports", 5)
                for addr in [comport.device for comport in serial.tools.list_ports.comports()]:
                    # If the lsof call returns an output, then the port is already in use!
                    try:
                        subprocess.check_output(['lsof', '+wt', addr])
                        continue
                    except subprocess.CalledProcessError:
                        self.ser = IDMindSerial(addr=addr, baudrate=115200, timeout=0.5)
                        imu_data = self.ser.read_until("\r\n")
                        try:
                            if self.parse_msg(imu_data):
                                connected = True
                                self.log("Imu found on {}".format(addr), 2)
                                self.publish_diagnostic(0, "IMU Detected on {}".format(addr))
                                break
                            else:
                                self.log("Imu not found on {}".format(addr), 5)
                        except IMUException:
                            self.log("Imu not found on {}".format(addr), 5)
                    except KeyboardInterrupt:
                        self.log("Node shutdown by user.", 2)
                        raise KeyboardInterrupt()
                    except SerialException:
                        self.log("Unable to connect to "+addr, 2)
                    except Exception as serial_exc:
                        self.log(serial_exc, 2)
                else:
                    self.log("No other devices found.", 5)

            if not connected:
                self.log("IMU not found. Waiting 5 secs to try again.", 1)
                rospy.sleep(5)
            else:
                self.calibration = True

        return connected

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

    def publish_diagnostic(self, level, message):
        """ Auxiliary method to publish Diagnostic messages """
        diag_msg = DiagnosticArray()
        diag_msg.header.frame_id = "imu"
        diag_msg.header.stamp = rospy.Time.now()
        imu_msg = DiagnosticStatus()
        imu_msg.name = "IMU"
        imu_msg.hardware_id = "Razor IMU"
        imu_msg.level = level
        imu_msg.message = message
        diag_msg.status.append(imu_msg)
        self.diag_pub.publish(diag_msg)

    def request_calibration(self, _req):
        self.calibration = True
        return TriggerResponse(True, "Requesting calibration")

    def parse_msg(self, imu_data):
        new_q = []
        a = []
        w = []
        dev_data = imu_data.split(" | ")
        for d in dev_data:
            values = d.split(" ")
            if values[0] == "Q:":
                q1 = Quaternion()
                q1.x = float(values[2])
                q1.y = float(values[3])
                q1.z = float(values[4])
                q1.w = float(values[1])
                q_off = self.imu_offset

                new_q = transformations.quaternion_multiply([q1.x, q1.y, q1.z, q1.w],
                                                            [q_off.x, q_off.y, q_off.z, q_off.w])
            elif values[0] == "A:":
                a = [float(values[1]), float(values[2]), float(values[3])]
            elif values[0] == "G:":
                w = [float(values[1]), float(values[2]), float(values[3])]
            else:
                self.log("Exception parsing message - {}".format(imu_data), 5)
                raise IMUException("razor", "Exception parsing message - {}".format(imu_data))
        return [new_q, a, w]

    def calibrate_imu(self):
        """
        This method will save the current orientation as the offset. All future publications will be adjusted in relation
        to the saved orientation
        :return:
        """
        self.log("Calibrating IMU", 3)
        r = rospy.Rate(20)
        calibrated = False
        reads = 0
        while not calibrated and not rospy.is_shutdown():
            try:
                reads = reads + 1
                imu_data = self.ser.read_until("\r\n").split(" | ")
                if reads > 100:
                    s = imu_data[0]
                    s_data = s.split(" ")
                    if s_data[0] == "Q:":
                        self.imu_offset.w = -float(s_data[1])
                        self.imu_offset.x = float(s_data[2])
                        self.imu_offset.y = float(s_data[3])
                        self.imu_offset.z = float(s_data[4])
                        calibrated = True
                        self.calibration = False
                    else:
                        rospy.logwarn("{}: IMU is giving bad answers - {}".format(rospy.get_name(), s_data[0]))
                else:
                    if reads == 1:
                        self.log("Discarding 100 readings for calibration", 3)
                    # rospy.loginfo(reads)
                r.sleep()

            except KeyboardInterrupt:
                raise KeyboardInterrupt()

    def update_imu(self):
        """
        Reads all characters in the buffer until finding \r\n
        Messages should have the following format: "Q: w x y z | A: x y z | G: x y z"
        :return:
        """
        # Create new message
        try:
            imu_msg = Imu()
            # Set the sensor covariances
            imu_msg.orientation_covariance = [
                0.0025, 0, 0,
                0, 0.0025, 0,
                0, 0, 0.0025
            ]
            imu_msg.angular_velocity_covariance = [
                0.02, 0, 0,
                0, 0.02, 0,
                0, 0, 0.02
            ]
            imu_msg.linear_acceleration_covariance = [
                0.04, 0, 0,
                0, 0.04, 0,
                0, 0, 0.04
            ]

            imu_data = self.ser.read_until("\r\n")
            if len(imu_data) == 0:
                self.log("IMU is not answering", 2)
                raise IMUException("razor", "IMU is not answering")
            try:
                [q, a, w] = self.parse_msg(imu_data)
                imu_msg.orientation.x = q[0]
                imu_msg.orientation.y = q[1]
                imu_msg.orientation.z = q[2]
                imu_msg.orientation.w = q[3]
                imu_msg.linear_acceleration.x = a[0]*9.82
                imu_msg.linear_acceleration.y = a[1]*9.82
                imu_msg.linear_acceleration.z = a[2]*9.82
                imu_msg.angular_velocity.x = w[0]
                imu_msg.angular_velocity.y = w[1]
                imu_msg.angular_velocity.z = w[2]

            except IMUException as err:
                raise IMUException
            except Exception as err:
                self.log("Exception generating IMU Message - {}".format(err), 5)
                raise IMUException("razor", "Exception generating IMU Message - {}".format(err))

            # Handle message header
            imu_msg.header.frame_id = self.tf_prefix+"imu"
            imu_msg.header.stamp = rospy.Time.now() + rospy.Duration(0.5)

            # Transform IMU message to another frame
            transf = self.get_transform(self.target_frame, imu_msg.header.frame_id)
            imu_msg = do_transform_imu(imu_msg, transf)

            self.publish_euler_imu(imu_msg)

            self.imu_reading = imu_msg

        except SerialException as serial_exc:
            self.log("SerialException while reading from IMU: {}".format(serial_exc), 3)
            self.calibration = True
        except ValueError as val_err:
            self.log("Value error from IMU data - {}".format(val_err), 5)
            self.val_exc = self.val_exc + 1
        except Exception as imu_exc:
            self.log(imu_exc, 3)
            raise imu_exc

    def get_transform(self, source="imu", target="imu"):
        """ Returns the transform between two frames """
        try:
            transformation = self.tf_buffer.lookup_transform(target, source, rospy.Duration(0))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            self.log('Unable to find the transformation from {} to {}'.format(source, target), 2, alert="error")
            transformation = TransformStamped()
        return transformation

    def publish_imu(self):
        self.imu_pub.publish(self.imu_reading)

    def publish_euler_imu(self, imu_reading):
        euler = transformations.euler_from_quaternion([imu_reading.orientation.x, imu_reading.orientation.y,
                                                           imu_reading.orientation.z, imu_reading.orientation.w])

        imu_quaternion = "quaternion = ({}, {}, {}, {})".format(imu_reading.orientation.x,
                                                                    imu_reading.orientation.y,
                                                                    imu_reading.orientation.z,
                                                                    imu_reading.orientation.w)
        imu_euler = "euler = ({}, {}, {})".format(euler[0], euler[1], euler[2])
        imu_euler_deg = "euler_deg = ({}, {}, {})".format(degrees(euler[0]),
                                                          degrees(euler[1]),
                                                          degrees(euler[2]))

        euler_imu_string = imu_quaternion + "\n" + imu_euler + "\n" + imu_euler_deg
        self.imu_euler_pub.publish(euler_imu_string)

    def start(self):

        r = rospy.Rate(20)
        imu_exception = 0
        while not rospy.is_shutdown():
            try:
                self.log("Bytes waiting: {}".format(self.ser.in_waiting), 7)
                if self.calibration:
                    self.publish_diagnostic(1, "Calibrating")
                    self.calibrate_imu()
                else:
                    self.update_imu()
                    self.publish_imu()
                imu_exception = 0
                self.publish_diagnostic(0, "OK")
                r.sleep()
            except IMUException as err:
                imu_exception = imu_exception + 1
                self.publish_diagnostic(1, "Error reading IMU")
                if imu_exception > 10:
                    self.log("Failure connecting to IMU. Restarting..", 3)
                    self.publish_diagnostic(2, "Failure connecting to IMU")
                    if not self.connection():
                        rospy.sleep(2)
            except KeyboardInterrupt:
                self.log("{}: Shutting down by user".format(rospy.get_name()), 2)
                break
            except IOError as io_exc:
                self.publish_diagnostic(1, "Lost connection to IMU")
                self.log("Lost connection to IMU", 3)
                if not self.connection():
                    rospy.sleep(2)


if __name__ == "__main__":
    rospy.init_node("idmind_razor")

    imu = IDMindIMU()
    imu.start()
    rospy.loginfo("{}: Node stopped".format(rospy.get_name()))
