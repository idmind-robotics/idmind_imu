#!/usr/bin/env python

import subprocess
import serial.tools.list_ports
from serial import SerialException
from math import degrees

import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Quaternion
from tf_conversions import transformations
from std_srvs.srv import Trigger, TriggerResponse

from idmind_serial2.idmind_serialport import IDMindSerial
from idmind_messages.msg import Log

VERBOSE = 5
LOGS = 5


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
        self.val_exc = 0

        self.imu_reading = Imu()
        self.calibration = True
        self.imu_offset = Quaternion()
        self.imu_offset.w = -1

        # Connect to IMU
        self.ser = None
        self.connection()

        self.imu_pub = rospy.Publisher("/imu", Imu, queue_size=10)
        self.imu_euler_pub = rospy.Publisher("/imu_euler", String, queue_size=10)

        rospy.Service("/idmind_razor/calibration", Trigger, self.request_calibration)

    def connection(self):
        """
        Function that connects to IMU port. Tries /dev/idmind-imu (created by udev rules) and then tries all available ports
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
                        if self.parse_msg(imu_data):
                            connected = True
                            self.log("Imu found on {}".format(addr), 5)
                            break
                        else:
                            self.log("Imu not found on {}".format(addr), 7)
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

    def log(self, msg, msg_level, log_level=-1):
        """
        Logging method for both verbose and logging topic
        :param msg: Message to be logged/displayed
        :param msg_level: if this value is lower than VERBOSE, display message on screen
        :param log_level: (optional) if this value is lower than LOGS, publish message
        :return:
        """

        if VERBOSE >= msg_level:
            rospy.loginfo("{}: {}".format(rospy.get_name(), msg))
        if LOGS >= (log_level if log_level != -1 else msg_level):
            self.logging.publish(rospy.Time.now().to_sec(), rospy.get_name(), msg)

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
                self.log("{}: IMU is giving bad answers - {}".format(rospy.get_name(), imu_data), 5)
                self.log(values[0], 5)
                return False
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
            imuMsg = Imu()
            # Set the sensor covariances
            imuMsg.orientation_covariance = [
                0.0025, 0, 0,
                0, 0.0025, 0,
                0, 0, 0.0025
            ]
            imuMsg.angular_velocity_covariance = [
                0.02, 0, 0,
                0, 0.02, 0,
                0, 0, 0.02
            ]
            imuMsg.linear_acceleration_covariance = [
                0.04, 0, 0,
                0, 0.04, 0,
                0, 0, 0.04
            ]

            imu_data = self.ser.read_until("\r\n")
            if len(imu_data) == 0:
                self.log("IMU is not answering", 2)
                return
            try:
                [q, a, w] = self.parse_msg(imu_data)
                imuMsg.orientation.x = q[0]
                imuMsg.orientation.y = q[1]
                imuMsg.orientation.z = q[2]
                imuMsg.orientation.w = q[3]
                imuMsg.linear_acceleration.x = a[0]
                imuMsg.linear_acceleration.y = a[1]
                imuMsg.linear_acceleration.z = a[2]
                imuMsg.angular_velocity.x = w[0]
                imuMsg.angular_velocity.y = w[1]
                imuMsg.angular_velocity.z = w[2]
            except:
                self.log("{}: IMU is giving bad answers - {}".format(rospy.get_name(), imu_data), 5)
                return
            # Handle message header
            imuMsg.header.frame_id = "base_link_imu"
            imuMsg.header.stamp = rospy.Time.now()+rospy.Duration(0.5)

            imuMsg_euler = transformations.euler_from_quaternion([imuMsg.orientation.x, imuMsg.orientation.y, imuMsg.orientation.z, imuMsg.orientation.w])
            pub_msg = "imuMsg_quaternion = (%s, %s, %s, %s)" % (imuMsg.orientation.x, imuMsg.orientation.y, imuMsg.orientation.z, imuMsg.orientation.w) + \
                      "\nimuMsg_euler = (%s, %s, %s)" % (imuMsg_euler[0], imuMsg_euler[1], imuMsg_euler[2]) +\
                      "\nimuMsg_euler deg = (%s, %s, %s)" % (degrees(imuMsg_euler[0]), degrees(imuMsg_euler[1]), degrees(imuMsg_euler[2]))
            self.imu_euler_pub.publish(pub_msg)
            # print "imuMsg_quaternion = (%s, %s, %s, %s)" % (imuMsg.orientation.x, imuMsg.orientation.y, imuMsg.orientation.z, imuMsg.orientation.w)
            # print "imuMsg_euler = (%s, %s, %s)" % (imuMsg_euler[0], imuMsg_euler[1], imuMsg_euler[2])

            self.imu_reading = imuMsg

        except SerialException as serial_exc:
            self.log("SerialException while reading from IMU: {}".format(serial_exc), 3)
            self.calibration = True
        except ValueError as val_err:
            self.log("Value error from IMU data - {}".format(val_err), 5)
            self.val_exc = self.val_exc + 1
        except Exception as imu_exc:
            self.log(imu_exc, 3)
            raise imu_exc

    def publish_imu(self):
        self.imu_pub.publish(self.imu_reading)

    def start(self):

        r = rospy.Rate(20)
        while not rospy.is_shutdown():
            try:
                self.log("Bytes waiting: {}".format(self.ser.in_waiting), 7)
                if self.calibration:
                    self.calibrate_imu()
                else:
                    self.update_imu()
                    self.publish_imu()
                r.sleep()
            except KeyboardInterrupt:
                self.log("{}: Shutting down by user".format(rospy.get_name()), 2)
                break
            except IOError as io_exc:
                self.log("Lost connection to IMU", 3)
                if not self.connection():
                    rospy.sleep(2)


if __name__ == "__main__":
    rospy.init_node("idmind_razor")
    imu = IDMindIMU()
    imu.start()
    rospy.loginfo("{}: Node stopped".format(rospy.get_name()))