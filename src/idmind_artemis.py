#!/usr/bin/env python

import numpy as np
import subprocess
import serial.tools.list_ports
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

VERBOSE = 5
LOGS = 5


class IDMindIMU:
    """
    This class extracts data from the Sparkfun OpenLog Artemis (with ICM 20948)
    The OpenLogArtemis sketch must be uploaded to the unit. It will publish to /imu the values of orientation,
    angular velocity and linear acceleration.
    In case the connection is lost, it will try to reconnect.

    TODO: Allow for calibration of components
    """
    def __init__(self):
        self.ready = False
        rospy.Service("~ready", Trigger, self.report_ready)
        # Logging
        self.logging = rospy.Publisher("/idmind_logging", Log, queue_size=10)
        self.diag_pub = rospy.Publisher("/diagnostics", DiagnosticArray, queue_size=10)
        self.val_exc = 0

        self.imu_data = ""
        self.last_imu = Imu()
        self.imu_reading = Imu()
        self.calibration = True
        self.tf_prefix = rospy.get_param("~tf_prefix", "")

        # Connect to IMU
        self.ser = None
        self.connection()

        self.imu_pub = rospy.Publisher("~imu", Imu, queue_size=10)
        self.imu_euler_pub = rospy.Publisher("~euler_string", String, queue_size=10)

        self.ready = True

    ###############
    #  CALLBACKS  #
    ###############
    def report_ready(self, _req):
        """ Simple Service callback to show node is ready """
        self.log("Replying to 'ready' request", 7)
        return TriggerResponse(self.ready, rospy.get_name()+" is " + ("ready" if self.ready else "not ready"))

    def connection(self):
        """
        Function that connects to IMU port, searching all available ports
        Repeats until found. Flags for calibration.
        :return:
        """
        connected = False
        while not connected and not rospy.is_shutdown():
            self.log("Searching for IMU", 5)
            for addr in [comport.device for comport in serial.tools.list_ports.comports()]:
                # If the lsof call returns an output, then the port is already in use!
                try:
                    subprocess.check_output(['lsof', '+wt', addr])
                    continue
                except subprocess.CalledProcessError:
                    self.ser = IDMindSerial(addr=addr, baudrate=115200, timeout=0.5)
                    rospy.sleep(2)
                    self.ser.flush()
                    self.ser.reset_input_buffer()
                    self.ser.reset_output_buffer()
                    self.ser.write(bytearray(['f']))
                    rospy.sleep(0.5)
                    imu_data = self.ser.readline()
                    # self.log("Data: {}".format(imu_data), 2)
                    if "IDMind OpenLog Artemis" in imu_data:
                        connected = True
                        self.log("Imu found on {}".format(addr), 5)
                        self.publish_diagnostic(0, "IMU Detected on {}".format(addr))
                        break
                    else:
                        try:
                            self.ser.close()
                        except:
                            pass
                        finally:
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
                self.publish_diagnostic(2, "IMU not found")
                rospy.sleep(5)
            else:
                self.calibration = True

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

    def get_imu_data(self):
        """
            This method will read the buffer from the IMU until it finds EOL.
            This should be able to handle the message flow
        """
        data = ""
        self.ser.write(bytearray(['r']))
        while not rospy.is_shutdown():
            data += self.ser.readline()
            if len(data) > 0 and data[-1] == "\n":
                self.log("Got a complete line", 9)
                self.imu_data = data[:-2]
                return self.imu_data
            else:
                self.log("Waiting for a complete line", 7, alert="warn")
        return data

    def parse_msg(self):
        try:
            dev_data = self.imu_data.split(" ")
            msg = []
            for elem in dev_data:
                msg.append(float(elem.split(":")[1]))
            return msg
        except IndexError as err:
            self.log("Short IMU message: {}".format(err), 2, alert="warn")
        except ValueError as err:
            self.log("Bad IMU message: {}".format(err), 2, alert="warn")
        except Exception as err:
            self.log("Unknown Exception parsing IMU message: {}".format(err), 2, alert="error")
            raise err

    def compute_imu_msg(self):
        # Create new message
        try:
            # Get data
            self.get_imu_data()
            [q1, q2, q3, accuracy, roll, pitch, yaw, w_x, w_y, w_z, acc_x, acc_y, acc_z] = self.parse_msg()

            imu_msg = Imu()
            imu_msg.header.frame_id = self.tf_prefix+"imu"
            imu_msg.header.stamp = rospy.Time.now()  # + rospy.Duration(0.5)

            if ((q1 * q1) + (q2 * q2) + (q3 * q3)) < 1:
                q4 = [q1, q2, q3, np.sqrt(1.0 - ((q1 * q1) + (q2 * q2) + (q3 * q3)))]
            else:
                self.log("Inconsistent readings from IMU", 2, alert="warn")
                return True

            # Compute the Orientation Quaternion
            new_q = Quaternion()
            new_q.x = q1
            new_q.y = q2
            new_q.z = q3
            new_q.w = q4
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
        except SerialException as serial_exc:
            self.log("SerialException while reading from IMU: {}".format(serial_exc), 3)
            self.calibration = True
        except ValueError as val_err:
            self.log("Value error from IMU data - {}".format(val_err), 5)
            self.val_exc = self.val_exc + 1
        except Exception as imu_exc:
            self.log(imu_exc, 3)
            raise imu_exc

    def calibrate_imu(self):
        """
            This method will save the current orientation as the offset.
            All future publications will be adjusted in relation to the saved orientation
            :return:
        """
        self.log("Calibrating IMU", 3)
        r = rospy.Rate(20)
        calibrated = False
        reads = 0
        while not calibrated and not rospy.is_shutdown():
            try:
                reads = reads + 1
                self.get_imu_data()
                if reads > 50:
                    self.get_imu_data()
                    [q1, q2, q3, accuracy, roll, pitch, yaw, w_x, w_y, w_z, acc_x, acc_y, acc_z] = self.parse_msg()
                    q = [q1, q2, q3, np.sqrt(1.0 - ((q1 * q1) + (q2 * q2) + (q3 * q3)))]
                    calibrated = True
                    self.calibration = False
                else:
                    if reads == 1:
                        self.log("Discarding 50 readings for calibration", 7)
                    # rospy.loginfo(reads)
                r.sleep()

            except KeyboardInterrupt:
                raise KeyboardInterrupt()

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
                self.log("Bytes waiting: {}".format(self.ser.in_waiting), 7)
                if self.calibration:
                    self.calibrate_imu()
                else:
                    # self.get_imu_data()
                    # self.parse_msg()
                    self.compute_imu_msg()
                self.publish_diagnostic(0, "OK")
                r.sleep()
            except KeyboardInterrupt:
                self.log("{}: Shutting down by user".format(rospy.get_name()), 2)
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
