#!/usr/bin/env python

import rospy
from numpy import pi
from threading import Lock
from sensor_msgs.msg import Imu
from idmind_messages.msg import Log
from serial import SerialException
from geometry_msgs.msg import Quaternion
from tf_conversions import transformations
from std_srvs.srv import Trigger, TriggerResponse
from idmind_serial2.idmind_serialport import IDMindSerial

VERBOSE = 3
LOGS = 3


class IDMindIMU:
    """
    This class extracts data from IDMind's Imu Board for Sparkfun MP6050 IMU.
    In case the connection is lost, it will try to reconnect.
    """
    def __init__(self, mode="sample", freq=20.0):
        """
        Initiates the IDMindImu Class, using a default "sample" mode at 20Hz
        :param mode:
        :param freq:
        """

        self.mode = 0x01 if mode == "stream" else 0x02
        self.rate = 110.0 if self.mode == "stream" else freq
        self.imu_lock = Lock()

        # Logging
        self.logging = rospy.Publisher("/idmind_logging", Log, queue_size=10)
        self.val_exc = 0

        # Connect to IMU
        self.connection()

        self.imu_reading = Imu()
        self.calibration = False
        self.imu_offset = Quaternion()
        self.imu_offset.w = -1

        self.imu_pub = rospy.Publisher("/imu", Imu, queue_size=10)

        rospy.Service("/idmind_razor/calibration", Trigger, self.request_calibration)

    def connection(self):
        """
        Function that connects to IMU port. Tries /dev/idmind-imu (created by udev rules) and then tries all ttyACM ports
        Repeats until found
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
                self.log(serial_exc, 2)
            if not connected:
                for i in range(0,10):
                    try:
                        self.ser = IDMindSerial("/dev/ttyACM"+str(i), baudrate=115200, timeout=0.5)
                        connected = True
                        break
                    except KeyboardInterrupt:
                        self.log("Node shutdown by user.", 2)
                        raise KeyboardInterrupt()
                    except SerialException:
                        self.log("Unable to connect to /dev/ttyACM"+str(i), 2)
                    except Exception as serial_exc:
                        self.log(serial_exc, 2)
            if not connected:
                self.log("IMU not found. Waiting 5 secs to try again.", 1)
                rospy.sleep(5)

        try:
            res = self.ser.command([0x50, self.mode], 4)
            if ord(res[0]) == 0x50:
                self.ser.flush()
            else:
                print "IDMindImu: Failed to set IMU mode"
        except Exception:
            print "IDMindImu: IMU communication failed"

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

    def read_data(self):
        """
        This methods reads a full message from the buffer.
        In case of "sample" mode, sends a request and returns the 10 byte response
        In case of "stream" mode, reads one byte until finding @ and then reads 9 more
        :return [yaw, pitch, roll]:
        """
        buffer_imu = []
        if self.mode == 0x02:   # Request Sample
            self.ser.send_command([0x40])
            buffer_imu = self.ser.read_command(10)
        else:                   # Read from stream
            b = ""
            while ord(b) != 0x40:
                b = self.ser.read_command(1)
            buffer_imu = self.ser.read_command(9)
            buffer_imu = b.extend(buffer_imu)

        if ord(buffer_imu[0]) == 0x40:
            yaw = (pi / 180.0) * round(self.ser.to_num(buffer_imu[1], buffer_imu[2]), 5) / 100
            pitch = (pi / 180.0) * round(self.ser.to_num(buffer_imu[3], buffer_imu[4]), 5) / 100
            roll = (pi / 180.0) * round(self.ser.to_num(buffer_imu[5], buffer_imu[7]), 5) / 100
            return [yaw, pitch, roll]
        else:
            return ["inf", "inf", "inf"]

    def calibrate_imu(self):
        """
        This method will save the current orientation as the offset. All future publications will be adjusted in relation
        to the saved orientation. Readings com in [yaw, pitch, roll]
        :return:
        """
        self.log("Calibrating IMU", 3)
        r = rospy.Rate(20)
        calibrated = False
        reads = 0
        while not calibrated and not rospy.is_shutdown():
            try:
                reads = reads + 1
                imu_data = self.read_data()

                if reads > 100:
                    q_data = transformations.quaternion_from_euler(imu_data[2], imu_data[1], imu_data[0])
                    self.imu_offset.x = float(q_data[0])
                    self.imu_offset.y = float(q_data[1])
                    self.imu_offset.z = float(q_data[2])
                    self.imu_offset.w = -float(q_data[3])
                    calibrated = True
                    self.calibration = False
                else:
                    if reads == 1:
                        self.log("Discarding 100 readings for calibration", 3)
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
                -1., 0, 0,
                0, 0, 0,
                0, 0, 0
            ]
            imuMsg.linear_acceleration_covariance = [
                -1., 0, 0,
                0, 0, 0,
                0, 0, 0
            ]

            imu_data = self.read_data()
            if len(imu_data) == 0:
                self.log("IMU is not answering", 2)
                return

            q_data = transformations.quaternion_from_euler(imu_data[2], imu_data[1], imu_data[0])
            q1 = Quaternion()
            q1.x = float(q_data[0])
            q1.y = float(q_data[1])
            q1.z = float(q_data[2])
            q1.w = float(q_data[3])

            q_off = self.imu_offset

            new_q = transformations.quaternion_multiply([q1.x, q1.y, q1.z, q1.w],
                                                        [q_off.x, q_off.y, q_off.z, q_off.w])
            imuMsg.orientation.x = new_q[0]
            imuMsg.orientation.y = new_q[1]
            imuMsg.orientation.z = new_q[2]
            imuMsg.orientation.w = new_q[3]

            # Handle message header
            imuMsg.header.frame_id = "base_link_imu"
            imuMsg.header.stamp = rospy.Time.now()+rospy.Duration(nsecs=5000)
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
                self.log("Bytes waiting: {}".format(self.ser.in_waiting), 5)
                if self.calibration:
                    self.calibrate_imu()
                else:
                    self.update_imu()
                    self.publish_imu()
                r.sleep()
            except KeyboardInterrupt:
                self.log("{}: Shutting down by user".format(rospy.get_name()), 2)
                self.shutdown()
                break
            except IOError as io_exc:
                self.log("Lost connection to IMU", 3)
                if not self.connection():
                    rospy.sleep(2)

    def shutdown(self):
        """
        Stops the IMU readings and closes the serial port
        :return:
        """
        if self.ser.send_command(bytearray([0x50, 0])) == 2:
            buffer_imu = self.ser.read_command(4)
            if buffer_imu[0] == 0x50:
                self.ser.flush()
                IDMindSerial.__del__(self)

    def __del__(self):
        self.shutdown()


if __name__ == "__main__":
    rospy.init_node("idmind_imu")
    imu = IDMindIMU()
    imu.start()
    rospy.loginfo("{}: Node stopped".format(rospy.get_name()))