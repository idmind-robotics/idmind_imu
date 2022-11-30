#!/usr/bin/env python

import numpy as np
from serial import SerialException

import rospy
from datetime import datetime
from std_msgs.msg import String
from geometry_msgs.msg import Quaternion
from tf_conversions import transformations
from std_srvs.srv import Trigger, TriggerResponse
from sensor_msgs.msg import Imu, NavSatFix, NavSatStatus, TimeReference

from idmind_serial2.idmind_serialport import IDMindSerial
from idmind_msgs.msg import Log

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus

# TESTING RESTARTING PORT
import os
import fcntl

VERBOSE = 7
LOGS = 7
USBDEVFS_RESET = 'U' << (4*2) | 20
epoch = datetime(year=1970, month=1, day=1)

class IDMindArtemisHub:
    """
    This class extracts data from the Sparkfun OpenLog Artemis (with integrated ICM 20948)
    The OpenLogArtemis Firmware must be uploaded to the unit and configured (explained in README.MD).
    In case the connection is lost, it will try to reconnect and even close and reopen USB port.
    It will publish sensors_msgs/Imu to ~imu and euler angles to ~euler_string.
    If QWICC devices are connected, it can parse and publish the available values.    
    """
    def __init__(self):
        # Logging
        self.ready = False
        rospy.Service("~ready", Trigger, self.report_ready)
        self.logging = rospy.Publisher("/idmind_logging", Log, queue_size=10)
        self.diag_pub = rospy.Publisher("/diagnostics", DiagnosticArray, queue_size=10)        
        self.tf_prefix = rospy.get_param("~tf_prefix", "")
                
        # Connect to Artemis HUB
        self.fails = 0
        self.reads = 0
        self.ser = None
        self.first_run = True
        self.connection()
        self.data_headers = {"Date": 0, "Time": 1, "Hz": -1, 
                             "IMU": {"enabled": False},
                             "GPS": {"enabled": False},
                             }

        # IMU
        self.imu_calibrated = False
        self.imu_pub = rospy.Publisher("~imu", Imu, queue_size=10)
        self.imu_euler_pub = rospy.Publisher("~euler_string", String, queue_size=10)

        # GPS
        self.gps_pub = rospy.Publisher("~gps", NavSatFix, queue_size=10)
        self.time_ref = rospy.Publisher("~time_reference", TimeReference, queue_size=10)

        self.ready = True
        self.log("Node is ready", 4)

    ###############
    #  CALLBACKS  #
    ###############
    def report_ready(self, _req):
        """ Simple Service callback to show node is ready """
        self.log("Replying to 'ready' request", 7)
        return TriggerResponse(self.ready, rospy.get_name()+" is " + ("ready" if self.ready else "not ready"))
    
    ####################
    #  Initialization  #
    ####################
    def connection(self):
        """
        Function that connects to IMU port. Repeats until found.
        :return:
        """
        connected = False
        while not connected and not rospy.is_shutdown():
            self.log("Searching for Hub", 4)
            try:
                self.ser = IDMindSerial("/dev/idmind-artemis", baudrate=115200, timeout=0.5)
                self.log("OpenLog Artemis found.", 4)
                connected = True
            except SerialException as err:
                
                if err.errno == 2:
                    self.log("Openlog Artemis Hub not found", 2, alert="warn")
                    rospy.sleep(1)
                elif "Inappropriate ioctl" in str(err):
                    self.log("Restarting USB port", 2, alert="error")
                    self.usb_port_restart()
                    # fd = os.open("/dev/idmind-artemis", os.O_WRONLY)
                    # try:
                    #     fcntl.ioctl(fd, USBDEVFS_RESET, 0)
                    # finally:
                    #     os.close(fd)
                    rospy.sleep(1)
                else:                                        
                    self.log("Exception connecting to IMU: {}".format(err), 2, alert="warn")
                    rospy.sleep(1)
        
        self.first_run = True
        return connected

    def usb_port_restart(self, port="/dev/idmind-artemis"):
        fd = os.open(port, os.O_WRONLY)
        try:
            fcntl.ioctl(fd, USBDEVFS_RESET, 0)
        finally:
            os.close(fd)

    def auto_detect_devices(self):
        """ 
            This method will listen to the initial communications and detect connected Qwicc devices:
            - IMU
            - GPS
        """
        headers = False
        devices_detected = False
        # Start by checking available devices
        while (not headers) and (not rospy.is_shutdown()):
            data = self.ser.readline()
            if len(data.split(",")) > 5:
                if not devices_detected:
                    self.log("Restarting USB port", 2, alert="error")
                    self.usb_port_restart()
                self.log("Data headers received", 5)
                headers = True
            elif "IMU online" in data:
                self.log("IMU Detected", 5)
                devices_detected = True
                self.data_headers["IMU"]["enabled"] = True
            elif "GPS-ublox online" in data:
                self.log("GPS Detected", 5)
                devices_detected = True
                self.data_headers["GPS"]["enabled"] = True
        
        split_data = data.split(",")
        # If IMU was detected, search for indexes for Q6_1, Q6_2, Q6_3, RawAX, RawAY, RawAZ, RawGX, RawGY, RawGZ and ignore Magnetometer.
        if self.data_headers["IMU"]["enabled"]:
            try:
                self.imu_calibrated = False
                self.imu_history = [0.0] * 3
                self.data_headers["IMU"]["QX"] = split_data.index("Q6_1")
                self.data_headers["IMU"]["QY"] = split_data.index("Q6_2")
                self.data_headers["IMU"]["QZ"] = split_data.index("Q6_3")
                self.data_headers["IMU"]["AX"] = split_data.index("RawAX")
                self.data_headers["IMU"]["AY"] = split_data.index("RawAY")
                self.data_headers["IMU"]["AZ"] = split_data.index("RawAZ")
                self.data_headers["IMU"]["GX"] = split_data.index("RawGX")
                self.data_headers["IMU"]["GY"] = split_data.index("RawGY")
                self.data_headers["IMU"]["GZ"] = split_data.index("RawGZ")
                self.log("IMU Header Indexes: {}".format(self.data_headers["IMU"]), 8)
            except ValueError:
                self.log("IMU Data not found on headers", 4, alert="warn")
                self.data_headers["IMU"]["enabled"] = False
        
        # If GPS was detected, search for indexes gps_Date,gps_Time,gps_Lat,gps_Long,gps_Alt,gps_SIV,gps_FixType,gps_GroundSpeed,gps_Heading,gps_pDOP
        if self.data_headers["GPS"]["enabled"]:
            try:
                self.data_headers["GPS"]["Date"] = split_data.index("gps_Date")
                self.data_headers["GPS"]["Time"] = split_data.index("gps_Time")
                self.data_headers["GPS"]["Lat"] = split_data.index("gps_Lat")
                self.data_headers["GPS"]["Long"] = split_data.index("gps_Long")
                self.data_headers["GPS"]["Alt"] = split_data.index("gps_Alt")
                self.data_headers["GPS"]["SIV"] = split_data.index("gps_SIV")
                self.data_headers["GPS"]["FixType"] = split_data.index("gps_FixType")
                self.data_headers["GPS"]["GroundSpeed"] = split_data.index("gps_GroundSpeed")
                self.data_headers["GPS"]["Heading"] = split_data.index("gps_Heading")
                self.data_headers["GPS"]["pDOP"] = split_data.index("gps_pDOP")
                self.log("GPS Header Indexes: {}".format(self.data_headers["GPS"]), 8)
            except ValueError:
                self.log("GPS Data not found on headers", 4, alert="warn")
                self.data_headers["GPS"]["enabled"] = False

        msg = "Devices detected: \n"
        for dev in self.data_headers.keys():
            if type(self.data_headers[dev]) == dict:
                if self.data_headers[dev]["enabled"]:
                    msg += "\t{}\n".format(dev)
        self.log(msg, 3)
        return True

    def handle_imu(self, data, timestamp):
        """
        Accelerometer	aX,aY,aZ	milli g
        Gyro	gX,gY,gZ	Degrees per Second
        Magnetometer	mX,mY,mZ	micro Tesla
        Temperature	imu_degC	Degrees Centigrade
        """
        # Compute Quaternion, if data is consistent
        q = [float(data[self.data_headers["IMU"]["QX"]]),
             float(data[self.data_headers["IMU"]["QY"]]),
             float(data[self.data_headers["IMU"]["QZ"]]), 
             0]

        # Data may be inconsistent, try to fix it
        if ((q[0] * q[0]) + (q[1] * q[1]) + (q[2] * q[2])) > 1.0:
            q_norm = (q[0] * q[0]) + (q[1] * q[1]) + (q[2] * q[2])
            q[0] = q[0]/q_norm
            q[1] = q[1]/q_norm
            q[2] = q[2]/q_norm
        q[3] = np.sqrt(1.0 - ((q[0] * q[0]) + (q[1] * q[1]) + (q[2] * q[2])))

        # Save IMU history for calibration
        self.imu_history.pop(0)
        self.imu_history.append(transformations.euler_from_quaternion(q)[2])       
        if not self.imu_calibrated:
            # Wait for heading readings to stabilize
            if self.reads > 50 and abs((self.imu_history[2]-self.imu_history[1]) - (self.imu_history[1]-self.imu_history[0])) < 1e-6:
                self.log("IMU is calibrated.", 5)
                self.imu_calibrated = True
        else:
            new_q = Quaternion()
            new_q.x = q[0]
            new_q.y = q[1]
            new_q.z = q[2]
            new_q.w = q[3]

            # Compute Linear Acceleration - converting to ms-2
            acc_x = float(data[self.data_headers["IMU"]["AX"]])/1000
            acc_y = float(data[self.data_headers["IMU"]["AY"]])/1000
            acc_z = float(data[self.data_headers["IMU"]["AZ"]])/1000

            # Compute Angular Velocity
            w_x = float(data[self.data_headers["IMU"]["GX"]])*np.pi/180
            w_y = float(data[self.data_headers["IMU"]["GY"]])*np.pi/180
            w_z = float(data[self.data_headers["IMU"]["GY"]])*np.pi/180

            # Compute IMU Msg
            imu_msg = Imu()
            imu_msg.header.frame_id = self.tf_prefix+"imu"
            imu_msg.header.stamp = timestamp
            imu_msg.orientation = new_q
            # Set the sensor covariances
            imu_msg.orientation_covariance = [
            0.01, 0, 0,
            0, 0.01, 0,
            0, 0, 0.01
            ]
            # Angular Velocity
            imu_msg.angular_velocity.x = w_x
            imu_msg.angular_velocity.y = w_y
            imu_msg.angular_velocity.z = w_z
            imu_msg.angular_velocity_covariance = [0.0] * 9
            imu_msg.angular_velocity_covariance[0] = -1
            # Linear Acceleration
            imu_msg.linear_acceleration.x = acc_x
            imu_msg.linear_acceleration.y = acc_y
            imu_msg.linear_acceleration.z = acc_z
            imu_msg.linear_acceleration_covariance = [0.0] * 9
            imu_msg.linear_acceleration_covariance[0] = 0.01
            imu_msg.linear_acceleration_covariance[4] = 0.01
            imu_msg.linear_acceleration_covariance[8] = 0.01

            # Message publishing
            self.imu_pub.publish(imu_msg)
            new_q = imu_msg.orientation
            [r, p, y] = transformations.euler_from_quaternion([new_q.x, new_q.y, new_q.z, new_q.w])
            self.imu_euler_pub.publish("Roll: {} | Pitch: {} | Yaw: {}".format(r, p, y))

        return True
    
    def handle_gps(self, data, timestamp):
        """
        Date	gps_Date	MM/DD/YYYY or DD/MM/YYYY
        Time	gps_Time	HH:MM:SS.SSS
        Lat & Lon	gps_Lat,gps_Long	Degrees-7
        Altitude	gps_Alt	mm
        Altitude MSL	gps_AltMSL	mm
        SIV	gps_SIV	Count
        Fix Type	gps_FixType	0-5
        Carrier Soln.	gps_CarrierSolution	0-2
        Ground Speed	gps_GroundSpeed	mm/s
        Heading	gps_Heading	Degrees-5
        PDOP	gps_pDOP	10-2 (dimensionless)
        Time Of Week	gps_iTOW	ms
        Lat = Latitude
        Lon = Longitude
        MSL = Metres above Sea Level
        SIV = Satellites In View
        PDOP = Positional Dilution Of Precision

        Fix Type:
        0: No
        1: Dead Reckoning Only
        2: 2D
        3: 3D
        4: GNSS + Dead Reckoning
        5: Time Only

        Carrier Solution:
        0: No
        1: Float Solution
        2: Fixed Solution
        """
        gps_msg = NavSatFix()
        gps_msg.header.frame_id = self.tf_prefix+"world"
        gps_msg.header.stamp = timestamp

        gps_msg.status = NavSatStatus(status=0, service=1)

        gps_msg.latitude = float(data[self.data_headers["GPS"]["Lat"]])
        gps_msg.longitude = float(data[self.data_headers["GPS"]["Long"]])
        gps_msg.altitude = float(data[self.data_headers["GPS"]["Alt"]])

        # COVARIANCE_TYPE_UNKNOWN = 0, COVARIANCE_TYPE_APPROXIMATED = 1
        # COVARIANCE_TYPE_DIAGONAL_KNOWN = 2, COVARIANCE_TYPE_KNOWN = 3
        gps_msg.position_covariance = [0.0] * 9
        gps_msg.position_covariance_type = 0

        self.gps_pub.publish(gps_msg)

        # Time Reference
        time_msg = TimeReference()
        time_msg.header.stamp = timestamp
        gps_time = datetime.strptime("{} {}".format(data[self.data_headers["GPS"]["Date"]], data[self.data_headers["GPS"]["Time"]]),
                                     "%d/%m/%Y %H:%M:%S.%f")
        total_secs = (gps_time - epoch).total_seconds()
        time_msg.time_ref.secs = int(total_secs)
        time_msg.time_ref.nsecs = total_secs-int(total_secs)
        time_msg.source = "GPS"
        self.time_ref.publish(time_msg)

        return True

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


    def publish_diagnostics(self, level=0, message=""):
        """ Auxiliary method to publish Diagnostic messages """
        diag_msg = DiagnosticArray()
        diag_msg.header.frame_id = self.tf_prefix+"artemis"
        diag_msg.header.stamp = rospy.Time.now()
        if level == 0:
            # IMU Diagnostics
            if self.data_headers["IMU"]["enabled"]:
                imu_msg = DiagnosticStatus()
                imu_msg.name = "IMU"
                imu_msg.hardware_id = "ICM 20948"
                imu_msg.level = 0 if self.imu_calibrated else 3
                imu_msg.message = "OK" if self.imu_calibrated else "Calibrating"
                diag_msg.status.append(imu_msg)
            # GPS Diagnostics
            if self.data_headers["GPS"]["enabled"]:
                gps_msg = DiagnosticStatus()
                gps_msg.name = "GPS"
                gps_msg.hardware_id = "GPS NEO-M9N"
                gps_msg.level = 0
                gps_msg.message = "OK"
                diag_msg.status.append(gps_msg)
        else:
            artemis_msg = DiagnosticStatus()
            artemis_msg.name = "Openlog Artemis"
            artemis_msg.hardware_id = "Openlog Artemis"
            artemis_msg.level = level
            artemis_msg.message = message
            diag_msg.status.append(artemis_msg)            
        self.diag_pub.publish(diag_msg)

    def publish_device_data(self):
        # TODO: Ensure message is recent and valid or discard everything
        data = (self.ser.readline()).split(',')
        timestamp = rospy.Time.now()
        if len(data) > 5:
            self.reads += 1
            if self.data_headers["IMU"]["enabled"]:
                self.handle_imu(data, timestamp)
            if self.data_headers["GPS"]["enabled"]:
                self.handle_gps(data, timestamp)
            self.publish_diagnostics(level=0)
        else:
            self.publish_diagnostics(level=1, message="Invalid/Old data")
        return True

    def start(self):
        r = rospy.Rate(20)        
        while not rospy.is_shutdown():
            try:
                if self.first_run:
                    self.auto_detect_devices()
                    self.first_run = False
                self.publish_device_data()
                r.sleep()
            except KeyboardInterrupt:
                self.log("Shutting down by user", 2)
                self.ser.close()
                break
            except IOError as io_exc:
                self.publish_diagnostics(1, "Lost connection to IMU")
                self.log("Lost connection to IMU", 3)
                if not self.connection():
                    rospy.sleep(2)            


if __name__ == "__main__":
    rospy.init_node("idmind_artemis")
    imu = IDMindArtemisHub()
    imu.start()
    rospy.loginfo("{}: Node stopped".format(rospy.get_name()))
    