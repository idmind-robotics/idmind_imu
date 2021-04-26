#!/usr/bin/env python
import rospy
from idmind_msgs.msg import Log
from sensor_msgs.msg import Imu
from std_msgs.msg import String
from std_srvs.srv import Trigger, TriggerRequest

VERBOSE = 8
LOGS = 7


class IDMindIMU:
    def __init__(self):
        self.logging = rospy.Publisher("/idmind_logging", Log, queue_size=10)

        self.imu_pub = rospy.Publisher("{}/imu".format(rospy.get_name()), Imu, queue_size=10)
        self.imu_euler_pub = rospy.Publisher("{}/euler_string".format(rospy.get_name()), String, queue_size=10)

        rospy.Service("{}/calibration".format(rospy.get_name()), Trigger, self.request_calibration)

        self.ready = True
        rospy.Service("{}/ready".format(rospy.get_name()), Trigger, self.report_ready)
        self.log("Node initiated", 3)

    ###############
    #  CALLBACKS  #
    ###############
    def report_ready(self, _req):
        """ Simple Service callback to show node is ready """
        return TriggerResponse(self.ready, rospy.get_name()+" is " + ("ready" if self.ready else "not ready"))

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
