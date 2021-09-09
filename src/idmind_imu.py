#!/usr/bin/env python
import rospy
import collections
import numpy as np
from idmind_messages.msg import Log
from sensor_msgs.msg import Imu
from std_msgs.msg import String
from std_srvs.srv import Trigger, TriggerRequest, TriggerResponse

VERBOSE = 8
LOGS = 7


def quat_mult(q, p):
    """ Quaternion multiplication """
    # Check if q1 and q2 are quaternions
    if q.size != 4 or p.size != 4:
        raise Exception("One or both inputs are not quaternions")
    new_q = np.zeros((4, 1))
    new_q[0] = q[3]*p[0] + q[2]*p[1] - q[1]*p[2] + q[0]*p[3]
    new_q[1] = -q[2]*p[0] + q[3]*p[1] + q[0]*p[2] + q[1]*p[3]
    new_q[2] = q[1]*p[0] - q[0]*p[1] + q[3]*p[2] + q[2]*p[3]
    new_q[3] = -q[0]*p[0] - q[1]*p[1] - q[2]*p[2] + q[3]*p[3]
    return new_q


def skew_symmetric(v):
    if v.size != 3:
        raise Exception("Input is not 3 element vector")
    skew = np.array([
        [0, -v[2], v[1]],
        [v[2], 0, -v[0]],
        [-v[1], v[0], 0],
    ])


def sigma(w):
    """ Auxiliary matrix for quaternion calculations """
    sigma = np.array([
        [0, w[2], -w[1], w[0]],
        [-w[2], 0, w[0], w[1]],
        [w[1], -w[0], 0, w[2]],
        [-w[0], -w[1], -w[2], 0],
    ])
    return sigma


def quat_integrator(q, w_last, w_curr, dt):
    w_avg = (w_curr - w_last)/dt
    term1 = np.exp(0.5*sigma(w_avg)*dt)
    term2 = (1./48.)*(sigma(w_curr)*sigma(w_last) - sigma(w_last)*sigma(w_curr))*pow(dt, 2)
    new_q = np.dot(term1 + term2, q)
    return new_q


class IDMindIMU:
    def __init__(self):
        self.logging = rospy.Publisher("/idmind_logging", Log, queue_size=10)

        # IMU Subscriber
        rospy.Subscriber
        self.imu_topic = "idmind_imu/imu"
        self.imu = collections.deque([Imu()] * 5)
        rospy.Subscriber(self.imu_topic, Imu, self.update_imu)

        # Corrected IMU Publisher
        self.imu_pub = rospy.Publisher("{}/imu".format(rospy.get_name()), Imu, queue_size=10)
        self.imu_euler_pub = rospy.Publisher("{}/euler_string".format(rospy.get_name()), String, queue_size=10)

        # EKF Variables
        # State x =[q b]t and state history
        self.x = np.zeros((7, 1))
        self.x[3] = 1.0
        self.x_history = collections.deque([self.x] * 5)
        
        # State Covariance Matrix P (propagated and updated)
        self.P_prop = np.zeros()
        self.P_upd = np.zeros()
        
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

    def update_imu(self, msg):
        """ Update IMU readings """
        self.imu.popleft()
        self.imu.append(msg)
        self.new_imu = True

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

    def state_transition(self, w_est, dt):
        """ Computes the state transition matrix """
        term1 = np.eye(3) - dt*skew_symmetric(w_est) + 0.5*pow(dt, 2)*pow(skew_symmetric(w_est), 2)
        term2 = -np.eye(3)*dt + 0.5*pow(dt, 2)*skew_symmetric(w_est) - (pow(dt, 3)/6)*pow(skew_symmetric(w_est), 2)
        top = np.append(term1, term2, 1)
        bottom = np.append(np.zeros((3, 3)), np.eye(3), 1)
        return np.append(top, bottom, 0)

    def noise_covariance(self):
        """ Computes the noise covariance matrix """
        term1 = np.eye(3)
        term2 = np.eye(3)
        term3 = pow(sigma_w, 2)*dt*np.eye(3)
        top = np.append(term1, term2, 1)
        bottom = np.append(np.transpose(term2), term3, 1)
        return np.append(top, bottom, 0)

    def propagate(self):
        """ Propagate stage of EKF """
        self.b_prop = self.b_est
        self.w_prop = self.w_measurement - self.b_prop
        self.q_prop = quat_integrator(self.q_upd)

        st_matrix = self.state_transition(w_est, dt)
        noise_covariance = 

        self.P_prop = np.matmul(np.matmul(state_transition, self.P_upd), np.transpose(state_transition)) + noise_covariance

    def update(self):
        """ Update stage of EKF """
        # Update measurement matrix H
        H = 
        # Compute residual r
        r = z_curr - z_est
        # Compute covariance of residual S = HPHt + R
        S = np.matmul(np.matmul(H, self.P_prop), np.transpose(H)) + self.R
        # Compute Kalman Gain
        K = np.matmul(np.matmul(self.P_prop, np.transpose(H)), np.linalg.inv(S))
        # Compute state correction
        x_corr = np.dot(K, r)
        dq_nxt = x_corr[0:4]/2
        db_nxt = x_corr[4:7]
        # Update the quaternion
        dq_norm = np.dot(np.transpose(dq_nxt), dq_nxt)
        if dq_norm <= 1:
            dq = dq_nxt/np.sqrt(1-dq_norm)
        else:
            dq = 1/np.sqrt(1+dq_norm)*np.append(dq_nxt, [[1]], 0)
        self.q_est = quat_mult(dq, self.q_est)
        # Update the bias
        self.b_est = self.b_est + db_nxt
        # Update the turn rate
        self.w_est = w_measured - self.b_est
        # Update the Covariance Matrix P = (I-KH)P(I-KH)t + KRKt
        term1 = np.eye(6)-np.matmul(K, H)
        term2 = np.matmul(np.matmul(K, self.R), np.transpose(K))
        self.P_upd = np.matmul(np.matmul(term1, self.P_prop), np.transpose(term1)) + term2
    
    def publish_orientation(self):
        return True
    
    def start(self):
        r = rospy.Rate(20)
        while not rospy.is_shutdown():
            try:
                self.propagate()
                self.update()
                self.publish_orientation()
                r.sleep()
            except KeyboardInterrupt:
                self.log("{}: Shutting down by user".format(rospy.get_name()), 2)
                break
            except Exception as err:
                self.log("Exception: {}".format(err), 2, alert="error")
                r.sleep()

if __name__ == "__main__":
    rospy.init_node("idmind_artemis")

    imu = IDMindIMU()
    imu.start()
    rospy.loginfo("{}: Node stopped".format(rospy.get_name()))
