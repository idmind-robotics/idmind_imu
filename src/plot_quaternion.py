import rospy
from sensor_msgs.msg import Imu

import numpy as np
import matplotlib
matplotlib.use('TKAgg')
from tf_conversions import transformations as trf
from matplotlib.lines import Line2D
import matplotlib.pyplot as plt
import matplotlib.animation as animation
HIST_SIZE = 50


class QuaternionPlotter:
    def __init__(self):
        self.imu_hist = [Imu()]*HIST_SIZE
        rospy.Subscriber("/miniadd/idmind_imu/imu", Imu, self.update_imu)

        self.fig, self.axs = plt.subplots(4)
        self.tdata = [a for a in range(0, HIST_SIZE)]
        self.qdata = [[0]*HIST_SIZE, [0]*HIST_SIZE, [0]*HIST_SIZE, [0]*HIST_SIZE]
        self.lines = [None] * 4
        for i in range(0, 4):
            self.lines[i] = Line2D(self.tdata, self.qdata[i])
            self.axs[i].add_line(self.lines[i])
            if i == 0:
                self.axs[i].set_ylim(-3.14, 3.14)
            else:
                self.axs[i].set_ylim(-1.1, 1.1)
            self.axs[i].set_xlim(0, HIST_SIZE)

    def update_imu(self, msg):        
        self.imu_hist.pop(0)
        self.imu_hist.append(msg)

    def data_gen(self):
        imu = self.imu_hist[-1]
        q = [imu.orientation.x, imu.orientation.y, imu.orientation.z, imu.orientation.w]
        [roll, pitch, yaw] = trf.euler_from_quaternion(q)
        yield [roll, pitch, yaw]

    def update(self, euler):
        # print("Q: {}".format(data))
        #yaw = trf.euler_from_quaternion(data)[2]
        data = trf.quaternion_from_euler(euler[0], euler[1], euler[2])
        data[0] = euler[2]

        for i in range(0, 4):
            self.qdata[i].pop(0)
            self.qdata[i].append(data[i])        
            self.lines[i].set_data(self.tdata, self.qdata[i])
        return self.lines

    def start(self):
        r = rospy.Rate(20)
        while not rospy.is_shutdown():
            ani = animation.FuncAnimation(self.fig, self.update, self.data_gen, interval=50, blit=True)
            plt.show()
            r.sleep()
            

rospy.init_node("quat_plotter")

scope = QuaternionPlotter()
scope.start()
# pass a generator in "emitter" to produce data for the update func

# plt.show()