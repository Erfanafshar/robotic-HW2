#! /usr/bin/env python3
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import tf
from gazebo_msgs.srv import GetModelState
import rospy
import math
import numpy as np

# set before execution ##############
path_type = 1               # 1 -> oval & 2 -> spiral
# ###################################


class PoseMonitor:
    def __init__(self):
        rospy.init_node('pose_monitor', anonymous=True)
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.callback_odometry)
        self.vel_change_sub = rospy.Subscriber('/change', Twist, self.callback_velocity_change)
        self.report_pose = False
        rospy.wait_for_service("gazebo/get_model_state")
        print("gazebo detected")
        self.get_ground_truth = rospy.ServiceProxy("gazebo/get_model_state", GetModelState)
        self.faults = []

        if path_type == 1:
            self.set_oval_path_points(1, 3)
        elif path_type == 2:
            self.set_spiral_path_points(0.1)

    def callback_velocity_change(self, msg):
        rospy.loginfo("Velocity has changed, now: %5.3f, %5.3f", msg.linear.x, msg.angular.z)
        rospy.sleep(0.75)
        self.report_pose = True

    def quaternion_to_euler(self, msg):
        quaternion = (msg.pose.pose.orientation.x, msg.pose.pose.orientation.y,
                      msg.pose.pose.orientation.z, msg.pose.pose.orientation.w)
        (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(quaternion)
        print("Roll: %5.2f Pitch: %5.2f Yaw: %5.2f" % (roll, pitch, yaw))

    def set_oval_path_points(self, a, b):
        self.path_x = []
        self.path_y = []
        for t in np.arange(0.0, 4 * math.pi, 0.25 * math.pi):
            x = a * math.cos(t)
            y = b * math.sin(t)
            self.path_x.append(x)
            self.path_y.append(y)

    def set_spiral_path_points(self, gf):
        self.path_x = []
        self.path_y = []
        for i in range(0, 20, 1):
            t = i * gf * math.pi
            x = (1 + t) * math.cos(t)
            y = (1 + t) * math.sin(t)
            self.path_x.append(x)
            self.path_y.append(y)

    def get_fault(self, cx, cy):
        dists = []
        for itr in range(len(self.path_x) - 1):
            cur_x = self.path_x[itr]
            cur_y = self.path_y[itr]
            nex_x = self.path_x[itr + 1]
            nex_y = self.path_y[itr + 1]
            dist_to_cur = math.sqrt((cur_x - cx) ** 2 + (cur_y - cy) ** 2)
            dist_to_nex = math.sqrt((nex_x - cx) ** 2 + (nex_y - cy) ** 2)
            distance = np.mean([dist_to_cur, dist_to_nex])
            dists.append(distance)
        m_dis = min(dists)
        return m_dis

    def get_avg(self):
        return sum(self.faults) / len(self.faults)

    def callback_odometry(self, msg):
        if self.report_pose:
            print("Position: (%5.2f, %5.2f, %5.2f)" %
                  (msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z))
            self.quaternion_to_euler(msg)
            print("Linear twist: (%5.2f, %5.2f, %5.2f)" %
                  (msg.twist.twist.linear.x, msg.twist.twist.linear.y, msg.twist.twist.linear.z))
            print("Angular twist: (%5.2f, %5.2f, %5.2f)" %
                  (msg.twist.twist.angular.x, msg.twist.twist.angular.y, msg.twist.twist.angular.z))
            # print("Ground Truth: ", self.get_ground_truth("mobile-base", "world"))

            cx = msg.pose.pose.position.x
            cy = msg.pose.pose.position.y

            f_distance = self.get_fault(cx, cy)
            self.faults.append(f_distance)
            c_avg = self.get_avg()
            print("current fault : " + str(f_distance))
            print("average fault until now  : " + str(c_avg))
            print()
            self.report_pose = False


if __name__ == '__main__':
    PoseMonitor()
    rospy.spin()

