#! /usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import tf
from gazebo_msgs.srv import SetModelState
from gazebo_msgs.msg import ModelState
import math
import numpy as np

# set before execution ##############
path_type = 1  # 1 -> oval & 2 -> spiral
start_point = [1, 2]
# ###################################

if path_type == 1:
    threshold = 0.25
    d_s = 0.1
elif path_type == 2:
    threshold = 0.5
    d_s = 0.25

k_p = 0.2
k_i = 0.05
k_t = 1.0


class Move:
    def __init__(self):
        rospy.init_node('move')

        self.vel = Twist()
        self.itr = 0
        self.list_e_p = []

        rospy.wait_for_service("/gazebo/set_model_state")
        print("gazebo detected")
        state_msg = ModelState()
        state_msg.model_name = "turtlebot3_burger"

        # set start point
        state_msg.pose.position.x = start_point[0]
        state_msg.pose.position.y = start_point[1]

        if path_type == 1:
            self.set_oval_path_points(1, 3)
        elif path_type == 2:
            self.set_spiral_path_points(0.1)

        self.cnt = 0
        self.set_next_destination()

        set_state = rospy.ServiceProxy("/gazebo/set_model_state", SetModelState)
        set_state(state_msg)
        rospy.on_shutdown(self.shutdown)

        self.vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=5)
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.callback_odometry)
        self.change_pub = rospy.Publisher('/change', Twist, queue_size=5)

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

    def set_next_destination(self):
        if self.cnt == len(self.path_x):
            print("finished")
            self.shutdown()
        else:
            self.destination = [self.path_x[self.cnt], self.path_y[self.cnt]]
            self.cnt += 1

    def reached(self, x, y, xd, yd):
        dist_to_dest = math.sqrt((xd - x) ** 2 + (yd - y) ** 2)
        if dist_to_dest < threshold:
            return True
        else:
            return False

    def check_position(self, x, y, xd, yd):
        if self.reached(x, y, xd, yd):
            self.itr += 1
            if self.itr == 40:
                print("10 laps finished")
                self.shutdown()
            else:
                self.set_next_destination()

    def callback_odometry(self, msg):
        x, y = msg.pose.pose.position.x, msg.pose.pose.position.y
        t = self.quaternion_to_euler(msg)
        xd, yd = self.destination

        self.check_position(x, y, xd, yd)
        xd, yd = self.destination

        # calculate velocity
        ep = math.sqrt((xd - x) ** 2 + (yd - y) ** 2) - d_s
        array_size = len(self.list_e_p)
        if array_size > 2:
            intg_ep = (self.list_e_p[array_size - 1] +
                       self.list_e_p[array_size - 2] +
                       self.list_e_p[array_size - 3]) / 3.0
            vs = k_p * ep + k_i * intg_ep
            self.list_e_p.append(ep)
        else:
            vs = k_p * ep
            self.list_e_p.append(ep)

        # calculate angular
        atan2 = math.atan2(yd - y, xd - x)
        omega = k_t * min((atan2 - t, atan2 - 2 * math.pi - t, atan2 + 2 * math.pi - t), key=abs)
        self.vel.linear.x = vs
        self.vel.angular.z = omega
        self.vel_pub.publish(self.vel)
        self.change_pub.publish(self.vel)

    def quaternion_to_euler(self, msg):
        quaternion = (msg.pose.pose.orientation.x, msg.pose.pose.orientation.y,
                      msg.pose.pose.orientation.z, msg.pose.pose.orientation.w)
        roll, pitch, yaw = tf.transformations.euler_from_quaternion(quaternion)
        return yaw

    def shutdown(self):
        print("shutdown occurred")
        self.vel.linear.x = 0.0
        self.vel.angular.z = 0.0
        rospy.sleep(1)


if __name__ == '__main__':
    try:
        move = Move()
        rospy.spin()
    except rospy.ROSInterruptException:
        print("error occurred")

