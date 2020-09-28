#!/usr/bin/env python3

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Vector3, Pose, PointStamped
import numpy as np
import math
import rospy
from tf.transformations import euler_from_quaternion, quaternion_from_euler

class SquareNode(object):
    def __init__(self):
        rospy.init_node('square')
        rospy.Subscriber('/odom', Odometry, self.makeHeading)
        rospy.Subscriber('/move_to_point', PointStamped, self.pointDefine)
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.Command = Twist()
        roll = pitch = yaw = 0
        global targetPos
        targetPos = np.array([0, 0, 1])
 
    def pointDefine(self, m):
        global targetPos
        targetPos = np.array([m.point.x, m.point.y, m.point.z])

    def makeHeading(self, odom_data):
        if targetPos[2] == 1:
            pose = odom_data.pose.pose
            orientation_list = [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]
            (roll, pitch, yaw) = euler_from_quaternion(orientation_list)

            transMatrix = np.array([[np.cos(yaw), -np.sin(yaw), pose.position.x],
                                    [np.sin(yaw),  np.cos(yaw), pose.position.y],
                                    [0,            0,           1]])
            transposeMatrix = np.linalg.inv(transMatrix)
            sub_target = transposeMatrix.dot(targetPos)
            theta = np.rad2deg(np.arctan(sub_target[1]/sub_target[0]))
            sub_target_2D = np.array([sub_target[0], sub_target[1]])

            if sub_target[0] < 0 and sub_target[1] > 0:
                rotate = 1
            elif sub_target[0] <0 and sub_target[1] < 0:
                rotate = -1
            else:
                rotate = theta/30
            if np.linalg.norm(sub_target_2D) <= 0.1:
                V = 0
            else:
                V = 0.5 - abs(theta)/360
            self.Command.linear.x = V
            self.Command.angular.z = rotate
            self.pub.publish(self.Command)
        #print('\r', targetPos, end='')

    def run(self):
        r = rospy.Rate(10)
        while not rospy.is_shutdown():
            r.sleep()

if __name__ == '__main__':
    driveSquare = SquareNode()
    driveSquare.run()
