#!/usr/bin/env python3

import rospy
import numpy as np
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Vector3
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from visualization_msgs.msg import Marker

class wallFollower(object):
    def __init__(self):
        rospy.init_node('wallFollower')
        rospy.Subscriber('/scan', LaserScan, self.arbiter)
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.Command = Twist()
        self.pub_vis = rospy.Publisher('/visualization', Marker, queue_size=10)
        
    def arbiter(self, msg):
        theta = np.argmin(msg.ranges)
        D = min(msg.ranges)
        V = 0
        rotate = 0

        if np.isinf(D) == True:
            V = 0
            print('\r', "No wall found             ", end='')
        elif D > 1:
            V, rotate = self.wallDistance(theta, 315)
            print('\r', "Moving to Wall            ", end='')
        elif D < 0.5:
            V, rotate = self.wallDistance(theta, 225)
            print('\r', "Moving away from Wall     ", end='')
        else:
            V, rotate = self.wallDistance(theta, 270)
            print('\r', "Following Wall            ", end='')

        self.Command.linear.x = V
        self.Command.angular.z = rotate

    def wallDistance(self, theta, AOA): #AOA = Angle of Attack towards wall
        if theta < AOA-180 or theta > AOA:
            rotate = .5
        elif theta >= AOA-180 and theta < AOA:
            rotate = -.5
        else:
            rotate = 0

        if theta < AOA+10 and theta > AOA-10:
            V = .5
        else:
            V = .2
        return V, rotate

    def run(self):
        r = rospy.Rate(10)
        while not rospy.is_shutdown():
            self.pub.publish(self.Command)
            self.pub_vis.publish()
            r.sleep()

if __name__ == '__main__':
    node = wallFollower()
    node.run()
    


#put closest point 270 degrees off robot.

#know what side the wall is on
#in event of a corner, turn towards the wall