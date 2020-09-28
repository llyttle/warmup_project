#!/usr/bin/env python3

import tty
import select
import sys
import termios
from geometry_msgs.msg import Twist, Vector3
import rospy

class teleopNode(object):
    def __init__(self):
        rospy.init_node('teleop')
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

    def getKey(self, settings):
        tty.setraw(sys.stdin.fileno())
        select.select([sys.stdin], [], [], 0)
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        return key;

    def command(self, vel_msg, key):
        vel_msg.linear.x /= 100
        vel_msg.angular.z /= 100
        if key == 'w':
            vel_msg.linear.x = 1
        if key == 'a':
            vel_msg.angular.z = 1
        if key == 's':
            vel_msg.linear.x = -1
        if key == 'd':
            vel_msg.angular.z = -1
        if key == 'e':
            vel_msg.linear.x = 0
            vel_msg.angular.z = 0
        return vel_msg

    def detect_keyState(self, key, lastkey):
        if key != lastkey:
            keyHold = True
        elif key == lastkey:
            keyHold = False

    def run(self):
        r = rospy.Rate(10)
        settings = termios.tcgetattr(sys.stdin)
        vel_msg = Twist()
        key = None
        lastkey = key
        while key != '\x03':
            key = self.getKey(settings)
            if self.detect_keyState(key, lastkey) == False:
                vel_msg.linear.x = 0
                vel_msg.angular.z = 0
            self.command(vel_msg, key)
            self.pub.publish(vel_msg)
            lastkey = key
            r.sleep()

if __name__ == '__main__':
    remote = teleopNode()
    remote.run()

