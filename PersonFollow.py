#!/usr/bin/env python3

import rospy
import numpy as np
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Vector3
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from visualization_msgs.msg import Marker

class personFollow(object):
    def __init__(self):
        rospy.init_node('personFollow')
        rospy.Subscriber('/scan', LaserScan, self.arbiter)
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.Command = Twist()
        self.pub_vis = rospy.Publisher('/visualization', Marker, queue_size=10)
    
    def arbiter(self, msg):
        scan = np.asarray(msg.ranges)
        Distance, intAngle = self.getPosition(scan)
        V, rotate = self.follow(Distance, intAngle)
        self.Command.linear.x = V
        self.Command.angular.z = rotate
        print('\r', Distance, intAngle, '                      ', end='')

    def getPosition(self, scan):
        #Distance = 0
        new_row = np.zeros(len(scan))
        sd = np.array([new_row])
        num_objects = 0
        alreadyCounting = True
        for i in range (len(scan)):
            if np.isinf(scan[i]) == False:
                if alreadyCounting == False:
                    num_objects += 1
                    if num_objects > 1:
                        sd = np.vstack((sd, new_row))
                    alreadyCounting = True
                sd[num_objects-1, i] = i
            elif np.isinf(scan[i]) == True:
                alreadyCounting = False
        Object_angle = []
        for x in range (len(sd)):
            no_zeroes = sd[x][sd[x]!=0]
            if 1 in no_zeroes and 360 in no_zeroes:
                d = np.sum(no_zeroes<180) - np.sum(no_zeroes>180)
                if d > 0:
                    Object_angle.append(d/2)
                else:
                    Object_angle.append(360+d/2)
            else:
                Object_angle.append(np.mean(no_zeroes))
        intAngle = Object_angle[0].astype(np.int32)     #only set up for one object at a time 
        if intAngle >= 0 and intAngle <= 361:
            Distance = scan[intAngle]   
        else:
            Distance = 0

        return Distance, intAngle

    def follow(self, Distance, intAngle):
        V = 0
        rotate = 0
        if intAngle < 180 and intAngle > 5:
            rotate = intAngle/100
        elif intAngle > 180 and intAngle < 355:
            rotate = -(intAngle-175)/100
        else:
            rotate = 0
        if Distance > 1 and intAngle < 30:
            V = Distance/4
        else:
            V = 0
        return V, rotate
        
    def run(self):
        r = rospy.Rate(10)
        while not rospy.is_shutdown():
            self.pub.publish(self.Command)
            self.pub_vis.publish()
            r.sleep()

if __name__ == '__main__':
    node = personFollow()
    node.run()

