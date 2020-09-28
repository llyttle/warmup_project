#!/usr/bin/env python3

import rospy
import numpy as np
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Vector3
from tf.transformations import euler_from_quaternion, quaternion_from_euler

class wallPersonNode(object):
    def __init__(self):
        rospy.init_node('wallPerson')
        rospy.Subscriber('/scan', LaserScan, self.arbiter)
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.Command = Twist()

    def arbiter(self, msg):
        rmsg = msg.ranges
        arb_scan = np.asarray(rmsg)
        farthest_obj = np.where(np.isinf(arb_scan), -np.inf, arb_scan).argmax()
        if farthest_obj > 0 and farthest_obj < 180:
            self.PersonFollow(rmsg)
            print('\r', 'following Person                    ', end='')
        else:
            self. WallFollow(rmsg)
            print('\r', 'following wall                      ', end='')
        #print farthest_obj
#---------------------------------------------------------------------------------
    def PersonFollow(self, rmsg):
        scan = np.asarray(rmsg)
        Distance, intAngle = self.getPosition(scan)
        V, rotate = self.follow(Distance, intAngle)
        self.Command.linear.x = V
        self.Command.angular.z = rotate
        #print('\r', Distance, intAngle, '                      ', end='')

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
        if intAngle < 90 and intAngle > 5:
            rotate = intAngle/100
        elif intAngle > 270 and intAngle < 355:
            rotate = -(intAngle-175)/100
        else:
            rotate = 0
        if Distance > 1 and intAngle < 30:
            V = Distance/4
        else:
            V = 0
        return V, rotate
#---------------------------------------------------------------------------------
    def WallFollow(self, rmsg):
        theta = np.argmin(rmsg)
        D = min(rmsg)
        V = 0
        rotate = 0
        if np.isinf(D) == True:
            V = 0
            #print('\r', "No wall found             ", end='')
        elif D > 1:
            V, rotate = self.wallDistance(theta, 315)
            #print('\r', "Moving to Wall            ", end='')
        elif D < 0.5:
            V, rotate = self.wallDistance(theta, 225)
            #print('\r', "Moving away from Wall     ", end='')
        else:
            V, rotate = self.wallDistance(theta, 270)
            #print('\r', "Following Wall            ", end='')

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
#-----------------------------------------------------------------------------------------
    def run(self):
        r = rospy.Rate(10)
        while not rospy.is_shutdown():
            self.pub.publish(self.Command)
            r.sleep()

if __name__ == '__main__':
    node = wallPersonNode()
    node.run()