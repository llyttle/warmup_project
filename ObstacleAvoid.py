#!/usr/bin/env python3

from geometry_msgs.msg import Twist, Vector3, PointStamped, Point
import numpy as np
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Header
import rospy

class ObstacleAvoidNode(object):
    def __init__(self):
        rospy.init_node("ObstacleAvoid")
        rospy.Subscriber('/scan', LaserScan, self.arbiter)
        self.pub_point = rospy.Publisher("/move_to_point", PointStamped, queue_size=10)
        self.pub_cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.Command = Twist()

    def arbiter(self, msg):
        scan = np.hstack((np.asarray(msg.ranges[269:359]), msg.ranges[0:89]))
        inPath = scan[15:165]
        Gaps = self.detectObject(scan)
        for z in range (len(inPath)):
            if np.isinf(inPath[z]) == False:
                m = self.setPoint(10, 0, 0)
                self.avoidObject(Gaps)
                print('avoiding object')
            else:
                m = self.setPoint(10, 0, 1)
                print('to waypoint')
            self.pub_point.publish(m)    

    def detectObject(self, scan):
        new_row = np.zeros(len(scan))
        g = np.array([new_row])
        num_objects = 0
        num_gaps = 1
        alreadyCounting = False
        for i in range (len(scan)):
            if np.isinf(scan[i]) == False and alreadyCounting == False:
                num_objects += 1
                alreadyCounting = True
            elif np.isinf(scan[i]) == True:
                if alreadyCounting == True:
                    num_gaps += 1
                    if num_gaps >= 1:
                        g = np.vstack((g, new_row))
                    alreadyCounting = False
                g[num_gaps-1, i] = i
        Gaps = []
        for x in range (len(g)):
            no_zeroes = g[x][g[x]!=0]
            #gap_size = np.amax(no_zeroes) - np.amin(no_zeroes)
            gap_center = np.mean(no_zeroes)
            Gaps.append((gap_center))
        return Gaps

    def avoidObject(self, Gaps):
        V = 0.5
        closest_gap = min(Gaps, key=lambda y:abs(y-90))
        k = abs(closest_gap-90)
        if closest_gap < 89:
            rotate = -k/50
        elif closest_gap > 91:
            rotate = k/50
        else:
            rotate = 0
        

        self.Command.linear.x = V
        self.Command.angular.z = rotate
        self.pub_cmd_vel.publish(self.Command)

    def setPoint(self, x, y, z):
        my_header = Header(stamp=rospy.Time.now(), frame_id="base_link")
        my_point = Point(x, y, z)
        m = PointStamped(header=my_header, point=my_point)
        return m


    def run(self):
        r = rospy.Rate(10)
        while not rospy.is_shutdown():
            r.sleep()

if __name__ == '__main__':
    node = ObstacleAvoidNode()
    node.run()