#!/usr/bin/env python3

from geometry_msgs.msg import PointStamped, Point
from std_msgs.msg import Header
import rospy

class PointDefineNode(object):
    def __init__(self):
        rospy.init_node("test_message")
        self.pub = rospy.Publisher("/move_to_point", PointStamped, queue_size=10)

    def squarePoints(self, t0):
        now = rospy.get_time()
        elapsed = now-t0
        my_header = Header(stamp=rospy.Time.now(), frame_id="base_link")

        if elapsed <= 3:
            my_point = Point(1, 0, 1)
        elif elapsed > 3 and elapsed <= 6:
            my_point = Point(1, 1, 1)
        elif elapsed > 6 and elapsed <= 9:
            my_point = Point(0, 1, 1)
        elif elapsed > 9:
            my_point = Point(0, 0, 1)
        
        m = PointStamped(header=my_header, point=my_point)
        return m

    def run(self):
        while rospy.get_time() == 0:
            t0 = 0
        t0 = rospy.get_time()
        r = rospy.Rate(10)
        while not rospy.is_shutdown():
            m = self.squarePoints(t0)
            self.pub.publish(m)
            r.sleep()

if __name__ == '__main__':
    node = PointDefineNode()
    node.run()
