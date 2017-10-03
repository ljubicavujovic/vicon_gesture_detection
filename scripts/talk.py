#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from sys import stdin
"""
    This node is used to simulate speech. 
"""
if __name__ == '__main__':
    publisher = rospy.Publisher('speech', String, queue_size=10)
    rospy.init_node('talk', anonymous=True)

    r = rospy.Rate(0.1)
    while not rospy.is_shutdown():
        print "Type"
        sentance = stdin.readline()
        publisher.publish(sentance)
        r.sleep()
