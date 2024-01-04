#!/usr/bin/env python3

import rospy
from nav_msgs.msg import Odometry

counter = 0

def odomCallback(msg):
    global counter
    curr_time = msg.header.stamp
    pose = msg.pose.pose
    counter += 1
    print(counter, curr_time)
    print(pose)

def odom_node():
    rospy.init_node('odom_node', anonymous=True)
    rospy.Subscriber('odom', Odometry, odomCallback)
    rospy.spin()

if __name__ == "__main__":
    try:
        odom_node()
    except rospy.ROSInterruptException:
        pass

