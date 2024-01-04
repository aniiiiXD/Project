#!/usr/bin/env python3



import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

class MazeNavigator:
    def __init__(self):
        rospy.init_node('maze_navigator_node', anonymous=True)
        
        # Subscribe to the /odom topic
        rospy.Subscriber('/odom', Odometry, self.odom_callback)

        # Publisher for controlling the car
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        # Adjust control parameters as needed
        self.linear_speed = 0.2  # Linear speed in m/s
        self.angular_speed = 0.5  # Angular speed in rad/s

    def odom_callback(self, msg):
        # Extract linear and angular velocities from odometry data
        linear_velocity = msg.twist.twist.linear.x
        angular_velocity = msg.twist.twist.angular.z

        # Implement wall-following logic
        cmd_vel_msg = Twist()
        cmd_vel_msg.linear.x = self.linear_speed
        cmd_vel_msg.angular.z = self.calculate_angular_velocity(linear_velocity, angular_velocity)

        # Publish control commands
        self.cmd_vel_pub.publish(cmd_vel_msg)

    def calculate_angular_velocity(self, linear_velocity, current_angular_velocity):
        # Simple wall-following logic (adjust as needed)
        if linear_velocity < 0.1:  # If linear velocity is low, turn left
            return self.angular_speed
        else:
            return current_angular_velocity

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        navigator = MazeNavigator()
        navigator.run()
    except rospy.ROSInterruptException:
        pass

