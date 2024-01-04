#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Path
from tf.transformations import euler_from_quaternion
import math

class PIDController:
    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.prev_error = 0
        self.integral = 0

    def compute(self, error):
        self.integral += error
        derivative = error - self.prev_error
        output = (self.kp * error) + (self.ki * self.integral) + (self.kd * derivative)
        self.prev_error = error
        return output

class DifferentialDriveController:
    def __init__(self):
        rospy.init_node('diff_drive_controller')
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)
        self.pid_controller = PIDController(kp=1.0, ki=0.0, kd=0.0)
        self.path_sub = rospy.Subscriber('/maze_path', Path, self.path_callback)
        self.current_goal = None

    def odom_callback(self, msg):
        if self.current_goal is None:
            return

       
        orientation = msg.pose.pose.orientation
        euler_angles = euler_from_quaternion([orientation.x, orientation.y, orientation.z, orientation.w])
        yaw = euler_angles[2]

        
        desired_yaw = math.atan2(self.current_goal[1] - msg.pose.pose.position.y, self.current_goal[0] - msg.pose.pose.position.x)
        error = desired_yaw - yaw

        
        angular_vel = self.pid_controller.compute(error)

        cmd_vel_msg = Twist()
        cmd_vel_msg.angular.z = angular_vel
        self.cmd_pub.publish(cmd_vel_msg)

    def path_callback(self, path_msg):
        if not path_msg.poses:
            rospy.logwarn('Received an empty path.')
            return

        # Set the current goal to the first point in the path
        self.current_goal = (path_msg.poses[0].pose.position.x, path_msg.poses[0].pose.position.y)

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    diff_drive_controller = DifferentialDriveController()
    diff_drive_controller.run()

