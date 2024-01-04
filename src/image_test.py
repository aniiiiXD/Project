#!/usr/bin/env python3


import cv2
import numpy as np
import rospy
from std_msgs.msg import Path
from geometry_msgs.msg import PoseStamped


def colorize(value):
    # Use ANSI escape codes for coloring
    if value == 1:
        return "\033[1;31m1\033[0m"  # Red for 1
    else:
        return "\033[1;34m0\033[0m"  # Blue for 0

def image_to_matrix(image_path, target_width, target_height):
    # Read the image using OpenCV
    img = cv2.imread(image_path, cv2.IMREAD_GRAYSCALE)

    # Resize the image to the target width and height
    img = cv2.resize(img, (target_width, target_height), interpolation=cv2.INTER_AREA)

    # Apply GaussianBlur for smoothing (optional)
    img = cv2.GaussianBlur(img, (5, 5), 0)

    # Apply adaptive thresholding to create a binary image
    _, thresholded = cv2.threshold(img, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)

    # Use morphological operations to make edges more sharp
    kernel = np.ones((3, 3), np.uint8)
    sharp_edges = cv2.morphologyEx(thresholded, cv2.MORPH_CLOSE, kernel)

    # Convert the binary image to a matrix with inverted values
    matrix = (sharp_edges == 0).astype(int)

    return matrix

def max_pool(matrix, pool_size):
    rows, cols = matrix.shape
    pooled_matrix = np.zeros((rows // pool_size, cols // pool_size), dtype=int)

    for i in range(0, rows, pool_size):
        for j in range(0, cols, pool_size):
            pooled_matrix[i // pool_size, j // pool_size] = np.max(matrix[i:i+pool_size, j:j+pool_size])

    return pooled_matrix

# Provide the full path to the image in the Downloads folder
downloads_folder = "/home/yuvi/Downloads/"
image_path = downloads_folder + "images.png"

# Replace 'yuvi' with your actual Ubuntu username
# Adjust target width and height as needed
target_width = 90
target_height = 90

maze_matrix = image_to_matrix(image_path, target_width, target_height)

# Choose the pool size for max pooling
pool_size = 2

# Apply max pooling to the matrix
pooled_matrix = max_pool(maze_matrix, pool_size)

# Print the resulting matrix with colors
for row in pooled_matrix:
    colored_row = [colorize(value) for value in row]
    print(' '.join(colored_row))
    
    
rospy.init_node('image_test')

# Find path
path = astar(maze, start, end)

# Publish path as a ROS Path message
if path:
    path_msg = Path()
    for point in path:
        pose = PoseStamped()
        pose.pose.position.x = point[0]
        pose.pose.position.y = point[1]
        path_msg.poses.append(pose)
    path_msg.header.stamp = rospy.Time.now()
    path_msg.header.frame_id = 'map'  # Adjust the frame ID as needed
    
    path_pub = rospy.Publisher('/maze_path', Path, queue_size=10)
    path_pub.publish(path_msg)
    rospy.loginfo('Path published to /maze_path')
else:
    rospy.logwarn('No path found')

# Spin ROS node
rospy.spin()    
    
    
    
    
