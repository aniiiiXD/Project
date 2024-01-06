#!/usr/bin/env python3
#this version has the ability to receive a .png file 

import rospy
from pathfinding.core.grid import Grid
from pathfinding.finder.a_star import AStarFinder
from geometry_msgs.msg import Point
import cv2
import numpy as np
from test.msg import PathCoordinates  # Update with your actual package name
from std_msgs.msg import String 
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

coordinates=[]
x=0
y=0
i=0

def maze_gen():
    # Read the image using OpenCV
    img = cv2.imread("/home/yuvi/catkin_ws/received_image.png", cv2.IMREAD_GRAYSCALE) #update the path (image is stored in the folder from where )the script is executed
    # Resize the image to the target width and height
    img = cv2.resize(img, (90, 90), interpolation=cv2.INTER_AREA)
    img = cv2.GaussianBlur(img, (5, 5), 0)
    # Apply adaptive thresholding to create a binary image
    _, thresholded = cv2.threshold(img, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
    # Use morphological operations to make edges more sharp
    kernel = np.ones((3, 3), np.uint8)
    sharp_edges = cv2.morphologyEx(thresholded, cv2.MORPH_CLOSE, kernel)
    # Convert the binary image to a matrix with inverted values
    maze_matrix = (sharp_edges == 0).astype(int)
    # Choose the pool size for max pooling
    pool_size = 2
    # Apply max pooling to the matrix
    rows, cols = maze_matrix.shape
    pooled_matrix = np.zeros((rows // pool_size, cols // pool_size), dtype=int)

    for i in range(0, rows, pool_size):
        for j in range(0, cols, pool_size):
            pooled_matrix[i // pool_size, j // pool_size] = np.max(maze_matrix[i:i+pool_size, j:j+pool_size])
    # Store the resulting matrix in a variable
    return pooled_matrix

def callback(data, matrix):
    global coordinates, x, y, i
    direction = data.data
    if direction == "n":
        y -= 1
    elif direction == "s":
        y += 1
    elif direction == "w":
        x -= 1
    elif direction == "e":
        x += 1

    grid = Grid(matrix=matrix)
    start = grid.node(x, y)
    end = grid.node(41, 44)
    finder = AStarFinder()
    path = finder.find_path(start, end, grid)
    coordinates = [int(node.x) for node in path[0]] + [int(node.y) for node in path[0]]
    i = 0

    print(coordinates)
def image_callback(msg):
    print("image_callbacked")
    try:
        bridge = CvBridge()
        cv_image = bridge.imgmsg_to_cv2(msg, "bgr8")  # Convert the ROS image message to OpenCV format
        # Your image processing or saving code here
        cv2.imwrite('received_image.png', cv_image)  # Save the image as .png
    except Exception as e:
        rospy.logerr(e)

def solve():
    rospy.init_node("pathgen_node", anonymous=True)
    rospy.init_node("image_node", anonymous=True)
    rospy.Subscriber("image_topic", Image, image_callback)
    rospy.sleep(0.2)
    pub = rospy.Publisher("path_topic", PathCoordinates, queue_size=10)
    matrix = maze_gen()
    grid = Grid(matrix=matrix)
    start = grid.node(0, 5)
    end = grid.node(41, 44)
    finder = AStarFinder()
    path = finder.find_path(start, end, grid)
    global coordinates, x, y, i
    coordinates = [(int(node.x), int(node.y)) for node in path[0]]
    i=0
    rospy.Subscriber("msg_topic", String, lambda data: callback(data, matrix))
    rospy.sleep(0.5)
    
    while not rospy.is_shutdown():
        path_msg = PathCoordinates()
        path_msg.x_coordinates = [int(coord[0]) for coord in coordinates]
        path_msg.y_coordinates = [int(coord[1]) for coord in coordinates]
        pub.publish(path_msg)
        i += 1
        if i == len(coordinates):
            break
        rospy.sleep(2)

if __name__ == "__main__":
    try:
        solve()
    except rospy.ROSInterruptException:
        pass
