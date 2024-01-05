#!/usr/bin/env python3

import rospy
from pathfinding.core.grid import Grid
from pathfinding.finder.a_star import AStarFinder
from geometry_msgs.msg import Point
import cv2
import numpy as np
from std_msgs.msg import String 

coordinates=[]
x=0
y=0
i=0
def callback(data, matrix):
    global coordinates, x, y, i
    direction=data.data
    if direction=="n":
        y -= 1
        grid = Grid(matrix=matrix)
        start = grid.node(x, y)
        end = grid.node(41, 44)
        finder = AStarFinder()
        path = finder.find_path(start, end, grid)
        coordinates = [(node.x, node.y) for node in path[0]]
        i = 0

    if direction=="s":
        y += 1
        grid = Grid(matrix=matrix)
        start = grid.node(x, y)
        end = grid.node(41, 44)
        finder = AStarFinder()
        path = finder.find_path(start, end, grid)
        coordinates = [(node.x, node.y) for node in path[0]]
        i = 0

    if direction=="w":
        x -= 1
        print("working")
        grid = Grid(matrix=matrix)
        start = grid.node(x, y)
        end = grid.node(41, 44)
        finder = AStarFinder()
        path = finder.find_path(start, end, grid)
        coordinates = [(node.x, node.y) for node in path[0]]
        i = 0

    if direction=="e":
        x += 1
        grid = Grid(matrix=matrix)
        start = grid.node(x, y)
        end = grid.node(41, 44)
        finder = AStarFinder()
        path = finder.find_path(start, end, grid)
        coordinates = [(node.x, node.y) for node in path[0]]
        i = 0

    print(coordinates)
    
def maze_gen():
    # Read the image using OpenCV
    img = cv2.imread("/home/yuvi/Downloads/images.png", cv2.IMREAD_GRAYSCALE)

    # Resize the image to the target width and height
    img = cv2.resize(img, (90, 90), interpolation=cv2.INTER_AREA)

    # Apply GaussianBlur for smoothing (optional)
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
def solve():
    rospy.init_node("pathgen_node", anonymous=True)
    pub=rospy.Publisher("path_topic", Point, queue_size=10)
    coord=Point()
    matrix = maze_gen()
    grid = Grid(matrix=matrix)
    start = grid.node(0, 5)
    end = grid.node(41, 44)
    finder = AStarFinder()
    path = finder.find_path(start, end, grid)
    global coordinates, x, y, i
    coordinates = [(node.x, node.y) for node in path[0]]
    print(coordinates)
    i=0
    rospy.Subscriber("msg_topic", String, lambda data: callback(data, matrix))
    rospy.sleep(0.5)

    while not rospy.is_shutdown():
        x, y = coordinates[i]
        coord.x = x
        coord.y = y
        coord.z = 0
        pub.publish(coord) #publish one coordinate every 2 seconds
        matrix[y][x] = 0
        i += 1
        if i==len(coordinates):
            break
        rospy.sleep(2)

if __name__=="__main__":
    try:
        solve()
    except rospy.ROSInterruptException:
        pass