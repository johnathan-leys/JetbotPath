# File contains code used in the ROS2 node onboard the jetbot

import rclpy
import numpy as np
import math
from rclpy.node import Node
from geometry_msgs.msg import Twist
from aruco_opencv_msgs.msg import ArucoDetection
#for astar
import numpy as np
from queue import PriorityQueue
#movement
import time


class ArucoNode(Node):

    def __init__(self):
        super().__init__('aruco_node')
        self.subscription = self.create_subscription(
            ArucoDetection,
            '/aruco_detections',
            self.listener_callback,
            10)
        self.publisher = self.create_publisher(Twist, 'jetbot/cmd_vel', 10)  
     
    def listener_callback(self, aruco_msg):        
        # Will be called for every publish to /aruco_detections
        
        # Twist message is what will be published... init to zero for now.
        twist_msg = Twist()
        twist_msg.linear.x = 0.0 #update
        twist_msg.linear.y = 0.0
        twist_msg.linear.z = 0.0
        twist_msg.angular.z = 0.0 #update
        twist_msg.angular.x = 0.0
        twist_msg.angular.y = 0.0
        
        theta = 0; # Current angle
        
        # Now publish needed vel mesgs
        
        # Basic test of twist messages: Turn and move forward
        twist_msg.angular.z = 0.94    
        self.publisher.publish(twist_msg) #publish the needed angular first
        time.sleep(2) # allow for movement
        twist_msg.angular.z = 0.0 # Reset angular
        twist_msg.linear.x = -0.075 # Move forward
        self.publisher.publish(twist_msg) # publish again
        time.sleep(1)
        twist_msg.linear.x = 0.0
        self.publisher.publish(twist_msg) 
        time.sleep(0.1) # settle robot
        

# Define astar algo, mostly from homework, textbook, GFG
# Will not be using in Final, replace with RRT*
def heuristic(a, b): #manhattan
    return abs(a[0] - b[0]) + abs(a[1] - b[1])

def astar(grid, start, goal):

    # moves possible (up down left right)
    neighbors = [(0,1),(1,0),(-1,0),(0,-1)]

    # prio q
    frontier = PriorityQueue()
    frontier.put(start, 0)

    # needed to track the path
    came_from = {}
    cost_so_far = {}
    came_from[start] = None
    cost_so_far[start] = 0

    while not frontier.empty():
        current = frontier.get()

        if current == goal:
            break

        for i, j in neighbors:
            next_node = current[0] + i, current[1] + j
            if 0 <= next_node[0] < grid.shape[0] and 0 <= next_node[1] < grid.shape[1]:
                if grid[next_node] == 1: # Obstacle
                    continue
                new_cost = cost_so_far[current] + 1
                if next_node not in cost_so_far or new_cost < cost_so_far[next_node]:
                    cost_so_far[next_node] = new_cost
                    priority = new_cost + heuristic(goal, next_node)
                    frontier.put(next_node, priority)
                    came_from[next_node] = current

    # reconstruct the path
    current = goal
    path = []
    while current != start:
        path.append(current)
        current = came_from[current]
    path.append(start)
    path.reverse()

    return path

# Main: do not change
def main(args=None):
    rclpy.init(args=args)
    node = ArucoNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
