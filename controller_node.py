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

#for RRT
import random
import matplotlib.pyplot as plt
#from rrt_weighted_progress import *

# Import the RRT algo: needed due to ROS2 interface/calling
import sys
import os
module_dir = '/home/jetbot/ece498/ws/src/aruco_controller/aruco_controller'
sys.path.append(module_dir)
from rrt_weighted_progress import *




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
        
        # Init with non-zero values for testing
        Goal_xpos = 0.5
        Goal_zpos = 0.5
        
        Obs1_xpos = 0.5
        Obs1_zpos = 0.3
        
        
        for i in range(0, len(aruco_msg.markers)):
            if (aruco_msg.markers[i].marker_id == 0):              #goal
                Goal_xpos = aruco_msg.markers[0].pose.position.x   #left/right, neg=left
                Goal_ypos = aruco_msg.markers[0].pose.position.y   #upd/down
                Goal_zpos = aruco_msg.markers[0].pose.position.z   #distance, meters
            # To start, only use goal
            elif (aruco_msg.markers[i].marker_id == 1):              #obstacle
                Obs1_xpos = aruco_msg.markers[0].pose.position.x   #left/right, neg=left
                Obs1_ypos = aruco_msg.markers[0].pose.position.y   #upd/down
                Obs1_zpos = aruco_msg.markers[0].pose.position.z   #distance, meters
            
        Start = (0.0,0.0)
        Goal = (Goal_xpos, Goal_zpos, 0.02)   #0.02 radius
        Obstacles =  [(Obs1_xpos, Obs1_zpos, 0.01), (2.5, 2.5, 0.1)]  # added fake obstacle
        
        rrt = RRT(Start, Goal, Obstacles, iterations=300, quit_on_goal=False,
               seeded=True, seed= 123)
        
        print(rrt.plan()) # returns
        
        # Now publish needed vel mesgs
        
        # Basic test of twist messages: Turn and move forward
#         twist_msg.angular.z = 0.94    
#         self.publisher.publish(twist_msg) #publish the needed angular first
#         time.sleep(2) # allow for movement
#         twist_msg.angular.z = 0.0 # Reset angular
#         twist_msg.linear.x = -0.075 # Move forward
#         self.publisher.publish(twist_msg) # publish again
#         time.sleep(1)
#         twist_msg.linear.x = 0.0
#         self.publisher.publish(twist_msg) 
#         time.sleep(0.1) # settle robot
        




# Main: do not change
def main(args=None):
    rclpy.init(args=args)
    node = ArucoNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
