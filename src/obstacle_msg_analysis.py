#! /usr/bin/env python
'''
Script buat subscribe dummy obstacles yg dibikin sama node obstacle_publisher trus dimirror posisinya dan dipublish ke /new_obstacles
'''
import rospy
from nav_msgs.msg import Odometry
from obstacle_detector.msg import Obstacles
from obstacle_detector.msg import CircleObstacle
from geometry_msgs.msg import Point
import time


class ObstacleListener:
    def __init__(self):
        
        #self.obstacle_sub = rospy.Subscriber('/raw_obstacles', Obstacles, self.sub_callback)
        self.obstacle_sub = rospy.Subscriber('/raw_obstacles/camera_obstacles', Obstacles, self.sub_callback)
        self.new_obstacles = Obstacles()
        self.new_circles = CircleObstacle()
        self.new_points = Point()
        self.new_obstacles.circles = [0]
        self.count = 0


    def sub_callback(self, msg):
        obstacles_full = msg
        obstacles = msg.circles
        print("length", len(obstacles))
        #if len(obstacles) != 1:
        #    self.count +=1
        #print("Count", self.count)
        #x = obstacles.center.x
        #y = obstacles.center.y
        #print([x, y])
        
        
if __name__ == "__main__":
    rospy.init_node('obstacle_analysis', log_level=rospy.INFO) 
    run = ObstacleListener()
    rospy.spin() # mantain the service open.
