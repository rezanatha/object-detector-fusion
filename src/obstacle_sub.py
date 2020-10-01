#! /usr/bin/env python
'''
Script buat subscribe dummy obstacles yg dibikin sama node obstacle_publisher trus dimirror posisinya dan dipublish ke /new_obstacles
'''
import rospy
import csv
from obstacle_detector.msg import Obstacles
import time


class ObstacleListener:
    def __init__(self):

        self.obstacle_sub = rospy.Subscriber(
            '/raw_obstacles/lidar_obstacles', Obstacles, self.sub_callback)
        self.print_read = []
        self.start_time = rospy.get_time()
        self.count = 0

    def sub_callback(self, msg):
        circles = msg.circles[0]
        time_elapsed = rospy.get_time() - self.start_time
        obj = [circles.center.x, circles.center.y,
               circles.true_radius, time_elapsed]
        self.count += 1
        print(obj, self.count)
        self.print_read.append(obj)


if __name__ == "__main__":
    rospy.init_node('obstacle_listener', log_level=rospy.INFO)
    run = ObstacleListener()
    rospy.spin()  # mantain the service open.
    with open('obstacle_data.csv', 'w') as f:
        csvwriter = csv.writer(f)
        csvwriter.writerow(["pos_x",
                            "pos_y",
                            "true_radius",
                            "time"])
        csvwriter.writerows(run.print_read)
        print("Printed to obstacle_data.csv")
    f.close()
