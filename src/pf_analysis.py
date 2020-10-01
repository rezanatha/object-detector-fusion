#!/usr/bin/env python
import roslib
from std_msgs.msg import Float32, Float32MultiArray, ColorRGBA
from rospy.numpy_msg import numpy_msg
from jsk_rviz_plugins.msg import OverlayText
from obstacle_detector.msg import Obstacles
import numpy as np
import rospy
import csv
import sys
import math
PKG = 'object-detector-fusion'
roslib.load_manifest(PKG)


np.set_printoptions(precision=3)
SAVE_TO_FILE = False
PUBLISH = False


class ParticleFilterViz:

    def __init__(self):
        self.start_time = rospy.get_time()
        self.time_elapsed = 0
        self.print_mu = [[], [], []]
        self.main_sub = rospy.Subscriber(
            '/estimate/values/mean', numpy_msg(Float32MultiArray), self.callback)
        self.main_sub = rospy.Subscriber(
            '/runtime', Float32, self.runtime_callback) 
        self.obstacle_sub = rospy.Subscriber(
            '/raw_obstacles/camera_obstacles', Obstacles, self.obstacle_callback)  
        self.lidar_sub = rospy.Subscriber(
            '/raw_obstacles/lidar_obstacles', Obstacles, self.lidar_callback)       
        self.text_pub_1 = rospy.Publisher(
            "/estimate/values/text_overlay/1", OverlayText, queue_size=1)
        self.text_pub_2 = rospy.Publisher(
            "/estimate/values/text_overlay/2", OverlayText, queue_size=1)     
        self.text_pub_3 = rospy.Publisher(
            "/estimate/values/text_overlay/3", OverlayText, queue_size=1)      
        if PUBLISH:
            self.dist = rospy.Publisher(
                '/estimate/values/1/dist', Float32, queue_size=1)
            self.x_1 = rospy.Publisher(
                '/estimate/values/1/pos_x', Float32, queue_size=1)
            self.y_1 = rospy.Publisher(
                '/estimate/values/1/pos_y', Float32, queue_size=1)
            self.dist = rospy.Publisher(
                '/estimate/values/1/vel', Float32, queue_size=1)
            self.x_vel_1 = rospy.Publisher(
                '/estimate/values/1/vel_x', Float32, queue_size=1)
            self.y_vel_1 = rospy.Publisher(
                '/estimate/values/1/vel_y', Float32, queue_size=1)
            self.x_acc_1 = rospy.Publisher(
                '/estimate/values/1/acc_x', Float32, queue_size=1)
            self.y_acc_1 = rospy.Publisher(
                '/estimate/values/1/acc_y', Float32, queue_size=1)
            self.rad_1 = rospy.Publisher(
                '/estimate/values/1/rad', Float32, queue_size=1)
    def runtime_callback(self, msg):
        self.runtime = msg.data

    def obstacle_callback(self, msg):
        self.obstacles = msg.circles
    def lidar_callback(self, msg):
        self.lidars = msg.circles
    def callback(self, msg):
        main_msg = np.reshape(msg.data, [3, 7])
        #print(main_msg)
        
        pos_x = main_msg[0, 0]
        pos_y = main_msg[0, 1]
        vel_x = main_msg[0, 2]
        vel_y = main_msg[0, 3]
        acc_x = main_msg[0, 4]
        acc_y = main_msg[0, 5]
        rad = main_msg[0, 6]
        if PUBLISH:
            self.x_1.publish(pos_x)
            self.y_1.publish(pos_y)
            self.x_vel_1.publish(vel_x)
            self.y_vel_1.publish(vel_y)
            self.x_acc_1.publish(acc_x)
            self.y_acc_1.publish(acc_y)
            self.rad_1.publish(rad)

        text = OverlayText()
        text.width = 400
        text.height = 600
        text.left = 10
        text.top = 10
        text.text_size = 12
        text.line_width = 2
        text.fg_color = ColorRGBA(25 / 255.0, 1.0, 240.0 / 255.0, 1.0)
        text.bg_color = ColorRGBA(0.0, 0.0, 0.0, 0.2)

        text.font = "DejaVu Sans Mono"
        arg = [x for x in main_msg[0, :]]
        dist = math.sqrt((arg[0]**2 + arg[1]**2))
        speed = math.sqrt((arg[2]**2 + arg[3]**2))
        text.text = """
Runtime: %.2f ms
Object 1
Position = [%.2f, %.2f]
Distance = %.2f
Velocity = [%.2f, %.2f]
Speed = %.2f
Acceleration = [%.2f, %.2f]
True Radius = %.2f
  """ % (self.runtime, arg[0], arg[1], dist, arg[2], arg[3], speed, arg[4], arg[5], arg[6])
        self.text_pub_1.publish(text)

        arg = [x for x in main_msg[1, :]]
        dist = math.sqrt((arg[0]**2 + arg[1]**2))
        speed = math.sqrt((arg[2]**2 + arg[3]**2))
        text.text = """
Object 2
Position = [%.2f, %.2f]
Distance = %.2f
Velocity = [%.2f, %.2f]
Speed = %.2f
Acceleration = [%.2f, %.2f]
True Radius = %.2f
  """ % (arg[0], arg[1], dist, arg[2], arg[3], speed, arg[4], arg[5], arg[6])
        self.text_pub_2.publish(text)

        arg = [x for x in main_msg[2, :]]
        dist = math.sqrt((arg[0]**2 + arg[1]**2))
        speed = math.sqrt((arg[2]**2 + arg[3]**2))
        text.text = """
Object 3
Position = [%.2f, %.2f]
Distance = %.2f
Velocity = [%.2f, %.2f]
Speed = %.2f
Acceleration = [%.2f, %.2f]
True Radius = %.2f
  """ % (arg[0], arg[1],dist, arg[2], arg[3], speed, arg[4], arg[5], arg[6])
        self.text_pub_3.publish(text)         
        
        for i, mu in enumerate(main_msg[:3]):
            self.time_elapsed = rospy.get_time() - self.start_time
            circles = self.obstacles[i]
            circles_lidar = self.lidars[i]
            
            self.print_mu[i].append(np.append(mu, 
            (self.runtime, 
            self.time_elapsed, 
            circles.center.x, 
            circles.center.y, 
            circles.true_radius,
            circles_lidar.x,
            circles_lidar.y,
            circles_lidar.true_radius)))
            #self.print_mu[i].append(np.append(mu, (self.runtime, self.time_elapsed)))
        


if __name__ == '__main__':
    rospy.init_node('pf_estimate_analysis')
    run = ParticleFilterViz()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down....")

    if SAVE_TO_FILE:
        for i in range(len(run.print_mu)):
            with open("estimate_data_{}.csv".format(i), "w") as f:
                csvwriter = csv.writer(f)
                csvwriter.writerow(["x_estimate",
                                    "y_estimate",
                                    "x_vel_estimate",
                                    "y_vel_estimate",
                                    "x_accel_estimate",
                                    "y_accel_estimate",
                                    "true_rad_estimate",
                                    "runtime",
                                    "time",
                                    "x_cam",
                                    "y_cam",
                                    "rad_cam",
                                    "x_lid",
                                    "y_lid",
                                    "rad_lid"])
                csvwriter.writerows(run.print_mu[i])
            print("Printed to estimate_data_{}.csv".format(i))
        f.close()
