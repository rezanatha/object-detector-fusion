#! /usr/bin/env python
'''
Script track obstacle yg terdeteksi oleh kamera & lidar dengan particle filter
'''
import roslib
import rospy
import csv

import numpy as np
import multi_particle_filter_a as pf
from scipy.optimize import linear_sum_assignment
from filterpy.stats import multivariate_multiply
from filterpy.monte_carlo import systematic_resample

from rospy.numpy_msg import numpy_msg
from std_msgs.msg import ColorRGBA, Float32MultiArray, Float32
from geometry_msgs.msg import Point, Pose
from visualization_msgs.msg import Marker
from obstacle_detector.msg import Obstacles

PKG = 'object-detector-fusion'
roslib.load_manifest(PKG)


np.set_printoptions(precision=3)
PRINT = False
SAVE_TO_FILE = False


class ParticleFilter:
    mu_array = []
    time_elapsed = 0

    def __init__(self):
        self.lidar_obstacle_sub = rospy.Subscriber(
            '/raw_obstacles/lidar_obstacles', Obstacles, self.lidar_callback)
        self.obstacle_sub = rospy.Subscriber(
            '/raw_obstacles/camera_obstacles', Obstacles, self.camera_callback)
        self.pf_sub = rospy.Subscriber(
            '/pub_pf', Float32, self.pf_callback)
        self.obstacle_pub_mean = rospy.Publisher(
            '/estimate/viz/mean', Marker, queue_size=10)
        self.obstacle_pub_particles = rospy.Publisher(
            '/estimate/viz/particles', Marker, queue_size=100)
        self.obstacle_pub_values = rospy.Publisher(
            '/estimate/values/mean', numpy_msg(Float32MultiArray), queue_size=10)
        self.pub_runtime = rospy.Publisher(
            '/runtime', Float32, queue_size=10)
        self.marker_particle = Marker()
        self.marker_mean = Marker()
        self.marker_pose = Pose()
        self.pub_particles = []
        self.list_of_pub_mean = []
        self.est_values = np.zeros([3, 7], dtype=np.float32)

        self.N = 1000
        self.pf_size = 3
        self.start_time = rospy.get_time()
        self.pf_is_running = False
        self.first_time = True

        self.old_camera_measurements = []

        self.new_particles = np.empty((self.N, 7, self.pf_size))
        self.new_weights = np.empty((self.N, self.pf_size))

        self.lidar_is_on = False
        self.lidar_time = self.start_time

    def initialize(self, initial_pos, iteration):
        initial_x = (initial_pos.center.x, initial_pos.center.y,
                     0.05, 0.05, 0, 0, initial_pos.true_radius)
        std = (0.2, 0.2, 0.5, 0.5, 0.5, 0.5, 0.1)
        self.new_particles[:, :, iteration] = pf.create_gaussian_particles(
            mean=initial_x, std=std, N=self.N)
        self.new_weights[:, iteration] = pf.create_initial_weights(self.N)

        self.old_camera_measurements.append(
            [initial_pos.center.x, initial_pos.center.y, initial_pos.true_radius])

        for particle in self.new_particles[:, :, iteration]:
            pub_particle = Point()
            pub_particle.x = particle[0]
            pub_particle.y = particle[1]
            pub_particle.z = 0.35
            self.pub_particles.append(pub_particle)

    def run_pf(self, camera, lidar, dt, i):
        #sigma (uncertainty) for the camera
        cam_err_x = 0.35
        cam_err_y = 0.25
        cam_err_rad = 0.1

        if lidar is not None:
            #sigma (uncertainty) for lidar
            lid_err_x = 0.05
            lid_err_y = 0.05
            lid_err_rad = 0.05
            noise_factor = 0

            lid_err = [[lid_err_x**2, 0, 0],
                       [0, lid_err_y**2, 0],
                       [0, 0, lid_err_rad**2]]
            cam_err = [[cam_err_x**2, 0, 0],
                       [0, cam_err_y**2, 0],
                       [0, 0, lid_err_rad**2]]


            zs, sigma = multivariate_multiply(camera, cam_err, lidar, lid_err)
            R = sigma * (1 + noise_factor)

        else:
            zs = camera
            R = [[cam_err_x**2, 0, 0],
                 [0, cam_err_y**2, 0],
                 [0, 0, cam_err_rad**2]]

        # do predict
        std_predict = (0.02, 0.02, 0.04, 0.04, 0.04, 0.04, 0.04)
        pf.predict(self.new_particles[:, :, i],
                   std=std_predict,
                   dt=dt)

        # incorporate measurements
        pf.update_multivariate(
            self.new_particles[:, :, i], self.new_weights[:, i], z=zs, R=R)

        # resample if too few effective particles
        if pf.neff(self.new_weights[:, i]) < self.N/2:
            indexes = systematic_resample(self.new_weights[:, i])
            self.new_weights[:, i] = pf.resample_from_index(
                self.new_particles[:, :, i], self.new_weights[:, i], indexes)

        self.mu, self.var = pf.estimate(
            self.new_particles[:, :, i], self.new_weights[:, i])

        # reinitialize
        t = 10
        time = round(self.time_elapsed, 1)
        mu_pos = np.array(self.mu[0:2])
        z_pos = np.array(zs[0:2])
        euclid_dist = np.linalg.norm(mu_pos-z_pos)
        if time % t == 10 or euclid_dist > 2:
            print("Reinitializing")
            initial = (zs[0], zs[1], 0.1, 0.1, 0, 0, zs[2])
            init_std = (0.01, 0.01, 0.1, 0.1, 0.1, 0.1, 0.01)
            self.new_particles[:, :, i] = pf.create_gaussian_particles(
                mean=initial, std=init_std, N=self.N)
            self.new_weights[:, i] = pf.create_initial_weights(self.N)

    def cost_function(self, x_i, x_j, y_i, y_j, r_i, r_j):
        #i = row; new measurements
        #j = column; old measurements
        cost = np.sqrt((x_i-x_j)**2 + (y_i-y_j)**2 + (r_i-r_j)**2)
        return cost

    def calibrate(self, old, a = 1, b = 0):
        return a*old + b

    def objects_assigner(self, old_measurements, new_measurements):
        old = np.array(old_measurements)
        new = np.array(new_measurements)

        cost_matrix = np.empty((len(new), len(old)))

        for i in range(len(cost_matrix)):
            for j in range(len(cost_matrix[i])):
                cost_matrix[i, j] = self.cost_function(
                    new[i, 0], old[j, 0], new[i, 1], old[j, 1], new[i, 2], old[j, 2])
        row_idx, col_idx = linear_sum_assignment(cost_matrix)
        return row_idx, col_idx, cost_matrix

    def lidar_callback(self, ros_data):
        self.new_lidar_measurements = []
        self.lidar_all = ros_data.circles[:self.pf_size]
        for i in range(len(self.lidar_all)):
            lidar_circle = self.lidar_all[i]
            new_lid_x = self.calibrate(lidar_circle.center.x, 1.0124, -0.0401)
            new_lid_y = self.calibrate(lidar_circle.center.y, 0.9982, -0.0215)
            self.new_lidar_measurements.append(
                [new_lid_x , new_lid_y, lidar_circle.true_radius])
            self.lidar_row_idx, self.lidar_col_idx, self.lidar_cost_matrix = self.objects_assigner(self.new_camera_measurements, self.new_lidar_measurements)
        #print("Lidar measurement in lidar callback", self.new_lidar_measurements)
        self.lidar_time = rospy.get_time()
        self.lidar_is_on = True

    def camera_callback(self, ros_data):
        self.obstacles_full = ros_data
        self.camera_all = ros_data.circles[:self.pf_size]

    def pf_callback(self, ros_data):
        init_runtime = rospy.get_time()
        time = 1 / ros_data.data
        print(time)

        fusion_idx = [None, None, None]
        self.camera_done = False

        if self.first_time:
            self.pf_is_running = [False for _ in range(self.pf_size)]
            self.first_time = False
        #print("Running", self.pf_is_running)

        self.new_camera_measurements = []
        self.run_fusion = [False for _ in range(self.pf_size)]
        #print("run fusion before", self.run_fusion)
        for i in range(len(self.camera_all)):
            camera_circle = self.camera_all[i]
            new_cam_x = self.calibrate(camera_circle.center.x, 0.7485, 0.2889)
            new_cam_y = self.calibrate(camera_circle.center.y, 0.9955, -0.0108)
            self.new_camera_measurements.append(
                [new_cam_x, new_cam_y, camera_circle.true_radius])
            self.row_idx, self.col_idx, self.cost_matrix = self.objects_assigner(
                self.old_camera_measurements, self.new_camera_measurements)

        #Data Gating for camera measurements
        for k in range(len(self.row_idx)):
            camera_cost = self.cost_matrix[self.row_idx[k], self.col_idx[k]]
            print("camera cost", camera_cost)
            if camera_cost > 5:
                self.row_idx = np.delete(self.row_idx, k)
                #self.row_idx[k] = -1
                print("Blocked")

        if self.lidar_is_on:
            #print("is ON?", self.lidar_is_on)
            lidars = self.new_lidar_measurements
            for k in range(len(self.lidar_row_idx)):
                lidar_cost = self.lidar_cost_matrix[self.lidar_row_idx[k],
                                                    self.lidar_col_idx[k]]
                #print("ROW AND COST", self.lidar_row_idx[k], lidar_cost)
                if lidar_cost < 0.5:
                    #print("{} with cost {} are fusioned with {}".format(self.lidar_row_idx[k], lidar_cost, self.lidar_col_idx[k]))
                    self.run_fusion[self.lidar_col_idx[k]] = True
                    #print("the measurement", self.new_lidar_measurements)
                    fusion_idx[self.lidar_col_idx[k]] = self.lidar_row_idx[k]

        #print("run fusion after", self.run_fusion)
        for i in range(len(self.new_camera_measurements)):
            if not self.pf_is_running[i]:
                initial_pos = self.camera_all[i]
                self.initialize(initial_pos, i)
                self.run_pf(
                    self.new_camera_measurements[i], None, dt=time, i=i)
                self.pf_is_running[i] = True

            elif self.run_fusion[self.col_idx[i]]:
                print("Fused")
                #print("lidar row", self.lidar_row_idx)
                #print("lidar col", self.lidar_col_idx)
                #print("camera row", self.row_idx)
                #print("camera col", self.col_idx)
                #print("but which one", self.lidar_row_idx[i_lidar], self.new_lidar_measurements[self.lidar_row_idx[i_lidar]])
                #print("fusion index is {}. Now we use {} indexed with {}".format(fusion_idx, fusion_idx[i], i))
                #print("fusion index (col) is {}. Now we use {} indexed with {}".format(fusion_idx, fusion_idx[self.col_idx[i]], self.col_idx[i]))
                self.run_pf(self.new_camera_measurements[self.row_idx[i]],
                            lidars[fusion_idx[self.col_idx[i]]], 
                            dt=time, i=self.col_idx[i])
            else:
                self.run_fusion[self.col_idx[i]] = False
                self.run_pf(
                    self.new_camera_measurements[self.row_idx[i]], None, dt=time, i=self.col_idx[i])
                    
            #Fill the marker_particle of Marker class

            if i < len(self.col_idx) - 1:
                i_particle = self.col_idx[i]
            else:
                i_particle = i          

            for j, particle in enumerate(self.new_particles[:, :, self.col_idx[i]]):
                pub_new_particle = Point()
                pub_new_particle.x = particle[0]
                pub_new_particle.y = particle[1]
                pub_new_particle.z = 0.35
                self.pub_particles[j+self.N*self.col_idx[i]] = pub_new_particle

            self.time_elapsed = rospy.get_time() - self.start_time

            self.est_values[i_particle] = self.mu
            if PRINT:
                print('object: ', self.col_idx[i], self.time_elapsed)
                print('Fused?', self.run_fusion[self.col_idx[i]])
                print('pos: ', np.array(self.new_camera_measurements[i][0:2]))
                print('distance:', np.linalg.norm(
                    [self.new_camera_measurements[i][0:2]], axis=1))
                print('velocity estimate: ', self.mu[2:4])
                print('acceleration estimate: ', self.mu[4:6])
                print('true radius estimate: ', self.mu[6])
                #print('camera: ', self.camera)
                #print('position error: ', self.obstacle_pos-self.mu[0:2])
                #test = True
            pub_mean = Point()
            pub_mean.x = self.mu[0]
            pub_mean.y = self.mu[1]
            pub_mean.z = 0.32
            self.list_of_pub_mean.append(pub_mean)

        print("--------")
        #Fill the marker_particle of Marker class
        self.marker_particle.header = self.obstacles_full.header
        self.marker_particle.type = 8  # POINTS
        self.marker_particle.points = self.pub_particles
        self.pub_particles = [Point() for _ in range(len(self.pub_particles))]
        #print(len(self.pub_particles))
        self.marker_particle.scale.x = 0.03
        self.marker_particle.scale.y = 0.03
       # self.marker_particle.scale.z = 0.02
        self.marker_particle.pose.orientation.x = 0
        self.marker_particle.pose.orientation.y = 0
        self.marker_particle.pose.orientation.z = 0
        self.marker_particle.pose.orientation.w = 1

        self.marker_particle.color.r = 0
        self.marker_particle.color.g = 0
        self.marker_particle.color.b = 0
        self.marker_particle.color.a = 1

        #self.marker_particle.lifetime = rospy.Duration(1)
        self.marker_particle.ns = "particles"

        self.obstacle_pub_particles.publish(self.marker_particle)

        #Fill the marker_mean of Marker class
        self.marker_mean.header = self.obstacles_full.header
        self.marker_mean.type = 6  # CUBE_LIST

        self.marker_pose.orientation.x = 0
        self.marker_pose.orientation.y = 0
        self.marker_pose.orientation.z = 0
        self.marker_pose.orientation.w = 1

        self.marker_mean.pose = self.marker_pose

        self.marker_mean.points = self.list_of_pub_mean
        self.marker_mean.scale.x = 0.15
        self.marker_mean.scale.y = 0.15
        self.marker_mean.scale.z = 0.15

        mean_color = ColorRGBA()
        mean_color.r = 230.0 / 255.0
        mean_color.g = 26.0 / 255.0
        mean_color.b = 102.0 / 255.0
        mean_color.a = 1

        self.marker_mean.colors = [
            mean_color for _ in range(len(self.list_of_pub_mean))]

        self.obstacle_pub_mean.publish(self.marker_mean)
        self.list_of_pub_mean = []

        val = self.est_values.ravel()
        #print("values", val)
        pub_values = Float32MultiArray()
        pub_values.data = np.array(val, dtype=np.float32)
        self.obstacle_pub_values.publish(pub_values)

        self.running_time = rospy.get_time()
        #apabila lidar gak ngirim data dalam waktu > 0.1 s, sensor fusionnya gak dipake
        self.time_difference = self.running_time - self.lidar_time
        if self.time_difference >= 0.08:
            self.lidar_is_on = False

        #Update Old measurement dgn yang new
        for i in range(len(self.row_idx)):
            self.old_camera_measurements[self.col_idx[i]
                                        ] = self.new_camera_measurements[self.row_idx[i]]
        self.pub_runtime.publish((self.running_time - init_runtime) * 1000)
        #print("Runtime", self.running_time-init_runtime)
        self.camera_done = True


if __name__ == "__main__":
    rospy.init_node('particle_filters')
    run = ParticleFilter()
    #write estimate data to file
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down....")

