#! /usr/bin/env python
'''
Script track obstacle yg terdeteksi oleh kamera dengan particle filter
'''
import roslib
import rospy
import csv

import numpy as np
import multi_particle_filter_a as pf
from scipy.optimize import linear_sum_assignment
from filterpy.stats import multivariate_multiply
from filterpy.monte_carlo import systematic_resample
from numpy.random import randn

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
            '/lidar_obstacles', Obstacles, self.lidar_callback)
        self.obstacle_sub = rospy.Subscriber(
            '/obstacles', Obstacles, self.camera_callback)
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
        self.count = 0

        self.N = 10
        self.pf_size = 3
        self.start_time = rospy.get_time()
        self.pf_is_running = False
        self.first_time = True

        self.old_camera_measurements = []
        self.runtime = []

        self.lidar_is_on = False
        #self.lidar_all = [CircleObstacle() for _ in range(self.pf_size)]
        self.lidar_time = self.start_time

        self.new_particles = np.empty((self.N, 7, self.pf_size))
        self.new_weights = np.empty((self.N, self.pf_size))

    def initialize(self, initial_pos, iteration):
        initial_x = (initial_pos.center.x, initial_pos.center.y,
                     0.05, 0.05, 0, 0, initial_pos.true_radius)
        std = (0.1, 0.1, 0.5, 0.5, 0.5, 0.5, 0.1)
        self.new_particles[:, :, iteration] = pf.create_gaussian_particles(
            mean=initial_x, std=std, N=self.N)
        self.new_weights[:, iteration] = pf.create_initial_weights(self.N)

        cam_circle_err_x = randn()*0.1
        cam_circle_err_y = randn()*0.1
        cam_circle_err_rad = randn()*0.1

        self.old_camera_measurements.append([initial_pos.center.x + cam_circle_err_x,
                                             initial_pos.center.y + cam_circle_err_y,
                                             initial_pos.true_radius + cam_circle_err_rad])

        for particle in self.new_particles[:, :, iteration]:
            pub_particle = Point()
            pub_particle.x = particle[0]
            pub_particle.y = particle[1]
            pub_particle.z = 0.35
            self.pub_particles.append(pub_particle)

    def run_pf(self, camera=None, lidar=None, dt=0, i=0):
        #self.camera_obstacle_pos = np.array([camera.center.x, camera.center.y])
        #self.camera_radius = np.array([camera.radius])
        #sigma (uncertainty) sensor with added noise
        cam_err_x = 0.3
        cam_err_y = 0.2
        cam_err_rad = 0.08

        if self.run_fusion[i]:
            lid_err_x = 0.1
            lid_err_y = 0.08
            lid_err_rad = 0.08
            lid_var = [[lid_err_x**2, 0, 0],
                       [0, lid_err_y**2, 0],
                       [0, 0, lid_err_rad**2]]
            cam_var = [[cam_err_x**2, 0, 0],
                       [0, cam_err_y**2, 0],
                       [0, 0, lid_err_rad**2]]
            zs, sigma = multivariate_multiply(camera, cam_var, lidar, lid_var)
            #fused_radius = (camera[2] * cam_err_rad**2 + lidar[2] * lid_err_rad**2)/(cam_err_rad**2+lid_err_rad**2)
            #camera_err_norm = camera_std_err/(camera_std_err+lidar_std_err)
            #lidar_err_norm = lidar_std_err/(camera_std_err+lidar_std_err)
            #R = np.average([camera_std_err, lidar_std_err],
            #                weights = [(1-camera_err_norm), (1-lidar_err_norm)])
            var = sigma * (1 + 0.5)

        else:
            #self.camera = self.camera_obstacle_pos + randn()* camera_std_err
            #self.camera = self.camera_obstacle_pos + np.array([randn()* camera_std_err_x, randn()* camera_std_err_y])
            #print(type(self.camera))
            #zs = np.concatenate((self.camera, self.camera_radius))
            zs = camera
            var = [[cam_err_x**2, 0, 0],
                   [0, cam_err_y**2, 0],
                   [0, 0, cam_err_rad**2]]
        # do predict
        std = (0.02, 0.02, 0.04, 0.04, 0.04, 0.04, 0.02)
        pf.predict(self.new_particles[:, :, i],
                   std=std,
                   dt=dt)

        # incorporate measurements
        #pf.update(self.new_particles[:, :,i], self.new_weights[:, i], z = zs, R = R)

        pf.update_multivariate(
            self.new_particles[:, :, i], self.new_weights[:, i], z=zs, R=var)

        # resample if too few effective particles
        if pf.neff(self.new_weights[:, i]) < self.N/2:
            indexes = systematic_resample(self.new_weights[:, i])
            self.new_weights[:, i] = pf.resample_from_index(
                self.new_particles[:, :, i], self.new_weights[:, i], indexes)
            #assert np.allclose(self.weights, 1/self.N)

        self.mu, self.var = pf.estimate(
            self.new_particles[:, :, i], self.new_weights[:, i])
        #reinitialize every t seconds
        #t = 0
        #time = round(self.time_elapsed, 1)
        if False:
            print("Reinitializing")
            initial = (zs[0], zs[1], 0.05, 0.05, 0, 0, zs[2])
            init_std = (0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1)
            self.new_particles[:, :, i] = pf.create_gaussian_particles(
                mean=initial, std=init_std, N=self.N)
            self.new_weights[:, i] = pf.create_initial_weights(self.N)

    def cost_function(self, x_i, x_j, y_i, y_j, r_i, r_j):
        #i = row; new measurements
        #j = column; old measurements
        cost = np.sqrt((x_i-x_j)**2 + (y_i-y_j)**2 + (r_i-r_j)**2)
        return cost

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
        self.lidar_all = ros_data.circles[:6]
        for i in range(len(self.lidar_all)):
            lidar_circle = self.lidar_all[i]
            self.new_lidar_measurements.append(
                [lidar_circle.center.x, lidar_circle.center.y, lidar_circle.true_radius])
            row_idx, col_idx, cost_matrix = self.objects_assigner(
                self.new_camera_measurements, self.new_lidar_measurements)
        #print("lidar new indices", row_idx)
        self.lidar_row_idx = row_idx
        self.lidar_col_idx = col_idx
        self.lidar_cost_matrix = cost_matrix
        self.lidar_time = rospy.get_time()
        self.lidar_is_on = True

    def camera_callback(self, ros_data):
        self.obstacles_full = ros_data
        self.camera_all = ros_data.circles[:self.pf_size]

    def pf_callback(self, ros_data):
        time = 1 / ros_data.data
        print(time)
        init_runtime = rospy.get_time()
        #obstacles_full = ros_data
        #camera_all = ros_data.circles[:self.pf_size]

        if self.first_time:
            self.pf_is_running = [False for _ in range(self.pf_size)]
            self.first_time = False
        #self.pf_size = len(camera_all)
        #print("old measurement", self.old_camera_measurements)
        self.new_camera_measurements = []

        cam_circle_err_x = randn()*0.1 # sigma (uncertainty sensor yg asli)
        cam_circle_err_y = randn()*0.1 # sigma (uncertainty sensor yg asli)
        cam_circle_err_rad = randn()*0.1  # sigma (uncertainty sensor yg asli)

        self.run_fusion = [False for _ in range(self.pf_size)]
        for i in range(len(self.camera_all)):
            camera_circle = self.camera_all[i]
            self.new_camera_measurements.append([camera_circle.center.x + cam_circle_err_x,
                                                 camera_circle.center.y + cam_circle_err_y,
                                                 camera_circle.true_radius + cam_circle_err_rad])
            self.row_idx, col_idx, cost_matrix = self.objects_assigner(
                self.old_camera_measurements, self.new_camera_measurements)
        #print("row", row_idx)
        #print("col", col_idx)
        #print("camera cost matrix", cost)
        #print("new measurement", self.new_camera_measurements)

        #Data Gating for camera measurements
        for k in range(len(self.row_idx)):
            camera_cost = cost_matrix[self.row_idx[k], col_idx[k]]
            if camera_cost > 2:
                self.row_idx = np.delete(self.row_idx, k)
                #self.row_idx[k] = -1

        if self.lidar_is_on:
            #Data Gating for lidar measurements
            #print("lidar cost matrix", self.lidar_cost_matrix)
            for k in range(len(self.lidar_row_idx)):
                #print("row", self.lidar_row_idx)
                #print("col", self.lidar_col_idx)
                #print("cost ", self.lidar_cost_matrix)
                lidar_cost = self.lidar_cost_matrix[self.lidar_row_idx[k],
                                                    self.lidar_col_idx[k]]
                #print(self.lidar_row_idx[k], lidar_cost)
                if lidar_cost < 0.4:
                    #print("cost", k)
                    self.run_fusion[self.lidar_col_idx[k]] = True
            #print("lidar row", self.lidar_row_idx)

        #print("cost matrix: ", cost_matrix)
        #print("new measurement indices (row)", self.row_idx)
        #print("old measurement indices (col)", col_idx)
        #print(len(col_idx))
        #self.pf_size = len(row_idx)

        i_lidar = 0
        for i in range(len(self.camera_all)):
            if not self.pf_is_running[i]:
                initial_pos = self.camera_all[i]
                self.initialize(initial_pos, i)
                print("Initialize for object {}".format(i))
                #print(len(self.pub_particles))
                self.run_pf(self.new_camera_measurements[i], dt=time, i=i)
                self.pf_is_running[i] = True

                #self.old_measurements.append(initial_pos)

            elif self.run_fusion[col_idx[i]]:
                #print("lidar measurements",len(self.new_lidar_measurements))
                #print("row_idx",len(self.lidar_row_idx))
                #print("Fused")
                self.run_pf(self.new_camera_measurements[self.row_idx[i]],
                            self.new_lidar_measurements[self.lidar_row_idx[i_lidar]], 
                            dt=time,
                            i=col_idx[i])
                i_lidar += 1
                #print("len", len(camera_all), len(self.lidar_all))
            else:
                self.run_pf(
                    self.new_camera_measurements[self.row_idx[i]], dt=time, i=col_idx[i])
            #print(self.mu)

            #print(len(self.new_particles[:,:,i]))
            #Fill the marker_particle of Marker class
            if i < len(col_idx) - 1:
                i_particle = col_idx[i]
            else:
                i_particle = i    

            for j, particle in enumerate(self.new_particles[:, :, i_particle]):
                pub_new_particle = Point()
                pub_new_particle.x = particle[0]
                pub_new_particle.y = particle[1]
                pub_new_particle.z = 0.35
                self.pub_particles[j+self.N*i_particle] = pub_new_particle

            

            self.est_values[i_particle] = self.mu
            #print("mu", self.mu)
            self.time_elapsed = rospy.get_time() - self.start_time

            if PRINT:
                print('object: ', i, self.time_elapsed)
                print('pos: ', np.array(self.new_camera_measurements[i][0:2]))
                print('distance:', np.linalg.norm(
                    [self.new_camera_measurements[i][0:2]], axis=1))
                print('velocity estimate: ', self.mu[2:4])
                print('acceleration estimate: ', self.mu[4:6])
                print('true radius estimate: ', self.mu[6])
                #print('camera: ', self.camera)
                #print('position error: ', self.obstacle_pos-self.mu[0:2])
                #test = True
            #print(self.mu[0], self.mu[1])

            pub_mean = Point()
            pub_mean.x = self.mu[0]
            pub_mean.y = self.mu[1]
            pub_mean.z = 0.32
            self.list_of_pub_mean.append(pub_mean)

        #print("New: ",np.shape(self.new_measurements))

        print("--------")
        pf_running_time = rospy.get_time()
        #Fill the marker_particle of Marker class
        self.marker_particle.header = self.obstacles_full.header
        self.marker_particle.type = 8  # POINTS
        self.marker_particle.points = self.pub_particles
        #print(len(self.pub_particles))
        self.marker_particle.scale.x = 0.02
        self.marker_particle.scale.y = 0.02
       # self.marker_particle.scale.z = 0.02
        self.marker_particle.pose.orientation.x = 0
        self.marker_particle.pose.orientation.y = 0
        self.marker_particle.pose.orientation.z = 0
        self.marker_particle.pose.orientation.w = 1

        self.marker_particle.color.r = 0
        self.marker_particle.color.g = 0
        self.marker_particle.color.b = 0
        self.marker_particle.color.a = 2

        self.obstacle_pub_particles.publish(self.marker_particle)

        #Fill the marker_mean of Marker class

        self.marker_mean.header = self.obstacles_full.header
        self.marker_mean.type = 6  # CUBE_LIST

        self.marker_pose.orientation.x = 0
        self.marker_pose.orientation.y = 0
        self.marker_pose.orientation.z = 0
        self.marker_pose.orientation.w = 1

        self.marker_mean.pose = self.marker_pose

        #print(self.list_of_pub_mean)

        self.marker_mean.points = self.list_of_pub_mean
        self.marker_mean.scale.x = 0.1
        self.marker_mean.scale.y = 0.1
        self.marker_mean.scale.z = 0.1

        mean_color = ColorRGBA()
        mean_color.r = 2.0 / 255.0
        mean_color.g = 255.0 / 255.0
        mean_color.b = 20.0 / 255.0
        mean_color.a = 1

        self.marker_mean.colors = [
            mean_color for _ in range(len(self.list_of_pub_mean))]
        #self.marker_mean.color.g = 0
        #self.marker_mean.color.b = 0
        #self.marker_mean.color.a = 1

        self.obstacle_pub_mean.publish(self.marker_mean)
        self.list_of_pub_mean = []

        #print(est_values)
        val = self.est_values.ravel()
        #print("values", val)
        pub_values = Float32MultiArray()
        pub_values.data = np.array(val, dtype=np.float32)
        self.obstacle_pub_values.publish(pub_values)

        self.running_time = rospy.get_time()
        #apabila lidar gak ngirim data dalam waktu > 0.1 s, sensor fusionnya gak dipake
        self.time_difference = self.running_time - self.lidar_time
        #print(self.time_difference)
        if self.time_difference >= 1/5.5:
            self.lidar_is_on = False

        if not len(col_idx) == 0:
            print("CHECK IT'S RUNNING ")
            for i in range(len(self.row_idx)):
                self.old_camera_measurements[col_idx[i]
                                             ] = self.new_camera_measurements[self.row_idx[i]]
        #print("PF Runtime", 1000 * (pf_running_time - init_runtime))
        #print("Runtime", 1000 * (self.running_time - init_runtime))
        #print("Count", self.count)
        self.count += 1
        self.pub_runtime.publish((self.running_time - init_runtime) * 1000)
        
        self.runtime.append([self.running_time - init_runtime, rospy.get_time() - self.start_time])

if __name__ == "__main__":
    rospy.init_node("particle_filters")
    run = ParticleFilter()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down....")
    if SAVE_TO_FILE:
        with open("runtime_data.csv", "w") as f:
            csvwriter = csv.writer(f)
            csvwriter.writerow(["runtime", "time"])
            csvwriter.writerows(run.runtime)
        print("Printed to runtime_data.csv")
        f.close()
