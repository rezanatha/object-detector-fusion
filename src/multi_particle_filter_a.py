#!/usr/bin/env python
'''
Script yg isinya fungsi-fungsi untuk keperluan particle filtering
'''
import numpy as np
from numpy.linalg import norm
from numpy.random import randn
from numpy.random import uniform
from filterpy.monte_carlo import systematic_resample
from filterpy.stats import multivariate_multiply
import matplotlib.pyplot as plt
import scipy.stats


def create_uniform_particles(x_range, y_range, xdot_range, ydot_range, N):
    particles = np.empty((N, 4))
    particles[:, 0] = uniform(x_range[0], x_range[1], size=N)
    particles[:, 1] = uniform(xdot_range[0], xdot_range[1], size=N)
    particles[:, 2] = uniform(y_range[0], y_range[1], size=N)
    particles[:, 3] = uniform(ydot_range[0], ydot_range[1], size=N)
    return particles

def create_gaussian_particles(mean, std, N):
    particles = np.empty((N, 7))
    for i in range(7):
        particles[:, i] = mean[i] + (randn(N)*std[i])
    return particles

def create_initial_weights(N):
    return np.ones(N) / N

def predict(particles, std, dt):
    
    N = len(particles)
    #predict object true radius
    particles[:, 6] += randn(N) *std[6]

    #predict acceleration
    particles[:, 4] += randn(N) * std[4]
    particles[:, 5] += randn(N) * std[5]
    #predict velocity
    particles[:, 2] += particles[:, 4] * dt + randn(N) * std[2]
    particles[:, 3] += particles[:, 5] * dt + randn(N) * std[3]

    #predict object position
    particles[:, 0] += randn(N) * std[0] + particles[:, 2] * dt + 0.5 * particles[:, 4] * dt**2 
    particles[:, 1] += randn(N) * std[1] + particles[:, 3] * dt + 0.5 * particles[:, 5] * dt**2

    return particles
    
def update(particles, weights, z, R):
    position_update = particles[:, 0:2]
    radius_update = np.transpose([particles[:, 6]]) 
    update = np.hstack((position_update, radius_update))
    distance = np.linalg.norm(update - z, axis = 1)
    weights *= scipy.stats.norm(distance, R).pdf(0)
    #weights *= scipy.stats.norm(distance, Ry).pdf(0)       
    weights += 1.e-300      # avoid round-off to zero
    weights /= sum(weights) # normalize
    return weights

def update_multivariate(particles, weights, z, R):
    position_update = particles[:, 0:2]
    radius_update = np.transpose([particles[:, 6]]) 
    update = np.hstack((position_update, radius_update))
    weights *= scipy.stats.multivariate_normal.pdf(x = update, mean = z, cov = R)
    weights += 1.e-300      # avoid round-off to zero
    weights /= sum(weights) # normalize
    return weights

def estimate(particles, weights):
    """returns mean and variance of the weighted particles"""

    pos = particles[:, :]
    mean = np.average(pos, weights=weights, axis=0)
    var  = np.average((pos - mean)**2, weights=weights, axis=0)
    return mean, var

def neff(weights):
    return 1. / np.sum(np.square(weights))

def resample_from_index(particles, weights, indexes):
    particles[:] = particles[indexes]
    #print("before: ", np.shape(weights))
    resized_weights = np.resize(weights, len(particles))
    #print("resized: ", np.shape(resized_weights))
    resized_weights.fill (1.0 / len(weights))
    return resized_weights

