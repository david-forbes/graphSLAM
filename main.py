import numpy as np
from helpers import *
from slam import *
import math

# import data viz resources
import matplotlib.pyplot as plt
from pandas import DataFrame
import seaborn as sns

# Display the final world!
num_landmarks      = 230    # number of landmarks
N                  = 20       # time steps
world_size         = 100.0    # size of world (square)

# robot parameters
measurement_range  = 50.0     # range at which we can sense landmarks
motion_noise       = 1.0      # noise in robot motion
measurement_noise  = 1.0      # noise in the measurements
distance           = 20.0     # distance by which robot (intends to) move each iteratation 


# make_data instantiates a robot, AND generates random landmarks for a given world size and number of landmarks
data,r = make_data(N, num_landmarks, world_size, measurement_range, motion_noise, measurement_noise, distance)

mu, omega, xi = slam(data, N, num_landmarks, world_size, motion_noise, measurement_noise)

print(r.get_landmarks())
# print out the resulting landmarks and poses
if(mu is not None):
    # get the lists of poses and landmarks
    # and print them out
    poses, landmarks = get_poses_landmarks(mu, N,num_landmarks)
    print_all(poses, landmarks)
# define figure size
plt.rcParams["figure.figsize"] = (20,20)

# check if poses has been created
if 'poses' in locals():
    # print out the last pose
    print('Last pose: ', poses[-1])
    # display the last position of the robot *and* the landmark positions
    display_world(int(world_size), poses[-1], landmarks)