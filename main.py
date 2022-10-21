import numpy as np
from helpers import *
from slam import *
import math

# import data viz resources
import matplotlib.pyplot as plt
from pandas import DataFrame
import seaborn as sns
from slam_class import *
# Display the final world!
num_landmarks      = 230    # number of landmarks
N                  = 20    # time steps
world_size         = 100.0    # size of world (square)

# robot parameters
measurement_range  =5.0     # range at which we can sense landmarks
motion_noise       = 1.0      # noise in robot motion
measurement_noise  = 1.0      # noise in the measurements
distance           = 5.0     # distance by which robot (intends to) move each iteratation 


# make_data instantiates a robot, AND generates random landmarks for a given world size and number of landmarks
data,r = make_data(N, num_landmarks, world_size, measurement_range, motion_noise, measurement_noise, distance)

print(len(data))

#mu1, omega1, xi1 = slam_pi([data[0]], 2, num_landmarks, world_size, motion_noise, measurement_noise)



slam_object = slam(N, num_landmarks, world_size, motion_noise, measurement_noise)
for x in range(N-1):
    slam_object.add_time_step(data[x])

    mu, omega, xi = slam_object.run_slam()


if(mu is not None):
    # get the lists of poses and landmarks
    # and print them out
    poses, landmarks = get_poses_landmarks(mu, N,num_landmarks)
    #print_all(poses, landmarks)
# define figure size

plt.rcParams["figure.figsize"] = (20,20)

# check if poses has been created
if 'poses' in locals():
    # print out the last pose
    print('Last pose: ', poses)
    # display the last position of the robot *and* the landmark positions
    #for x in range(len(landmarks)):
        #landmarks[x] = (landmarks[x][0]+50,landmarks[x][1]+50)

    display_world(int(world_size), poses[-1], landmarks)

    
