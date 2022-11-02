import numpy as np
from helpers import *
from slam import *
import math

# import data viz resources
import matplotlib.pyplot as plt
from pandas import DataFrame
import seaborn as sns
from slam_class import *
from matplotlib.animation import FuncAnimation
# Display the final world!

world_size         = 100.0    # size of world (square)
num_landmarks = 2*round(world_size*0.7 + world_size*0.7*0.7)   # number of landmarks
N                  =  100   # time steps


# robot parameters
measurement_range  =20.0     # range at which we can sense landmarks
motion_noise       = 1      # noise in robot motion
measurement_noise  = 1      # noise in the measurements
distance           = 5.0     # distance by which robot (intends to) move each iteratation 


print(num_landmarks)

data,r = initialize_robot(N, num_landmarks, world_size, measurement_range, motion_noise, measurement_noise, distance)
print(len(data))

#mu1, omega1, xi1 = slam_pi([data[0]], 2, num_landmarks, world_size, motion_noise, measurement_noise)



slam_object = slam(N, num_landmarks, world_size, motion_noise, measurement_noise)
for x in range(N-1):
    data=r.make_time_step()
    slam_object.add_time_step(data[x])

    
    mu, omega, xi,spotted_landmarks = slam_object.run_slam()


if(mu is not None):
    
    poses, landmarks = get_poses_landmarks(mu, N,num_landmarks)
    print(len(landmarks))
 

plt.rcParams["figure.figsize"] = (20,20)


if 'poses' in locals():
    
    print('Last pose: ', poses)
    

    display_world(int(world_size), poses[-1], landmarks)

    
