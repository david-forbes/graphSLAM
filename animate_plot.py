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
distance           = 1     # distance by which robot (intends to) move each iteratation 

position_list = []


data,r = initialize_robot(N, num_landmarks, world_size, measurement_range, motion_noise, measurement_noise, distance)





slam_object = slam(N, num_landmarks, world_size, motion_noise, measurement_noise)
fig,ax = plt.subplots()

def animate(i):
    data=r.make_time_step()
    slam_object.add_time_step(data[i])

    
    mu, omega, xi,spotted_landmarks = slam_object.run_slam()


    if(mu is not None):
    
        poses, landmarks = get_poses_landmarks(mu, N,num_landmarks)
        print(len(landmarks))
 

    


    if 'poses' in locals():
    
        #print('Last pose: ', poses)
        
        position=poses[i+1]
        

        

        world_grid = np.zeros((int(world_size+1), int(world_size+1)))

        # Set minor axes in between the labels
        
        cols = int(world_size+1)
        rows = int(world_size+1)

        ax.clear()
        ax.set_xticks([x for x in range(1,cols)],minor=True )
        ax.set_yticks([y for y in range(1,rows)],minor=True)
        
        # Plot grid on minor axes in gray (width = 1)
        #plt.grid(which='minor',ls='-',lw=1, color='white')
        
        # Plot grid on major axes in larger width
        #plt.grid(which='major',ls='-',lw=2, color='white')
        
        # Create an 'o' character that represents the robot
        # ha = horizontal alignment, va = vertical
        
        ax.text(position[0], position[1], 'o', ha='center', va='center', color='r', fontsize=30)
        
        # Draw landmarks if they exists
        if(landmarks is not None):
            # loop through all path indices and draw a dot (unless it's at the car's location)
            for pos in landmarks:
                if(pos != position and (pos[0]!=0 and pos[1]!=0)):
                    ax.text(pos[0], pos[1], 'X', ha='center', va='center', color='purple', fontsize=20)

    
ani = FuncAnimation(fig, animate, frames=N-1, interval=100, repeat=False)

plt.show()