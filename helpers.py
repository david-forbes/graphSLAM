from robot_class import robot
from math import *
import random
import numpy as np
import matplotlib.pyplot as plt
import seaborn as sns


# --------
# this helper function displays the world that a robot is in
# it assumes the world is a square grid of some given size
# and that landmarks is a list of landmark positions(an optional argument)
def display_world(world_size, position, landmarks=None):
    
    # using seaborn, set background grid to gray
    sns.set_style("dark")

    # Plot grid of values
    world_grid = np.zeros((world_size+1, world_size+1))

    # Set minor axes in between the labels
    ax=plt.gca()
    cols = world_size+1
    rows = world_size+1

    ax.set_xticks([x for x in range(1,cols)],minor=True )
    ax.set_yticks([y for y in range(1,rows)],minor=True)
    
    # Plot grid on minor axes in gray (width = 1)
    plt.grid(which='minor',ls='-',lw=1, color='white')
    
    # Plot grid on major axes in larger width
    plt.grid(which='major',ls='-',lw=2, color='white')
    
    # Create an 'o' character that represents the robot
    # ha = horizontal alignment, va = vertical
    ax.text(position[0], position[1], 'o', ha='center', va='center', color='r', fontsize=30)
    
    # Draw landmarks if they exists
    if(landmarks is not None):
        # loop through all path indices and draw a dot (unless it's at the car's location)
        for pos in landmarks:
            if(pos != position):
                ax.text(pos[0], pos[1], 'x', ha='center', va='center', color='purple', fontsize=20)
    
    # Display final result
    plt.show()


    
    # using seaborn, set background grid to gray
    

    # Plot grid of values
    
    
    # Display final result
       
# --------
# this routine makes the robot data
# the data is a list of measurements and movements: [measurements, [dx, dy]]
# collected over a specified number of time steps, N
#
def make_data(N, num_landmarks, world_size, measurement_range, motion_noise, 
              measurement_noise, distance):


    

    

        data = []

        # make robot and landmarks
        r = robot(world_size, measurement_range, motion_noise, measurement_noise)
        r.make_landmarks(num_landmarks)
        seen = [False for row in range(num_landmarks)]
        '''
        # guess an initial motion
        orientation = random.random() * 2.0 * pi
        dx = cos(orientation) * distance
        dy = sin(orientation) * distance
        
        '''
        for k in range(N-1):
            r.make_time_step()
            
            
            

          
        
   
        data = r.get_data()



        return data,r

def initialize_robot(N, num_landmarks, world_size, measurement_range, motion_noise, 
              measurement_noise, distance):

    data = []

    # make robot and landmarks
    r = robot(world_size, measurement_range, motion_noise, measurement_noise)
    r.make_landmarks(num_landmarks)
 

    



    return data,r




def initialize_constraints(N, num_landmarks, world_size):
        ''' This function takes in a number of time steps N, number of landmarks, and a world_size,
            and returns initialized constraint matrices, omega and xi.'''
        
        ## Recommended: Define and store the size (rows/cols) of the constraint matrix in a variable
        size = (N + num_landmarks) * 2
        
        # initial position weight
        weight = 100
        
        ## TODO: Define the constraint matrix, Omega, with two initial "strength" values
        ## for the initial x, y location of our robot
        omega = np.zeros((size, size))
        omega[0][0] = omega[1][1] = weight
        
        ## TODO: Define the constraint *vector*, xi
        ## you can assume that the robot starts out in the middle of the world with 100% confidence
        xi = np.zeros((size, 1))
        xi[0][0] = xi[1][0] = world_size / 2.0 * weight
        
        return omega, xi