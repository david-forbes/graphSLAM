
import numpy as np
from helpers import make_data
import slam
import math
# import data viz resources
import matplotlib.pyplot as plt
from pandas import DataFrame
import seaborn as sns
## slam takes in 6 arguments and returns mu, 
## mu is the entire path traversed by a robot (all x,y poses) *and* all landmarks locations
def slam(data, N, num_landmarks, world_size, motion_noise, measurement_noise):
    
    ## TODO: Use your initilization to create constraint matrices, omega and xi
    omega, xi = initialize_constraints(N, num_landmarks, world_size)    
    
    motion_strength = 1.0 / motion_noise
    measurement_strength = 1.0 / measurement_noise
    
    ## TODO: Iterate through each time step in the data
    ## get all the motion and measurement data as you iterate
    
    assert (len(data)+1)==N, "Data Error: The data size doesn't match N!"
    
    for time_step in range(N-1):
        measurements = data[time_step][0]
        motion = data[time_step][1]              
            
    ## TODO: update the constraint matrix/vector to account for all *measurements*
    ## this should be a series of additions that take into account the measurement noise
        for measurement in measurements:
            ldmk_idx = measurement[0]
            ldmk_dx = measurement[1]
            ldmk_dy = measurement[2]
            
            # the row/column index of x axis of pose and the current measured landmark in 
            # the omega matrix and xi vector; the y axis will simply be x + 1
            pose_row_col = (time_step) * 2
            ldmk_row_col = (N + ldmk_idx) * 2
            
            # update x axis
            omega[pose_row_col][pose_row_col] += measurement_strength
            omega[ldmk_row_col][ldmk_row_col] += measurement_strength
            omega[pose_row_col][ldmk_row_col] -= measurement_strength
            omega[ldmk_row_col][pose_row_col] -= measurement_strength
            
            xi[pose_row_col][0] -= measurement_strength * ldmk_dx
            xi[ldmk_row_col][0] += measurement_strength * ldmk_dx
            
            # update y axis
            omega[pose_row_col+1][pose_row_col+1] += measurement_strength
            omega[ldmk_row_col+1][ldmk_row_col+1] += measurement_strength
            omega[pose_row_col+1][ldmk_row_col+1] -= measurement_strength
            omega[ldmk_row_col+1][pose_row_col+1] -= measurement_strength
            
            xi[pose_row_col+1][0] -= measurement_strength * ldmk_dy
            xi[ldmk_row_col+1][0] += measurement_strength * ldmk_dy
            
            
    ## TODO: update the constraint matrix/vector to account for all *motion* and motion noise
        pose_pre = (time_step) * 2
        pose_cur = (time_step + 1) * 2  # this is a repeated definition only for easy to read purpose        
        
        pose_dx = motion[0]
        pose_dy = motion[1]
        
        # update x axis
        omega[pose_pre][pose_pre] += motion_strength
        omega[pose_cur][pose_cur] += motion_strength
        omega[pose_pre][pose_cur] -= motion_strength
        omega[pose_cur][pose_pre] -= motion_strength

        xi[pose_pre][0] -= motion_strength * pose_dx
        xi[pose_cur][0] += motion_strength * pose_dx

        # update y axis
        omega[pose_pre+1][pose_pre+1] += motion_strength
        omega[pose_cur+1][pose_cur+1] += motion_strength
        omega[pose_pre+1][pose_cur+1] -= motion_strength
        omega[pose_cur+1][pose_pre+1] -= motion_strength

        xi[pose_pre+1][0] -= motion_strength * pose_dy
        xi[pose_cur+1][0] += motion_strength * pose_dy
        
    
    ## TODO: After iterating through all the data
    ## Compute the best estimate of poses and landmark positions
    ## using the formula, omega_inverse * Xi
    omega_inv = np.linalg.inv(np.matrix(omega))
    mu = omega_inv*xi   
    
    return mu, omega, xi # return `mu`

## TODO: Complete the code to implement SLAM

## slam takes in 6 arguments and returns mu, 
## mu is the entire path traversed by a robot (all x,y poses) *and* all landmarks locations
def slam(data, N, num_landmarks, world_size, motion_noise, measurement_noise):
    
    ## TODO: Use your initilization to create constraint matrices, omega and xi
    omega, xi = initialize_constraints(N, num_landmarks, world_size)    
    
    motion_strength = 1.0 / motion_noise
    measurement_strength = 1.0 / measurement_noise
    
    ## TODO: Iterate through each time step in the data
    ## get all the motion and measurement data as you iterate
    
    assert (len(data)+1)==N, "Data Error: The data size doesn't match N!"
    
    for time_step in range(N-1):
        measurements = data[time_step][0]
        motion = data[time_step][1]              
            
    ## : update the constraint matrix/vector to account for all *measurements*
    ## this should be a series of additions that take into account the measurement noise
        for measurement in measurements:
            ldmk_idx = measurement[0]
            ldmk_dx = measurement[1]
            ldmk_dy = measurement[2]
            
            # the row/column index of x axis of pose and the current measured landmark in 
            # the omega matrix and xi vector; the y axis will simply be x + 1
            pose_row_col = (time_step) * 2
            ldmk_row_col = (N + ldmk_idx) * 2
            
            # update x axis
            omega[pose_row_col][pose_row_col] += measurement_strength
            omega[ldmk_row_col][ldmk_row_col] += measurement_strength
            omega[pose_row_col][ldmk_row_col] -= measurement_strength
            omega[ldmk_row_col][pose_row_col] -= measurement_strength
            
            xi[pose_row_col][0] -= measurement_strength * ldmk_dx
            xi[ldmk_row_col][0] += measurement_strength * ldmk_dx
            
            # update y axis
            omega[pose_row_col+1][pose_row_col+1] += measurement_strength
            omega[ldmk_row_col+1][ldmk_row_col+1] += measurement_strength
            omega[pose_row_col+1][ldmk_row_col+1] -= measurement_strength
            omega[ldmk_row_col+1][pose_row_col+1] -= measurement_strength
            
            xi[pose_row_col+1][0] -= measurement_strength * ldmk_dy
            xi[ldmk_row_col+1][0] += measurement_strength * ldmk_dy
            
            
    
        pose_pre = (time_step) * 2
        pose_cur = (time_step + 1) * 2  # this is a repeated definition only for easy to read purpose        
        
        pose_dx = motion[0]
        pose_dy = motion[1]
        
        # update x axis
        omega[pose_pre][pose_pre] += motion_strength
        omega[pose_cur][pose_cur] += motion_strength
        omega[pose_pre][pose_cur] -= motion_strength
        omega[pose_cur][pose_pre] -= motion_strength

        xi[pose_pre][0] -= motion_strength * pose_dx
        xi[pose_cur][0] += motion_strength * pose_dx

        # update y axis
        omega[pose_pre+1][pose_pre+1] += motion_strength
        omega[pose_cur+1][pose_cur+1] += motion_strength
        omega[pose_pre+1][pose_cur+1] -= motion_strength
        omega[pose_cur+1][pose_pre+1] -= motion_strength

        xi[pose_pre+1][0] -= motion_strength * pose_dy
        xi[pose_cur+1][0] += motion_strength * pose_dy
        
    
    ## TODO: After iterating through all the data
    ## Compute the best estimate of poses and landmark positions
    ## using the formula, omega_inverse * Xi
    omega_inv = np.linalg.inv(np.matrix(omega))
    mu = omega_inv*xi   
    
    return mu, omega, xi # return `mu`


# a helper function that creates a list of poses and of landmarks for ease of printing
# this only works for the suggested constraint architecture of interlaced x,y poses
 # a helper function that creates a list of poses and of landmarks for ease of printing
# this only works for the suggested constraint architecture of interlaced x,y poses
# 
# 
def get_poses_landmarks(mu, N,num_landmarks):
# create a list of poses
    poses = []
    for i in range(N):
        poses.append((mu[2*i].item(), mu[2*i+1].item()))

    # create a list of landmarks
    landmarks = []
    for i in range(num_landmarks):
        landmarks.append((mu[2*(N+i)].item(), mu[2*(N+i)+1].item()))

    # return completed lists
    return poses, landmarks

def print_all(poses, landmarks):
    print('\n')
    print('Estimated Poses:')
    for i in range(len(poses)):
        print('['+', '.join('%.3f'%p for p in poses[i])+']')
    print('\n')
    print('Estimated Landmarks:')
    for i in range(len(landmarks)):
        print('['+', '.join('%.3f'%l for l in landmarks[i])+']')

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