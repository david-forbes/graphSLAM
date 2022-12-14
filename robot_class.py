from math import *
import random
import math
from helpers import *
from circle import *

### ------------------------------------- ###
# Below, is the robot class
#
# This robot lives in 2D, x-y space, and its motion is
# pointed in a random direction, initially.
# It moves in a straight line until it comes close to a wall 
# at which point it stops.
#
# For measurements, it  senses the x- and y-distance
# to landmarks. This is different from range and bearing as
# commonly studied in the literature, but this makes it much
# easier to implement the essentials of SLAM without
# cluttered math.
#
class robot:
    
    # --------
    # init:
    #   creates a robot with the specified parameters and initializes
    #   the location (self.x, self.y) to the center of the world
    #
    def __init__(self, world_size = 100.0, measurement_range = 30.0,
                 motion_noise = 1.0, measurement_noise = 1.0):
        self.measurement_noise = 0.0
        self.world_size = world_size
        self.measurement_range = measurement_range
        self.x = world_size / 2.0
        self.y = (world_size / 2.0)#-self.world_size*0.4*0.5*0.85
        #self.x = world_size/2
        #self.y = world_size/2
        self.motion_noise = motion_noise
        self.measurement_noise = measurement_noise
        self.landmarks = []
        self.data = []
        self.num_landmarks = 0
    
    
    # returns a positive, random float
    def rand(self):
        return random.random() * 2.0 - 1.0
    
    
    # --------
    # move: attempts to move robot by dx, dy. If outside world
    #       boundary, then the move does nothing and instead returns failure
    #
    def move(self, dx, dy):
        
        x = self.x + dx + self.rand() * self.motion_noise
        y = self.y + dy + self.rand() * self.motion_noise
        
        if x < 0.0 or x > self.world_size or y < 0.0 or y > self.world_size:
            return False
        else:
            
            self.x = x
            self.y = y
            print(x,y)
            self.data.append([self.sense(), [dx, dy]])
            







            return True


    # --------
    # sense: returns x- and y- distances to landmarks within visibility range
    #        because not all landmarks may be in this range, the list of measurements
    #        is of variable length. Set measurement_range to -1 if you want all
    #        landmarks to be visible at all times
    #
    
    ## TODO: paste your complete the sense function, here
    ## make sure the indentation of the code is correct
    def sense(self):
        ''' This function does not take in any parameters, instead it references internal variables
            (such as self.landamrks) to measure the distance between the robot and any landmarks
            that the robot can see (that are within its measurement range).
            This function returns a list of landmark indices, and the measured distances (dx, dy)
            between the robot's position and said landmarks.
            This function should account for measurement_noise and measurement_range.
            One item in the returned list should be in the form: [landmark_index, dx, dy].
            '''
           
        measurements = []
        
        ## TODO: iterate through all of the landmarks in a world
        
        ## TODO: For each landmark
        ## 1. compute dx and dy, the distances between the robot and the landmark
        ## 2. account for measurement noise by *adding* a noise component to dx and dy
        ##    - The noise component should be a random value between [-1.0, 1.0)*measurement_noise
        ##    - Feel free to use the function self.rand() to help calculate this noise component
        ## 3. If either of the distances, dx or dy, fall outside of the internal var, measurement_range
        ##    then we cannot record them; if they do fall in the range, then add them to the measurements list
        ##    as list.append([index, dx, dy]), this format is important for data creation done later
        
        ## TODO: return the final, complete list of measurements
        
        for idx in range(self.num_landmarks):
            ldmk = self.landmarks[idx]
            ldmk_x = ldmk[0]
            ldmk_y = ldmk[1]
            
            dx = ldmk_x - self.x
            dy = ldmk_y - self.y
            
            dx += self.rand() * self.measurement_noise
            dy += self.rand() * self.measurement_noise
            
            if (abs(dx)<=self.measurement_range) and (abs(dy)<=self.measurement_range):
                measurements.append([idx, dx, dy])            
                
        return measurements


    # --------
    # make_landmarks:
    # make random landmarks located in the world
    #
    
        
    def make_landmarks(self, num_landmarks):
        self.landmarks = []
        num_landmarks_placed =0
        '''
        for i in range(num_landmarks):
            self.landmarks.append([round(random.random() * self.world_size),
                                   round(random.random() * self.world_size)])
        '''
        circle_radius = self.world_size*0.4*0.5


        for x in range(round(self.world_size)):
            if num_landmarks_placed<num_landmarks:
                
                
                y_upper,y_lower = circle_gen(x,self.world_size/2,circle_radius)
                if y_upper!=0:

                
                    if num_landmarks_placed<num_landmarks:
                        self.landmarks.append([x,y_upper])
                        num_landmarks_placed+=1
                    if y_upper!=y_lower and num_landmarks_placed<num_landmarks:
                        self.landmarks.append([x,y_lower])
                        num_landmarks_placed+=1

        for x in range(round(self.world_size)):
            if num_landmarks_placed<num_landmarks:
                
                
                y_upper,y_lower = circle_gen(x,self.world_size/2,(circle_radius*0.7))
                if y_upper!=0:

                
                    if num_landmarks_placed<num_landmarks:
                        self.landmarks.append([x,y_upper])
                        num_landmarks_placed+=1
                    if y_upper!=y_lower and num_landmarks_placed<num_landmarks:
                        self.landmarks.append([x,y_lower])
                        num_landmarks_placed+=1


        

        self.num_landmarks = num_landmarks_placed

    def make_time_step(self):
        dx,dy = 3*((random.random()*2)-1),10*((random.random()*2)-1)
        while not self.move(dx, dy):
                # if we'd be leaving the robot world, pick instead a new direction
                
                dx,dy = 3*((random.random()*2)-1),10*((random.random()*2)-1)
        return self.data
    def make_time_step_circle(self):
        dx = 1
        dy = circle_dy(dx, self.world_size/2)
        while not self.move(dx, dy):
                # if we'd be leaving the robot world, pick instead a new direction
                
                dx = 1
                dy = circle_dy(dx, self.world_size/2)
        return self.data
    # called when print(robot) is called; prints the robot's location
    def __repr__(self):
        return 'Robot: [x=%.5f y=%.5f]'  % (self.x, self.y)

    def get_landmarks(self):
        return self.landmarks
    def get_data(self):
        return self.data



####### END robot class #######