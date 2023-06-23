import matplotlib.pyplot as plt
import random
import numpy as np
import math

import statistics # added for the mean computation
from collections import defaultdict # added to compare elements of the list
from itertools import tee # to allow pairwise comparisons


class Target:
    def __init__(self,name,x,y): # no indetermination in the target's position
        self.name = name
        self.x = x
        self.y = y

class Obstacle: # Just a point for now
    _registry = []

    def __init__(self,name,x,y):
        self._registry.append(self)
        self.name = name
        self.x = x
        self.y = y

class Robotx(object):
    _registry = []

    # def __init__(self, name, alphax, betax, alphay, betay, gamma, delta):
    def __init__(self, name, alphax, betax, alphay, betay, gamma, delta, position):
        self._registry.append(self)
        self.name = name
        self.alphax = alphax
        self.betax = betax
        self.alphay = alphay
        self.betay = betay
        self.gamma = gamma
        self.delta = delta
        self.position = position # new -- I need it for sound
        
    # position is needed if we wanna add sonification

def reward(T, betax, betay):
    return 1 - ((T.x - betax)**2 + (T.y - betay)**2)**0.5
    # the closer the target, the less the distance, the higher the reward

# Dispaly the the robots,target and obstacles in a plot graph
def plot_scatterplot():
    for i in Robotx._registry:
        plt.scatter(i.betax, i.betay, s = 400, marker = 'o', color = 'black')
    
    plt.scatter(T.x, T.y, s = 400, marker = '*', color = 'turquoise')

    for o in Obstacle._registry:
        plt.scatter(o.x, o.y, s = 400, marker = 'p', color = 'red')

    plt.axis([0, 1, 0, 1])

    plt.show() 

# Change the position and reward of the robot
def reshuffling_position (r):
    r.alphax = round(random.uniform(0, 1), 2)
    r.betax = round(1 - r.alphax, 2)
    r.alphay = round(random.uniform(0, 1), 2)
    r.betay = round(1 - r.alphay, 2)
    r.delta = round(reward(T, r.betax, r.betay),2)
    r.gamma = round(1 - reward(T, r.betax, r.betay),2)

# Check if the robot is too near an obstacle
def obstacle_check(r,obs):
    diff_threshold = 0.2
    obs_colision = False
    for o in obs._registry:
        # Prevent the robots to spawn in a square define by diff_threshold length around the target
        if ((math.fabs(r.betax - o.x) <= diff_threshold) and (math.fabs(r.betay - o.y) <= diff_threshold)): # to avoid the obstacle (if at least one is True)
            obs_colision = True

    return obs_colision



num_of_robots = 5
threshold_delta = 0.8
num_of_obstacles = 3
num_tries = 50

# --- Define Target ---
T = Target("T", 0.7, 0.7)

num = 0
while(num < num_tries):


    # --- Define Obstacle ---
    dict_obs = {}
    dict_obs.clear()
    for obs in range (num_of_obstacles):
        shuffle_obstacle = True
        while shuffle_obstacle:
            shuffle_obstacle = False
            x_obs = round(random.uniform(0, 1), 2)
            y_obs = round(random.uniform(0, 1), 2)
            # Prevent the obstacles to spawn in a square of 0.2 length around the target
            if math.fabs(x_obs - T.x) <= 0.2 and math.fabs(y_obs - T.y) <= 0.2 :
                shuffle_obstacle = True
            else:
                dict_obs["O{0}".format(obs)] = Obstacle("O{0}".format(obs), x_obs, y_obs)


    # --- Creation of the robots ---
    dict_rob = {}
    dict_rob.clear()
    for tag in range(1,num_of_robots+1):
        x_pos = round(random.uniform(0, 1), 2)
        y_pos = round(random.uniform(0, 1), 2)
        x_pos_inv = round(1-x_pos,2)
        y_pos_inv = round(1-y_pos,2)

        dict_rob["R{0}".format(tag)] = Robotx("R{0}".format(tag), x_pos, x_pos_inv, y_pos, y_pos_inv, 1 - reward(T, x_pos_inv, y_pos_inv), reward(T, x_pos_inv, y_pos_inv), tag)


    # --- Shuffling position ---
    shuffling = True
    iter = 0
    while (shuffling):
        iter += 1    

        # --- Reshuffle ---
        for r in Robotx._registry:
            # If the robot is not near of the Target and not in an Obstacle, then reshuffle
            if (r.delta <= threshold_delta):
                flag = True
                # Until the robot are not in an obstacle, reshuffle
                while flag:
                    flag = False
                    reshuffling_position(r)
                    flag = obstacle_check(r,Obstacle)

            
        result = all(i.delta >= threshold_delta for i in Robotx._registry)

        # Untile all robot are not near the Target, reshuffle again
        shuffling = not result   

    num += 1
    print(iter)

    # Delet the register of the robots and obstacles
    Robotx._registry.clear()
    Obstacle._registry.clear()

