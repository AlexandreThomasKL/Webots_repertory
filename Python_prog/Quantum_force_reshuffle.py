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
        if ((math.fabs(r.betax - o.x) <= diff_threshold) and (math.fabs(r.betay - o.y) <= diff_threshold)): # to avoid the obstacle (if at least one is True)
            obs_colision = True

    return obs_colision


# --- Define Target ---
T = Target("T", 0.7, 0.7)

# --- Define Obstacle ---

for obs in range (3):
    shuffle_obstacle = True
    while shuffle_obstacle:
        shuffle_obstacle = False
        x_obs = round(random.uniform(0, 1), 2)
        y_obs = round(random.uniform(0, 1), 2)
        if math.fabs(x_obs - T.x) <= 0.1 or math.fabs(y_obs - T.y) <= 0.1 :
            shuffle_obstacle = True
        else:
            O = Obstacle("Oo", x_obs, y_obs)

"""
O1 = Obstacle("Oo", 0.8, 0.2)
O2 = Obstacle("Oo", 0.7, 0.6)
O3 = Obstacle("Oo", 0.7, 0.5)
O4 = Obstacle("Oo", 0.3, 0.4)
O5 = Obstacle("Oo", 0.2, 0.5)
"""
num_of_robots = 5
threshold_delta = 0.5


# --- Creation of the robots ---
dict_rob = {}
dict_rob.clear()

for tag in range(1,num_of_robots+1):
    x_pos = round(random.uniform(0, 1), 2)
    y_pos = round(random.uniform(0, 1), 2)
    x_pos_inv = round(1-x_pos,2)
    y_pos_inv = round(1-y_pos,2)

    dict_rob["R{0}".format(tag)] = Robotx("R{0}".format(tag), x_pos, x_pos_inv, y_pos, y_pos_inv, 1 - reward(T, x_pos_inv, y_pos_inv), reward(T, x_pos_inv, y_pos_inv), tag)
    R = dict_rob["R{0}".format(tag)]
    MessageR = [R.alphax, R.betax, R.alphay, R.betay, R.gamma, R.delta, R.position]


    with open('R{0}_values.txt'.format(tag), 'w') as dataFile:
        for d in MessageR:
            dataFile.write(f"{d}\n")
"""
print("--- Initial position ---")
print("name/pos_x/pos_y/delta/gamma/tag")
for k in Robotx._registry:
    print(f"{k.name} {k.betax:.2f} {k.betay:.2f} {k.delta:.2f} {k.gamma:.2f} {k.position}")
"""

#plot_scatterplot()


shuffling = True
iter = 0
while (shuffling):
    
    result = all(i.delta >= threshold_delta for i in Robotx._registry)
    #print("Do all the robots have a reward greater than 0.8? : " + str(result))

    # Untile all robot are not near the Target, reshuffle again
    shuffling = not result       

    # --- Reshuffle ---
    for r in Robotx._registry:
        # If the robot is not near of the Target and not in an Obstacle, then reshuffle
        if (r.delta <= threshold_delta):
            #print(r.name,"Reshuffle...")
            flag = True
            # Until the robot are not in an obstacle, reshuffle
            while flag:
                flag = False

                reshuffling_position(r)
                flag = obstacle_check(r,Obstacle)
                """
                if flag == False:
                    # Rewrite the final new position
                    MessageR = [r.alphax, r.betax, r.alphay, r.betay, r.gamma, r.delta, r.position]
                    with open(f"{r.name}_values_forced.txt", 'w') as dataFile:
                        for d in MessageR:
                            dataFile.write(f"{d}\n")      
    
    print("--- Position after reshuffle (low delta) ---")
    for k in Robotx._registry:
        print(f"{k.name} {k.betax:.2f} {k.betay:.2f} {k.delta:.2f} {k.gamma:.2f} {k.position}")
    """
    iter += 1
    #plot_scatterplot()

plot_scatterplot()
print("number of iteration :",iter)