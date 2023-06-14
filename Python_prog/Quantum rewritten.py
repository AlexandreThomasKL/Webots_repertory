from qiskit import QuantumRegister, ClassicalRegister, QuantumCircuit, Aer, transpile
from qiskit.visualization import plot_histogram
#from qiskit import *
from operator import attrgetter,itemgetter

import matplotlib.pyplot as plt
import heapq
import random
import numpy as np

import statistics # added for the mean computation
from collections import defaultdict # added to compare elements of the list
from itertools import tee # to allow pairwise comparisons
# from scipy.spatial.distance import cosine # to compute cosine distance

class Target:
    def __init__(self,name,x,y): # no indetermination in the target's position
        self.name = name
        self.x = x
        self.y = y

class Obstacle: # Just a point for now
    def __init__(self,name,x,y):
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
    plt.scatter(O.x, O.y, s = 400, marker = 'p', color = 'red')

    plt.axis([0, 1, 0, 1])

    plt.show() 

def reshuffling_position (r):
    r.alphax = round(random.uniform(0, 0.9), 2)
    r.betax = round(1 - r.alphax, 2)
    r.alphay = round(random.uniform(0, 0.9), 2)
    r.betay = round(1 - r.alphay, 2)

def print_formatted_vector(*args):
    for vector in args:
        print("[" + "".join(f"{val:.2f} " for val in vector).strip() + "]")

def eval_outcome(most_prob_dict, n_outcome):
    mapped_weights0 = list(map(lambda res: int(res[n_outcome*2])*most_prob_dict[res], most_prob_dict))
    return sum(mapped_weights0)/sum(most_prob_dict.values())

# --- Define Target ---
T = Target("T", 0.8, 0.7)

# --- Define Obstacle ---
O = Obstacle("Oo", 0.8, 0.2)

num_of_robots = 4

# --- Creation of the robots ---
dict_rob = {}
for tag in range(1,num_of_robots+1):
    x_pos = round(random.uniform(0, 0.9), 2)
    y_pos = round(random.uniform(0, 0.9), 2)
    x_pos_inv = round(1-x_pos,2)
    y_pos_inv = round(1-y_pos,2)

    dict_rob["R{0}".format(tag)] = Robotx("R{0}".format(tag), x_pos, x_pos_inv, y_pos, y_pos_inv, 1 - reward(T, x_pos_inv, y_pos_inv), reward(T, x_pos_inv, y_pos_inv), tag)
    R = dict_rob["R{0}".format(tag)]
    MessageR = [R.alphax, R.betax, R.alphay, R.betay, R.gamma, R.delta, R.position]


    with open('R{0}_values.txt'.format(tag), 'w') as dataFile:
        for d in MessageR:
            dataFile.write(f"{d}\n")

print("--- Initial position ---")
for k in Robotx._registry:
    print(f"{k.name} {k.betax:.2f} {k.betay:.2f} {k.gamma:.2f} {k.position}")

plot_scatterplot()

for r in Robotx._registry:
    if (r.delta < 0.5):
        print(f"{r.name} {r.delta:.2f} achtung!") # and start from this point to build the reshuffle section

# I'm adding this one as the only non-quantum thing:

result = all(i.delta < 0.8 for i in Robotx._registry)
print("Do all the robots have a reward lower than 0.8? : " + str(result))

# --- Reshuffle ---
for r in Robotx._registry:
    if (r.delta < 0.5):
        print(r.name,"Reshuffle...")
        flag = True
        while flag:
            flag = False
            reshuffling_position(r)
            if (r.betax - O.x <= 0.2 and r.betay - O.y <= 0.2 <= 0.2): # to avoid the obstacle
                flag = True

                # Rewrite the new position
                MessageR = [r.alphax, r.betax, r.alphay, r.betay, r.gamma, r.delta, r.position]
                with open(f"{r.name}_values.txt", 'w') as dataFile:
                    for d in MessageR:
                        dataFile.write(f"{d}\n")    

print("--- Position after reshuffle (low delta) ---")
for k in Robotx._registry:
    print(f"{k.name} {k.betax:.2f} {k.betay:.2f} {k.gamma:.2f} {k.position}")

plot_scatterplot()

# --- Quantum circuit construction ---
print("--- Quantum cicuit ---")
q = QuantumRegister(5, 'q') # qubits # changed to 9, formerly 15
m2 = ClassicalRegister(1, 'c1') # classical bits (separated is better)
m3 = ClassicalRegister(1, 'c2')
m4 = ClassicalRegister(1, 'c3')

qc3 = QuantumCircuit(q, m2, m3, m4) # to reach the target
qc4 = QuantumCircuit(q, m2, m3, m4) # to get back to the nest


closest_robot = max(Robotx._registry, key=attrgetter('delta'))
print(f"Closest robot to the target: {closest_robot.name} {closest_robot.betax:.2f} {closest_robot.betay:.2f} {closest_robot.delta:.2f}")


# and then it enters the gate
vector0 = [closest_robot.alphax, closest_robot.betax]
vector1 = [closest_robot.alphay, closest_robot.betay]
vector3 = [closest_robot.gamma, closest_robot.delta]

normalized_v0 = vector0/np.linalg.norm(vector0)
normalized_v1 = vector1/np.linalg.norm(vector1)
normalized_v3 = vector3/np.linalg.norm(vector3)

#print_formatted_vector(vector0, vector1, vector3)
#print_formatted_vector(normalized_v0, normalized_v1, normalized_v3)


# direct initialization with amplitudes vector
qc3.initialize(normalized_v0, q[0])
qc3.initialize(normalized_v1, q[1])
qc3.initialize(normalized_v3, q[2])

qc3.barrier(q)
qc3.ccx(q[0],q[1],q[3])
qc3.ccx(q[0],q[1],q[4])

qc3.reset(q[3])
qc3.reset(q[4])

qc3.ccx(q[0],q[2],q[3]) 
qc3.ccx(q[1],q[2],q[4])

qc3.x(q[2])

qc3.ch(q[2],q[3])
qc3.ch(q[2],q[4])

qc3.x(q[2])

qc3.barrier(q)

# perform measurements and store them in classical bits

qc3.measure(q[2],m2[0])
qc3.measure(q[3],m3[0])
qc3.measure(q[4],m4[0])

# visualization of the ciruit

# qc3.draw(fold=-1, output="mpl")
# plt.show();

print(qc3)



aer_sim = Aer.get_backend("aer_simulator")
transpiled_qc = transpile(qc3, aer_sim)
result = aer_sim.run(transpiled_qc).result()

counts = result.get_counts()
print("counts: ", counts)

plot_histogram(result.get_counts(),filename="histogram.png")



num_most_prob_states = 4

# https://docs.python.org/3/library/heapq.html: 
#
# heapq.nlargest(n, iterable, key=None) returns a list with the n largest element of iterable
#
most_prob_dict = dict(heapq.nlargest(num_most_prob_states, counts.items(), key=itemgetter(1)))
print(f"{num_most_prob_states} most probable states: {most_prob_dict}")

outcome0, outcome1 = eval_outcome(most_prob_dict, 0), eval_outcome(most_prob_dict, 1)

print(f"outcome0: {outcome0:.2f}\noutcome1: {outcome1:.2f}")


for r in Robotx._registry:
    if (r.delta != closest_robot.delta or all(r.delta == j.delta for j in Robotx._registry)): # excluding the robot that entered the gate
        print(r.name,"Reshuffle...")
        reshuffling_position(r)

print("--- Position after reshuffle (closest robot)---")
for k in Robotx._registry:
    print(f"{k.name} {k.betax:.2f} {k.betay:.2f} {k.gamma:.2f} {k.position}")

plot_scatterplot()
plot_scatterplot()

