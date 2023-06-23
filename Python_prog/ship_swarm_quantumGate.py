"""ship_swarm_quantumGate controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Supervisor

import math
import string

# create the Robot instance.
robot = Supervisor()
"""
robot_node = robot.getFromDef('SHIP')
translation_field = robot_node.getField('translation')
print(type(translation_field))
"""
# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())


# --- Setup & Initialisation ---
# Actuator
motor = []
motor_name = ["propeller_motor_right","propeller_motor_left"]
for idx in range(len(motor_name)):
    motor.append(robot.getDevice(motor_name[idx]))
    motor[idx].setPosition(float('inf')) 
    motor[idx].setVelocity(0.0)

# Ray sensor
sensor = []
sensor_name = ["area_detector","left_sensor","right_sensor"]
for idx in range(len(sensor_name)):
    sensor.append(robot.getDevice(sensor_name[idx]))
    sensor[idx].enable(timestep)

# Navigation tools
gps_ship = robot.getDevice("gps")
gyro_ship = robot.getDevice("gyro")
compass_ship = robot.getDevice("compass")

gps_ship.enable(timestep)
gyro_ship.enable(timestep)
compass_ship.enable(timestep)


# Variables
roataion_speed = 6.28
velocity = roataion_speed*0.6 # Max speed

# Create a txt file where state & position of robot are register
with open(robot.getName()+"_data.txt", "w") as fileData:
    fileData.write("Creation of the file of '" + robot.getName() +"'\n")

def bearing_position():
    angle_compass = round(math.atan2(compass_ship.getValues()[0],compass_ship.getValues()[1]),2)
    return angle_compass

def avoidance_state(velocity,ds_value_l,ds_value_r):
    if ds_value_l < 950.0:
        motor[0].setVelocity(velocity*0.2)
        motor[1].setVelocity(0.0)
    elif ds_value_r < 950.0:
        motor[0].setVelocity(0.0)
        motor[1].setVelocity(velocity*0.2)
        
def data_package (new_state):
    sep = ";"
    robot_time = str(round(robot.getTime(),4)) + sep
    actual_state = new_state + sep
    actual_position = str(round(gps_ship.getValues()[0],2)) + sep + str(round(gps_ship.getValues()[1],2)) + sep
    actual_rotation = str(-round(bearing_position(),2)) + sep
    actual_speed = str(round(gps_ship.getSpeed(),2))
    data_send = robot_time + actual_state + actual_position + actual_rotation  + actual_speed + "\n"

    with open(robot.getName()+"_data.txt", "a") as fileData:
        fileData.writelines(data_send)
     
        
def ratio_angle(x_pos,y_pos):
 # Give the rotation of the robot frame from the world
    angle_compass = -round(bearing_position(),2)
            
    # Calculate the relative vector from the robot frame
    def_x,def_y = -(round(gps_ship.getValues()[0],2) - x_pos),-(round(gps_ship.getValues()[1],2) - y_pos)
    
    def_x_r,def_y_r = round((math.cos(angle_compass)*def_x - math.sin(angle_compass)*def_y),2),round((math.sin(angle_compass)*def_x + math.cos(angle_compass)*def_y),2)

    # Calculte the angle between the vector and the front axis robot
    angle_converging = round(math.atan2(def_y_r,def_x_r)*(180.0/math.pi)-90.0,2)
    if angle_converging < -180.0:
        angle_converging = round(angle_converging + 360.0,2)
            
    # Give a positive or negative ratio of the angle (value near 0 mean the robot is front of the objective)
    ratio = round(angle_converging/180.0,2)

    return ratio


def main ():




    global velocity
    force_data = True
    state = "INITIALISATION"
    new_state = state
    delay = 0
    trigger_delay = 50


    while robot.step(timestep) != -1:
        delay += 1
        
        # Detection change state
        if (new_state != state):
            state = new_state
            print(robot.getName(),state)
        
        # Change to DATA state after the delay oe if forced
        if (state != "DATA" and delay > trigger_delay) or force_data == True:
            force_data = False
            delay = 0
            state = "DATA"
            
        # Wait for 
        if state == "INITIALISATION":

            print(robot.getName(),"initilaisation...")

            # Wait for the robot to fall
            while (delay < 20 and robot.step(timestep) != -1):
                delay += 1
            print(robot.getName(),"running")
            new_state = "SEARCHING"
            
        # Avoid the obstacle   
        if state == "AVOIDANCE":
                
                avoidance_state(velocity,round(sensor[1].getValue(),1),round(sensor[2].getValue(),1))
                if  sensor[1].getValue() > 950.0 and  sensor[2].getValue() > 950.0:
                    new_state = "SEARCHING"            
            
        if state == "SEARCHING":

            motor[0].setVelocity(velocity)
            motor[1].setVelocity(velocity)
            
            if  sensor[1].getValue() < 950.0 or sensor[2].getValue() < 950.0:
                new_state = "AVOIDANCE"
             
        if state == "DATA":
            data_package(new_state)
            
    return 0
    
if __name__ == "__main__":
    print(robot.getName(),"start program")
    main()
  