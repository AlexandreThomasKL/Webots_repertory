"""ship_swarm_quantumGate controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Supervisor

import math
import string
import time
import random



# create the Robot instance.
robot = Supervisor()

robot_node = robot.getFromDef('ship')
translation_field = robot_node.getField('translation')

area_node = robot.getFromDef('area_boundary')
floorSize_field_area = area_node.getField('floorSize')

target_node = robot.getFromDef('Target')
translation_field_target = target_node.getField('translation')


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



# Create a text file where state & position of robot are register
with open(robot.getName()+"_data.txt", "w") as fileData:
    fileData.write("Creation of the file of '" + robot.getName() +"'\n")

def target_estimation():
    pos_target = translation_field_target.getSFVec3f()
    pos_robot = translation_field.getSFVec3f()
    
    eucl_dist = math.sqrt( (pos_target[0] - pos_robot[0])**2 
    + (pos_target[1] - pos_robot[1])**2)
    
    return eucl_dist

def FileCheck(path_file):
    try:
        open(path_file,"r")
        return True
    except:
        print ("Error : file not found or exist")
        return False   
        
def package_data():    
    sep = ";"
    rob_name = robot.getName() + sep
    
    rob_pos_x = round(translation_field.getSFVec3f()[0],3)
    
    rob_pos_x = str(round(translation_field.getSFVec3f()[0],2)) + sep
    rob_pos_y = str(round(translation_field.getSFVec3f()[1],2)) + sep
    
    reward_estim = str(round(target_estimation(),3)) + sep
    
    msg_pack = rob_name + reward_estim + rob_pos_x + rob_pos_y + "\n"
    
    with open(robot.getName()+"_data.txt", "a") as fileData:
        fileData.writelines( msg_pack)
        
def text_file_assigned():
    file_order_name = "position_indicator"
    extension_end = ".txt" 
    number_assigned = robot.getName()[-2]
    
    if number_assigned.isnumeric():
        file_assignation = file_order_name + "_" + number_assigned + extension_end
    else:
        file_assignation = file_order_name + "_0" + extension_end
 
        
    return file_assignation

def main ():

    delay = 0
    idx = 0
    count_line = 0
    
    x,y = random.uniform(0.5, 14.5),random.uniform(0.5, 14.5)
    new_value=[x,y,0.672]
    translation_field.setSFVec3f(new_value)
    
    position_file = text_file_assigned()
    print(robot.getName(), "assigned to the text file",position_file)
    package_data()
    
    file_exist = False
    while file_exist == False and robot.step(timestep) != -1:
        fileVar = "static_variables.txt"
        if FileCheck(fileVar) :
            print("Storing variable file exsit, writting...")
            with open(fileVar, "a+") as fileDataVar:
                msg_package = str(round(floorSize_field_area.getSFVec3f()[0],1)) + "\n"
                fileDataVar.write(msg_package)
                
            file_exist = True
        else:
            print("Storing variable file exsit doesn't existe...")
    
  
    while robot.step(timestep) != -1:
  
        delay +=1

        # After some time, read the next line and go to the position indicate
        if delay > 50:
            delay = 0
            
            new_count_line = count_line
            with open(position_file, "r") as fileData:
                lines_pos = fileData.readlines()
                count_line = len(lines_pos)
                if new_count_line != count_line:
                    print("new line added")
                fileData.close()
            
            if not idx >= len(lines_pos):
                line_strip = lines_pos[idx].strip()
                pos_coord = [float(coord) for coord in line_strip.split(';')]
                print(pos_coord)
                new_value = [pos_coord[0],pos_coord[1],0.672]

                translation_field.setSFVec3f(new_value)
                package_data()
                idx +=1
        
    return 0

    
if __name__ == "__main__":
    print(robot.getName(),"start program")
    main()
  