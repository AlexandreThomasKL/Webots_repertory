"""ship_swarm controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot
from controller import Keyboard
from controller import Emitter
from controller import Receiver
from controller import Camera
import math
import string
"""
import time
time.sleep(2.4)
"""

# create the Robot instance.
robot = Robot()

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

# Communication tools
rx = robot.getDevice("receiver")
rx.enable(timestep)
rx.setChannel(5)
tx = robot.getDevice("emitter")
tx.setChannel(5)

# Variables
roataion_speed = 6.28
velocity = roataion_speed*0.6 # Max speed

areaHasBeFound = False # The area has be found by a robot
spotHasBeFound = False # The objective has be found by a robot
onTheSpot = False # robot on the objective

# Create a txt file where state & position of robot are register
with open(robot.getName()+"_data.txt", "w") as fileData:
    fileData.write("Creation of the file of '" + robot.getName() +"'\n")

def bearing_position():
    angle_compass = round(math.atan2(compass_ship.getValues()[0],compass_ship.getValues()[1]),2)
    return angle_compass
 
def emission_position(priority):
    pos_x = round(gps_ship.getValues()[0],2)
    pos_y = round(gps_ship.getValues()[1],2)
    message = str(priority) +" "+ str(pos_x) +" "+ str(pos_y)
    print("Coordinates area :",message)
    tx.send(message)

def avoidance_state(velocity,ds_value_l,ds_value_r):
    if ds_value_l < 950.0:
        motor[0].setVelocity(velocity*0.2)
        motor[1].setVelocity(0.0)
    elif ds_value_r < 950.0:
        motor[0].setVelocity(0.0)
        motor[1].setVelocity(velocity*0.2)
        
def data_package (new_state):
    sep = ";"
    robot_time = str(round(robot.getTime(),4))
    actual_state = new_state
    actual_position = str(round(gps_ship.getValues()[0],2)) + sep + str(round(gps_ship.getValues()[1],2))
    actual_rotation = str(-round(bearing_position(),2))
    actual_speed = str(round(gps_ship.getSpeed(),2))
    objective_achived = str(onTheSpot)
    data_send = robot_time + sep + actual_state + sep + objective_achived +  sep + actual_position + sep + actual_rotation + sep  + actual_speed + "\n"
    with open(robot.getName()+"_data.txt", "a") as fileData:
        fileData.writelines(data_send)
     
        
def ratio_angle(x_pos,y_pos):
 # Give the rotation of the robot frame from the world
    angle_compass = -round(bearing_position(),2)
    #print("Compass angle :" ,angle_compass)
            
    # Calculate the relative vector from the robot frame
    def_x = -(round(gps_ship.getValues()[0],2) - x_pos)
    def_y = -(round(gps_ship.getValues()[1],2) - y_pos)
    def_x_r = round((math.cos(angle_compass)*def_x - math.sin(angle_compass)*def_y),2)
    def_y_r = round((math.sin(angle_compass)*def_x + math.cos(angle_compass)*def_y),2)
            
    # Calculte the angle between the vector and the front axis robot
    angle_converging = round(math.atan2(def_y_r,def_x_r)*(180.0/math.pi)-90.0,2)
    if angle_converging < -180.0:
        angle_converging = round(angle_converging + 360.0,2)
 
    # Give a positive or negative ratio of the angle (value near 0 mean the robot is front of the objective)
    ratio = round(angle_converging/180.0,2)


    return ratio


def main ():
    global areaHasBeFound,spotHasBeFound,onTheSpot, velocity
    force_data = True
    state = "INITIALISATION"
    new_state = state
    roadToTheObjective = False
    
    stabel_balance = 0
    priority_level = 0
    actual_priority = 0
    priority = 0
    delay = 0
    delay_reach = delay
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
            
            zone_value = round(sensor[0].getValue()/1000.0,2)
            
            new_state = "SEARCHING"
            
            
        if state == "SEARCHING":
            motor[0].setVelocity(velocity)
            motor[1].setVelocity(velocity)
            
            if  sensor[1].getValue() < 950.0 or sensor[2].getValue() < 950.0:
                new_state = "AVOIDANCE"
            
            zone_value_old = zone_value
            zone_value = round(sensor[0].getValue()/1000.0,2)
            diff_zone = round(math.fabs(zone_value - zone_value_old),3)
            if diff_zone > 0.2 :
                new_state = "FOUND"
            
            if rx.getQueueLength() > 0:
                new_state = "RECEVING"

                 
# Avoid the obstacle   
        if state == "AVOIDANCE":
                avoidance_state(velocity,round(sensor[1].getValue(),1),round(sensor[2].getValue(),1))
                if  sensor[1].getValue() > 950.0 and  sensor[2].getValue() > 950.0:
                    new_state = "SEARCHING"
                    
# Receive the data from the broadcast            
        if state == "RECEVING":
            message = rx.getString()
            list_message = list(map(float,message.split(" ")))
            pos_coord = list_message[1:3]
            priority = list_message[0]
            rx.nextPacket()
            data_package(state)
            new_state = "STABILISING"
            
# Calculate and orient himself to the objectif coordinates      
        if state == "CONVERGING": 
        
            if rx.getQueueLength() > 0:
                message = rx.getString()
                list_message = list(map(float,message.split(" ")))
                pos_coord = list_message[1:3]
                new_priority = list_message[0]
                rx.nextPacket()
                if new_priority > priority:
                    print("higher priority taken...")
                    pos_coord = list_message[1:3]
            
            # Turn in function of ratio value and sign, use ratio & PID corrector to ajust the velocity
            ratio = ratio_angle(pos_coord[0],pos_coord[1])
            
            Kp_corr = 0.8 # proportional corrector : value < 1.0
            Ki_coor = 0.9 # Integral corrector
            Kd_coor = 0.1 # Derivate corrector
            # Simplift FTBF : PID / (1+ PID)
            PID_corr = (Kp_corr + Ki_coor*timestep/1000 + Kd_coor/(timestep/1000)) / (1+ (Kp_corr + Ki_coor*timestep/1000) + Kd_coor/(timestep/1000))
            # s(t)=(PID(t) / (1+ PID(t))) * e(t)
            velocity_corr =round(PID_corr*velocity*math.fabs(ratio),2)
            
            
            #print("Velocity coorection : ",velocity_corr)
            if ratio < 0.0:
                motor[0].setVelocity(-velocity_corr)
                motor[1].setVelocity(velocity_corr)
            if ratio > 0.0:
                motor[0].setVelocity(velocity_corr)
                motor[1].setVelocity(-velocity_corr)
   
            if math.fabs(velocity_corr) < 0.004:
                stabel_balance += 1
            else:
                stabel_balance = 0
            
            if stabel_balance > 20:
                print("stabilisation goood")
                roadToTheObjective = True
                new_state = "REACH"
        
# Stabilising the boat to emply inertia
        if state == "STABILISING":
            if not ((math.fabs(round(gyro_ship.getValues()[2],3)) < 0.05) and (gps_ship.getSpeed() < 0.05)):
  
                if round(gyro_ship.getValues()[2],3) > 0.25:
                    motor[0].setVelocity(velocity*0.2)
                elif round(gyro_ship.getValues()[2],3) < -0.25:
                    motor[1].setVelocity(velocity*0.2)
                else :
                    motor[0].setVelocity(0.0)
                    motor[1].setVelocity(0.0)

            elif roadToTheObjective == True:
                new_state = "SEARCHING"
            else :
                new_state = "CONVERGING"
                     
        if state == "REACH":
            delay_reach += 1
            motor[0].setVelocity(velocity)
            motor[1].setVelocity(velocity)

            
            zone_value_old = zone_value
            zone_value = round(sensor[0].getValue()/1000.0,2)
            if zone_value < zone_value_old :
                if zone_value < 0.3:
                    new_state = "WAITING"
                else:    
                    new_state = "FOUND"
            
            if delay_reach > 400 :
                delay_reach = 0
                new_state = "CONVERGING"
                       
        if state == "WAITING":
            motor[0].setVelocity(0.0)
            motor[1].setVelocity(0.0)
 
                         
# The robot has found the area   
        if state == "FOUND":
            if zone_value < 0.3:
                priority_level = 3
                """priority_level > actual_priority and"""
                if priority_level > actual_priority and priority < priority_level:
                    print(robot.getName(),"found area lvl 3, broadcast...")
                    emission_position(priority_level)
                    data_package(state)
                    actual_priority = priority_level
                    new_state = "WAITING"
            elif zone_value < 0.5:
                priority_level = 2
                """priority_level > actual_priority and"""
                if priority_level > actual_priority and priority < priority_level:
                    print(robot.getName(),"found area lvl 2, broadcast...")
                    emission_position(priority_level) 
                    data_package(state)
                    actual_priority = priority_level
                    new_state = "SEARCHING"
            elif zone_value < 0.7:
                priority_level = 1
                """priority_level > actual_priority and"""
                if priority_level > actual_priority and priority < priority_level:
                    print(robot.getName(),"found area lvl 1, broadcast...")
                    emission_position(priority_level)
                    data_package(state)
                    actual_priority = priority_level
                    new_state = "SEARCHING"
                    
            if roadToTheObjective == True:
                new_state = "REACH"
                         
        if state == "DATA":
            data_package(new_state)
            
    return 0
    

if __name__ == "__main__":
    print(robot.getName(),"start program")
    main()
  