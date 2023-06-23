"""ship_swarm controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot
from controller import Keyboard
from controller import Emitter
from controller import Receiver
import math
import string

# create the Robot instance.
robot = Robot()

# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())

# Setup & Initialisation
motor = []
name_motor = ["propeller_motor_right","propeller_motor_left","propeller_motor_front","propeller_motor_back"]
for i in range(len(name_motor)):
    motor.append(robot.getDevice(name_motor[i]))
    motor[i].setPosition(float('inf'))
    motor[i].setVelocity(0.0)
    
sensor = []
name_sensor = ["area_detector","sensor_NE","sensor_SE","sensor_SO","sensor_NO"]
for i in range(len(name_sensor)):
    sensor.append(robot.getDevice(name_sensor[i]))
    sensor[i].enable(timestep)


gps_ship = robot.getDevice("gps")
gyro_ship = robot.getDevice("gyro")
find_area = robot.getDevice("area_detector")
compass_ship = robot.getDevice("compass")
rx = robot.getDevice("receiver")
tx = robot.getDevice("emitter")

keyboard_control = Keyboard()
keyboard_control.enable(timestep)

delay = 0
roataion_speed = 6.28
areaHasBeFound = False
onTheSpot = False
velocity = roataion_speed*0.25

gps_ship.enable(timestep)
gyro_ship.enable(timestep)
find_area.enable(timestep)
compass_ship.enable(timestep)

rx.setChannel(5)
tx.setChannel(5)
rx.enable(timestep)


f = open("test_text.txt", "a")
f.write("Now the file has more content!\n")
f.close()


def transition_speed():
    vect_x = round(gps_ship.getSpeedVector()[0],2)
    vect_y = round(gps_ship.getSpeedVector()[1],2)
    return vect_x,vect_y,
    
def bearing_position():
    angle_compass = round(math.atan2(compass_ship.getValues()[0],compass_ship.getValues()[1]),2)
    return angle_compass
 
def emission_position():
    pos_x = round(gps_ship.getValues()[0],2)
    pos_y = round(gps_ship.getValues()[1],2)
    message = str(pos_x) +" "+ str(pos_y)
    print("Coordinates area :",message)
    tx.send(message)

def control_keyboard(velocity):
    key = keyboard_control.getKey()
    print(key)
    if key == 316:
        motor[0].setVelocity(velocity)
        motor[1].setVelocity(0.0)
        motor[2].setVelocity(0.0)
        motor[3].setVelocity(0.0)
    if key == 314:
        motor[0].setVelocity(0.0)
        motor[1].setVelocity(velocity)
        motor[2].setVelocity(0.0)
        motor[3].setVelocity(0.0)
    if key == 315:
        motor[0].setVelocity(0.0)
        motor[1].setVelocity(0.0)
        motor[2].setVelocity(0.0)
        motor[3].setVelocity(velocity)
    if key == 317: 
        motor[0].setVelocity(0.0)
        motor[1].setVelocity(0.0)
        motor[2].setVelocity(velocity)
        motor[3].setVelocity(0.0)
    if key == -1: 
        motor[0].setVelocity(0.0)
        motor[1].setVelocity(0.0)
        motor[2].setVelocity(0.0)
        motor[3].setVelocity(0.0)
        
def avoidance_state(velocity,ds_value_NE,ds_value_SE,ds_value_SO,ds_value_NO):
    if ds_value_l < 950.0:
        propeller_ship_right.setVelocity(velocity*0.2)
        propeller_ship_left.setVelocity(0.0) 
    elif ds_value_r < 950.0:
        propeller_ship_right.setVelocity(0.0)
        propeller_ship_left.setVelocity(velocity*0.2)

def main ():
    global areaHasBeFound,onTheSpot, velocity
    state = "INITIALISATION"
    new_state = state
    stabel_balance = 0
    while robot.step(timestep) != -1:
      
   
        if (new_state != state):
            state = new_state
            print(robot.getName(),state)
    
        if state == "INITIALISATION":
        
            print(robot.getName(),"initilaisation...")
            # Wait for the robot to fall
            delay = 0
            while (delay < 20 and robot.step(timestep) != -1):
                delay += 1
            print(robot.getName(),"running")
            
            new_state = "SEARCHING"
            
     
        if state == "SEARCHING":
        
            control_keyboard(velocity)
            
            """  
            if left_dt.getValue() < 950.0 or right_dt.getValue() < 950.0:
                new_state = "AVOIDANCE"
                
            if find_area.getValue() < 600.0 and onTheSpot == False:
                areaHasBeFound = True
                onTheSpot = True
                new_state = "FOUND"
              
            if rx.getQueueLength() > 0 and areaHasBeFound == False:
                areaHasBeFound = True
                new_state = "RECEVING"
               
        if state == "AVOIDANCE":
                avoidance_state(velocity,round(left_dt.getValue(),1),round(right_dt.getValue(),1))
                if  left_dt.getValue() > 950.0 and  right_dt.getValue() > 950.0:
                    new_state = "SEARCHING"
                    
                    
        if state == "RECEVING":
            message = rx.getString()
            pos_coord = list(map(float,message.split(" ")))
            rx.nextPacket()
            new_state = "STABILISING"
            
            
        if state == "CONVERGING":
            
            
            
            angle_compass = -round(bearing_position(),2)
            
            print("Compass angle :" ,angle_compass)
            
            def_x = -(round(gps_ship.getValues()[0],2) - pos_coord[0])
            def_y = -(round(gps_ship.getValues()[1],2) - pos_coord[1])
            
            def_x_r = round((math.cos(angle_compass) - math.sin(angle_compass))*def_x,2)
            def_y_r = round((math.sin(angle_compass) + math.cos(angle_compass))*def_y,2)

            angle_converging = round(math.atan2(def_y,def_x)*(180.0/math.pi)-90.0,2)
            angle_converging_r = round(math.atan2(def_y_r,def_x_r)*(180.0/math.pi)-90.0,2)

            print("position robot :" ,def_x_r ,def_y_r)
            print("position :" ,def_x, def_y)
            
            if angle_converging < -180.0:
                angle_converging = round(angle_converging + 360.0,2)
            print("Converging angle :" ,angle_converging)
            print("Converging angle robot :" ,angle_converging_r)
            angle_rotate = round( math.fmod(
                            angle_converging + angle_compass
                            ,360.0),2)
            #0.261799
            
            print("Angle 1 :" ,angle_rotate)
            
            ratio = round(math.fabs(angle_rotate)/180.0,2)
            print(ratio)
            
            #velocity_ratio = round((velocity-velocity*math.fabs(ratio))/roataion_speed ,2)
            #print(ratio,velocity_ratio)
            
            if ratio > -0.1:
                propeller_ship_right.setVelocity(-velocity*0.2)
                propeller_ship_left.setVelocity(velocity*0.2)
            if ratio < 0.1:
                propeller_ship_right.setVelocity(velocity*0.2)
                propeller_ship_left.setVelocity(-velocity*0.2)
             
            print("Gyro speed :",math.fabs(round(gyro_ship.getValues()[2],2)))
            if math.fabs(round(gyro_ship.getValues()[2],2)) < 0.03:
                stabel_balance += 1
            else:
                stabel_balance = 0
            
            if stabel_balance > 10:
                print("stabilisation goood")       
                #new_state = "WAITING"
       
   
        if state == "STABILISING":
            print("Gyro speed :",round(gyro_ship.getValues()[2],2))
            if not ((math.fabs(round(gyro_ship.getValues()[2],2)) < 0.01) and (gps_ship.getSpeed() < 0.01)):
                if round(gyro_ship.getValues()[2],2) > 0.25:
                    propeller_ship_right.setVelocity(velocity*0.2)
                elif round(gyro_ship.getValues()[2],2) < -0.25:
                    propeller_ship_left.setVelocity(velocity*0.2)
                else :
                    propeller_ship_right.setVelocity(0.0)
                    propeller_ship_left.setVelocity(0.0)
            else :
                new_state = "CONVERGING"
                      
               
        if state == "FOUND":
            areaHasBeFound = True
            print(robot.getName(),"found area, broadcast...")
            emission_position()
            new_state = "WAITING"
            
            
        if state == "WAITING":
            propeller_ship_right.setVelocity(0.0)
            propeller_ship_left.setVelocity(0.0)
            if not (find_area.getValue() < 600.0) and onTheSpot == False:
                new_state = "SEARCHING"
        
   ²"""
    return 0

if __name__ == "__main__":
    print(robot.getName(),"start program")
    main()
  