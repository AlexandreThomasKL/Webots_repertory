from controller import Robot

TIME_STEP = 64
robot = Robot()
#ds = []
#dsNames = ['ds_right', 'ds_left']
#for i in range(2):
    #ds.append(robot.getDevice(dsNames[i]))
    #ds[i].enable(TIME_STEP)
wheels = []
wheelsNames = ['Right_motor', 'Left_motor']
for i in range(2):
    wheels.append(robot.getDevice(wheelsNames[i]))
    wheels[i].setPosition(float('inf'))
    wheels[i].setVelocity(0.0)
avoidObstacleCounter = 0
while robot.step(TIME_STEP) != -1:
    leftSpeed = 1.0
    rightSpeed = 1.0
              
    wheels[0].setVelocity(leftSpeed)
    wheels[1].setVelocity(rightSpeed)
