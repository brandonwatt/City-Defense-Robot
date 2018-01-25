# Brandon Watt, Vlad Psarev and Isaiah Forward
# 1/21/17
# Embedded Systems Design
# Defend the City Robot Navigation Code
#
# This code dictates the navigation behavior of the robot.
# The robot will avoid walls, white lines and stop on magnets it finds, when in 'fire' mode.
# Once it has expended it's ammunition it will enter 'reload' mode where it will contiune
# it's avoidance of walls and white lines, but it will ignore magnets and track towards upside
# down cones.



from __future__ import print_function
from __future__ import division
from builtins import input

import time
from BrickPi import *   # import BrickPi.py file to use BrickPi operations
from MultiMotorDriving import *
from MPU9250 import *
import RPi.GPIO as GPIO
from multiprocessing import *
from LineSensors import *


# Globals and Constants

GRAY_MIN = 0.0012   # An approximate minimum value of the range of gray colors. Anything less is considered white.
MAG_CHANGE = 250    # The difference in magnetic readings needed to indicate the presence of a magnet.

numberOfMotors = 2 # The number of motors driving the robot.

rightPin = 11   # The GPIO pin number for the right ultrasonic sensor.
leftPin = 16    # The GPIO pin number for the left ultrasonic sensor.
ledPin = 13     # The GPIO pin number for the BrickPi board LED

lightPinLeft = 21   # The GPIO pin number for the left color sensor.
lightPinRight = 23  # The GPIO pin number for the right color sensor.

# Speeds for each motor, to be adjusted by the motorAlign function.
Speed = [95, 95]

# Setup the MPU.
mpu9250 = MPU9250()

# The current motor encoder readings.
currentTics = [0] * numberOfMotors

# The previous motor encoder readings.
previousTics = [0] * numberOfMotors

# The differences between a motors current readings and previous readings.
ticDiff = [0] * numberOfMotors

motorL = PORT_C # The port for the left motor.
motorR = PORT_B # The port for the right motor.


# For navigating the robot.
# The cameraPipe allows the navigation process to send and recieve information from the camera process.
# The turretPipe allows the navigation process to send and recieve information from the turret process.
def navigate(cameraPipe = None, turretPipe = None):
    mode = 'fire'   # Initialize the robot in the fire mode so it will travel looking for magnets and firing at targets.
    
    rightPin = 11   # The GPIO pin number for the right ultrasonic sensor.
    leftPin = 16    # The GPIO pin number for the left ultrasonic sensor.
    ledPin = 13     # The GPIO pin number for the BrickPi board LED

    # Sent up the MPU.
    mpu9250 = MPU9250()

    # Initialized to false to keep motorAlign from running in the first pass through the loop.
    RUN = False

    # An alternating variable so that motor alignment and magnet readings only occur
    # every other loop.
    switch = True
    
    # Setting up The shield.
    BrickPiSetup()  # setup the serial port for communication

    # Setting the ports for the sensors.
    # Color_Sensor_Port = PORT_2
    Sonic_Sensor_Port = PORT_1  

    BrickPi.SensorType[Sonic_Sensor_Port] = TYPE_SENSOR_ULTRASONIC_CONT
    #BrickPi.SensorType[Color_Sensor_Port] = TYPE_SENSOR_RCX_LIGHT  


    motorL = PORT_C # The port for the left motor.
    motorR = PORT_B # The port for the right motor.
    motors = [motorL, motorR] # Listing the motors for the motorRotateDegrees and motorAlign functions.

    BrickPi.MotorEnable[motorL] = 1 #Enable the left Motor
    BrickPi.MotorEnable[motorR] = 1 #Enable the right Motor

    #Send the properties of sensors to BrickPi
    BrickPiSetupSensors()   

    # Default function timeout.
    BrickPi.Timeout=3000

    # Setting up the GPIO pin for the led.
    GPIO.setmode(GPIO.BOARD)
    GPIO.setwarnings(False)
    GPIO.setup(ledPin, GPIO.OUT)    #GPIO 27

    # Initialize the LED to off.
    GPIO.output(ledPin, False)

    # Getting initial readings for the magnet.
    mag = mpu9250.readMagnet()
    curZ = abs(mag['z'])
    prevZ = curZ

    # The frontal distance readings.
    preDist = 0
    dist = 0
    preRDist = 0
    rightDist = 0
    preLDist = 0
    leftDist = 0


    # The color readings.
    leftColor = 0.0029
    rightColor = 0.0029

    # The flags to indicate whether or not said color sensor has recently detected white.
    rightColorFlag = False
    leftColorFlag = False

    
    flagResetTime = 3           # The amount of time that must pass before the flags are reset.
    lastFlagTime = time.time()  # The last time one of the flags was set to True.
    
    msgWait = 0.5
    lastMsg = time.time()
    
    BrickPiUpdateValues()

    # Initialize the motor speeds
    BrickPi.MotorSpeed[motorL] = 100
    BrickPi.MotorSpeed[motorR] = 100
    BrickPiUpdateValues()

    # Variables used to set a maximum run time when testing.
    st = time.time()
    wait = 40

    # Wait for a message from the turret to indicate that it has finished calibrating.
    turretPipe.recv()

    # Used to determine a maximum run time when testing.
    #time.time() - st < wait

    # The main navigating loop.
    while (True):
        try:
            # Align the motors and check the magnet reading every other loop.
            if switch:
                # update values.
                result = BrickPiUpdateValues()
                # Read in the motor encoder values.
                for i in range(numberOfMotors):
                    currentTics[i] = BrickPi.Encoder[motors[i]]
                
                # Realign the motors
                motorAlign(motors, True, Speed, RUN)

                # Get the magnet reading and calculate the difference from the last value.
                mag = mpu9250.readMagnet()
                curZ = abs(mag['z'])
                zDif = abs(curZ - prevZ)

            # Ask BrickPi to update values for sensors/motors
            result = BrickPiUpdateValues()
            if not result:
                # Read in the distances and the color from the sensors.
                dist = BrickPi.Sensor[Sonic_Sensor_Port]
                rightDist = ReadDistance(rightPin)
                leftDist = ReadDistance(leftPin)
                
                leftColor = readQD(lightPinLeft)
                rightColor = readQD(lightPinRight)

                # Prints for testing.
                #print ('Distance: %d' % dist)
                #print ('PreDistance: %d' % preDist)
                #print ('Right Distance: %d' % rightDist)
                #print ('Left Distance: %d' % leftDist)
                #print ('Left Color: ', leftColor, 'Right Color: ', rightColor)
                #print ('Mag dif: ', zDif )
                #print ('Speed left: %d, Speed right: %d' % (Speed[0], Speed[1]))

            # If one of the color flags is true and the reset time has elapsed, set the flag false.
            if leftColorFlag and time.time() - lastFlagTime >= flagResetTime:
                leftColorFlag = False

            if rightColorFlag and time.time() - lastFlagTime >= flagResetTime:
                rightColorFlag = False
            
            #  Check to see if the distances, color and magnet readings are within acceptable ranges.
            #
            if (dist < 20 and dist > 0 and preDist < 20 and preDist > 0) or (rightDist < 20 and rightDist > 0 and preRDist < 20 and preRDist > 0) or (leftDist < 20 and leftDist >= 0 and preLDist < 20 and preLDist > 0)or rightColor <= GRAY_MIN  or leftColor <= GRAY_MIN or (zDif >= MAG_CHANGE and mode == 'fire'):                
                

                # If the color is white back up and turn around.   
                if leftColor <= GRAY_MIN or rightColor <= GRAY_MIN:

                    # Check the color again.
                    leftColor = readQD(lightPinLeft)
                    rightColor = readQD(lightPinRight)
                    #print ('Left Color: ', leftColor, 'Right Color: ', rightColor)

                    # White detected on both sides, back up and go towards the shortest distance, the city.
                    if leftColor <= GRAY_MIN and rightColor <= GRAY_MIN:
                        backUp()
                        
                        # Read the left and right distances.
                        rightDistance = ReadDistance(rightPin)
                        leftDistance = ReadDistance(leftPin)

                        #print ('Left Distance: %d' % leftDistance)
                        #print ('Right Distance: %d' % rightDistance)
                        
                        # Turn direction with the greatest distance.
                        if leftDistance > rightDistance:
                            rightTurn = 1
                        else:
                            rightTurn = -1

                        # Turn the appropriate direction.
                        power=[100, 100]
                        deg = [-120 * rightTurn, 120 * rightTurn]
                        maxWheelSpeedDiff = motorRotateDeg (power, deg, motors, sampling_time=0.0)
                        
                    # Detect white on the left side, turn right.
                    elif leftColor <= GRAY_MIN:
                        if rightColorFlag:
                            backUp()

                            # Turn right
                            rightTurn = 1
                            # Turn the appropriate direction.
                            power=[100, 100]
                            deg = [-120 * rightTurn, 120 * rightTurn]
                            maxWheelSpeedDiff = motorRotateDeg (power, deg, motors, sampling_time=0.0)

                            # Recheck the sensors, if either is white, back up again.
                            rightColor = readQD(lightPinRight)
                            leftColor = readQD(lightPinLeft)
                            if rightColor <= GRAY_MIN or leftColor <= GRAY_MIN:
                                backUp()                            

                            lastFlagTime = time.time()
                            
                        else:
                            # Turn the robot right.
                            BrickPi.MotorSpeed[motorL] = -120
                            BrickPi.MotorSpeed[motorR] = 120
                            BrickPiUpdateValues()
                            time.sleep(0.01)

                            leftColorFlag = True
                            lastFlagTime = time.time()
                            
                            rightColor = readQD(lightPinRight)
                            if rightColor <= GRAY_MIN:
                                backUp()

                            if (time.time() - lastMsg) >= msgWait:
                                turretPipe.send([90, 'right'])
                                lastMsg = time.time()
                        
                    # Detect white on the right side, turn left. 
                    elif rightColor <= GRAY_MIN:
                        # If the left color sensor has recently detected white as well.
                        if leftColorFlag:
                            backUp()

                            # Turn left
                            rightTurn = -1
                            
                            # Turn the appropriate direction.
                            power=[100, 100]
                            deg = [-120 * rightTurn, 120 * rightTurn]
                            maxWheelSpeedDiff = motorRotateDeg (power, deg, motors, sampling_time=0.0)

                            # Recheck the sensors, if either is white, back up again.
                            rightColor = readQD(lightPinRight)
                            leftColor = readQD(lightPinLeft)
                            if rightColor <= GRAY_MIN or leftColor <= GRAY_MIN:
                                backUp() 

                            # Reset the flag time.
                            lastFlagTime = time.time()
                            
                        else:
                            # Turn the robot left.
                            BrickPi.MotorSpeed[motorL] = 120  
                            BrickPi.MotorSpeed[motorR] = -120
                            BrickPiUpdateValues()
                            time.sleep(0.01)

                            # Set the right flag true and reset the flage timer.
                            rightColorFlag = True
                            lastFlagTime = time.time()
                        
                            leftColor = readQD(lightPinLeft)
                            if leftColor <= GRAY_MIN:
                                backUp()

                            if (time.time() - lastMsg) >= msgWait:
                                turretPipe.send([90, 'left'])
                                lastMsg = time.time()
                        
                # If a magnet is found
                elif zDif >= MAG_CHANGE and mode == 'fire':
                    # Turn on the led.
                    GPIO.output(ledPin, True)

                    # Re-update the magnet values to avoid constantly reading the same values.
                    mag = mpu9250.readMagnet()
                    curZ = abs(mag['z'])
                    prevZ = curZ

                    mode = turretPipe.recv()
                    GPIO.output(ledPin, False)
                    time.sleep(2) # Give the turret time to fire before navigation begins.
                    
                # If a wall is found.   
                else:
                    #print ('Left Distance: ', leftDist, ' Right Distance: ', rightDist, ' Front Distance: ', dist)
                    # If the robot is very close to a wall, back up to avoid hitting it when turning.
                    if dist < 5 or rightDist < 1 or leftDist < 1:
                        
                        backUp()

                    # There are walls close by on both sides.
                    if (rightDist < 5 and leftDist < 5) or (dist < 5 and leftDist < 5) or (dist < 5 and rightDist < 5):
                        backUp()
                        backUp()

                        # Read the left and right distances.
                        rightDistance = ReadDistance(rightPin)
                        leftDistance = ReadDistance(leftPin)

                        #print ('Left Distance: %d' % leftDistance)
                        #print ('Right Distance: %d' % rightDistance)
                        
                        # Turn direction with the greatest distance.
                        if leftDistance > rightDistance:
                            rightTurn = -1
                        else:
                            rightTurn = 1

                        # Turn the appropriate direction.
                        power=[100, 100]
                        deg = [-120 * rightTurn, 120 * rightTurn]
                        maxWheelSpeedDiff = motorRotateDeg (power, deg, motors, sampling_time=0.0)

                        # Send whether you turn left or right to turret, with a base power.
                        if rightTurn == 1 and (time.time() - lastMsg) >= msgWait:
                            turretPipe.send([60, 'right'])
                            lastMsg = time.time()
                        elif (time.time() - lastMsg) >= msgWait:
                            turretPipe.send([60, 'left'])
                            lastMsg = time.time()
                            
                    # If there is a nearby wall on the right side, turn left.
                    elif rightDist < 20 and preRDist < 20:
                        change =  90 - (int(rightDist) * 2)

                        #print('Turn Left')
                        BrickPi.MotorSpeed[motorL] += change  #Set the speed of MotorA (-255 to 255)
                        BrickPi.MotorSpeed[motorR] -= change
                        BrickPiUpdateValues()

                        # Tell the turret that it's turning left at such a speed.
                        # Limit the number of messages sent to 2 per second.
                        if (time.time() - lastMsg) >= msgWait:
                            turretPipe.send([change, 'left'])
                            lastMsg = time.time()

                    # If there is a nearby wall on the left side, turn right.
                    elif leftDist < 20 and preLDist < 20:
                        change =  90 - (int(leftDist) * 2)

                        #print('Turn Right')
                        BrickPi.MotorSpeed[motorL] -= change 
                        BrickPi.MotorSpeed[motorR] += change
                        BrickPiUpdateValues()

                        # Tell the turret that it's turning right at such a speed.
                        # Limit the number of messages sent to 2 per second.
                        if (time.time() - lastMsg) >= msgWait:
                            turretPipe.send([change, 'right'])
                            lastMsg = time.time()
                        
                    # If there is a nearby wall straight ahead.
                    else:
                        backUp()
                        
                        # Read the left and right distances.
                        rightDistance = ReadDistance(rightPin)
                        leftDistance = ReadDistance(leftPin)

                        #print ('Left Distance: %d' % leftDistance)
                        #print ('Right Distance: %d' % rightDistance)
                        
                        # Turn direction with the greatest distance.
                        if leftDistance > rightDistance:
                            rightTurn = -1
                        else:
                            rightTurn = 1

                        # Turn the appropriate direction.
                        power=[100, 100]
                        deg = [-120 * rightTurn, 120 * rightTurn]
                        maxWheelSpeedDiff = motorRotateDeg (power, deg, motors, sampling_time=0.0)

                        # Send whether you turn left or right to turret, with a base power.
                        if rightTurn == 1 and (time.time() - lastMsg) >= msgWait:
                            turretPipe.send([60, 'right'])
                            lastMsg = time.time()
                        elif (time.time() - lastMsg) >= msgWait:
                            turretPipe.send([60, 'left'])
                            lastMsg = time.time()
                            
            # If no obsticle is encountered the robot should track to the reload station.
            elif mode == 'reload' and turretPipe.poll():

                # Recieve the position of the turret relative to the body in seconds.
                # Negitve indicates left, positive indicates right and center is zero.
                # The time it takes for the turret to travel from the center position to a far side is also sent.
                dif, centerTime = turretPipe.recv()

                # Calculate the amount of speed change needed to face the robot toward the target.
                change =  int ((abs(dif) / centerTime) * 280)
                    
                # If the turret is facing left. The body should be rotated left.
                if dif < 0:
                    # Move left
                    # If it is a small short adjustment.
                    if change <= 80:
                        #print('Turn Left')
                        BrickPi.MotorSpeed[motorL] += change # Increase left motor speed.
                        BrickPi.MotorSpeed[motorR] -= change # Reduce right motor speed.
                        BrickPiUpdateValues()
                        time.sleep(0.03)
                    # If the turn is fast, reduce the power and increase the delay,
                    # to allow for a less 'twitchy' turn.
                    else:
                        #print('Turn Left')
                        BrickPi.MotorSpeed[motorL] += int(change/3) # Increase left motor speed.
                        BrickPi.MotorSpeed[motorR] -= int(change/3) # Reduce right motor speed.
                        BrickPiUpdateValues()
                        time.sleep(0.09)

                    
                # If the turret is facing right. The body should be rotated right.
                elif dif > 0:
                    # Move right
                    # If it is a small short adjustment.
                    if change <= 80:
                        BrickPi.MotorSpeed[motorL] -= change # Reduce left motor speed.
                        BrickPi.MotorSpeed[motorR] += change # Increase right motor speed.
                        BrickPiUpdateValues()
                        time.sleep(0.03)
                    # If the turn is large, reduce the power and increase the delay,
                    # to allow for a less 'twitchy' turn.
                    else:
                        BrickPi.MotorSpeed[motorL] -= int(change/3) # Reduce left motor speed.
                        BrickPi.MotorSpeed[motorR] += int(change/3) # Increase right motor speed.
                        BrickPiUpdateValues()
                        time.sleep(0.09)
                    

                
            # Update values.
            result = BrickPiUpdateValues()

            # The current distance readings become the previous readings.
            preDist = dist
            preRDist = rightDist
            preLDist = leftDist

            # Set the global so that motorAlign will always run.
            RUN = True

            # Every other loop update the motor encoder readings and the magnet readings.
            if switch:
                for i in range(numberOfMotors):
                    previousTics[i] = BrickPi.Encoder[motors[i]]
                
                prevZ = curZ
            
            # Alternate whether the every other checks will occur or not.
            switch = not switch
            
        # End the loop when ctrl-c is pressed.
        except KeyboardInterrupt:
            break
        
    # End of while loop
    
    # The loop is done, turn off the LED, stop the motors and clean up the GPIO pins.
    GPIO.output(ledPin, False)  
    BrickPi.MotorSpeed[motorL] = 0
    BrickPi.MotorSpeed[motorR] = 0
    BrickPiUpdateValues()
    GPIO.cleanup()

# Causes the robot to back up a short distance.
def backUp():
    # Set the motors to reverse.
    BrickPi.MotorSpeed[motorL] = -100
    BrickPi.MotorSpeed[motorR] = -100

    # Update the motors for 0.5 seconds to ensure the robot stays in reverse for 0.5 seconds.
    ot = time.time()
    while(time.time() - ot < .5):    
        BrickPiUpdateValues()       
        time.sleep(.01)

    # Stop the motors.
    BrickPi.MotorSpeed[motorL] = 0  
    BrickPi.MotorSpeed[motorR] = 0
    BrickPiUpdateValues()

# Rotates the Robot approximately 180 degrees.
def turn_180(right = 1, motors  = [PORT_C, PORT_B]):
    # Encoder turn
    result = BrickPiUpdateValues() 
    if not result : # if updating values succeedes
        power=[100, 100]
        deg = [-90 * right, 90 * right] # Approximate value for a 180 degree turn
        maxWheelSpeedDiff = motorRotateDeg (power, deg, motors, sampling_time=0.0)
        
        BrickPiSense()

    # Stop moving
    BrickPi.MotorSpeed[PORT_A] = 0
    BrickPi.MotorSpeed[PORT_D] = 0
    BrickPiUpdateValues()

# Rotates the Robot approximately 90 degrees.
def turn_90(right = 1, motors = [PORT_C, PORT_B]):
    result = BrickPiUpdateValues() 
    if not result : # if updating values succeedes
        power=[100, 100]
        deg = [-200 * right, 200 * right]
        maxWheelSpeedDiff = motorRotateDeg (power, deg, motors, sampling_time=0.0)        
        BrickPiSense()
    
    # Stop moving
    BrickPi.MotorSpeed[PORT_A] = 0
    BrickPi.MotorSpeed[PORT_D] = 0
    BrickPiUpdateValues()

# Adjust the motor speeds so that the robot will drive straight.
def motorAlign(motors, switch, Speed, RUN):
    # Won't run on the intial pass through the navigation loop.
    if RUN:

        # Calculates the difference between current and previous encoder readings.
        # This can be thought as a more accurate current speed of each motor.
        ticDiff0 = currentTics[0] - previousTics[0]
        ticDiff1 = currentTics[1] - previousTics[1]
        
        #print ('left motor: %d, right motor: %d' % (ticDiff0, ticDiff1))

        # If the diffences are to great or small, it is due to a turn, and
        # adjustsments to speed should not be made. Otherwise overcompensation occurs.
        if (ticDiff0 > 60 or ticDiff0 < 20 or ticDiff1 > 60 or ticDiff1 < 20):
            BrickPi.MotorSpeed[motors[0]] = Speed[0]
            BrickPi.MotorSpeed[motors[1]] = Speed[1]
        # If the motor is faster than the other, increase it's speed.   
        elif ticDiff0 < ticDiff1:
            Speed[0] += 1
            BrickPi.MotorSpeed[motors[0]] = Speed[0]
        # If the motor is slower than the other, decrease it's speed.  
        elif ticDiff0 > ticDiff1:
            Speed[0] -= 1
            BrickPi.MotorSpeed[motors[0]] = Speed[0]
        # If the motors are equal don't adjust speed.
        else:
            BrickPi.MotorSpeed[motors[0]] = Speed[0]

        # Keep the base motor at a constant speed.
        BrickPi.MotorSpeed[motors[1]] = Speed[1]

    # Ensure that motorAlign will run after the initial loop.
    RUN = True
    # update the motor speeds.
    BrickPiUpdateValues()

# Gets the distance from the ultransonic sensors that aren't through brick pi.
def ReadDistance(pin):  
    wait = 0.25 # The maximum amount of time the loops should wait for a signal.

    # Initial times incase of loop failure.
    initialTime = time.time()
    starttime = time.time()
    endtime = time.time()

    GPIO.setup(pin, GPIO.OUT)
    GPIO.output(pin, 0)
    time.sleep(0.000002)

    # send trigger signal  
    GPIO.output(pin, 1)
    # wait
    time.sleep(0.000005)
    
    # set the pin to low.
    GPIO.output(pin, 0)
    
    # Make the pin read input
    GPIO.setup(pin, GPIO.IN)

    
    # Wait for the pin to go high.
    while GPIO.input(pin) == 0:  
        starttime=time.time() # record the start time.
        # If the method waits for too long, end the loop.
        if starttime - initialTime > wait:
            break
    
    # Wait for the pin to go low.
    while GPIO.input(pin)==1:  
        endtime=time.time()  # record the end time.
        # If the method waits for too long, end the loop.
        if endtime - initialTime > wait:
            break

    # calculate the duration of the pulse.
    duration=endtime-starttime  
    # Distance is defined as time/2 (there and back) * speed of sound 34000 cm/s   
    distance=duration*34000/2
    return distance

# Main method, only used for individual testing.
if __name__ == '__main__':
    ntTurretPipe, ntNavPipe = Pipe()
    cnCameraPipe, cnNavPipe = Pipe()
    
    navigate(ntNavPipe, cnNavPipe)
