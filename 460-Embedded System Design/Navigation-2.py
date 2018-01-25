
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

GRAY_MIN = 0.0012 # An approximate minimum value of the range of gray colors. Anything less is considered white.
MAG_CHANGE = 250 # The difference in magnetic readings needed to indicate the presence of a magnet.

numberOfMotors = 2 # The number of motors driving the robot.

rightPin = 11 # The GPIO pin number for the right ultrasonic sensor.
leftPin = 16 # The GPIO pin number for the left ultrasonic sensor.
ledPin = 13 # The GPIO pin number for the BrickPi board LED

lightPinLeft = 21
lightPinRight = 23

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
    motors = [motorL, motorR]

    BrickPi.MotorEnable[motorL] = 1 #Enable the left Motor
    BrickPi.MotorEnable[motorR] = 1 #Enable the right Motor

    #Send the properties of sensors to BrickPi
    BrickPiSetupSensors()   

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


    leftColor = 0.0029
    rightColor = 0.0029
    rightColorFlag = False
    leftColorFlag = False
    flagResetTime = 3
    lastFlagTime = time.time()
    
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

    turretPipe.recv()
    # Used to determine a maximum run time when testing.
    #time.time() - st < wait

    # The main navigating loop.
    #print('Starting While')
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
                motorAlign2(motors, True, Speed, RUN)

                # Get the magnet reading and calculate the difference from the last value.
                mag = mpu9250.readMagnet()
                curZ = abs(mag['z'])
                zDif = abs(curZ - prevZ)

            # Ask BrickPi to update values for sensors/motors
            #print('Before result')
            result = BrickPiUpdateValues()
            #print(result)
            if not result:
                # Read in the distances and the color from the sensors.
                dist = BrickPi.Sensor[Sonic_Sensor_Port]
                rightDist = ReadDistance(rightPin)
                leftDist = ReadDistance(leftPin)
                
                leftColor = readQD(lightPinLeft)
                rightColor = readQD(lightPinRight)

                #print ('Distance: %d' % dist)
                #print ('PreDistance: %d' % preDist)
                #print ('Right Distance: %d' % rightDist)
                #print ('Left Distance: %d' % leftDist)
                #print ('Left Color: ', leftColor, 'Right Color: ', rightColor)
                # Prints for testing.
                
                #print ('Mag dif: ', zDif )
                #print ('Speed left: %d, Speed right: %d' % (Speed[0], Speed[1]))

            # If one of the color flags is true and the reset time has elapsed, set the flag false.
            if leftColorFlag and time.time() - lastFlagTime >= flagResetTime:
                leftColorFlag = False

            if rightColorFlag and time.time() - lastFlagTime >= flagResetTime:
                rightColorFlag = False
            
            #  Check to see if the distances, color and magnet readings are within acceptable ranges.
            #
            if (dist < 20 and dist > 0 and preDist < 20 and preDist > 0) or (rightDist < 20 and rightDist > 0 and preRDist < 20 and preRDist > 0) or (leftDist < 20 and leftDist >= 0  and preLDist < 20 and preLDist > 0)or rightColor <= GRAY_MIN  or leftColor <= GRAY_MIN or (zDif >= MAG_CHANGE and mode == 'fire'):                
                

                # If the color is white back up and turn around.   
                if leftColor <= GRAY_MIN or rightColor <= GRAY_MIN:

                    # Check the color again.
                    leftColor = readQD(lightPinLeft)
                    rightColor = readQD(lightPinRight)
                    #print ('Left Color: ', leftColor, 'Right Color: ', rightColor)
                    # Backup
                    #backUp()

                    # Detect white on both sides, back up and go towards the shortest distance, the city.
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
            
                            rightColor = readQD(lightPinRight)
                            leftColor = readQD(lightPinLeft)

                            if rightColor <= GRAY_MIN or leftColor <= GRAY_MIN:
                                backUp()                            

                            lastFlagTime = time.time()
                            
                        else:
                            change = 90
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
                        if leftColorFlag:
                            backUp()

                            # Turn left
                            rightTurn = -1
                            
                            # Turn the appropriate direction.
                            power=[100, 100]
                            deg = [-120 * rightTurn, 120 * rightTurn]
                            maxWheelSpeedDiff = motorRotateDeg (power, deg, motors, sampling_time=0.0)
            
                            rightColor = readQD(lightPinRight)
                            leftColor = readQD(lightPinLeft)

                            rightColor = readQD(lightPinRight)
                            leftColor = readQD(lightPinLeft)
                            if rightColor <= GRAY_MIN or leftColor <= GRAY_MIN:
                                backUp() 

                            lastFlagTime = time.time()
                            
                        else:
                            change = 90
                            BrickPi.MotorSpeed[motorL] = 120  
                            BrickPi.MotorSpeed[motorR] = -120
                            BrickPiUpdateValues()
                            time.sleep(0.01)

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
                    
                    #print('Found Mag')
                    # Turn on the led and wait for a moment.
                    GPIO.output(ledPin, True)
                    #time.sleep(2)

                    # Turn off the led.
                    #GPIO.output(ledPin, False)

                    # Re-update the magnet values to avoid constantly reading the same values.
                    mag = mpu9250.readMagnet()
                    curZ = abs(mag['z'])
                    prevZ = curZ

                    #print('Defending')
                    #turretPipe.send('Defend')
                    mode = turretPipe.recv()
                    GPIO.output(ledPin, False)
                    #print('mode: ', mode)
                    time.sleep(2)
                    #break # Exit the main loop
                    
                # If a wall is found.   
                else:
                    #print ('Left Distance: ', leftDist, ' Right Distance: ', rightDist, ' Front Distance: ', dist)
                    #print('Wall detected')
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
                        # Limit the number of messages sent to 1 per second.
                        if (time.time() - lastMsg) >= msgWait:
                            turretPipe.send([change, 'left'])
                            lastMsg = time.time()

                    # If there is a nearby wall on the left side, turn right.
                    elif leftDist < 20 and preLDist < 20:
                        change =  90 - (int(leftDist) * 2)

                        #print('Turn Right')
                        BrickPi.MotorSpeed[motorL] -= change  #Set the speed of MotorA (-255 to 255)
                        BrickPi.MotorSpeed[motorR] += change
                        BrickPiUpdateValues()

                        # Tell the turret that it's turning right at such a speed.
                        # Limit the number of messages sent to 1 per second.
                        if (time.time() - lastMsg) >= msgWait:
                            turretPipe.send([change, 'right'])
                            lastMsg = time.time()
                        
                    # If there is a nearby wall straight ahead.
                    else: 
                        #print('Straight Ahead')
            
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
                #print('Center difference: ', dif)

                # If the turret is facing left. The body should be rotated left.
                if dif < 0:
                    # Move left
                    change =  int ((abs(dif) / centerTime) * 280)
                    #print(change)

                    # If it is a small short adjustment.
                    if change <= 80:
                        #print('Turn Left')
                        BrickPi.MotorSpeed[motorL] += change  #Set the speed of MotorA (-255 to 255)
                        BrickPi.MotorSpeed[motorR] -= change
                        BrickPiUpdateValues()
                        time.sleep(0.03)
                    # If the turn is fast, reduce the power and increase the delay,
                    # to allow for a less 'twitchy' turn.
                    else:
                        #print('Turn Left')
                        BrickPi.MotorSpeed[motorL] += int(change/3)  #Set the speed of MotorA (-255 to 255)
                        BrickPi.MotorSpeed[motorR] -= int(change/3)
                        BrickPiUpdateValues()
                        time.sleep(0.09)

                    
                # If the turret is facing right. The body should be rotated right.
                elif dif > 0:
                    # Move right
                    change =  int((abs(dif) / centerTime) * 280)
                    #print(change)

                    # If it is a small short adjustment.
                    if change <= 80:
                        #print('Turn Right')
                        BrickPi.MotorSpeed[motorL] -= change  #Set the speed of MotorA (-255 to 255)
                        BrickPi.MotorSpeed[motorR] += change
                        BrickPiUpdateValues()
                        time.sleep(0.03)
                    # If the turn is fast, reduce the power and increase the delay,
                    # to allow for a less 'twitchy' turn.
                    else:
                        #print('Turn Right')
                        BrickPi.MotorSpeed[motorL] -= int(change/3)  #Set the speed of MotorA (-255 to 255)
                        BrickPi.MotorSpeed[motorR] += int(change/3)
                        BrickPiUpdateValues()
                        time.sleep(0.09)
                    

                
            # Update values.
            result = BrickPiUpdateValues()
            
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
            
            # Alternate the whether the every other checks will occur or not.
            switch = not switch
            #print('Loop completed')
            

        except KeyboardInterrupt:
            break
            #print('break attempt')
    # End of while loop
    
    # The loop is done, turn off the LED and stop the motors.
    GPIO.output(ledPin, False)  
    BrickPi.MotorSpeed[motorL] = 0  #Set the speed of MotorA (-255 to 255)
    BrickPi.MotorSpeed[motorR] = 0
    BrickPiUpdateValues()
    GPIO.cleanup()

def backUp():
    BrickPi.MotorSpeed[motorL] = -100
    BrickPi.MotorSpeed[motorR] = -100
    ot = time.time()
    while(time.time() - ot < .5):    
        BrickPiUpdateValues()       
        time.sleep(.01)

    BrickPi.MotorSpeed[motorL] = 0  
    BrickPi.MotorSpeed[motorR] = 0
    BrickPiUpdateValues()

# Rotates the Robot approximately 180 degrees.
def turn_180(right = 1, motors  = [PORT_C, PORT_B]):
    # Encoder turn
    result = BrickPiUpdateValues() 
    if not result :					# if updating values succeedes
        #print('turn 180')
        power=[100, 100] # 0 to 255
        deg = [-90 * right, 90 * right] # Approximate value for a 180 degree turn
        maxWheelSpeedDiff = motorRotateDeg (power, deg, motors, sampling_time=0.0)		#to use MultiMotorDriving's super version
        
        BrickPiSense()

    # Stop moving
    BrickPi.MotorSpeed[PORT_A] = 0
    BrickPi.MotorSpeed[PORT_D] = 0
    BrickPiUpdateValues()

# Rotates the Robot approximately 90 degrees.
def turn_90(right = 1, motors = [PORT_C, PORT_B]):
    result = BrickPiUpdateValues() 
    if not result : # if updating values succeedes
        #print('turn 90')
        power=[100, 100]
        deg = [-200 * right, 200 * right]
        maxWheelSpeedDiff = motorRotateDeg (power, deg, motors, sampling_time=0.0)        
        BrickPiSense()
    
    # Stop moving
    BrickPi.MotorSpeed[PORT_A] = 0
    BrickPi.MotorSpeed[PORT_D] = 0
    BrickPiUpdateValues()

# Adjust the motor speeds so that the robot will drive straight.
def motorAlign2(motors, switch, Speed, RUN):
    #print('motor align')
    # Won't run on the intial pass through the navigation loop.
    if RUN:

        # Calculates the difference between current and previous encoder readings.
        # This can be thought as a more accurate current spped of each motor.
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
def ReadDistance(pin1):  
    #print('Reading Dist')
    wait = 0.25
    initialTime = time.time()
    starttime = time.time()
    endtime = time.time()

    GPIO.setup(pin1, GPIO.OUT)
    GPIO.output(pin1, 0)
    time.sleep(0.000002)

    # send trigger signal  
    GPIO.output(pin1, 1)
    # wait
    time.sleep(0.000005)
    
    # set the pin to low.
    GPIO.output(pin1, 0)
    
    # Make the pin read input
    GPIO.setup(pin1, GPIO.IN)

    
    # Wait for the pin to go high.
    while GPIO.input(pin1) == 0:  
        starttime=time.time() # record the start time.
        if starttime - initialTime > wait:
            break
    
    # Wait for the pin to go low.
    while GPIO.input(pin1)==1:  
        endtime=time.time()  # record the end time.
        if endtime - initialTime > wait:
            break

    # calculate the duration of the pulse.
    duration=endtime-starttime  
    # Distance is defined as time/2 (there and back) * speed of sound 34000 cm/s   
    distance=duration*34000/2
    #print('Returning Distance')
    return distance

if __name__ == '__main__':
    ntTurretPipe, ntNavPipe = Pipe()
    cnCameraPipe, cnNavPipe = Pipe()
    
    navigate(ntNavPipe, cnNavPipe)


# End of used code.

# Old turn 90 degrees code
##    BrickPi.MotorSpeed[PORT_A] = -100 * right  #Set the speed of MotorA (-255 to 255)
##    BrickPi.MotorSpeed[PORT_D] = 102 * right  #Set the speed of MotorB (-255 to 255)
##    ot = time.time()
##    wait = .72/2 + 0.11  # Half of 180
##    while(time.time() - ot < wait):    #running while loop for 3 seconds
##        BrickPiUpdateValues()       # Ask BrickPi to update values for sensors/motors
##        time.sleep(.01)

# Original 180 degree Turn Code
##    BrickPi.MotorSpeed[PORT_A] = -100 * right  #Set the speed of MotorA (-255 to 255)
##    BrickPi.MotorSpeed[PORT_D] = 102 * right  #Set the speed of MotorB (-255 to 255)
##    ot = time.time()
##    while(time.time() - ot < .87):    #running while loop for 3 seconds
##        BrickPiUpdateValues()       # Ask BrickPi to update values for sensors/motors
##        time.sleep(.01)


# Old head turning code
##    BrickPiUpdateValues()
##    head = ((BrickPi.Encoder[hmotor[0]] % 720)/2)
##    #print('head %d' % head)
##    if head >= 77 and head < 120:
##        degChange = head - 45
##        hdeg = [-degChange]
##        headTurn = motorRotateDeg (hpower, hdeg, hmotor, sampling_time=0.0)
##        #print('Adjusting head left: %d' % head)
##    elif head < 292 and head > 180:
##        degChange = 315 - head
##        hdeg = [degChange]
##        headTurn = motorRotateDeg (hpower, hdeg, hmotor, sampling_time=0.0)
##        #print('Adjust head right : %d' % head)
##    
##    power=[100, 100] # 0 to 255
##    deg = [60, 60]
##
##     # 0 to 255
##    degreeVal = -45 * alternate
##    hdeg = [degreeVal]
##    headTurn = motorRotateDeg (hpower, hdeg, hmotor, sampling_time=0.0)
##    altCount += 1


# Old head turning alternation code.
##    if altCount >= 2:
##        alternate *= -1
##        altCount = 0


# Old head checking code.
#headPosition = (BrickPi.Encoder[hmotor[0]] % 720) / 2
#difference = 0
#print(headPosition)
#if headPosition > 300:
#    difference = 360 - headPosition
#else:
#    difference = headPosition * -1

#headTurn = motorRotateDeg ([30], [difference], hmotor, sampling_time=0.0)
#print('Head Position: %d' % ((BrickPi.Encoder[hmotor[0]] % 720)/2))
#time.sleep(1)

              # Old turning code  
##            # Turn right and check the distance.
##            turn_90(1, motors)
##            time.sleep(1)
##        
##            result = BrickPiUpdateValues()  # Ask BrickPi to update values for sensors/motors 
##            if not result :
##                distRight = BrickPi.Sensor[Sonic_Sensor_Port]
##            print (distRight)
##            time.sleep(1)
##
##            # Turn around (left) and check distance.
##            turn_180(-1, motors)
##            time.sleep(1)
##            
##            result = BrickPiUpdateValues()  # Ask BrickPi to update values for sensors/motors 
##            if not result :
##                distLeft = BrickPi.Sensor[Sonic_Sensor_Port]
##            print (distLeft)
##            time.sleep(1)
##
##            # If the right had more distance turn around and head in that direction.
##            if distLeft <= distRight:
##                turn_180(1, motors)
##            time.sleep(1)

                # Old head turning and distance cheacking code
##                BrickPiUpdateValues()
##                headPosition = ((BrickPi.Encoder[hmotor[0]] % 720) / 2)
##                difference = 0
##                print(headPosition)
##                if headPosition > 300:  
##                    difference = 360 - headPosition
##                else:
##                    difference = headPosition * -1
##                
##                headTurn = motorRotateDeg ([30], [difference], hmotor, sampling_time=0.0)
##                #print('Head Position: %d' % ((BrickPi.Encoder[hmotor[0]] % 720)/2))
##                
##                hdeg = [60]
##                headTurn = motorRotateDeg (hpower, hdeg, hmotor, sampling_time=0.0)
##                rightDist = BrickPi.Sensor[Sonic_Sensor_Port]
##
##                hdeg = [-120]
##                headTurn = motorRotateDeg (hpower, hdeg, hmotor, sampling_time=0.0)
##                
##                leftDist = BrickPi.Sensor[Sonic_Sensor_Port]
##
##                hdeg = [60]
##                headTurn = motorRotateDeg (hpower, hdeg, hmotor, sampling_time=0.0)

