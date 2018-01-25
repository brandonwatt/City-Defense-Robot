# Brandon Watt, Vlad Psarev and Isaiah Forward
# 1/21/17
# Embedded Systems Design
# Defend the City Robot Turret Code
#
# Controls the movement and firing of the large turret and the
# firing of the small turret.


import time
import RPi.GPIO as GPIO

# Pins
# 7, 15, 18, 22
# 29, 31, 32, 33, 35, 36, 37, 38, 40


downPin = 40        # Moves the turret down, when high.
upPin = 38          # Moves the turret up, , when high.
downLimitPin = 37   # Goes high when the turret is at it's lowest position.
upLimitPin = 15     # Goes high when the turret is at it's highest position.


leftPin = 36        # Moves the turret left, when high.
rightPin = 32       # Moves the turret right, when high.
leftLimitPin = 7    # Goes high when the turret is at it's left most position.
rightLimitPin = 18  # Goes high when the turret is at it's right most position.

shootPin = 22       # Fires the turret, when high.
firedPin = 35       # Goes high when the turret has fired.

smallTurretPin = 19 # Pin that controls the firing of the small turret.

# Tests the movement and pins of the large turret.
def turretTest():
    GPIO.setwarnings(False)
    GPIO.setmode(GPIO.BOARD)

    # Set up and initialize the firing pins.
    GPIO.setup(firedPin, GPIO.IN, GPIO.PUD_DOWN)

    GPIO.setup(shootPin, GPIO.OUT)
    GPIO.output(shootPin, False)
    
    # Set up and initialize the down pins.
    GPIO.setup(downLimitPin, GPIO.IN, GPIO.PUD_DOWN)

    GPIO.setup(downPin, GPIO.OUT)
    GPIO.output(downPin, False)
    
    # Set up and initialize the up pins.
    GPIO.setup(upLimitPin, GPIO.IN, GPIO.PUD_DOWN)

    GPIO.setup(upPin, GPIO.OUT)
    GPIO.output(upPin, False)
    
    # Set up and initialize the left pins.
    GPIO.setup(leftLimitPin, GPIO.IN, GPIO.PUD_DOWN)

    GPIO.setup(leftPin, GPIO.OUT)
    GPIO.output(leftPin, False)

    # Set up and initialize the right pins.
    GPIO.setup(rightLimitPin, GPIO.IN, GPIO.PUD_DOWN)

    GPIO.setup(rightPin, GPIO.OUT)
    GPIO.output(rightPin, False)

    # Left Test
    while GPIO.input(leftLimitPin) == 0:
        GPIO.output(leftPin, True)
        print('Left {0}'.format(GPIO.input(leftLimitPin)))
        print('Left {0}, Right {1}, Up {2}, Down {3}, Shoot {4}'.format(GPIO.input(leftLimitPin), GPIO.input(rightLimitPin), GPIO.input(upLimitPin), GPIO.input(downLimitPin), GPIO.input(shootPin)))
    GPIO.output(leftPin, False)
    time.sleep(1)
    
    # Right Test
    while GPIO.input(rightLimitPin) == 0:
        GPIO.output(rightPin, True)
        print('Right {0}'.format(GPIO.input(rightLimitPin)))
        print('Left {0}, Right {1}, Up {2}, Down {3}, Shoot {4}'.format(GPIO.input(leftLimitPin), GPIO.input(rightLimitPin), GPIO.input(upLimitPin), GPIO.input(downLimitPin), GPIO.input(shootPin)))
    GPIO.output(rightPin, False)
    time.sleep(1)

    # Shoot Test
    st = time.time()
    wait = 2.91
    GPIO.output(shootPin, True)
    previousVal = 0
    #time.sleep(2.6)
    while not (previousVal == 1 and GPIO.input(firedPin) == 0):
        GPIO.output(shootPin, True)
        previousVal = GPIO.input(firedPin)
        print('Shoot {0}'.format(GPIO.input(firedPin)))
        print('Left {0}, Right {1}, Up {2}, Down {3}, Shoot {4}'.format(GPIO.input(leftLimitPin), GPIO.input(rightLimitPin), GPIO.input(upLimitPin), GPIO.input(downLimitPin), GPIO.input(shootPin)))
    GPIO.output(shootPin, False)
    time.sleep(1)
    
    # Down test
    while GPIO.input(downLimitPin) == 0:
        GPIO.output(downPin, True)
        print('Down {0}'.format(GPIO.input(downLimitPin)))
        print('Left {0}, Right {1}, Up {2}, Down {3}, Shoot {4}'.format(GPIO.input(leftLimitPin), GPIO.input(rightLimitPin), GPIO.input(upLimitPin), GPIO.input(downLimitPin), GPIO.input(shootPin)))
    GPIO.output(downPin, False)
    time.sleep(1)
    
    # Up Test
    while GPIO.input(upLimitPin) == 0:
        GPIO.output(upPin, True)
        print('Up {0}'.format(GPIO.input(upLimitPin)))
        print('Left {0}, Right {1}, Up {2}, Down {3}, Shoot {4}'.format(GPIO.input(leftLimitPin), GPIO.input(rightLimitPin), GPIO.input(upLimitPin), GPIO.input(downLimitPin), GPIO.input(shootPin)))
    GPIO.output(upPin, False)
    time.sleep(1)
    
    GPIO.cleanup()

# Returns the time it takes for the turret to fully rotate from
# the left most position to the right most position.
def getAngleToDelay():
    GPIO.setwarnings(False)
    GPIO.setmode(GPIO.BOARD)

    # Set up and initialize the left pins.
    GPIO.setup(leftLimitPin, GPIO.IN, GPIO.PUD_DOWN)

    GPIO.setup(leftPin, GPIO.OUT)
    GPIO.output(leftPin, False)
    
    # Set up and initialize the right pins.
    GPIO.setup(rightLimitPin, GPIO.IN, GPIO.PUD_DOWN)

    GPIO.setup(rightPin, GPIO.OUT)
    GPIO.output(rightPin, False)

    # Rotate all the way left
    while GPIO.input(leftLimitPin) == 0:
        GPIO.output(leftPin, True)
    GPIO.output(leftPin, False)
    time.sleep(1)

    # Rotate Right and time it
    st = time.time()
    while GPIO.input(rightLimitPin) == 0:
        GPIO.output(rightPin, True)
    GPIO.output(rightPin, False)
    et = time.time()
    time.sleep(1)

    GPIO.cleanup()
    timedif = et - st
    return timedif

# Returns the time it takes for the turret to fully rotate from
# the lowest position to the hightest position.
def upDownDelay():
    GPIO.setwarnings(False)
    GPIO.setmode(GPIO.BOARD)
    
    # Set up and initialize the down pins.
    GPIO.setup(downLimitPin, GPIO.IN, GPIO.PUD_DOWN)

    GPIO.setup(downPin, GPIO.OUT)
    GPIO.output(downPin, False)
    
    # Set up and initialize the up pins.
    GPIO.setup(upLimitPin, GPIO.IN, GPIO.PUD_DOWN)

    GPIO.setup(upPin, GPIO.OUT)
    GPIO.output(upPin, False)

    # Rotate the turret all the way down.
    while GPIO.input(downLimitPin) == 0:
        GPIO.output(downPin, True)
        #print('Down {0}'.format(GPIO.input(downLimitPin)))
    GPIO.output(downPin, False)
    time.sleep(1)
    
    # Rotate fully up and record the time it takes.
    st = time.time()
    while GPIO.input(upLimitPin) == 0:
        GPIO.output(upPin, True)
        #print('Up {0}'.format(GPIO.input(upLimitPin)))
    GPIO.output(upPin, False)
    et = time.time()
    time.sleep(1)

    GPIO.cleanup()
    timedif = et - st
    return timedif

# Returns the times it takes the turret to preform full
# vertical and horizontal rotations. Also centers the turret
# horizontally and vertically.
def calibrateAndCenter():
    GPIO.setwarnings(False)
    GPIO.setmode(GPIO.BOARD)

    t = getAngleToDelay()
    
    GPIO.setmode(GPIO.BOARD)

    # Set up and initialize the left pin.
    GPIO.setup(leftLimitPin, GPIO.IN, GPIO.PUD_DOWN)

    GPIO.setup(leftPin, GPIO.OUT)
    GPIO.output(leftPin, False)

    # Center the turret horizontally.
    GPIO.output(leftPin, True)
    time.sleep(t/2)
    GPIO.output(leftPin, False)

    u = upDownDelay()

    # Set up and initialize the down pin.
    GPIO.setmode(GPIO.BOARD)
    GPIO.setup(downPin, GPIO.OUT)
    GPIO.output(downPin, False)

    # Center the turret vertically.
    GPIO.output(downPin, True)
    time.sleep(u/2)
    GPIO.output(downPin, False)

    return t, u

# The targeting method of the turret. Receives information from the camera and moves
# the turret to center the target and fires when the target is centered.
def turretCommand(CameraPipe, NavPipe, firePipe):
    xtolerance = 16 # The tolerance for aiming in the x-axis.
    tolerance = 16 # The tolerance for aiming in the y-axis.
    yOffset = 35 # A pixel offset so that the turret will aim high enough to hit targets.

    bigAmmo = 4         # The number of large turret missiles.
    smallAmmo = 6       # The number of small turret missiles.
    reloadAmmo = False  # Indicate whether or not the robot needs to reload.
    
    wait = 5            # The wait interval for shooting. Prevents the queueing of fire commands.
    lastFireTime = time.time() - 5   # Tracks the last time the turret fired. Initialized to allow for immediate firing.

    vertAdjust = 2.3    # Adjustment for vertical movements, 1.5 for 200
    horiAdjust = 2.5    # Adjustments for horizontal movements 2.5 for 200

    initialLoop = True  # Inidicates that the method is in it's initial loop.

    # Holds the previous and current positions of an object.
    # Will only work if it is tracking a single object and not multiple.
    # NOT CURRENTLY IMPLEMENTED
    predictiveTrack = [[0,0],[0,0]] 
    xChange = 0
    yChange = 0
    
    # Initialize the mid points of the image.
    xMid = 100
    yMid = 100

    SHIFT = 0.005       # The delay time for the turret to do a shift when searching.
    targetWait = 3      # The amount of time without a target before the turret will start searching.
    lastTargetTime = time.time() - targetWait # The time that the camera last had a target. Initialized so that it will start searching.
    direction = True    # The direction the turret will be turning while searching. True is left, false is right.

    difFromCenter = 0.0 # A value used to track the position of the turret. Negative is left, positive is right and center is 0.

    horiTime, vertTime = calibrateAndCenter()

    centerTime = horiTime / 2 # The amount of time it takes the turret to rotate all the way left or all the way right from center.

    vertCenter = vertTime / 2 # The amount of time it takes the turret to rotate all the way up or all the way down from center.
    vertDir = True
    udOrLR = False
    
    adjustRatio = 4 # The amount the turret will adjust to a turn when in reload mode. 

    NavPipe.send('Calibration Complete') # Notify the turret that it can start running.
    
    GPIO.setwarnings(False)
    GPIO.setmode(GPIO.BOARD)
    
    # Set up and initialize the down pins.
    GPIO.setup(downLimitPin, GPIO.IN, GPIO.PUD_DOWN)

    GPIO.setup(downPin, GPIO.OUT)
    GPIO.output(downPin, False)
    
    # Set up and initialize the up pins.
    GPIO.setup(upLimitPin, GPIO.IN, GPIO.PUD_DOWN)

    GPIO.setup(upPin, GPIO.OUT)
    GPIO.output(upPin, False)
    
    # Set up and initialize the left pins.
    GPIO.setup(leftLimitPin, GPIO.IN, GPIO.PUD_DOWN)

    GPIO.setup(leftPin, GPIO.OUT)
    GPIO.output(leftPin, False)

    # Set up and initialize the right pins.
    GPIO.setup(rightLimitPin, GPIO.IN, GPIO.PUD_DOWN)

    GPIO.setup(rightPin, GPIO.OUT)
    GPIO.output(rightPin, False)

    # The main loop.
    while True:
        try:        
            # Tracking and firing.
            # If there is information from the camera.
            if CameraPipe.poll():
                t, x, y, xMax, yMax = CameraPipe.recv() # Receive information from the camera.
                predictiveTrack[0] = [x,y] # Save the current position.
                
                #print('x: {0}, y: {1}, xMax: {2}, yMax: {3}'.format(x, y, xMax, yMax))

                # If this is the first loop calculate the the midpoint.
                if initialLoop:
                    xMid = xMax // 2
                    yMid = (yMax // 2) + yOffset
                # If it is not the initial loop calculate the object movement.
                else:
                    xChange = predictiveTrack[0][0] - predictiveTrack[1][0]
                    yChange = predictiveTrack[0][1] - predictiveTrack[1][1]

                #print('x: {0}, y: {1}, xMax: {2}, yMax: {3}, xMid: {4}, yMid {5}'.format(x, y, xMax, yMax, xMid, yMid))        
                #print('Left {0}, Right {1}, Up {2}, Down {3}'.format(GPIO.input(leftLimitPin), GPIO.input(rightLimitPin), GPIO.input(upLimitPin), GPIO.input(downLimitPin)))

                # Will send the fire command if the target is roughly centered, and the turret isn't currently firing.
                if (y > yMid - tolerance and y < yMid + tolerance) and (x > xMid - xtolerance and x < xMid + xtolerance) and time.time() - lastFireTime > wait and not reloadAmmo:
                    firePipe.send(['Fire', t])

                    # Decriment the appropriate ammo.
                    if t == 's' and bigAmmo > 0:
                        bigAmmo -= 1
                    elif t == 'c' and smallAmmo > 0:
                        smallAmmo -= 1
                    
                    
                    # If there is no more ammo, enter the reload state.
                    if bigAmmo <= 0 and smallAmmo <= 0:
                        time.sleep(5)
                        reloadAmmo = True
                        CameraPipe.send(['reload', 'c'])
                        NavPipe.send('reload')
                    # If there is no more big ammo, only target cones.
                    elif bigAmmo <= 0:
                        CameraPipe.send(['fire', 'c'])
                    # If there is no more small ammo, only target spheres.
                    elif smallAmmo <= 0:
                        CameraPipe.send(['fire', 's'])

                    # Reset the fire time.
                    lastFireTime = time.time()

                    
                # The target is not centered so the turret should be adjusted.
                elif not reloadAmmo:

                    # The target is left and the turret has not fully rotated left.
                    if x > xMid and GPIO.input(leftLimitPin) == 0:
                        # Move left
                        xdif = x - xMid
                        GPIO.output(leftPin, True)
                        ratio = ((float(xdif)) / xMax) / horiAdjust #  + xChange
                        if ratio < 0:
                            ratio = 0
                        print('L ratio {0}'.format(ratio))
                        time.sleep(ratio)
                        GPIO.output(leftPin, False)

                        # Record the change in position.
                        # If the adjustment would put it past the max position, just set the value to the
                        # max position.
                        if difFromCenter - ratio <=  (-1*centerTime):
                            difFromCenter = -1 * centerTime
                        else:
                            difFromCenter -= ratio

                        
                    # The target is right and the turret has not fully rotated right.
                    elif x < xMid and GPIO.input(rightLimitPin) == 0:
                        # Move right
                        xdif = xMid - x
                        GPIO.output(rightPin, True)
                        ratio = ((float(xdif)) / xMax) / horiAdjust # - xChange
                        if ratio < 0:
                            ratio = 0
                        print('R ratio {0}'.format(ratio))
                        time.sleep(ratio)
                        GPIO.output(rightPin, False)

                        # Record the change in position.
                        # If the adjustment would put it past the max position, just set the value to the
                        # max position.
                        if difFromCenter + ratio >=  centerTime:
                            difFromCenter = centerTime
                        else:
                            difFromCenter += ratio
                        
                    # If the left or right limit pin is on, set the position to the
                    # respective max value.
                    if GPIO.input(leftLimitPin) == 1:
                        difFromCenter = -1 * centerTime
                    elif GPIO.input(rightLimitPin) == 1:
                        difFromCenter = centerTime
                    
                    # The target is down and the turret has not fully rotated down.
                    if y > yMid and GPIO.input(downLimitPin) == 0:
                        # Move down
                        ydif = y - yMid
                        GPIO.output(downPin, True)
                        ratio = ((float(ydif)) / yMax) / vertAdjust # + yChange
                        if ratio < 0:
                            ratio = 0
                        print('D ratio {0}'.format(ratio))
                        time.sleep(ratio)
                        GPIO.output(downPin, False)
                    # The target is up and the turret has not fully rotated up.
                    elif y < yMid and GPIO.input(upLimitPin) == 0:
                        # Move up
                        ydif = yMid - y
                        GPIO.output(upPin, True)
                        ratio = ((float(ydif))/ yMax) / vertAdjust #   - yChange
                        if ratio < 0:
                            ratio = 0
                        print('U ratio {0}'.format(ratio))
                        time.sleep(ratio)
                        GPIO.output(upPin, False)

                    # Consume unused navigation messages.
                    if NavPipe.poll():
                        NavPipe.recv()
                        
                elif reloadAmmo:
                    # The target is left and the turret has not fully rotated left.
                    if x > xMid and GPIO.input(leftLimitPin) == 0:
                        # Move left
                        xdif = x - xMid
                        GPIO.output(leftPin, True)
                        ratio = (float(xdif) / xMax) / horiAdjust
                        if ratio < 0:
                            ratio = 0
                        #print('L ratio {0}'.format(ratio))
                        time.sleep(ratio)
                        GPIO.output(leftPin, False)

                        if difFromCenter - ratio <=  (-1 * centerTime):
                            difFromCenter = -1 * centerTime
                        else:
                            difFromCenter -= ratio
                        
                    # The target is right and the turret has not fully rotated right.
                    elif x < xMid and GPIO.input(rightLimitPin) == 0:
                        # Move right
                        xdif = xMid - x
                        GPIO.output(rightPin, True)
                        ratio = (float(xdif) / xMax) / horiAdjust
                        #print('R ratio {0}'.format(ratio))
                        time.sleep(ratio)
                        GPIO.output(rightPin, False)

                        if difFromCenter + ratio >=  centerTime:
                            difFromCenter = centerTime
                        else:
                            difFromCenter += ratio

                    # The target is down and the turret has not fully rotated down.
                    if y > yMid and GPIO.input(downLimitPin) == 0:
                        # Move down
                        ydif = y - yMid
                        GPIO.output(downPin, True)
                        ratio = (float(ydif) / yMax) / vertAdjust
                        #print('D ratio {0}'.format(ratio))
                        time.sleep(ratio)
                        GPIO.output(downPin, False)
                    # The target is up and the turret has not fully rotated up.
                    elif y < yMid and GPIO.input(upLimitPin) == 0:
                        # Move up
                        ydif = yMid - y
                        GPIO.output(upPin, True)
                        ratio = (float(ydif) / yMax) / vertAdjust
                        #print('U ratio {0}'.format(ratio))
                        time.sleep(ratio)
                        GPIO.output(upPin, False)

                    NavPipe.send([difFromCenter, centerTime])

                    # Consume unused navigation messages.
                    if NavPipe.poll():
                        NavPipe.recv()

                    # Adjust the turret postion to compensate for vehicle movement
                    if difFromCenter >= 0 and difFromCenter < centerTime and GPIO.input(leftLimitPin) == 0:
                        
                        ratio = (abs(difFromCenter) / adjustRatio)
                        
                        
                        GPIO.output(leftPin, True)
                        if difFromCenter - ratio <=  (-1*centerTime):
                            ratio = abs(-1 * centerTime - difFromCenter)
                            #print('Compensate L ratio {0}'.format(ratio))
                            difFromCenter = -1 * centerTime
                            time.sleep(ratio)
                        else:
                            #print('Compensate L ratio {0}'.format(ratio))
                            difFromCenter -= ratio
                            time.sleep(ratio)

                        GPIO.output(leftPin, False)
                        
                    elif difFromCenter < 0 and difFromCenter > (-1*centerTime) and GPIO.input(rightLimitPin) == 0:
                        
                        ratio = (abs(difFromCenter) / adjustRatio)
                        
                        GPIO.output(rightPin, True)
                        if difFromCenter + ratio >=  centerTime:
                            ratio = centerTime - difFromCenter
                            #print('Compensate R ratio {0}'.format(ratio))
                            difFromCenter = centerTime
                            time.sleep(ratio)
                        else:
                            #print('Compensate R ratio {0}'.format(ratio))
                            difFromCenter += ratio
                            time.sleep(ratio)

                        GPIO.output(rightPin, False)
                lastTargetTime = time.time()
                
            # Searching
            elif not CameraPipe.poll() and time.time() - lastTargetTime > targetWait:
                # The scanning left and the turret has not fully rotated left.
                if GPIO.input(leftLimitPin) == 1:
                    direction = False
                elif  GPIO.input(rightLimitPin) == 1:
                    direction = True

                # The target is down and the turret has not fully rotated down.
                if GPIO.input(upLimitPin) == 1 and (GPIO.input(rightLimitPin) == 1 or GPIO.input(leftLimitPin) == 1) and udOrLR:
                    # Move down
                    GPIO.output(downPin, True)
                    st = time.time()
                    while GPIO.input(downLimitPin) == 0 and time.time()-st <= vertCenter:
                        GPIO.output(downPin, True)
                    GPIO.output(downPin, False)

                    udOrLR = False
                    vertDir = False
                    
                # The target is up and the turret has not fully rotated up.
                elif GPIO.input(downLimitPin) == 1  and (GPIO.input(rightLimitPin) == 1 or GPIO.input(leftLimitPin) == 1) and udOrLR:
                    # Move up
                    GPIO.output(upPin, True)
                    st = time.time()
                    while GPIO.input(upLimitPin) == 0 and time.time()-st <= vertCenter:
                        GPIO.output(upPin, True)

                    GPIO.output(upPin, False)

                    udOrLR = False
                    vertDir = True

                elif (GPIO.input(rightLimitPin) == 1 or GPIO.input(leftLimitPin) == 1) and udOrLR:
                    # Move up
                    GPIO.output(upPin, vertDir)
                    GPIO.output(downPin, (not vertDir))
                    st = time.time()
                    while (GPIO.input(upLimitPin) == 0 and GPIO.input(downLimitPin) == 0)and time.time()-st <= vertCenter:
                        GPIO.output(upPin, vertDir)
                        GPIO.output(downPin,  (not vertDir))

                    GPIO.output(upPin, False)
                    GPIO.output(downPin, False)
                    udOrLR = False

                #print('Direction ', direction, ' vertDir ', vertDir, ' right pin', GPIO.input(rightLimitPin), ' left pin ', GPIO.input(leftLimitPin))

                if direction and GPIO.input(leftLimitPin) == 0:
                    # Move left
                    GPIO.output(leftPin, True)
                    time.sleep(SHIFT)
                    GPIO.output(leftPin, False)

                    if GPIO.input(leftLimitPin) == 1:
                        udOrLR = True
                        
                    if difFromCenter - SHIFT <=  (-1 * centerTime):
                        difFromCenter = -1 * centerTime
                    else:
                        difFromCenter -= SHIFT
                    
                # The scanning right and the turret has not fully rotated right.
                elif GPIO.input(rightLimitPin) == 0:
                    # Move right
                    GPIO.output(rightPin, True)
                    time.sleep(SHIFT)
                    GPIO.output(rightPin, False)

                    if GPIO.input(rightLimitPin) == 1:
                        udOrLR = True
                        
                    if difFromCenter + SHIFT >=  centerTime:
                        difFromCenter = centerTime
                    else:
                        difFromCenter += SHIFT

                

##                elif not vertDir and (GPIO.input(rightLimitPin) == 1 or GPIO.input(leftLimitPin) == 1):
##                    # Move down
##                    GPIO.output(downPin, True)
##                    st = time.time()
##                    while GPIO.input(downLimitPin) == 0 and time.time()-st <= vertCenter:
##                        GPIO.output(downPin, True)
##                    GPIO.output(downPin, False)
                
                if GPIO.input(leftLimitPin) == 1:
                    difFromCenter = -1 * centerTime
                elif GPIO.input(rightLimitPin) == 1:
                    difFromCenter = centerTime

            
            elif NavPipe.poll() and not CameraPipe.poll():
                speed, direction = NavPipe.recv()

                if direction == 'left' and  GPIO.input(rightLimitPin) == 0:
                    
                    ratio = (float(speed) / 255)
                    
                    GPIO.output(rightPin, True)
                    if difFromCenter + ratio >=  centerTime:
                        ratio = abs(centerTime - difFromCenter)
                        #print('Nav R ratio {0}'.format(ratio))
                        difFromCenter = centerTime
                        time.sleep(ratio)
                    else:
                        #print('Nav R ratio {0}'.format(ratio))
                        difFromCenter += ratio
                        time.sleep(ratio)

                    GPIO.output(rightPin, False)
                    
                elif direction == 'right' and  GPIO.input(leftLimitPin) == 0:
                    
                    ratio = (float(speed) / 255)
                    
                    GPIO.output(leftPin, True)
                    if difFromCenter - ratio <=  (-1*centerTime):
                        ratio = abs(-1 * centerTime - difFromCenter)
                        #print('Nav L ratio {0}'.format(ratio))
                        difFromCenter = -1 * centerTime
                        time.sleep(ratio)
                    else:
                        #print('Nav L ratio {0}'.format(ratio))
                        difFromCenter -= ratio
                        time.sleep(ratio)
                        
                    GPIO.output(leftPin, False)
                    
                    
                # Save the current position as the previous position.
                predictiveTrack[1] = predictiveTrack[0] 

                
                
                # Indicate the completion of the first loop.
                initialLoop = False
            #CameraPipe.send('Fired!')
            
        except KeyboardInterrupt:
            break

    print('Turret control ending.')  
    firePipe.send(['stop', 'n'])
    GPIO.cleanup()

def fire(turret= 's'):
    wait = 5
    GPIO.setwarnings(False)
    GPIO.setmode(GPIO.BOARD)

    # Set up and initialize the firing pins.
    GPIO.setup(firedPin, GPIO.IN, GPIO.PUD_DOWN)

    GPIO.setup(shootPin, GPIO.OUT)
    GPIO.output(shootPin, False)

    GPIO.setup(smallTurretPin, GPIO.OUT)
    GPIO.output(smallTurretPin, False)
    
    if turret == 's':
        #print('Fire larger turret')
        previousVal = 0
        st = time.time()
        ct = time.time()
        GPIO.output(shootPin, True)
        
        while not (previousVal == 1 and GPIO.input(firedPin) == 0) and ct - st < wait:
            GPIO.output(shootPin, True)
            previousVal = GPIO.input(firedPin)
            ct = time.time()
            #print('Shoot {0}'.format(GPIO.input(firedPin)))
            #print('In loop')
        GPIO.output(shootPin, False)
        time.sleep(.3)
    elif turret == 'c':
        #print('Fire smaller turret')
        GPIO.output(smallTurretPin, True) # Change to match smaller turret pin.
        time.sleep(0.054)
        GPIO.output(smallTurretPin, False)
        
def fireCommand(commandPipe):

    GPIO.setwarnings(False)
    GPIO.setmode(GPIO.BOARD)
    
    # Set up and initialize the firing pins.
    GPIO.setup(firedPin, GPIO.IN, GPIO.PUD_DOWN)

    GPIO.setup(shootPin, GPIO.OUT)
    GPIO.output(shootPin, False)
    
    command = 'Hold'

    while not command == 'stop':

        command, t = commandPipe.recv()
        if command == 'Fire':
            #print(command)
            st = time.time()
            fire(t)
            if t == 's':
                GPIO.output(shootPin, True)
                time.sleep(2)
                GPIO.output(shootPin, False)
            et = time.time()
            #print(et-st)
            command = 'Hold'
        else:
            GPIO.output(shootPin, False)

    GPIO.cleanup()

# main method, only used for individual testing.
if __name__ == '__main__':
    GPIO.setwarnings(False)
    GPIO.setmode(GPIO.BOARD)

    turretTest()

    GPIO.cleanup()
