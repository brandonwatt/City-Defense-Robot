# Brandon Watt, Vlad Psarev and Isaiah Forward
# 2/2/17
# Embedded Systems Design
# Defend the City Robot Computer Vision Code
#
# The process will use openCV 2 to detect objects for the turret to track and
# fire at. The objects targeted will change as the turret uses up it's ammo. This information
# will be sent in the format [target type, x-coordinate of center, y-coordinate of center, width of frame, height of frame].



from picamera.array import PiRGBArray
from picamera import PiCamera
import time
import cv2
from multiprocessing import *


# Width and height of the display.
# Also the resolution of the display.
width = 304
height = 304

def sphereSearch(turretPipe, NavPipe):
        # initialize the camera and grab a reference to the raw camera capture
        camera = PiCamera()
        camera.resolution = (width, height)
        camera.framerate = 32
        camera.vflip = True
        rawCapture = PiRGBArray(camera, size=(width, height))
         
        # allow the camera to warmup 
        time.sleep(0.1)

        # Ball_with_lines.xml for stripped cone.
        # stage20upcone.xml for right side up cones.
        # stage_20_upsidedown_cone.xml for inverted cones.
        sphere_cascade = cv2.CascadeClassifier('Ball_with_lines.xml')
        cone_cascade = cv2.CascadeClassifier('stage20upcone.xml')
        invert_cone = cv2.CascadeClassifier('stage_20_upsidedown_cone.xml') 


        mode = 'fire'   # Start in 'fire' mode where the camera should find targets to shoot at.
        targets = 'sc'  # The default targets are spheres and cones.

        # capture frames from the camera
        for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
                # grab the raw NumPy array representing the image, then initialize the timestamp
                # and occupied/unoccupied text
                image = frame.array
                gray = cv2.cvtColor(image,cv2.COLOR_BGR2GRAY)

                # If in fire mode. 
                if mode == 'fire':
                        # If the targets are both spheres and cones, use both cascades.
                        if targets == 'sc':
                                primary = sphere_cascade.detectMultiScale(gray, 1.3, 3)
                                secondary = cone_cascade.detectMultiScale(gray, 1.2, 3)
                        # If the camera should only target spheres, use only the sphere casade.
                        elif targets == 's':
                                primary = sphere_cascade.detectMultiScale(gray, 1.3, 3)
                        # If the camera should only target cones, use only the cone casade.
                        else:
                                primary = cone_cascade.detectMultiScale(gray, 1.2, 3)
                # If the turret is out of ammo, look for the inverted cones.
                elif mode == 'reload':
                        primary = invert_cone.detectMultiScale(gray, 1.2, 3)

                # If there is mode and target information from the turret.
                if turretPipe.poll():
                        mode, targets = turretPipe.recv() # Update the mode and targets depending upon the turrets ammo.

                pCount = 0      # A count of the number of main targets found, only the first targets information
                                # will be sent to the turret.

                # Go through the list of primary targets.
                for (x,y,w,h) in primary:
                        pCount += 1
                        cv2.rectangle(image,(x,y),(x+w,y+h),(255,255,0),2) # Draw a light blue rectangle around the target.

                        # If this is the first primary target, send the turret information.
                        if len(primary) >= 1 and pCount == 1:
                                # If targeting spheres and cones, indicate that the target is a sphere.
                                if targets == 'sc':
                                        turretPipe.send(['s',x+w//2, y+h//2, width, height])
                                # All other cases the targets variable indicates the type of target.
                                else:
                                        turretPipe.send([targets, x+w//2, y+h//2, width, height])
                                        
                cCount = 0      # A count of the number of secondary targets found, only the first targets information
                                # will be sent to the turret.

                # Only look for secondary targets if the turret is targeting spheres and cones.
                if mode == 'fire' and targets == 'sc':
                        # Go through the list of secondary targets.
                        for (x,y,w,h) in secondary:
                                cCount += 1
                                cv2.rectangle(image,(x,y),(x+w,y+h),(255,100,100),2) # Draw a purple rectangle around the target.

                                # If this is the first secondary target and there are no primary targets,
                                # send the coordinate information to the turret.
                                if len(secondary) >= 1 and len(primary) < 1 and cCount == 1:
                                        turretPipe.send(['c',x+w//2, y+h//2, width, height])    


                                                        
                # show the frame
                cv2.imshow("Frame", image)
                key = cv2.waitKey(1) & 0xFF
         
                # clear the stream in preparation for the next frame
                rawCapture.truncate(0)
         
                # if the `q` key was pressed, break from the loop
                if key == ord("q"):
                        break
        
        # close the camera when the program is finished.
        camera.close()
        #cv2.destroyAllWindows()

# Main method, only used for individual testing.
if __name__ == '__main__':
        ctTurretPipe, ctCameraPipe = Pipe()
        ntTurretPipe, ntNavPipe = Pipe()
        cnCameraPipe, cnNavPipe = Pipe()
    
        sphereSearch(ctCameraPipe, cnCameraPipe)
