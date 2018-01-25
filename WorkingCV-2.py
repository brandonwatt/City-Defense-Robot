# import the necessary packages
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
        # stage_20_upsidedown_cone.xml for upside down cones.
        face_cascade = cv2.CascadeClassifier('Ball_with_lines.xml')
        cone_cascade = cv2.CascadeClassifier('stage20upcone.xml')
        invert_cone = cv2.CascadeClassifier('stage_20_upsidedown_cone.xml') # cone.xml 
        # capture frames from the camera
        mode = 'fire'
        targets = 'sc'
        for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
                # grab the raw NumPy array representing the image, then initialize the timestamp
                # and occupied/unoccupied text
                image = frame.array
                gray = cv2.cvtColor(image,cv2.COLOR_BGR2GRAY)
                
                
                # 1.2, 2 best for stripped cone.
                #
                
                if mode == 'fire' and targets == 'sc':
                        faces = face_cascade.detectMultiScale(gray, 1.3, 3)
                        cones = cone_cascade.detectMultiScale(gray, 1.2, 3)
                elif mode == 'fire' and targets == 's':
                        faces = face_cascade.detectMultiScale(gray, 1.3, 3)
                elif mode == 'fire' and targets == 'c':
                        faces = cone_cascade.detectMultiScale(gray, 1.2, 3)
                elif mode == 'reload':
                        faces = invert_cone.detectMultiScale(gray, 1.2, 3)

                #print "Found "+str(len(faces))+" face(s)"
                #print "Found "+str(len(eyes))+" eyes(s)"

                if turretPipe.poll():
                        mode, targets = turretPipe.recv()

                fCount = 0
                for (x,y,w,h) in faces:
                        fCount += 1
                        cv2.rectangle(image,(x,y),(x+w,y+h),(255,255,0),2)
                        #roi_gray = gray[y:y+h, x:x+w]
                        #roi_color = image[y:y+h, x:x+w]
                        #eyes = eye_cascade.detectMultiScale(roi_gray, 1.1, 1)
                        
                        if len(faces) >= 1 and fCount == 1:
                                if targets == 's' or targets == 'sc':
                                        turretPipe.send(['s',x+w//2, y+h//2, width, height])
                                elif targets == 'c':
                                        turretPipe.send(['c',x+w//2, y+h//2, width, height])
                                else:
                                        turretPipe.send(['c',x+w//2, y+h//2, width, height])
                                        
                cCount = 0
                if mode == 'fire' and targets == 'sc':
                        for (x,y,w,h) in cones:
                                cCount += 1
                                cv2.rectangle(image,(x,y),(x+w,y+h),(255,100,100),2)
                                #roi_gray = gray[y:y+h, x:x+w]
                                #roi_color = image[y:y+h, x:x+w]
                                
                                if len(cones) >= 1 and len(faces) < 1 and cCount == 1:
                                        turretPipe.send(['c',x+w//2, y+h//2, width, height])
                                        #turretPipe.recv()     


                                                        
                # show the frame
                cv2.imshow("Frame", image)
                key = cv2.waitKey(1) & 0xFF
         
                # clear the stream in preparation for the next frame
                rawCapture.truncate(0)
         
                # if the `q` key was pressed, break from the loop
                if key == ord("q"):
                        break
        
        camera.close()
        #cv2.destroyAllWindows()

if __name__ == '__main__':
        ctTurretPipe, ctCameraPipe = Pipe()
        ntTurretPipe, ntNavPipe = Pipe()
        cnCameraPipe, cnNavPipe = Pipe()
    
        sphereSearch(ctCameraPipe, cnCameraPipe)
