#!/usr/bin/python
"""
This program is demonstration for face and object detection using haar-like features.
The program finds faces in a camera image or video stream and displays a red box around them,
then centers the webcam via two servos so the face is at the center of the screen
Based on facedetect.py in the OpenCV samples directory
"""
import sys
from optparse import OptionParser
import serial
import cv2.cv as cv
 
# Parameters for haar detection
# From the API:
# The default parameters (scale_factor=2, min_neighbors=3, flags=0) are tuned
# for accurate yet slow object detection. For a faster operation on real video
# images the settings are:
# scale_factor=1.2, min_neighbors=2, flags=CV_HAAR_DO_CANNY_PRUNING,
# min_size=<minimum possible face size
 
min_size = (20, 20)
image_scale = 2
haar_scale = 1.2
min_neighbors = 2
haar_flags = cv.CV_HAAR_DO_CANNY_PRUNING

max_pwm = 180
min_pwm = 1
midScreenWindow = 40  # acceptable 'error' for the center of the screen.
panStepSize = 1 # degree of change for each pan update
tiltStepSize = -1 # degree of change for each tilt update
servoPanPosition = 90 # initial pan position
servoTiltPosition = 160 # initial tilt position
panGpioPin = 0  # arduino pan servo id (NOT pin number)
tiltGpioPin = 1  # arduino tilt servo id (NOT pin number)
 
def detect_and_draw(img, cascade):
    gray = cv.CreateImage((img.width,img.height), 8, 1)
    small_img = cv.CreateImage((cv.Round(img.width / image_scale),
                   cv.Round (img.height / image_scale)), 8, 1)
 
    # convert color input image to grayscale
    cv.CvtColor(img, gray, cv.CV_BGR2GRAY)
 
    # scale input image for faster processing
    cv.Resize(gray, small_img, cv.CV_INTER_LINEAR)
 
    cv.EqualizeHist(small_img, small_img)
 
    midFace = None
 
    if(cascade):
        t = cv.GetTickCount()
        # HaarDetectObjects takes 0.02s
        faces = cv.HaarDetectObjects(small_img, cascade, cv.CreateMemStorage(0),
                                     haar_scale, min_neighbors, haar_flags, min_size)
        t = cv.GetTickCount() - t
        if faces:
            for ((x, y, w, h), n) in faces:
                # the input to cv.HaarDetectObjects was resized, so scale the
                # bounding box of each face and convert it to two CvPoints
                pt1 = (int(x * image_scale), int(y * image_scale))
                pt2 = (int((x + w) * image_scale), int((y + h) * image_scale))
                cv.Rectangle(img, pt1, pt2, cv.RGB(255, 0, 0), 3, 8, 0)
                # get the xy corner co-ords, calc the midFace location
                x1 = pt1[0]
                x2 = pt2[0]
                y1 = pt1[1]
                y2 = pt2[1]
                midFaceX = x1+((x2-x1)/2)
                midFaceY = y1+((y2-y1)/2)
                midFace = (midFaceX, midFaceY)
 
    cv.ShowImage("result", img)
    return midFace
    
def move(servo, angle):
    '''Moves the specified servo to the supplied angle.

    Arguments:
        servo
          the servo number to command, an integer from 1-4
        angle
          the desired servo angle, an integer from 0 to 180

    (e.g.) >>> servo.move(2, 90)
           ... # "move servo #2 to 90 degrees"'''

    if (min_pwm <= angle <= max_pwm):
        ser.write(chr(255))
        ser.write(chr(servo))
        ser.write(chr(angle))
    else:
        print "Servo angle must be an integer between 0 and 180.\n"
 
if __name__ == '__main__':
    #ser=serial.Serial(port='/dev/ttyUSB0',baudrate=9600,timeout=1)
    ser=serial.Serial(port='/dev/ttyACM0',baudrate=9600,timeout=1)
 
    # parse cmd line options, setup Haar classifier
    parser = OptionParser(usage = "usage: %prog [options] [camera_index]")
    parser.add_option("-c", "--cascade", action="store", dest="cascade", type="str", help="Haar cascade file, default %default", default = "./haarcascade_frontalface_alt.xml")
    (options, args) = parser.parse_args()
 
    cascade = cv.Load(options.cascade)
 
    if len(args) != 1:
        parser.print_help()
        sys.exit(1)
 
    input_name = args[0]
    if input_name.isdigit():
        capture = cv.CreateCameraCapture(int(input_name))
    else:
        print "We need a camera input! Specify camera index e.g. 0"
        sys.exit(0)
 
    cv.NamedWindow("result", 1)
 
    if capture:
        frame_copy = None
        
        move(panGpioPin, servoPanPosition)
        move(tiltGpioPin, servoTiltPosition)
 
        while True:
            frame = cv.QueryFrame(capture)
            if not frame:
                cv.WaitKey(0)
                break
            if not frame_copy:
                frame_copy = cv.CreateImage((frame.width,frame.height),
                                            cv.IPL_DEPTH_8U, frame.nChannels)
            if frame.origin == cv.IPL_ORIGIN_TL:
                cv.Copy(frame, frame_copy)
            else:
                cv.Flip(frame, frame_copy, 0)
            
            midScreenX = (frame.width/2)
            midScreenY = (frame.height/2)
  
            midFace = detect_and_draw(frame_copy, cascade)
            
            if midFace is not None:
                midFaceX = midFace[0]
                midFaceY = midFace[1]
                
                #Find out if the X component of the face is to the left of the middle of the screen.
                if(midFaceX < (midScreenX - midScreenWindow)):
                    #Update the pan position variable to move the servo to the right.
                    servoPanPosition += panStepSize
                    print str(midFaceX) + " > " + str(midScreenX) + " : Pan Right : " + str(servoPanPosition)
                #Find out if the X component of the face is to the right of the middle of the screen.
                elif(midFaceX > (midScreenX + midScreenWindow)):
                    #Update the pan position variable to move the servo to the left.
                    servoPanPosition -= panStepSize
                    print str(midFaceX) + " < " + str(midScreenX) + " : Pan Left : " + str(servoPanPosition)
                else:
                    print str(midFaceX) + " ~ " + str(midScreenX) + " : " + str(servoPanPosition)
                
                servoPanPosition = min(servoPanPosition, max_pwm)
                servoPanPosition = max(servoPanPosition, min_pwm)               
                move(panGpioPin, servoPanPosition)
                
                #Find out if the Y component of the face is below the middle of the screen.
                if(midFaceY < (midScreenY - midScreenWindow)):
                    if(servoTiltPosition <= max_pwm):
                        #Update the tilt position variable to lower the tilt servo.
                        servoTiltPosition -= tiltStepSize
                        print str(midFaceY) + " > " + str(midScreenY) + " : Tilt Down : " + str(servoTiltPosition)
                #Find out if the Y component of the face is above the middle of the screen.
                elif(midFaceY > (midScreenY + midScreenWindow)):
                    if(servoTiltPosition >= 1):
                        #Update the tilt position variable to raise the tilt servo.
                        servoTiltPosition += tiltStepSize
                        print str(midFaceY) + " < " + str(midScreenY) + " : Tilt Up : " + str(servoTiltPosition)
                else:
                    print str(midFaceY) + " ~ " + str(midScreenY) + " : " + str(servoTiltPosition)
                
                servoTiltPosition = min(servoTiltPosition, max_pwm)
                servoTiltPosition = max(servoTiltPosition, min_pwm)  
                move(tiltGpioPin, servoTiltPosition)
                
            if cv.WaitKey(10) >= 0: # 10ms delay
                break
 
    cv.DestroyWindow("result")
