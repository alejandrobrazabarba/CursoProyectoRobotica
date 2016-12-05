#!/usr/bin/python

# Import the necessary modules
from picamera.array import PiRGBArray
from picamera import PiCamera
import numpy
import cv2
import time

# Initialize the camera and grab a reference to the raw camera capture
camera = PiCamera()
camera.resolution = (640, 480)
camera.framerate = 4
rawCapture = PiRGBArray(camera, size=(640, 480))

# Allow the camera to warmup
time.sleep(0.1)

img_width = 640
img_height = 480

# Define the regions of the image to analyse
NUM_REGIONS = 10
reg_vert_offset = 400
reg_vert_width = 20
reg_vert_end = reg_vert_offset + reg_vert_width
reg_horiz_width = img_width/NUM_REGIONS
reg_horiz_divs = numpy.arange(0,img_width+1,img_width/NUM_REGIONS)
reg_num_pixels = 128*20
intensity_threshold = 30

GREEN = (0,255,0)
RED = (0,0,255)

for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):

    image = frame.array
    # Our operations on the frame come here
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

    # For each region of interest in the image
    for i in range(0,NUM_REGIONS):
        # Calculate average pixel intensity
        avg_int = numpy.median( gray[reg_vert_offset:reg_vert_end, reg_horiz_divs[i]:reg_horiz_divs[i+1]] )
        # Select color for rectangle depending on pixel intensity
        reg_rect_color = RED if avg_int < intensity_threshold else GREEN
        # Draw rectangle
        cv2.rectangle(image, (reg_horiz_divs[i],reg_vert_offset),
                     (reg_horiz_divs[i+1]-1, reg_vert_end-1),reg_rect_color,1)

    # cv2.rectangle(frame, (330,0), (630,430),(0,255,0),3)
    # Histogram equalization
    # equR = cv2.equalizeHist(frame[:,:,2])
    # equG = cv2.equalizeHist(frame[:,:,1])
    # equB = cv2.equalizeHist(frame[:,:,0])
    # res = numpy.hstack((frame,cv2.merge((equB,equG,equR))))
    rawCapture.truncate(0)

    # Display the resulting frame come here
    cv2.imshow('Image', image)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cv2.destroyAllWindows()
