#!/usr/bin/python

# Import the necessary modules
from picamera.array import PiRGBArray
from picamera import PiCamera
import numpy
import cv2
import time

# ROS specific imports
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError


class ImageProcessing:

    def __init__(self):
        # Initialize node
        rospy.init_node('image_processing', anonymous=False)
        self.bridge = CvBridge()
        self.image_pub = rospy.Publisher("processed_image", Image)
        # Initialize the camera and grab a reference to the raw camera capture
        self.camera = PiCamera()
        self.camera.resolution = (640, 480)
        self.camera.framerate = 4
        self.rawCapture = PiRGBArray(camera, size=(640, 480))

        # Allow the camera to warmup
        time.sleep(0.1)

        self.img_width = 640
        self.img_height = 480

        # Define the regions of the image to analyse
        self.NUM_REGIONS = 10
        self.reg_vert_offset = 400
        self.reg_vert_width = 20
        self.reg_vert_end = self.reg_vert_offset + self.reg_vert_width
        self.reg_horiz_width = self.img_width/self.NUM_REGIONS
        self.reg_horiz_divs = numpy.arange(0, self.img_width+1, self.img_width/self.NUM_REGIONS)
        self.reg_num_pixels = 128*20
        self.intensity_threshold = 30

        self.GREEN = (0, 255, 0)
        self.RED = (0, 0, 255)

    def main(self):
        for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):

            image = frame.array
            # Our operations on the frame come here
            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

            # For each region of interest in the image
            for i in range(0, self.NUM_REGIONS):
                # Calculate average pixel intensity
                avg_int = numpy.median(gray[self.reg_vert_offset:self.reg_vert_end,
                                       self.reg_horiz_divs[i]:self.reg_horiz_divs[i+1]])
                # Select color for rectangle depending on pixel intensity
                reg_rect_color = RED if avg_int < self.intensity_threshold else GREEN
                # Draw rectangle
                cv2.rectangle(image, (self.reg_horiz_divs[i], self.reg_vert_offset),
                                     (self.reg_horiz_divs[i+1]-1, self.reg_vert_end-1), reg_rect_color, 1)

                self.image_pub.publish(self.bridge.cv2_to_imgmsg(image, "bgr8"))
                rawCapture.truncate(0)


if __name__ == '__main__':
    imageProcessing = ImageProcessing()
    imageProcessing.main()
