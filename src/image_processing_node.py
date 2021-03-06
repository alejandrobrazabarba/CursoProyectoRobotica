#!/usr/bin/python

# Import the necessary modules
from picamera.array import PiRGBArray
from picamera import PiCamera
import numpy
import cv2
import time
# from datetime import datetime

# ROS specific imports
import rospy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge, CvBridgeError
from proyecto_curso_robotica.srv import *

class ImageProcessing:
    def __init__(self):
        # Initialize node
        rospy.init_node('image_processing', anonymous=False)
        self.bridge = CvBridge()
        self.image_pub = rospy.Publisher("processed_image", Image, queue_size=2)
        self.cmd_vel_pub = rospy.Publisher("cmd_vel", Twist, queue_size=1)

        self.img_broadcast_on = rospy.get_param('~broadcast_initial_status', 'False')
        self.img_proc_on = False

        img_broadcast_srv = rospy.Service('~image_broadcast_on_off', ImgBroadcastTurnOnOff,
                                          self.handle_img_broadcast_turn_on_off)
        img_proc_srv = rospy.Service('~image_proc_on_off', ImgProcTurnOnOff,
                                     self.handle_img_proc_turn_on_off)

        # Initialize the camera and grab a reference to the raw camera capture
        self.camera = PiCamera()
        self.camera.vflip = True
        self.camera.hflip = False
        self.camera.resolution = (640, 480)
        self.camera.framerate = rospy.get_param('~framerate', '4')
        self.rawCapture = PiRGBArray(self.camera, size=(640, 480))

        # Allow the camera to warmup
        time.sleep(0.1)

        self.img_width = 640
        self.img_height = 480

        # Define the regions of the image to analyse
        self.NUM_REGIONS = 10
        self.reg_vert_offset = rospy.get_param('~reg_vert_offset', '400')
        self.reg_vert_width = 20
        self.reg_vert_end = self.reg_vert_offset + self.reg_vert_width
        self.reg_horiz_width = self.img_width/self.NUM_REGIONS
        self.reg_horiz_divs = numpy.arange(0, self.img_width+1, self.img_width/self.NUM_REGIONS)
        self.reg_num_pixels = 128*20
        self.intensity_threshold = rospy.get_param('~intensity_threshold','30')

        self.threshold_method = rospy.get_param('~threshold_method','median')

        self.GREEN = (0, 255, 0)
        self.RED = (0, 0, 255)

        self.regionLineDetectedFlags = numpy.zeros(self.NUM_REGIONS, dtype=numpy.uint8)
        self.twistCommand = Twist()

    def handle_img_broadcast_turn_on_off(self, req):
        self.img_broadcast_on = req.on
        return ImgBroadcastTurnOnOffResponse()

    def handle_img_proc_turn_on_off(self, req):
        self.img_proc_on = req.on
        return ImgProcTurnOnOffResponse()

    def main(self):
        # previous_time = datetime.now()
        for frame in self.camera.capture_continuous(self.rawCapture, format="bgr", use_video_port=True):
            # current_time = datetime.now()
            # elapsed_time = current_time - previous_time
            # previous_time = current_time
            # print "Elapsed time: ", elapsed_time
            image = frame.array
            image_available = False
            if self.img_proc_on or self.img_broadcast_on:
                # The goal of this flag is to protect the code against the case
                # where the flag self.img_proc_on is changed between the execution of this block of code
                # and the one below the next if statement
                # in that case we would send a speed command based on a previous line detection
                image_available = True
                # Our operations on the frame come here
                gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

                # Apply Otsu's binarization
                ret, thresh_img = cv2.threshold(gray,0,255,cv2.THRESH_BINARY+cv2.THRESH_OTSU)

                # For each region of interest in the image
                for i in range(0, self.NUM_REGIONS):
                    if self.threshold_method == 'median':
                        # Calculate average pixel intensity
                        avg_int = numpy.median(thresh_img[self.reg_vert_offset:self.reg_vert_end,
                                                          self.reg_horiz_divs[i]:self.reg_horiz_divs[i+1]])
                    elif self.threshold_method == 'average':
                        avg_int = numpy.average(thresh_img[self.reg_vert_offset:self.reg_vert_end,
                                                           self.reg_horiz_divs[i]:self.reg_horiz_divs[i+1]])
                    else:
                        # If the parameter doesn't match any of the possible options, it defaults to average
                        avg_int = numpy.average(thresh_img[self.reg_vert_offset:self.reg_vert_end,
                                                           self.reg_horiz_divs[i]:self.reg_horiz_divs[i+1]])
                    # Select color for rectangle depending on pixel intensity
                    self.regionLineDetectedFlags[i] = 1 if avg_int < self.intensity_threshold else 0
                    
                    if self.img_broadcast_on:
                        reg_rect_color = self.RED if avg_int < self.intensity_threshold else self.GREEN
                        # Draw rectangle
                        cv2.rectangle(image, (self.reg_horiz_divs[i], self.reg_vert_offset),
                                            (self.reg_horiz_divs[i+1]-1, self.reg_vert_end-1), reg_rect_color, 1)
            if self.img_proc_on and image_available:
                self.calc_and_send_speed_cmd()

            # print "Publishing image"
            if self.img_broadcast_on:
                self.image_pub.publish(self.bridge.cv2_to_imgmsg(image, "bgr8"))
                #self.image_pub.publish(self.bridge.cv2_to_imgmsg(thresh_img, "mono8"))
            self.rawCapture.truncate(0)
            if rospy.is_shutdown():
                break

    def calc_and_send_speed_cmd(self):
        leftRegionsOccupied = numpy.sum(self.regionLineDetectedFlags[0:self.NUM_REGIONS/2])
        rightRegionsOccupied = numpy.sum(self.regionLineDetectedFlags[self.NUM_REGIONS/2:self.NUM_REGIONS])
    
        self.twistCommand.linear.x = 0.1
        if leftRegionsOccupied > rightRegionsOccupied:
            self.twistCommand.angular.z = 0.2
        elif leftRegionsOccupied < rightRegionsOccupied:
            self.twistCommand.angular.z = -0.2
        else:
            self.twistCommand.angular.z = 0

        self.cmd_vel_pub.publish(self.twistCommand)

if __name__ == '__main__':
    imageProcessing = ImageProcessing()
    imageProcessing.main()
