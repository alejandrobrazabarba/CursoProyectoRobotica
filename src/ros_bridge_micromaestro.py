#!/usr/bin/python

# system imports
import time
# micromaestro imports
import maestro
# ros specific imports
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32

class RosBridgeMicromaestro:
    def __init__(self):
        # Instantiate micromaestro controller.
        self.maestroController = maestro.Controller()

        # Channel assignment
        self.LEFT_SERVO = 4
        self.RIGHT_SERVO = 5
        self.SHARP_SENSOR = 0
        self.SHARP_CLB_A = 0
        self.SHARP_CLB_B = 0
        self.SHARP_CLB_C = 0
        self.period = 0.125
        self.vel_rot_desired = 0
        self.vel_trans_desired = 0
        self.vel_rot = 0
        self.vel_trans = 0
        self.k_rot = 0.5
        self.k_trans = 0.5
        self.wheel_basis = 16.438
        self.wheel_radius = 6.6675/2
        self.speed_cmd_received_flag = False
        self.periods_without_speed_cmd = 0
        # Number of periods we are going to wait before motors stop when
        # no speed cmd is recieved
        self.SAFE_STOP_WAITING_THRESHOLD = 1

    def stop_motors(self):
        self.maestroController.setTarget(self.LEFT_SERVO, 0)
        self.maestroController.setTarget(self.RIGHT_SERVO, 0)

    def speed_callback(self, speed_msg):
        self.vel_rot_desired = speed_msg.angular.z
        self.vel_trans_desired = speed_msg.msg.linear.x

        self.vel_trans = self.k_trans*(self.vel_trans_desired - self.vel_trans)
        self.vel_rot = self.k_rot*(self.vel_rot_desired - self.vel_rot)

        r_angular_speed = self.vel_trans/self.wheel_radius + self.vel_rot*self.wheel_basis/self.wheel_radius/2
        l_angular_speed = self.vel_trans/self.wheel_radius - self.vel_rot*self.wheel_basis/self.wheel_radius/2

        # set servo target accordingly to desired speed
        # we need to adjust this to compensate friction and allow
        # the robot to move at a known speed in m/s
        r_servo_target = r_angular_speed * 1
        l_servo_target = l_angular_speed * 1

        self.maestroController.setTarget(self.LEFT_SERVO, l_servo_target)
        self.maestroController.setTarget(self.RIGHT_SERVO, r_servo_target)

    def main(self):
        rospy.init_node('ros_bridge_micromaestro', anonymous=False)
        rospy.Subscriber("cmd_vel", Twist, self.speed_callback)
        sharp_pub = rospy.Publisher("sharp_distance", Int32, queue_size=10)
        rate = rospy.Rate(1 / self.period)

        while not rospy.is_shutdown():
            output = self.maestroController.getPosition(self.SHARP_SENSOR)
            # publish distance in cm using conversion from ADC reading to cm
            # with parameters obtained by calibration
            # distance = SHARP_CAL_A / (output-SHARP_CLB_B)-SHARP_CLB_C
            # publish raw ADC reading from sensor
            distance = output
            sharp_pub.publish(distance)

            # Check if we are not receiving speed commands, so we need to stop the motors
            if not self.speed_cmd_received_flag:
                self.periods_without_speed_cmd += 1
                if self.periods_without_speed_cmd==1:
                    self.stop_motors()
            else:
                self.speed_cmd_received_flag = False
            rate.sleep()

        # Stop motors before exit the program
        self.stop_motors()


if __name__ == '__main__':
    rosBridgeMicromaestro = RosBridgeMicromaestro()
    rosBridgeMicromaestro.main()
