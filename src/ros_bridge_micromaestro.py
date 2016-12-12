#!/usr/bin/python

# system imports
import time
# micromaestro imports
import maestro
# ros specific imports
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32

from threading import Lock
from threading import Timer

class RosBridgeMicromaestro:
    def __init__(self):
        # Instantiate micromaestro controller.
        self.maestroController = maestro.Controller()

        # Channel assignment
        self.LEFT_SERVO = 4
        self.RIGHT_SERVO = 5
        self.SHARP_SENSOR = 0
        # Parameters obtained from a least-square fitting using Matlab
        # from pairs of values (distance , reading)
        self.SHARP_P1 = 1352000.0
        self.SHARP_Q1 = 26380.0
        self.SHARP_Q2 = 76730.0
        self.SHARP_DISPLACEMENT = 0
        self.period = 0.125
        self.vel_rot_desired = 0
        self.vel_trans_desired = 0
        self.vel_rot = 0
        self.vel_trans = 0
        self.k_rot = 0.5
        self.k_trans = 0.5
        self.wheel_basis = 0.16438
        self.wheel_radius = 0.066675/2
        self.speed_cmd_received_flag = False
        self.periods_without_speed_cmd = 0
        # Number of periods we are going to wait before motors stop when
        # no speed cmd is received
        self.SAFE_STOP_WAITING_THRESHOLD = 0
        self.SerialAccessLock = Lock()
        self.NullSpeedCmd = Twist()
        self.NullSpeedCmd.linear.x = 0
        self.NullSpeedCmd.angular.z = 0

        rospy.init_node('ros_bridge_micromaestro', anonymous=False)
        rospy.Subscriber("cmd_vel", Twist, self.speed_callback)
        self.sharp_pub = rospy.Publisher("sharp_distance", Int32, queue_size=10)
        self.sharp_raw_pub = rospy.Publisher("sharp_raw", Int32, queue_size=10)
        self.r_servo_target_pub = rospy.Publisher("r_servo_target", Int32, queue_size=10)
        self.l_servo_target_pub = rospy.Publisher("l_servo_target", Int32, queue_size=10)

        self.timer = None

    def stop_motors(self):
        self.maestroController.setTarget(self.LEFT_SERVO, 0)
        self.maestroController.setTarget(self.RIGHT_SERVO, 0)
    # We use speed_callback for the subscriber of external speed commands and
    # from a timer that reduces the speed of the motors when no external command
    # is received
    def speed_callback(self, speed_msg, external_cmd=True):
        apply_motor_speed_cmd = True
        if external_cmd :
            self.speed_cmd_received_flag = True
        else:
            if not self.speed_cmd_received_flag :
                apply_motor_speed_cmd = True
            else:
                apply_motor_speed_cmd = False
                self.speed_cmd_received_flag = False

        if apply_motor_speed_cmd :
            self.vel_rot_desired = speed_msg.angular.z
            self.vel_trans_desired = speed_msg.linear.x

            # self.vel_trans = self.k_trans*(self.vel_trans_desired - self.vel_trans)
            # self.vel_rot = self.k_rot*(self.vel_rot_desired - self.vel_rot)

            self.vel_trans = 0.4*self.vel_trans + 0.6*self.vel_trans_desired
            self.vel_rot = 0.4*self.vel_rot + 0.6*self.vel_rot_desired

            r_angular_speed = self.vel_trans/self.wheel_radius + self.vel_rot*self.wheel_basis/self.wheel_radius/2
            l_angular_speed = self.vel_trans/self.wheel_radius - self.vel_rot*self.wheel_basis/self.wheel_radius/2

            # set servo target accordingly to desired speed
            # we need to adjust this to compensate friction and allow
            # the robot to move at a known speed in m/s
            # We know that the maximum PWM increment with respect to center position (1500 us),
            # for which the motor speed will saturate is approximately 340 us, that multiplied by 4 is 1360
            # We also know that, the maximum speed will be 0.19 m/s approx, and with the radius of the wheel,
            # that corresponds to an angular speed of 5.699287589 rad/s
            # So, for obtaining the servo target that we will command, we'll use the next equation,
            # supposing that the system is linear, which is not true
            # servo_target = center_position_us * 4 - angular_speed * Constant
            # making a substitution with the values corresponding to max speed, we can extract the constant
            # max_servo_target = center_position_us * 4 - max_angular_speed * constant
            # constant = (center_position_us * 4 - max_servo_target) / max_angular_speed
            # constant = (6000 - 4640) / 5.699 = 238.638
            r_servo_target = 6000 - r_angular_speed * 238.638
            l_servo_target = 6000 - l_angular_speed * 238.638

            # saturate servo target values
            if r_servo_target > 9000:
                r_servo_target = 9000
            elif r_servo_target < 3000:
                r_servo_target = 3000
            if l_servo_target > 9000:
                l_servo_target = 9000
            elif l_servo_target < 3000:
                l_servo_target = 3000

            self.r_servo_target_pub.publish(int(r_servo_target))
            self.l_servo_target_pub.publish(int(l_servo_target))

            with self.SerialAccessLock:
                self.maestroController.setTarget(self.LEFT_SERVO, int(l_servo_target))
                self.maestroController.setTarget(self.RIGHT_SERVO, int(r_servo_target))

        # Program next timed execution of the callback
        if not external_cmd :
            # If the flag of received cmd is down we need to reduce motor speed as soon as posible
            if apply_motor_speed_cmd :
                self.timer = Timer(0.25, self.speed_callback, [self.NullSpeedCmd, False])
                self.timer.start()
            # If the flag is up we don't need to hurry
            else:
                self.timer = Timer(1.0, self.speed_callback, [self.NullSpeedCmd, False])
                self.timer.start()
            
    def main(self):

        rate = rospy.Rate(1 / self.period)
        # We program the execution of speed_callback with a null speed command
        self.timer = Timer(1.0, self.speed_callback, args=[ self.NullSpeedCmd, False ])
        self.timer.start()
        
        while not rospy.is_shutdown():
            with self.SerialAccessLock:
                output = self.maestroController.getPosition(self.SHARP_SENSOR)
            # Publish raw ADC reading from sensor
            self.sharp_raw_pub.publish(output)
            # publish distance in cm using conversion from ADC reading to cm
            distance = 100 * (self.SHARP_P1 / (output**2 - 2*self.SHARP_DISPLACEMENT*output
                                               + self.SHARP_DISPLACEMENT**2
                                               + self.SHARP_Q1*output
                                               - self.SHARP_Q1*self.SHARP_DISPLACEMENT
                                               + self.SHARP_Q2))
            # distance = output
            self.sharp_pub.publish(distance)

            # Check if we are not receiving speed commands, so we need to stop the motors
            # if not self.speed_cmd_received_flag:
            #    self.periods_without_speed_cmd += 1
            #    if self.periods_without_speed_cmd == self.SAFE_STOP_WAITING_THRESHOLD:
            #        self.stop_motors()
            #        self.periods_without_speed_cmd = 0
            # else:
            #    self.speed_cmd_received_flag = False
            # rate.sleep()

        # Stop motors before exit the program
        self.stop_motors()
        self.timer.cancel()


if __name__ == '__main__':
    rosBridgeMicromaestro = RosBridgeMicromaestro()
    rosBridgeMicromaestro.main()
