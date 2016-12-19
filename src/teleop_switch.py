#!/usr/bin/python

import rospy
from proyecto_curso_robotica.srv import *
from sensor_msgs.msg import Joy

class TeleopSwitch:
    def __init__(self):
        rospy.init_node('teleop_switch', anonymous=False)
        self.img_proc_state = False
        self.img_broadcast_state = True
        self.procButtonLastState = 0
        self.broadButtonLastState = 0
        self.turn_on_off_img_proc = rospy.ServiceProxy('image_proc_on_off', ImgProcTurnOnOff)
        turn_on_off_img_broadcast = rospy.ServiceProxy('image_broadcast_on_off', ImgBroadcastTurnOnOff)
        
        self.procButtonIndex = rospy.get_param('~mode_switch_button', '2')
        self.broadButtonIndex = rospy.get_param('~broadcast_button', '3')
        
        self.joy_sub = rospy.Subscriber("joy", Joy, self.joy_callback)
        

    def joy_callback(self,data):
        procButtonState = data.buttons[self.procButtonIndex]
        broadButtonState = data.buttons[self.broadButtonIndex]
        if procButtonState == 1 and (self.procButtonLastState == 0):
            try:
                self.turn_on_off_img_proc(not self.img_proc_state)
                self.img_proc_state = not self.img_proc_state
            except rospy.ServiceException, e:
                pass
        if broadButtonState == 1 and (self.broadButtonLastState == 0):
            try:
                self.turn_on_off_img_broadcast(not self.img_broadcast_state)
                self.img_broadcast_state = not self.img_broadcast_state
            except rospy.ServiceException, e:
                pass 
        self.procButtonLastState = procButtonState
        self.broadButtonLastState = broadButtonState   

if __name__ == '__main__':
    teleopSwitch = TeleopSwitch()
    
    rospy.spin()
