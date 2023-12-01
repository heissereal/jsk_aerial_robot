#!/usr/bin/env python

import rospy
from std_msgs.msg import UInt8
from spinal.msg import PwmState

from sensor_msgs.msg import Joy

class Perching():
    def __init__(self):
        # self.flight_state_sub = rospy.Subscriber('/quadrotor/flight_state',UInt8,self.flight_state_cb)
        # self.flight_state_msg = UInt8()
        # self.flight_state_flag = False

        self.PWM_pub = rospy.Publisher('/pwm_indiv_test',PwmState,queue_size=1)
        self.PWM_msg = PwmState()
        self.PWM = 0.6
        self.init_PWM = 0.6

        # self.joy_sub = rospy.Subscriber('/joy', Joy, self.joy_cb)
        # self.joy = Joy()
        
        ##self.timer = rospy.Timer(rospy.Duration(0.05),self.timerCallback)

    def flight_state_cb(self,msg):
        self.flight_state_msg = msg

    def joy_cb(self,msg):
        self.joy = msg

    # def flight_state(self):
    #     if self.flight_state_msg.data == 5:
    #         self.flight_state_flag = True
    #     else:
    #         self.flight_state_flag = False


    def stop_propeller_start_pump(self):
        # self.PWM_msg.index = [0,1,2,3,4]
        # self.PWM_msg.percentage = [0.6,0.6,0.6,0.6,0.3]
        self.PWM_msg.index = []
        self.PWM_msg.percentage = []
        for i in range(5):
            self.PWM_msg.index.append(i)
            self.PWM_msg.percentage.append(self.init_PWM)
        print(self.PWM_msg.percentage)
        self.PWM_pub.publish(self.PWM_msg)

    def stop_pump(self):
        self.PWM_msg.index= [4]
        self.PWM_msg.percentage = [0.0]
        self.PWM_pub.publish(self.PWM_msg)


    def main(self):
        while not rospy.is_shutdown():
            # self.flight_state()
            # if self.flight_state_flag:
            #     if self.joy.buttons[3] == 1:
            rospy.sleep(0.3)
            self.stop_propeller_start_pump()
            rospy.sleep(2.0)
            self.stop_pump() #push button and stop 
            rospy.sleep(1.0)
            break

if __name__ == '__main__':
    rospy.init_node("Perching")
    node = Perching()
    node.main()
