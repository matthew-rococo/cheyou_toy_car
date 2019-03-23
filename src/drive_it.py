#!/usr/bin/env python
import rospy
import Adafruit_PCA9685 
from cheyou_toy_car.msg import MsgDriverCmd

pwm = Adafruit_PCA9685.PCA9685() 
pwm.set_pwm_freq(100) 
steer_channel=12
throttle_channel=15


def drive_with_cmd(throttle,steer):
    throttle_pulse_width=fix(2.048*throttle+614.6)
    steer_pulse_width=fix(2.048*steer+614.6)                                                    
    pwm.set_pwm(throttle_channel,0, throttle_pulse_width) 
    pwm.set_pwm(steer_channel,0, throttle_pulse_width)



def callback(MsgDriverCmd):
    throttle=MsgDriverCmd.throttle_pos
    steer=MsgDriverCmd.steer_pos

    rospy.loginfo(rospy.get_caller_id() + "throttle=%d steer=%d", throttle,steer)
    drive_with_cmd(throttle,steer)


def drive_it():
        rospy.init_node("drive_it")
        rospy.Subscriber("car_cmd",MsgDriverCmd,callback)
        rospy.spin()

if __name__ == '__main__':
    drive_it()        
