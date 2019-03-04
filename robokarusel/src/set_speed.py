#!/usr/bin/env python
import RPi.GPIO as GPIO
from robokarusel.srv import *
import rospy

def init():
    global IN2, IN3, IN4, right1,right2, left1, left2
    rospy.init_node('set_speed_server')

    GPIO.setmode(GPIO.BOARD)
    GPIO.setwarnings(False)

    IN1 = 37
    IN2 = 35

    IN3 = 40
    IN4 = 38

    GPIO.setup(IN1, GPIO.OUT)
    GPIO.setup(IN2, GPIO.OUT)

    GPIO.setup(IN3, GPIO.OUT)
    GPIO.setup(IN4, GPIO.OUT)

    right1 = GPIO.PWM(IN3, 1000)
    right2 = GPIO.PWM(IN4, 1000)

    left1 = GPIO.PWM(IN1, 1000)
    left2 = GPIO.PWM(IN2, 1000)


    rospy.loginfo("GPIO are inited")

    a = rospy.Service('set_speed', SetSpeed, handle_speed)
    rospy.loginfo("Service SetSpeed is Inited")
    rospy.spin()

def handle_speed(req):
    res = {
        'newRightSpeed': req.newRightSpeed,
        'newLeftSpeed': req.newLeftSpeed,
    }

    right1.start((abs(req.newRightSpeed)+req.newRightSpeed)/2)
    right2.start((abs(req.newRightSpeed)-req.newRightSpeed)/2)
    left1.start((abs(req.newLeftSpeed)+req.newLeftSpeed)/2)
    left2.start((abs(req.newLeftSpeed)-req.newLeftSpeed)/2)


    return res



if __name__ == "__main__":
	init()
