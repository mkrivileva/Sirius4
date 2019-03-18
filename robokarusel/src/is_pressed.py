#!/usr/bin/env python
import RPi.GPIO as GPIO
from robokarusel.srv import *
import rospy

def init():
	global PIN
	rospy.init_node('button_state_server')

	GPIO.setmode(GPIO.BOARD)
	GPIO.setwarnings(False)

	PIN = 33

	GPIO.setup(PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)

	#rospy.loginfo("GPIO are inited")

	a = rospy.Service('is_pressed', ButtonState, handle_state)
	rospy.loginfo("Service ButtonState is Inited")
	rospy.spin()

def handle_state(req):
	return GPIO.input(PIN)

if __name__ == '__main__':
	init()
