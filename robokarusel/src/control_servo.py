#!/usr/bin/env python
import RPi.GPIO as GPIO
import time
from robokarusel.srv import *
import rospy

def handle(req):
	if req.direction == "center":
		p.ChangeDutyCycle(9)
		time.sleep(0.5)
		#p.ChangeDutyCycle(7.5)
		#time.sleep(1)
		p.ChangeDutyCycle(0)
		rospy.loginfo("center")
	elif req.direction == "right":
		p.ChangeDutyCycle(2.5)
		time.sleep(.5)
		p.ChangeDutyCycle(0)
		rospy.loginfo("right")
	elif req.direction == "left":
		p.ChangeDutyCycle(12.5)
		time.sleep(.5)
		p.ChangeDutyCycle(0)
		rospy.loginfo("left")

	res = True

	return res


def init():

	global p
	rospy.init_node('control_servo_server')
	GPIO.setmode(GPIO.BOARD)
	GPIO.setwarnings(False)
	PIN = 32
	GPIO.setup(PIN, GPIO.OUT)
	p = GPIO.PWM(PIN, 50)
	p.start(7.5)
	time.sleep(.5)
	p.ChangeDutyCycle(0)

	#rospy.loginfo("GPIO are inited")

	a = rospy.Service('control_servo', ControlServo, handle)
	rospy.loginfo("Service ControlServo is Inited")
	rospy.spin()


if __name__ == "__main__":
	init()
