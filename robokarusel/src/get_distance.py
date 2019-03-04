#!/usr/bin/env python
import time
import threading
import pigpio
import collections
import numpy
import rospy
from line.srv import *

def init():
    global pi, done, low,history
    pi = pigpio.pi()
    done = threading.Event()
    low = 0.0
    history = collections.deque(maxlen=10)
    rospy.init_node('get_distance_server')

    global TRIG, ECHO
    TRIG = 23
    ECHO = 18
    pi.set_mode(TRIG, pigpio.OUTPUT)
    pi.set_mode(ECHO, pigpio.INPUT)

    pi.callback(ECHO, pigpio.RISING_EDGE, rise)
    pi.callback(ECHO, pigpio.FALLING_EDGE, fall)

    rospy.loginfo("GPIO are inited")

    a = rospy.Service('get_distance', GetDistance, handle_dist)
    rospy.loginfo("Service GetDistance is inited")
    rospy.spin()



def rise(gpio, level, tick):
        global high
        high = tick


def fall(gpio, level, tick):
        global low, high
        low = tick - high
        done.set()


def read_distance():
        global low, done
        done.clear()
        pi.gpio_trigger(TRIG, 50, 1)
        done.wait(timeout=5)
        return low * 343.0 / 2/ 10000

def read_distance_filtered():
    history.append(read_distance())
    return numpy.median(history)


def handle_dist(req):
    return read_distance_filtered()


if __name__ == "__main__":
    init()
