#!/usr/bin/env python
from lib_oled96 import ssd1306
from smbus import SMBus
from time import sleep
from PIL import ImageFont, ImageDraw, Image
from line.srv import *
import rospy

def handle(req):
	oled.cls()
	# Write two lines of text.
	oled.canvas.text((5,15), req.color, font=font, fill=1)
	# now display that canvas out to the hardware
	oled.display()
	res = True
	return res


def init():
	rospy.init_node('view_text_server')

	global oled, font
	i2cbus = SMBus(1) # 1 = Raspberry Pi but NOT early REV1 board
	oled = ssd1306(i2cbus) # create oled object, nominating the correct I2C bus, default address
	font = ImageFont.truetype('FreeSans.ttf', 30)
	oled.cls()

	oled.canvas.text((5,15), "", font=font, fill=1)
	oled.display()

	#rospy.loginfo("Inited: view_text.py")

	a = rospy.Service('view_text', ViewText, handle)
	rospy.loginfo("Service ViewText is inited")
	rospy.spin()


if __name__ == '__main__':
	init()



