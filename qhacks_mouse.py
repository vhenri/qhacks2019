#!/usr/bin/env python

import serial
import time
from pynput.mouse import Button, Controller

mouse = Controller()

ser = serial.Serial('COM6', 9600)
while(1):
	res = ser.readline()
	res = res.decode("utf-8").strip()

	if res == 'E':
		mouse.move(3, 0)

	elif res == 'W':
		mouse.move(-3, 0)

	elif res == 'N':
		mouse.move(0, 3)

	elif res == 'S':
		mouse.move(0, -3)

	elif res == 'NE':
		mouse.move(3, 3)

	elif res == 'NW':
		mouse.move(-3, 3)

	elif res == 'SW':
		mouse.move(-3, -3)

	elif res == 'SE':
		mouse.move(3, -3)

	elif res == 'RC':
		mouse.press(Button.right)
		mouse.release(Button.right)
		time.sleep(2)

	elif res == 'LC':
		mouse.press(Button.left)
		mouse.release(Button.left)
		time.sleep(2)

	else:
		pass