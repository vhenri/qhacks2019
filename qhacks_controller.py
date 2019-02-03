#!/usr/bin/env python

import serial
import time
from pynput.keyboard import Key, Controller

keyboard = Controller()

ser = serial.Serial('COM6', 9600)
while(1):
	res = ser.readline()
	res = res.decode("utf-8").strip()

	if res == 'L':
		#mouse.move(-2, 0)
		keyboard.press('a')
		keyboard.release('a')

	elif res == 'R':
		#mouse.move(2, 0)
		keyboard.press('d')
		keyboard.release('d')

	elif res == 'LB':
		keyboard.press(Key.shift)
		keyboard.press('a')
		keyboard.release(Key.shift)
		keyboard.release('a')

	elif res == 'RB':
		keyboard.press(Key.shift)
		keyboard.press('d')
		keyboard.release(Key.shift)
		keyboard.release('d')

	else:
		pass