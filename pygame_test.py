import pygame
import sys
from PyQt4.QtGui import *
from PyQt4.QtCore import *
from tcgui import *
import win32api
import Leap
from constants import *
import math
import win32api, win32con, win32gui
import pyagg


# calculates distance between two given points
def distance(x, y, x2, y2):
	dx = x - x2
	dy = y - y2
	return math.sqrt(dx*dx + dy*dy)

class leap_listener(Leap.Listener):
	def on_init(self, controller):
		print("on_init")
		self.flag = False
		self.fingers_pos = [[0,0,0] for c in range(5)]

	def on_connect(self, controller):
		print("conected")
		
	def on_frame(self, controller):
		frame = controller.frame()
		for hand in frame.hands:
			if hand.is_right:
				for finger in hand.fingers:
					f_pos = finger.tip_position
					if not self.flag:
						x = f_pos.x
						y = LEAP_H - f_pos.y
						z = f_pos.z
						x = x + LEAP_W/2
						x = (W * x) / LEAP_W
						y = (H * y) / LEAP_H

					z = 20

					start_click_movement = 280
					end_click_movement = 30
					"""
					print(abs(finger.tip_velocity.z))
					# if finger 1 and negative Z movement and icreased Z finger velocity and not inside this
					if finger.type == 1 and hand.palm_velocity.z < 0 and abs(finger.tip_velocity.z) > start_click_movement and not self.flag:
						# perform mouse click
						print("click")
						win32api.mouse_event(win32con.MOUSEEVENTF_LEFTDOWN, int(x), int(y), 0, 0)
						#time.sleep(.2)
						win32api.mouse_event(win32con.MOUSEEVENTF_LEFTUP, int(x), int(y), 0, 0)
						self.flag = True  # this is for not to move cursor when we moving finger to do click
						
						canvas.pen.setColor(Qt.green)
						canvas.add_to_path(x, y, z)
					
					# when click movement is near to reach its end
					elif abs(finger.tip_velocity.z) < end_click_movement:
						self.flag = False
					"""	
					if finger.type == 1 and not self.flag and abs(finger.tip_velocity.z) < 200 and distance(self.fingers_pos[finger.type][0], self.fingers_pos[finger.type][1], x, y) > 5:
						# perform mouse movement
						#win32api.SetCursorPos((int(x), int(y)))
						self.fingers_pos[finger.type] = (x, y, finger.tip_position.z)
				

if __name__ == "__main__":
	listener = leap_listener()
	controller = Leap.Controller()
	controller.add_listener(listener)
	# keeping Leap Motion working from background
	controller.set_policy(Leap.Controller.POLICY_BACKGROUND_FRAMES)

	
	canvas = pyagg.Canvas("210mm", "297mm", background=(222,222,222), ppi=96)
	canvas.percent_space()
	canvas.draw_line([10,10, 50,90, 90,10], smooth=True, fillcolor=(222,0,0), fillsize="2cm")

	canvas.draw_triangle((50,50),fillsize="30px", fillcolor=(0,0,0, 111))
	canvas.view()
