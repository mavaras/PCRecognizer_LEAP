# -*- coding: utf-8 -*-

import time
import ctypes
import sys
import os
import math
import numpy as np
import threading
from collections import deque
import Leap
import win32api, win32con, win32gui
import cv2
from constants import *
"""
from kivy.config import Config
Config.set('graphics', 'resizable', 0)
Config.set('graphics', 'width', W/2)
Config.set('graphics', 'height', H/2)
Config.write()xs
from kivy.app import App
from kivy.uix.widget import Widget
from kivy.graphics import Color, Ellipse, Line, Rectangle
from kivy.core.window import Window
from kivy.uix.label import Label
from kivy.uix.boxlayout import BoxLayout
from kivy.lang import Builder
#allmyapp = Builder.load_file("gui.kv")
from Tkinter import *
import ttk"""

from PyQt4.QtGui import *
from PyQt4.QtCore import *
from gui import *


# GLOBALS
INF = 999999
exit = False
cv_frame = np.zeros((H/2, W/3, 3), np.uint8)  # XY


# calculates the 3d distance between two given points
def distance_3d(x, y, z,
				x2, y2, z2):
	return math.sqrt(abs(x-x2) + abs(y-y2) + abs(z-z2))

# calculates distance between two given points
def distance(p1, p2):
	dx = p2.x - p1.x
	dy = p2.y - p1.y
	return math.sqrt(dx*dx + dy*dy)

# CLASS CONTAINING CONFIGURATION DATA
class Conf():
	file_name = ""
	file_date = ""
	file_path = ""

	def __init__(self):
		self.basic = self.Basic()
		self.extra = self.Extra()
	
	class Basic():
		def __init__(self):
			self.mm = "default"
			self.lclick = "default"
			self.rclick = "default"
			self.scroll = "default"
			self.grabb = "default"
			self.ch_window = "default"
			self.cl_window = "default"
			self.mm_window = "default"

		def get_conf(self):
			aux = ("Basic Conf:\n"
				  +str(self.mm)+"\n"
				  +str(self.lclick)+"\n"
				  +str(self.rclick)+"\n"
				  +str(self.scroll)+"\n"
				  +str(self.grabb)+"\n"
				  +str(self.ch_window)+"\n"
				  +str(self.cl_window)+"\n"
				  +str(self.mm_window)+"\n"
			)

			return aux			
			
	class Extra():
		def __init__(self):
			self.open_calc = "default"
			self.open_texteditor = "default"
			self.open_console = "default"
			self.open_browser = "default"
			
			self.open_custom_1 = "empty"
			self.open_custom_2 = "empty"
			self.open_custom_3 = "empty"
			self.open_custom_4 = "empty"

		def get_conf(self):
			aux = ("Extra Conf:\n"
				  +str(self.open_calc)+"\n"
				  +str(self.open_texteditor)+"\n"
				  +str(self.open_console)+"\n"
				  +str(self.open_browser)+"\n"
				  +str(self.open_custom_1)+"\n"
				  +str(self.open_custom_2)+"\n"
				  +str(self.open_custom_3)+"\n"
				  +str(self.open_custom_4)+"\n"
			)

			return aux


# CLASS CONTAINING LEAP API
class leap_listener(Leap.Listener):
	frame = 0
	mouse = 0
	capture_frame = False

	# this class represents a mouse object for better handling
	class Mouse:
		active = False
		left_clicked = False
		x, y = (0, 0)
		ydir = 1
		xdir = 1
		acc = 1
		vel = 1

		switch_mode = False
		switching = False

		def __init__(self, x, y, dircx, dircy, acc, vel, active):
			self.x, self.y = x, y
			self.xdir = dircx
			self.ydir = dircy
			self.acc = acc
			self.vel = vel
			self.active = active
			self.left_clicked = False
			self.left_pressed = False
			self.switch_mode = False

		def click(self):  # ?
			win32api.mouse_event(win32con.MOUSEEVENTF_LEFTDOWN, self.x, self.y, 0, 0)
			#time.sleep(.2)
			win32api.mouse_event(win32con.MOUSEEVENTF_LEFTUP, self.x, self.y, 0, 0)

	def clear_variables(self):
		self.gesture = [[]*5 for c in range(600)]
		self.c = 0
		self.tail_points = []
		for c in range(0, 5):
			aux = deque([], 18)
			self.tail_points.append(aux)
		
	def on_init(self, controller):
		global cv_frame
		
		self.frame = controller.frame()
		self.mouse = self.Mouse(0, 0, 1, 1, 1, 5, False)
		self.capture_frame = False
		self.scrolling = False
		self.plane_mode = False
		self.deep_mode = False

		# this all temporary variables, just for testing
		p = []
		lim_points = 600
		self.gesture = [[]*5 for c in range(lim_points)]
		self.c = 0                              # contador gesture points
		self.d01 = 0
		self.on_frame_c = 0                     # contador pitch average
		self.lim_points = 400
		self.hand_vel = 0
		self.fingers_pos = [[0,0,0] for c in range(5)]
		self.fingers_vel = [[-1] for c in range(5)]
		self.tail_points = []
		for c in range(0, 5):
			aux = deque([], 18)
			self.tail_points.append(aux)

		print("initialized")

	def on_connect(self, controller):
		print("conected")

		controller.enable_gesture(Leap.Gesture.TYPE_SCREEN_TAP)
		controller.enable_gesture(Leap.Gesture.TYPE_SWIPE)
		
		time.sleep(.5)
		main_window.change_leap_status(True)  # label

	def on_focus_lost(self, controller):
		print("Unfocused")
		controller.add_listener(listener)

	def on_disconnect(self, controller):
		print("disconnected")
		main_window.change_leap_status(False)

	def on_exit(self, controller):
		print("exited")

	def leap_to_screen(self, leap_x, leap_y):
		# avoiding to invert X when nearing windows borders
		x = -LEAP_W/2 if int(leap_x) < -LEAP_W/2 else leap_x
		# y invertion
		y = leap_y - canvas_height
		y = y - abs(y)
		x = x + LEAP_W/2
		x = (W * x) / LEAP_W
		y = (H * y) / LEAP_H
		#y += 1500
		y += 1800

		return abs(x), abs(y)
		
	def on_frame(self, controller):
		self.frame = controller.frame()
		frame = self.frame

		# opencv canvas
		cv_frame_loc = np.zeros((H/2, W/3, 3), np.uint8)  # XY

		if len(frame.hands) == 2:
			# two hands if frame
			cv2.putText(cv_frame_loc, "Two hands in frame",
					   (190, H/2 - 50), cv2.FONT_HERSHEY_SIMPLEX,
						0.8, (255,255,255), 1, cv2.LINE_AA)
			cv2.circle(cv_frame_loc, (H/4 + 50, W/6 - 50), 100, [255, 0, 0], 1)
			
		for hand in frame.hands:
			if hand.is_right:
				# angle between X and Y (Z rotations)
				roll = abs(hand.palm_normal.roll * Leap.RAD_TO_DEG)
				yaw = hand.direction.yaw * Leap.RAD_TO_DEG
				#print(yaw)
				if roll > 50:  # we are into switch mode
					self.mouse.switch_mode = True
					direction = int(yaw/abs(yaw))
					if hand.palm_velocity.x > 150 and not self.mouse.switching:
						# utf-8 esta jodiendo por ejemplo en (u)Torrent
						
						self.mouse.switching = True
						#print("direction: "+str(direction))
						aux = get_current_window_name()
						print("aux: "+aux)
						curr_window_index = opened_windows_titles.index(aux)
						if direction == 1:
							print("right SWIPE")
							if curr_window_index == len(opened_windows_titles) - 1:
								curr_window_index = 0
							name = opened_windows_titles[curr_window_index+1]
						else:
							print("left SWIPE")
							if curr_window_index == 1:  # always Start it's the first one in list(0) (Windows only?)
								curr_window_index = len(opened_windows_titles)-1
							name = opened_windows_titles[curr_window_index-1]
							
						bring_window_to_top(win32gui.FindWindow(None, name))
						time.sleep(.5)
					else:
						#print("else")
						self.mouse.switching = False
					
				else:
					self.mouse.switch_mode = False
					self.mouse.switching = False
				
				cv2.putText(cv_frame_loc, "X to Y representation (vertical plane)",
						   (50, H/2 - 50), cv2.FONT_HERSHEY_SIMPLEX,
							0.8, (255,255,255), 1, cv2.LINE_AA)
				
				#elif len(frame.hands) == 1:
				self.hand_vel = max(abs(hand.palm_velocity[0]),
									abs(hand.palm_velocity[1]),
									abs(hand.palm_velocity[2]))
				
				# one hand into frame
				if self.mouse.active == True:
					if self.scrolling:
						# scrolling
						# angle between Z and Y (X rotations)
						pitch = hand.direction.pitch * Leap.RAD_TO_DEG
						if pitch < -10:
							print("scroll down")
							cx, cy = win32api.GetCursorPos()
							vel = -int(40-((90-abs(pitch))/3))
							win32api.mouse_event(win32con.MOUSEEVENTF_WHEEL, cx, cy, vel, 0)

						elif pitch > 38:
							print("scroll up")
							cx, cy = win32api.GetCursorPos()
							vel = int(30-((90-pitch)/3))
							win32api.mouse_event(win32con.MOUSEEVENTF_WHEEL, cx, cy, vel, 0)

					# INTERACTION modes
					if self.deep_mode:
						finger1 = hand.fingers[1]
						f1_pos = finger1.tip_position
						"""print(f1_pos.z)

						x, y = self.leap_to_screen(f1_pos.x, f1_pos.y)

						# mouse movement:
						self.mouse.xdir = 1 if x > W/2 else -1
						self.mouse.ydir = 1 if abs(y) < H/2 else -1
						win32api.SetCursorPos((int(abs(x)+self.mouse.xdir),
											   int(abs(y))+self.mouse.ydir))
						self.mouse.x, self.mouse.y = int(abs(x)), int(abs(y))"""
						
					elif self.plane_mode:
						f1 = hand.fingers[0].tip_position
						f2 = hand.fingers[1].tip_position
						f3 = hand.fingers[2].tip_position
						dist_0_1 = distance(Point(f1.x,f1.z,-1), Point(f2.x,f2.z,-1))   # this works better on XZ plane
						#dist_1_2 = distance(Point(f3.x,f3.z,-1), Point(f2.x,f2.z,-1))
						dist_1_2 = distance_3d(f3.x,f3.y,f3.z, f2.x,f2.y,f2.z)
						#print(">"+str(dist_0_1))
						# for keep performance when hand pitch (X rotations) >>
						pitch = hand.direction.pitch * Leap.RAD_TO_DEG
						#print(str(pitch)+" "+str(abs(pitch)//35))
						#print("_"+str(9.5 - abs(pitch)//35))
						if dist_0_1 < 80 - 15*abs(pitch)//35:
							print("!")
							if not self.mouse.left_clicked and not self.mouse.left_pressed:
								print("click")
								self.mouse.left_clicked = True
								cx, cy = win32api.GetCursorPos()
								win32api.mouse_event(win32con.MOUSEEVENTF_LEFTDOWN, cx, cy, 0, 0)
								time.sleep(.2)
								win32api.mouse_event(win32con.MOUSEEVENTF_LEFTUP, cx, cy, 0, 0)
						else:
							self.d01 = dist_0_1
							if self.mouse.left_clicked:
								print("released")
								self.mouse.left_clicked = False

						#print(dist_1_2)
						if dist_1_2 < 5.8:
							if not self.mouse.left_clicked and not self.mouse.left_pressed:
								print("grabbed")
								cx, cy = win32api.GetCursorPos()
								win32api.mouse_event(win32con.MOUSEEVENTF_LEFTDOWN, cx, cy, 0, 0)
								self.mouse.left_pressed = True
						else:
							if self.mouse.left_pressed:
								print("ungrabbed")
								cx, cy = win32api.GetCursorPos()
								#time.sleep(.2)
								win32api.mouse_event(win32con.MOUSEEVENTF_LEFTUP, cx, cy, 0, 0)
								self.mouse.left_pressed = False

				for finger in hand.fingers:
					self.fingers_vel[finger.type] = max(abs(finger.tip_velocity.x), abs(finger.tip_velocity.y), abs(finger.tip_velocity.z))
					fx, fy, fz = finger.tip_position.x, finger.tip_position.y, finger.tip_position.z
					sx, sy = self.leap_to_screen(fx, fy)
					if distance(Point(sx, sy, -1), Point(self.fingers_pos[finger.type][0],
														 self.fingers_pos[finger.type][1], -1)) > 3:
						self.fingers_pos[finger.type] = (sx, sy, fz)

					# drawing things
					cv2.circle(cv_frame_loc, (int(fx) + 250, int(H/2 - abs(fy))), 11, [255, 255, 255], 1)
					if finger.type != 0:
						cv2.line(cv_frame_loc, (int(fx) + 250, int(H/2 - abs(fy))), (int(hand.fingers[finger.type-1].tip_position.x) + 250,
																					 int(H/2 - abs(hand.fingers[finger.type-1].tip_position.y))),
																					(255, 255, 255), 1)

					self.tail_points[finger.type].append((int(fx) + 250, int(H/2 - abs(fy))))
					for point in self.tail_points[finger.type]:
						cv2.circle(cv_frame_loc, point, 6, [205, 205, 205], 1)

					if len(self.tail_points) == 10:
						self.tail_points[finger.type].popleft()

					# recording coordinates
					if self.capture_frame:
						#print(str(finger.type)+" "+str(finger.tip_position))
						self.gesture[finger.type].append(Point(fx, fy, -1))
						self.gesture[finger.type][len(self.gesture[finger.type])-1].convert_to(W/2, H/2)
						#self.c += 1

					# cursor movement
					if len(frame.hands) == 1 and finger.type == 1 and self.mouse.active and not self.mouse.switch_mode:
						# cursor movement
						self.mouse.xdir = 1 if sx > W/2 else -1
						self.mouse.ydir = 1 if abs(sy) < H/2 else -1

						win32api.SetCursorPos((int(abs(self.fingers_pos[1][0])),
											   int(abs(self.fingers_pos[1][1]))))
						self.mouse.x, self.mouse.y = int(abs(sx)), int(abs(sy))

					"""if finger.type == 1 and self.mouse.active == True:
						# avoiding to invert X and Y when nearing windows borders
						x = -LEAP_W/2 if int(finger.tip_position.x) < -LEAP_W/2 else finger.tip_position.x
						# y invertion
						y = finger.tip_position.y - canvas_height
						y = y - abs(y)

						x = x + LEAP_W/2
						x = (W * x) / LEAP_W
						y = (H * y) / LEAP_H
						y += 1500

						self.mouse.xdir = 1 if x > W/2 else -1
						self.mouse.ydir = 1 if abs(y) < H/2 else -1

						win32api.SetCursorPos((int(abs(x)+self.mouse.vel*self.mouse.xdir),
											   int(abs(y))+self.mouse.vel*self.mouse.ydir))
						self.mouse.x, self.mouse.y = int(abs(x)), int(abs(y))"""

					self.on_frame_c += 1

		global cv_frame
		cv_frame = cv_frame_loc
		#cv2.imshow(cv2_window_name, cv_frame)
		key = cv2.waitKey(5) & 0xFF
		if key == ord("q"):
			exit_app()


# ===============PCRecognizer things (functions + classes)===============

# match two point_cloud by calculating distance between their points
# between our points and the template
def greedy_cloud_match(points, pc):
	e = 0.50
	step = math.floor(math.pow(len(points), 1.0 - e))
	minimum = INF
	for c in range(0, len(points)):
		d1 = cloud_distance(points, pc.points, c)
		d2 = cloud_distance(pc.points, points, c)
		minimum = min(minimum, min(d1, d2))
		c += step

	return minimum

# geometric distance between two point_clouds
def cloud_distance(pc1, pc2, start):
	aux = [False] * len(pc1)    # empty auxiliar array
	suma = 0
	w = start
	while True: # something like a do while?
		index = -1
		minimum = INF
		for j in range(0, len(aux)):
			if False == aux[j]:
				dist = distance(pc1[w], pc2[j])
				#print("d: "+str(minimum))
				if dist < minimum:
					minimum = dist
					index = j

		aux[index] = True
		# this float parsing is neccesary for this python 2.x
		weigth = 1 - (float((w - start + len(pc1) % len(pc1))) / float(len(pc1)))
		suma += weigth * minimum
		w = (w + 1) % len(pc1)

		if w == start:
			break

	return suma

# resamples a point cloud in order to set homogenous lengths for compare them properly.
# resample_length indicates the length which to resample the pc.
def resample(points, resample_len):
	interval = path_length(points) / (resample_len - 1)
	d = 0.0
	new_points = []
	new_points.append(points[0])
	c = 1
	for p in points:
		try:
			if points[c].id == points[c - 1].id:                 # we are int he same stroke
				dist = distance(points[c - 1], points[c])
				if d + dist >= interval:
					px = points[c - 1].x + ((interval - d) / dist) * (points[c].x - points[c - 1].x)
					py = points[c - 1].y + ((interval - d) / dist) * (points[c].y - points[c - 1].y)
					p = Point(px, py, points[c].id)
					new_points.append(p)
					points.insert(c, p)                          # insert p in c position, reasigning all elements
					d = 0.0

				else:
					d += dist

			c += 1
		except:
			break

	if len(new_points) == resample_len - 1:
		new_points.append(Point(points[len(points) - 1].x,
								points[len(points) - 1].y,
								points[len(points) - 1].id))

	return new_points

# provides the same point_cloud in different scales in order to compare
def scale(points):
	min_x = INF
	min_y = INF
	max_x = -INF
	max_y = -INF
	for c in range(len(points)):
		min_x = min(min_x, points[c].x)
		min_y = min(min_y, points[c].y)
		max_x = max(max_x, points[c].x)
		max_y = max(max_y, points[c].y)

	scale = max(max_x - min_x, max_y - min_y)
	new_points = []
	for c in range(len(points)):
		px = (points[c].x - min_x) / scale
		py = (points[c].y - min_y) / scale
		new_points.append(Point(px, py, points[c].id))

	return new_points

# translates a point_cloud to the provided centroid. It maps all pc to origin,
# in order to recognize pc that are similar but in different coordinates
def translate_to(points, where):
	centroid = get_centroid(points)
	new_points = []
	for c in range(0, len(points)):
		px = points[c].x + where.x - centroid.x
		py = points[c].y + where.y - centroid.y
		new_points.append(Point(px, py, points[c].id))

	return new_points

# amplifies a collection of points keeping its distances between each other attending to mult argument
def amplify(points, mult):
	new_points = []
	x = points[0].x
	y = points[0].y
	new_points.append(Point(x, y, points[0].id))
	for c in range(1, len(points)):
		x += mult*(points[c].x - points[c - 1].x)
		y += mult*(points[c].y - points[c - 1].y)
		new_points.append(Point(x, y, points[c].id))

	return new_points

# calculates the centroid of given cloud of points
def get_centroid(points):
	x = 0.0
	y = 0.0
	for c in range(0, len(points)):
		x += points[c].x
		y += points[c].y

	x /= len(points)
	y /= len(points)

	return Point(x, y, 0)

# calculates the length of a single point in a point_cloud
def path_length(points):
	dist = 0.0
	for c in range(1, len(points)):
		if points[c].id == points[c - 1].id:
			dist += distance(points[c - 1], points[c])

	return dist

# initialize templates array for PCRecognizer class
def init_templates():
	templates = []  
	# single stroke templates (all fingers doing the same if various fingers) (1 finger)
	templates.append(Template("T", [
		# Template point clouds
		# different ways of drawing Template.name (T) for better recognition
		Point_cloud("T1", [Point(30, 7, 1), Point(103, 7, 1),
						   Point(66, 7, 2), Point(66, 87, 2)])
		,
		Point_cloud("T2", [Point(30, 7, 1), Point(123, 7, 1),
						   Point(80, 17, 2),Point(30, 7, 2),
						   Point(80, 17, 3),Point(80, 77, 3)])
		,
		Point_cloud("T3", [Point(30, 7, 1), Point(123, 7, 1),
						   Point(80, 17, 2),Point(30, 7, 2),
						   Point(80, 17, 3),Point(80, 50, 3)])
		], None)
	)
	templates.append(Template("V", [
		Point_cloud("V1", [Point(30, 7, 1), Point(40, 37, 1),
						   Point(40, 37, 2),Point(50, 7, 2)])
		,
		Point_cloud("V2", [Point(0, 7, 1), Point(25, 37, 1),
						   Point(25, 37, 2),Point(50, 7, 2)])
		,
		Point_cloud("V3", [Point(30, 7, 1), Point(40, 25, 1),
						   Point(40, 25, 2),Point(50, 7, 2)])
		,
		Point_cloud("V4", [Point(30, 16, 1), Point(33, 25, 1),
						   Point(33, 25, 2),Point(38, 7, 2)])
		,
		Point_cloud("V5", [Point(30, 7, 1), Point(33, 25, 1),
						   Point(33, 25, 2),Point(38, 16, 2)])
		], None)
	)
	templates.append(Template("RIGHT", [
		Point_cloud("right1", [Point(30, 47, 1), Point(96, 7, 1)])
		,
		Point_cloud("right2", [Point(30, 37, 1), Point(96, 20, 1)])
		], None)
	)
	templates.append(Template("LEFT", [
		Point_cloud("left1", [Point(30, 7, 1), Point(96, 47, 1)])
		,
		Point_cloud("left2", [Point(30, 20, 1), Point(96, 37, 1)])
		], None)
	)
	templates.append(Template("X", [
		Point_cloud("X1", [Point(30, 7, 1), Point(60, 47, 1),
						   Point(60, 7, 2), Point(30, 47, 2)])
		,
		Point_cloud("X1_2", [Point(30, 7, 1), Point(60, 34, 1),
							 Point(60, 7, 2), Point(30, 34, 2)])
		,
		Point_cloud("X2", [Point(30, 7, 1), Point(60, 47, 1),
						   Point(60, 7, 2), Point(30, 47, 2),
						   Point(30, 7, 3), Point(60, 7, 3)])
		,
		Point_cloud("X3", [Point(30, 7, 1), Point(60, 47, 1),
						   Point(60, 7, 2), Point(30, 47, 2),
						   Point(30, 47, 3),Point(60, 47, 3)])
		,
		Point_cloud("X4", [Point(30, 7, 1), Point(60, 47, 1),
						   Point(60, 7, 2), Point(30, 47, 2),
						   Point(30, 7, 3), Point(30, 47, 3)])
		], None)
	)
	templates.append(Template("W", [
		Point_cloud("W1", [Point(30, 7, 1), Point(40, 37, 1),
						   Point(40, 37, 2),Point(50, 7, 2),
					   	   Point(50, 7, 1), Point(60, 37, 1),
				     	   Point(60, 37, 2),Point(70, 7, 2)])
		], None)
	)
	
	# complex stroke templates (each finger analized individually) (ALL fingers)    
	
	return templates


# CLASS DEFINITIONS
# this class represents one single point
# convert_to method does the translation between Leap coordinates to canvas or something ones
class Point:
	def __init__(self, x, y, id):
		self.x = x
		self.y = y
		self.id = id

	def convert_to(self, where_W, where_H):
		self.y = where_H - abs(self.y)   # Y invertion
		self.x = (W * self.x) / where_W
		self.y = (H * self.y) / where_H

	def draw_on_canvasTK(self, radius):
		canvas.create_oval(self.x-radius, self.y-radius,
						   self.x+radius, self.y+radius, fill = "#aaaaaa")

	def draw_on_canvas(self, radius, path):
		eval("main_window.widget_canvas."+path+
			 ".addEllipse(QtCore.QRectF(self.x, self.y, radius, radius))")
		main_window.widget_canvas.update()

# a point cloud is a collection of points defining a shape, it's like a points array
class Point_cloud:
	def __init__(self, name, points, where_to_translate=Point(W/4, H/4, -1)):
		self.origin = where_to_translate
		self.name = name
		self.points = resample(points, 32)                       # point cloud resizing
		self.points = scale(self.points)                         # point cloud scaling
		self.points = translate_to(self.points, self.origin)     # point cloud centering
		print(self.points[0].x)

	def draw_on_canvas(self, flag=True):
		aux_points = []                                          # to not overwritting original self.points array
		aux_points = amplify(self.points, 200)                   # kind of scale reversion
		aux_points = translate_to(aux_points, Point(self.origin.x, self.origin.y/2, -1))  # translating amplilfied pc to center
		dic = {"f0":"path_points_0", "f1":"path_points_1",
			   "f2":"path_points_2", "f3":"path_points_3", "f4":"path_points_4"}

		#aux_points = points
		c = 0
		if flag:
			path = dic.get(self.name)
		else:                                                    # black color by default
			path = "path_points_1"
			
		for p in aux_points:
			if c == num_points:
				p.draw_on_canvas(10, path)
			else:
				p.draw_on_canvas(6, path)
				
			c += 1

# this class represents a full Template, that's it a template representing a form (name)
# but with some different ways of representing it (point_cloud array)
class Template:
	def __init__(self, name, point_cloud, fingers_point_cloud):
		self.name = name
		self.point_cloud = point_cloud
		self.fingers_point_cloud = fingers_point_cloud           # this if template is a complex one

# this class stores the final result of the algorithm, containing the match
class Result:
	def __init__(self, name, score, ms):
		self.name = name
		self.score = score
		self.ms = ms


# MAIN ALGORITHM CLASS
# contains all the templates and starts all the process
num_points = 32                                                  # points number to resample to
origin = Point(W/4, H/4, -1)                                     # canvas point where to translate_to (canvas center)
class PCRecognizer:
	templates = init_templates()                                 # array storing all Template objects

	# this sets up points array received for proper algorithm application
	def normalize(self, points):
		points = resample(points, 32)
		points = scale(points)
		points = translate_to(points, origin)
		return points

	# arr_points is an array containing the points array of each finger
	# if we are working with 1 finger, its points array is into arr_points[0]
	def recognize(self, arr_points, print_all_matches = False):
		t_ini = time.clock()

		# normalizing stroke(s)  
		for c in range(0, len(arr_points)):                      # (if 1 finger len(arr_points) = 1
			arr_points[c] = self.normalize(arr_points[0])

		score = INF
		template_n = -1
		for c in range(0, len(self.templates)):
			if self.templates[c].fingers_point_cloud == None:
				# its a single stroke template (1 finger)
				for j in range(0, len(self.templates[c].point_cloud)):
					# normalizing template
					dist = greedy_cloud_match(arr_points[0], self.templates[c].point_cloud[j])
					# coincidence between points and all "subtemplates"
					if print_all_matches:
						coinc = "    similar to \""+str(self.templates[c].point_cloud[j].name+"\" about "+str(max((dist - 2.0) / -2.0, 0.0)))
						print(coinc)
						main_window.text_edit_2.append(coinc)
					if dist < score:
						score = dist
						template_n = c
			else:
				# its a complex stroke template (ALL fingers)
				pass

		t_fin = time.clock()
		if template_n == -1:
			return Result("no match", 0.0, t_fin - t_ini)

		else:
			# process is finished, we return the result
			print("score: "+str(score))
			return Result(self.templates[template_n].name,      # template matched
						  max((score - 2.0) / -2.0, 0.0),       # score achieved
						  t_fin - t_ini)                        # time in ms


# ===============GUI CLASSES===============
# CANVAS CLASS
class Widget_canvas(QWidget):
	lp = Point(0, 0, -1)
	np = Point(0, 0, -1)
	path_points_0 = QPainterPath()
	path_points_1 = QPainterPath()
	path_points_2 = QPainterPath()
	path_points_3 = QPainterPath()
	path_points_4 = QPainterPath()
	canvas = None
	pen_color = Qt.black

	def __init__(self, parent):
		super(Widget_canvas, self).__init__(parent)

	def clear(self):
		aux = QPainterPath()
		self.path_points_0 = aux
		aux = QPainterPath()
		self.path_points_1 = aux
		aux = QPainterPath()
		self.path_points_2 = aux
		aux = QPainterPath()
		self.path_points_3 = aux
		aux = QPainterPath()
		self.path_points_4 = aux
		aux = QPainterPath()
		self.update()
		
	def paintEvent(self, event):
		canvas = QtGui.QPainter(self)
		pen = QPen()

		# drawing grid
		pen.setWidth(1.4)
		pen.setColor(Qt.black)
		canvas.setPen(pen)
		interval = 20
		for c in range(interval, canvas_width, interval):
			for j in range(interval, canvas_height, interval):
				canvas.drawLine(c, 0, c, canvas_height)
				canvas.drawLine(0, j, canvas_width, j)

		# finger 0 path
		pen.setWidth(2.4)
		pen.setColor(Qt.red)
		canvas.setPen(pen)
		canvas.drawPath(self.path_points_0)

		# finger 1 path
		pen.setColor(Qt.black)
		canvas.setPen(pen)
		canvas.drawPath(self.path_points_1)
		
		# finger 2 path
		pen.setColor(Qt.blue)
		canvas.setPen(pen)
		canvas.drawPath(self.path_points_2)

		# finger 3 path
		pen.setColor(Qt.green)
		canvas.setPen(pen)
		canvas.drawPath(self.path_points_3)

		# finger 4 path
		pen.setColor(Qt.yellow)
		canvas.setPen(pen)
		canvas.drawPath(self.path_points_4)
		

	def mousePressEvent(self, event):
		print("click")
		x = event.x()
		y = event.y()
		print("start point: ("+str(x)+","+str(y)+")")
		#self.path.moveTo(e.pos())
		self.path_points_1.addEllipse(QtCore.QRectF(x, y, 16, 16))
		global stroke_id, points
		stroke_id += 1
		points.append(Point(x, y, stroke_id))
		self.lp.x, self.lp.y = x, y

	def mouseMoveEvent(self, event):
		#self.path.lineTo(event.pos())
		x = event.x()
		y = event.y()
		self.np = Point(x, y, -1)
		if distance(self.lp, self.np) > 5:
			self.path_points_1.addEllipse(QtCore.QRectF(x, y, 8, 8))
			global stroke_id, points
			points.append(Point(x, y, stroke_id))
			self.lp.x, self.lp.y = x, y
			self.update()

	def mouseReleaseEvent(self, event):
		print("release")
		print("end point: ("+str(event.x())+","+str(event.y())+")")

# CV FRAME CLASS (Configuration Tab)		
class Cv_Frame:
	def __init__(self):
		global cv_frame

		# show_frame each second
		self.timer = QtCore.QTimer(main_window)
		self.timer.timeout.connect(self.show_frame)
		self.timer.start(1)

	# load frame (image) into label
	def show_frame(self):
		frame = cv_frame
		frame = cv2.resize(frame, None, fx=.7, fy=.7, interpolation=cv2.INTER_CUBIC)
		
		height, width, size = frame.shape
		step = frame.size / height
		qformat = QImage.Format_RGBA8888 if size == 4 else QImage.Format_RGB888
		frame = QImage(frame, width, height, step, qformat)
		
		main_window.label_img.setPixmap(QtGui.QPixmap.fromImage(frame))
		main_window.label_img.setContentsMargins(45, 5, 0, 0)
		
# MAIN WINDOW CLASS
class MainWindow(QtGui.QMainWindow, Ui_MainWindow):
	# last and new points for propper drawing
	lp = Point(0, 0, -1)
	np = Point(0, 0, -1)
	n_of_fingers = 1

	def __init__(self, *args, **kwargs):
		QtGui.QMainWindow.__init__(self, *args, **kwargs)
		#self.setWindowFlags(Qt.FramelessWindowHint)
		#self.setWindowOpacity(0)
		
		self.systray = QSystemTrayIcon(QIcon("icon.ico"), self)
		self.set_notification_area()
		
		self.setupUi(self)
		
	# making app icon appear into notification area
	def set_notification_area(self):
		self.systray.show()
		self.systray.showMessage("PCRecognizer",
								 "App working",
								 QSystemTrayIcon.Warning)
		# right click options
		self.systray_menu = QMenu(self)
		self.hide_systray = self.systray_menu.addAction("Opt. 1")
		self.systray.setContextMenu(self.systray_menu)

	# specific widget initialization and positioning
	def initUI(self):
		# canvas setup
		self.widget_canvas = Widget_canvas(self.tab_canvas)     # linking widget_canvas to tab1
		self.widget_canvas.move(20, 20)
		self.widget_canvas.resize(canvas_width, canvas_height)

		# configuration tab
		self.button_save_conf.clicked.connect(self.save_conf)
		
		# text edits setup
		self.button_load_text.clicked.connect(self.load_text_B)
		self.text_edit.setReadOnly(True)
		self.text_edit_2.setReadOnly(True)
		self.combo_box.currentIndexChanged["int"].connect(self.combo_box_selection_changed)

		# label_leap_status default value
		self.change_leap_status(False)
		
		# Tab2 cv_frame representation
		self.cvF = Cv_Frame()

	# this two functions are about loading text from text_edit and converting it to points array
	def to_point(self, text):
		parts = text.split(",")
		return int(float(parts[0][1:])), int(float(parts[1][:-1]))
	def load_text_B(self):
		text = str(self.text_edit.toPlainText())
		arr = text.split("\n")

		loaded_points = []                                      # containing new read points
		for c in range(2, len(arr)-1):
			x, y = self.to_point(arr[c])
			loaded_points.append(Point(x, y, -1))

		pc = Point_cloud("loaded_points", loaded_points);
		self.widget_canvas.path = QPainterPath()                # clear canvas
		self.update()
		pc.draw_on_canvas()                                     # drawing loaded stroke
		global points
		points = loaded_points                                  # allowing "F"

	# save configuration to file
	def save_conf(self):
		print("save_conf()")
		fname = QFileDialog.getSaveFileName(self, "Save Configuration", "c:\\", "(*.txt)")
		if fname:
			conf.file_name = str(QFileInfo(fname).fileName())
			conf.file_path = str(QFileInfo(fname).path())
			conf.file_date = time.ctime(os.path.getctime(fname))
			
			f = open(fname, "w")
			f.write(conf.file_name+"\n"++conf.file_path+"\n"+conf.file_date+"\n\n")
			f.write(conf.basic.get_conf())
			f.write("\n")
			f.write(conf.extra.get_conf())
			f.close()
		
	# Leap status label
	def change_leap_status(self, conn):
		if conn:
			self.label_leap_status.setText("CONNECTED")
			self.label_leap_status.setStyleSheet("QLabel {color: green;}")
		else:
			self.label_leap_status.setText("DISCONNECTED")
			self.label_leap_status.setStyleSheet("QLabel {color: red;}")
		
	# collection of key events binded to GUI
	def keyPressEvent(self, event):
		if event.key() == QtCore.Qt.Key_Q:
			exit_app()

		elif event.key() == QtCore.Qt.Key_W:                    # app interraction example/test
			handle = win32gui.FindWindow(None, r"Reproductor multimedia VLC")
			#win32gui.PostMessage(handle, win32con.WM_CLOSE, 0, 0)
			win32gui.CloseWindow(handle)

		elif event.key() == QtCore.Qt.Key_S:
			listener.mouse.active = True if not listener.mouse.active else False

		elif event.key() == QtCore.Qt.Key_F:                    # start stroke recognition
			global points
			if self.n_of_fingers == 1:
				pc = Point_cloud("f3", points, Point(W/4, H/4 + 50, -1))
				#pc.draw_on_canvas()    normalized pc
				result = recognize_stroke(points)
				gesture_match(result.name)

				str_accum = "Last/Current gesture points array\n\n"
				for c in range(0, len(points)):
					str_accum += "("+str(points[c].x)+","+str(points[c].y)+")"
					if c != len(points) - 1:
						str_accum += "\n"

				print_score(result)
				self.text_edit.setText(str_accum)

			elif self.n_of_fingers == -1:
				results = []
				for c in range(0, 5):
					results.append(recognize_stroke(listener.gesture[c]))
					gesture_match(results[len(results) - 1].name)

				results_names = []
				results_names = [res.name for res in results]
				print(results_names)
				if len(set(results_names)) <= 1:
					print_score(results[1])
				else:
					print("inconsistent matches of each finger")

			# reseting values, clearing arrays
			stroke_id = 0                            
			points = []                              
			listener.clear_variables()

		elif event.key() == QtCore.Qt.Key_C:                    # clear canvas
			print("clear")
			self.widget_canvas.clear()
			self.label_score.setText("")

		elif event.key() == QtCore.Qt.Key_G and sys.argv[1] != "-thread":
			global points
			if self.n_of_fingers == 1:                          # recordin only 1 finger(indice)
				print("1 finger recording mode")
				if listener.capture_frame:                      # 2nd "G" press
					print("record captured")
					listener.capture_frame = False
					pc = Point_cloud("f1", listener.gesture[1])
					pc.draw_on_canvas()
					# getting finger 1 points
					points = listener.gesture[1]                # this allows "F" to work with mouse and hand stroke
					listener.c = 0

				else:                                           # 1st "G" press
					print("recording")
					self.label_count.setText("4")               # countdown after getting gesture
					# this is like a thread with no wait
					QtCore.QTimer.singleShot(1000, lambda: self.updateLabel(self.label_count))
					
			elif self.n_of_fingers == -1:                       # recording ALL fingers
				print("all fingers recording mode")
				aux = -100
				if listener.capture_frame:
					print("record captured")
					listener.capture_frame = False
					for c in range(0, 5):
						pc = Point_cloud("f"+str(c), listener.gesture[c], Point(W/4 + aux, H/4, -1))
						pc.draw_on_canvas()
						aux += 50

					points = listener.gesture[1]  # just for testing
					
				else:                                           # 1st "G" press
					print("recording")
					self.label_count.setText("4")               # countdown after getting gesture
					# this is like a thread with no wait
					QtCore.QTimer.singleShot(1000, lambda: self.updateLabel(self.label_count))
			
	def combo_box_selection_changed(self):
		print("selection changed"+str(self.combo_box.currentIndex()))
		if self.combo_box.currentIndex() == 0:
			self.n_of_fingers = 1
		elif self.combo_box.currentIndex() == 1:
			self.n_of_fingers = -1
			
		self.setFocus()                                         # getting focus back on main_window

	def set_label_leap_status(self, img):
		img,_,_ = load_image(img)
		self.label_leap_status.setPixmap(QPixmap.fromImage(img))

	def updateLabel(self, label):
		# change the following line to retrieve the new voltage from the device
		t = int(label.text()) - 1
		if t == 0:
			label.setText("")
			while(listener.hand_vel < 300 or listener.fingers_vel[4] < 100):
				pass

			listener.capture_frame = True
			return

		label.setText(str(t))
		QtCore.QTimer.singleShot(1000, lambda: self.updateLabel(label))

# various functions
# this function recognize one SINGLE stroke (if ALL fingers, one by one)
def recognize_stroke(points):
	print("recognizing stroke")
	aux = []
	aux.append(points)
	result = pcr.recognize(aux)
	
	return result

# this shows final score of current stroke (red label on canvas)
def print_score(result):
	score = "Result: matched with "+result.name+" about "+str(round(result.score, 2))
	main_window.label_score.setStyleSheet("color: red")
	main_window.label_score.setText(str(score))
	main_window.text_edit_2.append("\n"+str(score))

# handling gesture stroke match actions
def gesture_match(gesture_name):
	if gesture_name == "T":
		print("T gesture")
		if sys.argv[1] == "-thread":
			#handle = win32gui.FindWindow(None, r"Reproductor multimedia VLC")
			hwnd = get_current_window_hwnd()
			minimize_window(hwnd)
	
	elif gesture_name == "V":
		print("V gesture")
	elif gesture_name == "LEFT":
		print("LEFT gesture")
	elif gesture_name == "RIGHT":
		print("RIGHT gesture")
	print("")

# exits applications, closes all windows
def exit_app():
	print("exiting...")
	global exit
	exit = True
	main_window.close()
	cv2.destroyAllWindows()
	controller.remove_listener(listener)
	sys.exit()
	
# thread which allows to make "G" + "G" by putting two hands on frame (python .py -thread)
# enables stroke gesture recognition (two hands on frame)
def thread_handler():
	print("thread_handler_init")
	global exit
	while not exit:
		if len(listener.frame.hands) == 2 and not listener.capture_frame:
			# here listener.frame_capture is False (we starting a new recording)
			time.sleep(.5)
			print("recording")
			while(listener.hand_vel < 300):
				pass

			listener.capture_frame = True

		elif len(listener.frame.hands) == 1 and listener.capture_frame:
			print("recording and recognizing captured")
			listener.capture_frame = False                      # end of Leap capture
			pc = Point_cloud("f1", listener.gesture[1])         # pc containing our new gesture
			pc.draw_on_canvas()                                 # drawing pc on canvas to see the shape

			global points, stroke_id
			points = listener.gesture[1]                        # this allows "F" to work with mouse and hand stroke
			listener.c = 0

			result = recognize_stroke(points)
			gesture_match(result.name)
			#print_score(result)
			
			stroke_id = 0                                       # reseting values
			points = []
			listener.clear_variables()

	print("thread_handler_end")

def load_image(path):
	img = cv2.imread(path)
	data = np.array(bytearray(open(path, "rb").read()))
	#img = cv2.imdecode(data, cv2.IMREAD_UNCHANGED)
	height, width, size = img.shape
	step = img.size / height
	qformat = QImage.Format_Indexed8

	if len(img.shape) == 3:
		if size == 4:
			qformat = QImage.Format_RGBA8888
		else:
			qformat = QImage.Format_RGB888

	img = QImage(img, width, height, step, qformat)
	img = img.rgbSwapped()

	return img, height, width


# WIN32 CONTROL FUNCTIONS
def get_opened_windows_list():
	EnumWindows(EnumWindowsProc(foreach_window), 0)
	
# this is passed as argument when we call EnumWindows, fills titles array with current opened windows names
def foreach_window(hwnd, lParam):
	if IsWindowVisible(hwnd):
		length = GetWindowTextLength(hwnd)
		buff = ctypes.create_unicode_buffer(length + 1)
		GetWindowText(hwnd, buff, length + 1)
		if buff.value != "":
			opened_windows_titles.append(buff.value)
		
	return True

# do alt+tab to hwnd window
def bring_window_to_top(hwnd):
	win32gui.ShowWindow(hwnd, win32con.SW_MINIMIZE)
	win32gui.ShowWindow(hwnd, win32con.SW_RESTORE)

# minimizes hwnd window
def minimize_window(hwnd):
	win32gui.CloseWindow(hwnd)

# closes hwnd window
def close_window(hwnd):
	win32gui.PostMessage(hwnd, win32con.WM_CLOSE, 0, 0)

# return current window hwnd
def get_current_window_hwnd():
	return win32gui.GetForegroundWindow()
	
# returns current window name
def get_current_window_name():
	#return win32gui.GetWindowText(win32gui.GetForegroundWindow())
	hwnd = get_current_window_hwnd()
	length = GetWindowTextLength(hwnd)
	buff = ctypes.create_unicode_buffer(length + 1)
	GetWindowText(hwnd, buff, length + 1)
	#print("buff.value: "+buff.value.encode('latin1'))
	return buff.value
	#return unicode(win32gui.GetWindowText(win32gui.GetForegroundWindow()), errors="ignore")
	
	
if __name__ == "__main__":
	# scaled monitor dimensions for canvas
	window_width = W/2
	window_height = H/2
	canvas_width = 900
	canvas_height = 400
	
	points = []                                                 # where to store drawn points
	stroke_id = 0
	result = -1                                                 # result object
	pcr = PCRecognizer()                                        # algorithm class initialization
	cv2_window_name = "LEAP DATA"

	opened_windows_titles = []									# array storing all currently opened window titles
	
	# setting up win32 stuff
	EnumWindows = ctypes.windll.user32.EnumWindows
	EnumWindowsProc = ctypes.WINFUNCTYPE(ctypes.c_bool, ctypes.POINTER(ctypes.c_int), ctypes.POINTER(ctypes.c_int))
	GetWindowText = ctypes.windll.user32.GetWindowTextW
	GetWindowTextLength = ctypes.windll.user32.GetWindowTextLengthW
	IsWindowVisible = ctypes.windll.user32.IsWindowVisible

	"""handle = win32gui.FindWindow(None, opened_windows_titles[4])
	if handle == None:
		print("handle = None")
	#bring_window_to_top(handle)
	
	while True:
		s = win32gui.GetWindowText(win32gui.GetForegroundWindow())
		s = unicode(s, errors='replace')
		print(s.encode("utf-8"))
		print(opened_windows_titles.index(s))"""

	# Leap setting up
	listener = leap_listener()
	controller = Leap.Controller()

	# listener initializer
	controller.add_listener(listener)

	# keeping Leap Motion working from background
	controller.set_policy(Leap.Controller.POLICY_BACKGROUND_FRAMES)

	# Window setting up
	app = QtGui.QApplication([])
	"""diag = my_dialog()
	diag.resize(600, 400)
	diag.show()"""
	main_window = MainWindow()
	"""main_window.label_img.setPixmap(QPixmap.fromImage(img))
	main_window.label_img.setFixedSize(height, width)"""
	main_window.initUI()
	main_window.show()

	# Configuration
	conf = Conf()
	
	# shell arguments handling
	for arg in sys.argv:
		if arg == "-thread":                                    # no keys gesture recognition enabled
			thread = threading.Thread(target=thread_handler)
			thread.start()
		elif arg == "-allf":                                    # ALL fingers capture mode by default
			main_window.combo_box.setCurrentIndex(1)
		elif arg == "-scroll":                                  # scroll enabled
			listener.scrolling = True
		elif arg == "-deepm":                                   # INTERACTION MODE: deep mode by default
			listener.deep_mode = True
		elif arg == "-planem":                                  # INTERACTION MODE: plane mode by default
			listener.plane_mode = True

	# win32 stuff
	get_opened_windows_list()
	
	print("currently opened windows")
	print(opened_windows_titles) #.encode("utf-8")
	#print("-> "+opened_windows_titles[8])
	print(get_current_window_name())
	
	# drawing tests
	pcr.templates[5].point_cloud[0].draw_on_canvas(False)
	app.exec_()

"""
_____________KIVY IMPLEMENTATION_____________
class Canvas(Widget):
q   def on_touch_down(self, touch):
		print("pressed")
		with self.canvas:
			Color(1, 1, 0)
			d = 15
			Ellipse(pos=(touch.x - d / 2, touch.y - d / 2), size=(d, d))

	def on_touch_move(self, touch):
		with self.canvas:
			Color(1, 1, 0)
			d = 8
			Ellipse(pos=(touch.x - d / 2, touch.y - d / 2), size=(d, d))

	def on_touch_up(self, touch):
		print("released")

class Container(BoxLayout):
	pass

class MyApp(App):
	title = "joder"
	def build(self):
		self._keyboard = Window.request_keyboard(self._keyboard_closed, self)
		self._keyboard.bind(on_key_down = self._on_keyboard_down)
		b = BoxLayout(orientation="vertical")
		b.size_hint = (1, 1)
		label_info = Label(text="label text", color=(0,0,1,1), font_size=30)
		label_info.font_size = 30
		label_info.size_hint = (None, None)
		label_info2 = Label(text="label text2", color=(0,0,1,1), font_size=30)
		label_info2.font_size = 30
		label_info2.size_hint = (1, 1)
		#b.add_widget(label_info)
		b.add_widget(label_info2)
		return b

	def _keyboard_closed(self):
		self._keyboard.unbind(on_key_down=self._on_keyboard_down)
		self._keyboard = None

	def _on_keyboard_down(self, keyboard, keycode, text, modifiers):
		if keycode[1] == 'q':
			App.get_running_app().stop()

		elif keycode[1] == "f":
			# recognize stroke

if __name__ == "__main__":
	# scaled monitor dimensions for canvas
	canvas_width = W/2
	canvas_height = H/2

	points = []                 # where to store drawn points
	stroke_id = 0
	result = -1                 # result object
	pcr = PCRecognizer()        # algorithm class initialization
	cv2_window_name = "LEAP DATA"

	MyApp().run()

"""
"""
_________________TKINTER GUI_________________

# MAIN BLOCK
if __name__ == "__main__":
	# scaled monitor dimensions for canvas
	canvas_width = W/2
	canvas_height = H/2

	points = []                 # where to store drawn points
	stroke_id = 0
	result = -1                 # result object
	clicked = False
	radius = 2                  # size of point on canvas
	lp = Point(0, 0, -1)        # auxiliar point for mouse drawing
	pcr = PCRecognizer()        # algorithm class initialization
	window_name = "LEAP DATA"
	recording = False

	# Leap setting up
	listener = leap_listener()
	controller = Leap.Controller()

	# listener initializer
	controller.add_listener(listener)

	# keeping Leap Motion working from background
	controller.set_policy(Leap.Controller.POLICY_BACKGROUND_FRAMES)


	# CANVAS EVENTS
	def click(e):
		print("click")
		if e.num == 1:
			print("start point: (",e.x,",",e.y,")")
			x1, y1 = (e.x - radius+1), (e.y - radius+1)
			x2, y2 = (e.x + radius+1), (e.y + radius+1)

			clicked = True
			global stroke_id, num_points
			stroke_id += 1
			points.append(Point(e.x, e.y, stroke_id))
			canvas.create_oval(x1, y1, x2, y2, fill = "#aaaaaa")

	def move(e):
		np = Point(e.x, e.y, -1)
		if distance(lp, np) > 10:
			x1, y1 = (e.x - radius), (e.y - radius)
			x2, y2 = (e.x + radius), (e.y + radius)
			global stroke_id, points
			points.append(Point(e.x, e.y, stroke_id))
			canvas.create_oval(x1, y1, x2, y2, fill = "#000000")
			lp.x, lp.y = e.x, e.y

	def release(e):
		print("release")
		if e.num == 1:
			print("ending point: (",e.x,",",e.y,")")

		if e.num == 3:
			print("recognizing stroke")
			global stroke_id, pcr, points
			stroke_id = 0
			print("points_l: ",len(points))
			result = pcr.recognize(points)

			score = "Result: matched with "+result.name+" about "+str(round(result.score, 2))
			str_var.set(score)

			points = []  # clear points array


	# WINDOW.BIND EVENTS

	# exiting application
	def quit():
		window.destroy()
		cv2.destroyAllWindows()
		controller.remove_listener(listener)
		sys.exit()

	# clear canvas
	def clear():
		canvas.delete("all")

		str_var.set("")
		listener.c = 0
		listener.gesture = []

	# starting Leap mouse controll
	def s():
		print("s")
		listener.mouse.active = True if not listener.mouse.active else False

	# recognizing stroke
	def f_pressed():
		result = pcr.recognize(listener.gesture)
		score = "Result: matched with "+result.name+" about "+str(round(result.score, 2))
		str_var.set(score)

	# like left mouse click pressed
	def d_pressed():
		global pressed
		if not pressed:
			pressed = True
			cx, cy = win32api.GetCursorPos()
			win32api.mouse_event(win32con.MOUSEEVENTF_LEFTDOWN, cx, cy, 0, 0)
			time.sleep(.1)
		else:
			pressed = False
			cx, cy = win32api.GetCursorPos()
			time.sleep(.1)
			win32api.mouse_event(win32con.MOUSEEVENTF_LEFTUP, cx, cy, 0, 0)

	def g_pressed():
		print("g_pressed()")

		if listener.capture_frame: # 2nd g press
			listener.capture_frame = False
			pc = Point_cloud("algo", listener.gesture)
			pc.draw_on_canvas()
			listener.c = 0
			#listener.lim_points = 20

		else: # 1st g press
			# countdown after getting gesture
			t = 5
			while t > 0:
				print(str(t))
				str_var_count.set(str(t))
				window.update()
				time.sleep(1)
				t -= 1

			str_var_count.set("")
			listener.capture_frame = True


	window = Tk()
	window.title("python canvas")
	window.bind("q", lambda e: quit())
	window.bind("c", lambda e: clear())
	window.bind("s", lambda e: s())             # starting Leap capture
	window.bind("d", lambda e: d_pressed())
	window.bind("f", lambda e: f_pressed())
	window.bind("g", lambda e: g_pressed())     # starting Leap frame capture
	str_var = StringVar()                       # variable for actualize label text
	str_var_count = StringVar()

	tab_control = ttk.Notebook(window)
	tab1 = ttk.Frame(tab_control)
	tab_control.add(tab1, text="canvas")
	tab_control.pack(expand=1, fill="both")

	canvas = Canvas(tab1, width=canvas_width, height=canvas_height)
	canvas.pack(expand=YES, fill=BOTH)
	canvas.bind("<Button>", click)
	canvas.bind("<B1-Motion>", move)
	canvas.bind("<ButtonRelease>", release)
	message = Label(tab1, text="Left click to recognize. q for quit. c for clear.\nTemplates: T", font=("Helvetica", 14))
	message.pack(side=BOTTOM)
	str_var.set("")
	label_score = Label(tab1, textvariable=str_var, font=("Helvetica", 14))
	label_score.pack(side=TOP)
	str_var_count.set("")
	label_count = Label(tab1, textvariable=str_var_count, font=("Helvetica", 14))
	label_count.grid(row=0, column=0)
	label_count.pack(side=TOP)

	tab2 = ttk.Frame(tab_control)
	tab_control.add(tab2, text="templates")
	tab_control.pack(expand=1, fill="both")

	if len(listener.frame.hands) == 2:
		print("recording")
		if listener.capture_frame: # 2nd g press
			listener.capture_frame = False
			pc = Point_cloud("algo", listener.gesture)
			pc.draw_on_canvas()
			listener.c = 0
			#listener.lim_points = 20

		else: # 1st g press
			# countdown after getting gesture
			t = 5
			while t > 0:
				print(str(t))
				str_var_count.set(str(t))
				window.update()
				time.sleep(1)
				t -= 1

			str_var_count.set("recording")
			listener.capture_frame = True


	mainloop()
"""
