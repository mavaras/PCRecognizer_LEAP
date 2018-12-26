import sys
from PyQt4.QtGui import *
from PyQt4.QtCore import *
from opgui import *
import win32api
import Leap
from constants import *
import math

# calculates distance between two given points
def distance(x, y, x2, y2):
	dx = x - x2
	dy = y - y2
	return math.sqrt(dx*dx + dy*dy)

class leap_listener(Leap.Listener):
	def on_init(self, controller):
		print("on_init")
		self.fingers_pos = [[0,0,0] for c in range(5)]

	def on_connect(self, controller):
		print("conected")
		
	def on_frame(self, controller):
		canvas.clear()
		frame = controller.frame()
		global canvas
		for hand in frame.hands:
			if hand.is_right:
				for finger in hand.fingers:
					f_pos = finger.tip_position
					x = f_pos.x
					y = LEAP_H - f_pos.y
					z = f_pos.z
					x = x + LEAP_W/2
					x = (W * x) / LEAP_W
					y = (H * y) / LEAP_H
					print(finger.tip_velocity.z)
					"""if finger.type == 1:
						if z < -20:
							z = 7
						elif z < 0:
							z = 15
						elif z < 30:
							z = 23
						elif z < 70:
							z = 34
						elif z < 130:
							z = 45"""
					
					z = 20
						
					"""if finger.type == 1:
						z = (60-z)*40 / 10
					else:
						z = 20"""
					if abs(finger.tip_velocity.z) < 200 and finger.type == 1 and distance(self.fingers_pos[finger.type][0], self.fingers_pos[finger.type][1], x, y) > 2:
						canvas.add_to_path(x, y, z)
						self.fingers_pos[finger.type] = (x, y, z)
					else:
						canvas.add_to_path(self.fingers_pos[finger.type][0], self.fingers_pos[finger.type][1], self.fingers_pos[finger.type][2])
				
				canvas.draw_path()

# GUI code begins here
# canvas object
class Widget_canvas(QWidget):
	path = QPainterPath()
	def __init__(self, parent):
		print("canvas init")
		super(Widget_canvas, self).__init__(parent)

	def clear(self):
		aux = QPainterPath()
		self.path = aux
		
	def keyPressEvent(self, event):
		if event.key() == Qt.Key_Q:
			global exit
			exit = True
			self.close()
			sys.exit()
		
	def paintEvent(self, event):
		canvas = QtGui.QPainter(self)
		pen = QPen()

		# drawing grid
		pen.setWidth(1.4)
		pen.setColor(Qt.black)
		canvas.setPen(pen)
		interval = 40
		for c in range(interval, canvas_w, interval):
			for j in range(interval, canvas_h, interval):
				canvas.drawLine(c, 0, c, canvas_h)
				canvas.drawLine(0, j, canvas_w, j)

		pen.setWidth(3.4)
		pen.setColor(Qt.red)
		canvas.setPen(pen)	
		canvas.drawPath(self.path)

	def draw_path(self):
		self.update()
		
	def add_to_path(self, x, y, radius):
		self.path.addEllipse(QtCore.QRectF(x, y, radius, radius))

"""class MainWindow(QtGui.QMainWindow, Ui_MainWindow):
	def __init__(self, *args, **kwargs):
		QtGui.QMainWindow.__init__(self, *args, **kwargs)
		self.setWindowFlags(Qt.FramelessWindowHint)
		#self.setWindowOpacity(0)
		
		self.widget_canvas = canvas
		self.setupUi(self)

	# collection of key events binded to GUI
	def keyPressEvent(self, event):
		if event.key() == Qt.Key_Q:
			main_window.close()
			sys.exit()
"""
if __name__ == "__main__":
	listener = leap_listener()
	controller = Leap.Controller()
	controller.add_listener(listener)

	app = QtGui.QApplication([])
	canvas = Widget_canvas(None)
	canvas_h = win32api.GetSystemMetrics(1)
	canvas_w = win32api.GetSystemMetrics(0)
	print("screen resolution: "+str(canvas_w)+" "+str(canvas_h))

	canvas.setWindowFlags(Qt.FramelessWindowHint)
	canvas.setAttribute(Qt.WA_TranslucentBackground)
	canvas.showFullScreen()

	app.exec_()

	"""main_window = MainWindow()
	main_window.show()"""
