#!/usr/bin/env python
from plutodrone.srv import *
import rospy
from std_msgs.msg import String

class request_data():
	"""docstring for request_data"""
	def __init__(self):
		rospy.init_node('drone_board_data')
		data = rospy.Service('PlutoService', PlutoPilot, self.access_data)
		self.pub = rospy.Publisher('/temp',String, queue_size=5)
		rospy.spin()

	def access_data(self, req):
		 
                 print "accx = " + str(req.accX), "accy = " + str(req.accY), "accz = " + str(req.accZ)
		 print "gyrox = " + str(req.gyroX), "gyroy = " + str(req.gyroY), "gyroz = " + str(req.gyroZ)
		 print "magx = " + str(req.magX), "magy = " + str(req.magY), "magz = " + str(req.magZ)
		 print "roll = " + str(req.roll), "pitch = " + str(req.pitch), "yaw = " + str(req.yaw)
		 print "altitude = " +str(req.alt)
                 
		 self.yaw = req.yaw
		 self.roll = req.roll
		 self.pitch = req.pitch
		 self.altitude = req.alt
		 list1=str(self.roll)+" "+str(self.pitch)+" "+str(self.yaw)+" "+str(self.altitude)
                 
		 self.pub.publish(list1)
		 rospy.sleep(.1)
		 return PlutoPilotResponse(rcAUX2 =1500)

test = request_data()
		
