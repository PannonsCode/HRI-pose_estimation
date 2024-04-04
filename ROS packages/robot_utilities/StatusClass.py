#!/usr/bin/env python
import rospy
from custom_msgs.msg import state, person

#class to manage the state of the robot 
class Status:
	
    #initialize the state msg
	def __init__(self):
		
	    self.status = state()
	    self.status.person_which_interact = person()
	    self.status.face_detected = False
	    self.status.keyword_detected = False
	    self.person_ready = False
	    self.session_ended = False

    #return the entire state msg
	def get_status(self):
	    return self.status
   
	#set a field of the state msg   
	def set_status_param(self, param, value):
		
		if param == "face_detected":
			self.status.face_detected = value
		elif param == "keyword_detected":
			self.status.keyword_detected = value
		elif param == "person_ready":
			self.status.person_ready = value
		elif param == "session_ended":
			self.status.session_ended = value

	#set the data of the person who is interacting
	def set_person(self, value):
		self.status.person_which_interact=value
