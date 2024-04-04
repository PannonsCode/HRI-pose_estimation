#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from custom_srvs.srv import setStatus, getStatus, setPerson
from robot_utilities.StatusClass import Status

#set the variable to store the state
robot_state = Status()

#fuction called by the service to return the actual state
def get_status(req):
    print("Service executed")
    return robot_state.get_status()
	
#fuction called by the service to set a field of the state
def set_status(req):
    robot_state.set_status_param(req.field, req.value)
    print("Service executed")
    return ""

#fuction called by the service to set a field of the state
def set_person(req):
    robot_state.set_person(req.value)
    print("Service executed")
    return ""
	
#main loop
if __name__ == '__main__':
	
    #initailize node
    rospy.init_node("robot_state")
    
    #define services
    s1 = rospy.Service("/get_robot_state", getStatus, get_status)
    s2 = rospy.Service("/set_robot_state", setStatus, set_status)
    s3 = rospy.Service("/set_robot_state/person", setPerson, set_person)
    
    #keeps python from exiting until this node is stopped
    rospy.spin()
    
