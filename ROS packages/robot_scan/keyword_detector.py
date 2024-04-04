#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from custom_msgs.msg import state
from custom_srvs.srv import setStatus, getStatus
from robot_utilities.pubs_and_srvs import readyPub, textPub, setStatusService, getStatusService

KEYWORD = "hello"

#fuction called by the subscriber
def callback(data):
    
    #check: if keyword detected -> set the correspondig state field and publish a state msg
    if data.data == KEYWORD:
		print("Keyword detected")
		setStatusService("keyword_detected", True)
		response = getStatusService()
		readyPub.publish(response.robot_state)
		textPub.publish("KEYWORD DETECTED")
		rospy.signal_shutdown("END")

#main loop
if __name__ == '__main__':
    
    #initialize node
    rospy.init_node('keyword_detector')

	#when the node start (or restart), the keyword is always still not detected
    setStatusService("keyword_detected", False)
    starting_text = "Waiting for detect a person: I'm looking for a face and listening for the keyword 'HELLO'"
    textPub.publish(starting_text)

    #initialize subscriber and listen messages
    sub = rospy.Subscriber("/words_detected", String, callback)

    #keeps python from exiting until this node is stopped
    rospy.spin()

    #disconnect subscribers
    sub.unregister()
