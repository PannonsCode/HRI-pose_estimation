#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge
from custom_msgs.msg import state
from custom_srvs.srv import setStatus, getStatus
from std_msgs.msg import String
from robot_utilities.pubs_and_srvs import readyPub, textPub, imagePub, setStatusService, getStatusService

#path of the model for face detection
MODEL_PATH = "/home/mattia/workspaces/robot_functions/src/robot_utilities/models/"

#fuction called by the subscriber
def callback(data):
	
    try:
	#convert Image msg into an Image OpenCV
	bridge = CvBridge()
	cv_image = bridge.imgmsg_to_cv2(data, "bgr8")

	#do face detection
	face_cascade = cv2.CascadeClassifier(MODEL_PATH+'haarcascade_frontalface_default.xml')
	gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
	faces = face_cascade.detectMultiScale(gray, scaleFactor=1.1, minNeighbors=10)

	#check: if face detected -> set the correspondig state field and publish a state msg
	if len(faces)>0:
	    print("face detected")
	    for (x,y,w,h) in faces:
			cv2.rectangle(cv_image, (x,y), (x+w, y+h), (255,0, 0), 2)
	    frame = bridge.cv2_to_imgmsg(cv_image, "rgb8")
	    imagePub.publish(frame)
	    setStatusService("face_detected", True)
	    response = getStatusService()
	    readyPub.publish(response.robot_state)
	    textPub.publish("FACE DETECTED")
	    rospy.signal_shutdown("END")
		
    except Exception as e:
	rospy.logerr(e)
	rospy.signal_shutdown("END")

#main loop
if __name__ == '__main__':
    
    #initialize node
    rospy.init_node('face_detector', anonymous=True)
    
    #when the node start (or restart), the face is always still not detected
    setStatusService("face_detected", False)
    starting_text = "Waiting for detect a person: I'm looking for a face and listening for the keyword 'HELLO'"
    textPub.publish(starting_text)

    #initialize subscriber and listen
    sub = rospy.Subscriber("/camera/rgb/image_color", Image, callback)

    #keeps python from exiting until this node is stopped
    rospy.spin()
    
    #disconnect subscribers
    sub.unregister()
