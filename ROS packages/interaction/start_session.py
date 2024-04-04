#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image
from custom_msgs.msg import state, person
from custom_srvs.srv import findPerson, recoverData, storeData
from robot_utilities.textGenerationAPI import generate_text
from robot_utilities.pubs_and_srvs import textPub, imagePub, endSessionPub, setStatusService, getStatusService
import cv2
from cv_bridge import CvBridge
from robot_utilities.PoseEstimatorClass import PoseEstimator
import numpy as np
import time
from robot_utilities.utils import convert_image, get_image_advice, get_ref_keypoinys

poseEstimator = PoseEstimator('localhost', 8888)
LEVELS = ["Level 1", "Level 2"]
IMG4LEVEL = 2
bridge = CvBridge()

#callback for '/status/start_session' subscriber
def start_session(data):

    if data.person_ready:

	#connect to the server for pose estimation
	poseEstimator.connect()

	#loop to show all images for each level
	scores = []
	for l in LEVELS:
	    
	    if l != "Level 1":
		prompt="Tell me that a level has been passed and that we will take a break for a few seconds, during which you must tell a short joke."
		text = generate_text(prompt)
		textPub.publish(text)
		time.sleep(15)
		textPub.publish("Now let's go to next level!!")
		time.sleep(3)
	    
	    
	    for i in range(IMG4LEVEL):
		
		image_index = l+"-"+"image"+str(i)
		
		#get text for explaining next image
		advice = get_image_advice(image_index)

		#Show reference image
		image_ref = convert_image(str(i)+"_"+l+".jpg", from_="disk", to_topic=True)
		imagePub.publish(image_ref)
		textPub.publish(advice)
		time.sleep(10)
		keypoints_ref = get_ref_keypoinys(image_index)
		textPub.publish("Now replicate the pose and stand still until the message of \"Pose executed\" appear...")
		time.sleep(7)

		try:
		    #take photo from camera and detect pose
		    image_cam = rospy.wait_for_message("/camera/rgb/image_color", Image, timeout=10000.0)
		    textPub.publish("Pose executed, evaluation...")
		    cv_image = convert_image(image_cam, from_="topic")
		    image_cam_with_pose, keypoints_cam = poseEstimator.send_and_receive(cv_image)
		    image_cam_with_pose = np.uint8(image_cam_with_pose)
		    image_cam_pub = convert_image(image_cam_with_pose, to_topic=True)
		    imagePub.publish(image_cam_pub)
		
		#if the pose is not detected
		except ValueError or TypeError:
		    
		    textPub.publish("Pose not detected, restarting session...")
		    #update status and publish it
		    setStatusService("session_ended", True)
		    res = getStatusService()
		    endSessionPub.publish(res.robot_state)

		    #shutdown node
		    rospy.signal_shutdown("END SESSION")
		
		#pose comparison
		similarity = poseEstimator.compare_poses(keypoints_ref, keypoints_cam)
		scores.append(similarity)
		
		#give a mark (0-100%) on the accuracy in replicating the pose
		textPub.publish("You get: "+str(similarity)+"% of accuracy")
		time.sleep(3)

	#disconnect from server for pose estimation
	poseEstimator.send_and_receive(b"STOP")
	poseEstimator.close_connection()
	textPub.publish("Ending session, wait a moment please...")
	
        #give a final mark (mean of the collected marks)
        img = convert_image("nao3.jpeg", from_="disk", to_topic=True)
        imagePub.publish(img)
	final_score = round(sum(scores)/len(scores), 2)
        prompt = "Report that the session is over and that all levels have been successfully completed. Communicate the final score of ({}%) to the person in front of you and thank them for trusting you with these exercises, hoping to see them again soon. Before saying goodbye, he tells her a curiosity about the world of physical fitness and at the end he says goodbye.".format(str(final_score))
        text = generate_text(prompt)
        textPub.publish(text)
        time.sleep(50)
	textPub.publish("BYE!! See you sooon!!")
	time.sleep(10)

	#update status and publish it
	setStatusService("session_ended", True)
	res = getStatusService()
	endSessionPub.publish(res.robot_state)

	#shutdown node
	rospy.signal_shutdown("END SESSION")

if __name__ == "__main__":
	
    #initialize node
    rospy.init_node("start_session")

    #define subscriber
    sub = rospy.Subscriber("/status/start_session", state, start_session)
    
    #keeps python from exiting until this node is stopped
    rospy.spin()

    #disconnect subscribers
    sub.unregister()
