#!/usr/bin/env python
import cv2
from cv_bridge import CvBridge
import rospy
import json
import numpy as np

IMAGES_PATH = "/home/mattia/workspaces/robot_functions/src/robot_utilities/images/image_"

bridge = CvBridge()

with open(IMAGES_PATH+"info.json", 'r') as file:
	info = json.load(file)

'''This is a function to elaborate images in different formats:
   It is possible read it from disk with argument from_="disk" (the argument image is a string);
   It is possible to read it from a ROS topic message with argument from_="topic" (the argument image is an image in a ROS topic message);
   If from_="" it is supposed the image is already in a cv2 format (the argument image is an image in cv2 format) .
   The function return an image in a ROS topic message if the argument to_topic=True;
   else return an image in a cv2 format "'''
def convert_image(image, from_="", to_topic=False):

	if from_ == "disk":
		image = cv2.imread(IMAGES_PATH+image)
	elif from_ == "topic":
		image = bridge.imgmsg_to_cv2(image, "bgr8")

	#resize the image
	image = cv2.resize(image, (600, 500))

	if to_topic:
		image = bridge.cv2_to_imgmsg(image, "rgb8")

	return image


def get_image_advice(index):
	return info[index]['advice']

def get_ref_keypoinys(index):
	return np.array(info[index]['keypoints'])
		
