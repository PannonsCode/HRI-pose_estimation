#!/usr/bin/env python
# -*- coding: utf-8 -*-
import socket
import pickle
import struct
import cv2
import numpy as np

#list of points reference to access keypoints and take points of interest to compare poses
pose_points_ref = [(6, 8, 10), (8, 6, 12), (6, 12, 14), (12, 14, 16), (12, 6, 5), 
				   (6, 5, 11), (5, 11, 13), (11, 13, 15), (11, 5, 7), (5, 7, 9)]

class PoseEstimator:
	
	#initialization
	def __init__(self, IP, PORT):
		self.client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
		self.server_address = (IP, PORT)

	#connect to a sever
	def connect(self):
		self.client_socket.connect(self.server_address)

	#close connection
	def close_connection(self):
		self.client_socket.close()

	#send and receive data
	def send_and_receive(self, img):

		#send data
		frame_data = pickle.dumps(img, protocol=2)
		msg_size = struct.pack(">L", len(frame_data))
		self.client_socket.sendall(msg_size + frame_data)

		#receive a string to disconnect from server
		if b"STOP" in frame_data:
			return ""

		#receive data
		data = b""
		payload_size = struct.calcsize(">L")

		while len(data) < payload_size:
			data += self.client_socket.recv(4096)

		packed_msg_size = data[:payload_size]
		data = data[payload_size:]
		msg_size = struct.unpack(">L", packed_msg_size)[0]

		while len(data) < msg_size:
			data += self.client_socket.recv(4096)

		processed_frame_data = data[:msg_size]
		data = data[msg_size:]

		#access to data
		processed_data_dict = pickle.loads(processed_frame_data)
		processed_frame = processed_data_dict['frame']
		keypoints = processed_data_dict['keypoints']

		return processed_frame, keypoints

	#compare cosine similarities of two set of keypoints
	def compare_poses(self, keypoints1, keypoints2):

		sim1 = self.find_cos_similarities(keypoints1)
		sim2 = self.find_cos_similarities(keypoints2)
		
		sim = [abs(abs(l1)-abs(l2)) for l1,l2 in zip(sim1,sim2)]
		similarity_score = 1 - (sum(sim)/len(sim))
		similarity_score = round(similarity_score, 4)*100

		#return 1 -> poses equals
		#return 0 -> poses different
		return similarity_score

	#output a list of cosine similarities
	def find_cos_similarities(self, keypoints):
		
		cos_similarities = []

		#take interest point from keypoints
		for p in pose_points_ref:
			x1 = keypoints[p[0], 0]
			y1 = keypoints[p[0], 1]
			xc = keypoints[p[1], 0]
			yc = keypoints[p[1], 1]
			x2 = keypoints[p[2], 0]
			y2 = keypoints[p[2], 1]

			#compute normalized vectors
			v1 = self.compute_unit_vector([xc,yc],[x1,y1])
			v2 = self.compute_unit_vector([xc,yc],[x2,y2])

			#compute cosine similarity
			cos_similarities.append(self.compute_cosine_similarity(v1,v2))

		return cos_similarities

	#compute the vector from p1 to p2 of lenght 1
	def compute_unit_vector(self, p1, p2):

		v = np.array(p2) - np.array(p1)
		norm_v = np.linalg.norm(v)

		return v / norm_v

	#compute cosine similarity
	def compute_cosine_similarity(self, vector1, vector2):

		dot_product = np.dot(vector1, vector2)
		norm_vector1 = np.linalg.norm(vector1)
		norm_vector2 = np.linalg.norm(vector2)

		return dot_product / (norm_vector1 * norm_vector2)

