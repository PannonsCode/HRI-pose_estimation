import socket
import pickle
import cv2
import struct
import torch
import torchvision
import numpy as np
from PIL import Image
from torchvision.transforms import transforms as transforms
import matplotlib
import matplotlib.pyplot as plt
import time
import sys
import warnings

class PoseEstimationServer:
	
	def __init__(self, BIND="0.0.0.0", PORT=8888):

		#model and tools for pose estimation
		self.model = torchvision.models.detection.keypointrcnn_resnet50_fpn(pretrained=True, num_keypoints=17)
		self.device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
		self.model.to(self.device).eval()
		self.transform = transforms.Compose([transforms.ToTensor()])
		self.edges = [(0, 1), (0, 2), (2, 4), (1, 3), (6, 8), (8, 10),
					  (5, 7), (7, 9), (5, 11), (11, 13), (13, 15), (6, 12),
			          (12, 14), (14, 16), (5, 6)]

		#set the server
		self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
		server_address = (BIND, PORT)
		self.server_socket.bind(server_address)
		self.server_socket.listen(1)

	def connect(self):
		#wait for a client and accept connection
		print("\nWait for a connection...")
		self.client_socket, self.client_address = self.server_socket.accept()
		print("Connection accepted by: ", self.client_address)

	def listen_data(self):

		#receive data
		data = b""
		payload_size = struct.calcsize(">L")
		while True:
			while len(data) < payload_size:
				data += self.client_socket.recv(4096)

			packed_msg_size = data[:payload_size]
			data = data[payload_size:]
			msg_size = struct.unpack(">L", packed_msg_size)[0]

			while len(data) < msg_size:
				data += self.client_socket.recv(4096)

			frame_data = data[:msg_size]
			data = data[msg_size:]
			frame = pickle.loads(frame_data, encoding='bytes')

			with warnings.catch_warnings():
				warnings.simplefilter("ignore")
				if (b"STOP") in frame:
					print("\nClosing connection to: ", self.client_address)
					self.close_connection()
					break

			#process image
			print("\nNew data received")
			print("Processing image...")
			processed_frame, keypoints = self.process_image(frame)
			print("Image processed")

			#send back the image to the client
			print("Sending image")
			processed_frame_data = pickle.dumps({'frame':processed_frame, 'keypoints':keypoints}, protocol=2)
			msg_size = struct.pack(">L", len(processed_frame_data))
			self.client_socket.sendall(msg_size + processed_frame_data)
			print("Image sent")

	def close_connection(self):
		self.client_socket.close()
		self.server_socket.close()
		print("\nConnection closed")

	def process_image(self, img):

		#prepare image for the input of the model
		orig_numpy = np.array(img, dtype=np.float32)
		orig_numpy = cv2.cvtColor(orig_numpy, cv2.COLOR_BGR2RGB) / 255.
		image = self.transform(Image.fromarray(np.uint8(orig_numpy * 255)))
		image = image.unsqueeze(0).to(self.device)

		#prediction
		with torch.no_grad():
			outputs = self.model(image)
		output_image, keyP = self.draw_keypoints(outputs, orig_numpy)
		output_image = cv2.cvtColor(output_image, cv2.COLOR_BGR2RGB)*255

		return output_image, keyP

	def draw_keypoints(self, outputs, image):

		keypoints = []
		# the `outputs` is list which in-turn contains the dictionaries 
		for i in range(len(outputs[0]['keypoints'])):
			candidate_keypoints = outputs[0]['keypoints'][i].cpu().detach().numpy()

			# proceed to draw the lines if the confidence score is above 0.9
			if outputs[0]['scores'][i] > 0.9:

				keypoints = candidate_keypoints[:, :].reshape(-1, 3)

				for p in range(keypoints.shape[0]):
					# draw the keypoints
					cv2.circle(image, (int(keypoints[p, 0]), int(keypoints[p, 1])), 
								3, (0, 0, 255), thickness=2, lineType=cv2.FILLED)

				for e in self.edges:
					# join the keypoint pairs to draw the skeletal structure
					cv2.line(image, (int(keypoints[e, 0][0]), int(keypoints[e, 1][0])),
							(int(keypoints[e, 0][1]), int(keypoints[e, 1][1])),
							(255, 0, 0), thickness=2, lineType=cv2.LINE_AA)
			else:
				continue
		
		return image, keypoints

#check if connection to the server succeed
def connection_succes():
	try:
		server = PoseEstimationServer()
		server.connect()
		return server
	except OSError or KeyboardInterrupt or BrokenPipeError:
		return False

if __name__=='__main__':

	print("Server opened")
	server = False
	try:
		while True:
			server = connection_succes()
			if server:
				server.listen_data()
			else:
				print("\nWait a moment to free net resources... ")
				time.sleep(10)
	except KeyboardInterrupt:
		print("\Closing Server...")
		if server:
			server.close_connection()
		print("Closed.")
		sys.exit(0)
