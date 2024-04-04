#!/usr/bin/env python
import cv2
from cv_bridge import CvBridge
from PyQt5.QtWidgets import QApplication, QWidget, QLabel, QVBoxLayout, QHBoxLayout, QLineEdit
from PyQt5.QtGui import QPixmap, QImage
from PyQt5.QtCore import QTimer, Qt, QThread
from PyQt5.QtGui import QImageReader
import sys
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image
from robot_utilities.pubs_and_srvs import inputPub, imagePub, textPub
from robot_utilities.utils import IMAGES_PATH

bridge = CvBridge()

#implement the class for the thread to show live video from cam
class LiveCam(QThread):
    
    def __init__(self, label):
	super(LiveCam, self).__init__()
	self.video_label = label

    def run(self):

        while(True):
	    try:
		frame = rospy.wait_for_message("/camera/rgb/image_color", Image, timeout=1000.00) #receive images from topic
	    except:
		print("Live cam not available: program terminated")
		break
	    cv_image = bridge.imgmsg_to_cv2(frame, "bgr8")
	    rgbImage = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
	    convertToQtFormat = QImage(rgbImage.data, rgbImage.shape[1], rgbImage.shape[0],
				       QImage.Format_RGB888)
	    convertToQtFormat = QPixmap.fromImage(convertToQtFormat)
	    pixmap = QPixmap(convertToQtFormat)
	    pixmap = pixmap.scaled(600, 500, Qt.KeepAspectRatio)
	    QApplication.processEvents()
	    self.video_label.setPixmap(pixmap)

#implement the main class for the graphic interface
class App(QWidget):

    def __init__(self):
        super(App, self).__init__()

	self.setWindowTitle('HUMAN-ROBOT INTERFACE')

	#flags
	self.data_inserted = ""
	self.inserted = False

	#Create layouts
	self.main_layout = QVBoxLayout(self)
        self.upper_layout = QHBoxLayout()
	self.upperLeft_layout = QHBoxLayout()
	self.upperRight_layout = QHBoxLayout()
	self.lower_layout = QHBoxLayout()
	self.lowerInput_layout = QHBoxLayout()

        #Compose layout
	self.upper_layout.addLayout(self.upperLeft_layout)
	self.upper_layout.addLayout(self.upperRight_layout)
        self.main_layout.addLayout(self.upper_layout)
        self.main_layout.addLayout(self.lower_layout)
	self.main_layout.addLayout(self.lowerInput_layout)

	#Add box for video
        self.video_label = QLabel(self)
	self.video_label.setAlignment(Qt.AlignCenter)
        self.upperLeft_layout.addWidget(self.video_label)
	
        #Add box for images
        self.image_label = QLabel(self)
	self.image_label.setAlignment(Qt.AlignCenter)
        self.upperRight_layout.addWidget(self.image_label)

        #Add box for texts
        self.text_label = QLabel(self)
	self.text_label.setAlignment(Qt.AlignCenter)
        self.lower_layout.addWidget(self.text_label)

	#Add box for input
        self.name_input = QLineEdit(self)
	self.lowerInput_layout.addWidget(self.name_input)
        self.name_input.returnPressed.connect(self.insert_data)

	#set final layout
	self.setLayout(self.main_layout)

    #Video live
    def start_video(self):
	self.video_thread = LiveCam(self.video_label)
	self.video_thread.start()

    '''NOTE: the functions show_image and show_text are duplicated with the suffix _callback:
	     the latter take as arguments the correspondent msg type (String and Image),
	     while the first two take as arguments the standard object types. '''
    #Show images on the interface
    def show_image(self, image_path):
        pixmap = QPixmap(image_path)
        pixmap = pixmap.scaled(400, 500)
        self.image_label.setPixmap(pixmap)

    #as show_text, but used by subscriber on "/show_image" topic
    def show_image_callback(self, image):
	cv_image = bridge.imgmsg_to_cv2(image, "bgr8")
	cv_image = cv2.resize(cv_image, (400, 500))
	height, width, channel = cv_image.shape
        bytes_per_line = 3 * width
        q_image = QImage(cv_image.data, width, height, bytes_per_line, QImage.Format_RGB888)
        pixmap = QPixmap(q_image)
        self.image_label.setPixmap(pixmap)

    #show text on the interface
    def show_text(self, text):
	self.text_label.setText(text)
	self.text_label.setAlignment(Qt.AlignCenter)
	self.text_label.setStyleSheet('background-color: rgb(33, 33, 33);'
				      'border-color: rgb(18, 18, 18);'
				      'color: rgb(255, 255, 255);'
				      'font: bold italic 20pt "Times New Roman";')
	self.text_label.setWordWrap(True)

    #As show_text, but used by subcriber on "/show_text" topic
    def show_text_callback(self, text):
	self.text_label.setText(text.data)
	self.text_label.setAlignment(Qt.AlignCenter)
	self.text_label.setStyleSheet('background-color: rgb(33, 33, 33);'
				      'border-color: rgb(18, 18, 18);'
				      'color: rgb(255, 255, 255);'
				      'font: bold italic 20pt "Times New Roman";')
	self.text_label.setWordWrap(True)

    #what happen when press "Enter" key on the keyboard
    def insert_data(self):
        self.data_inserted = self.name_input.text()
        self.name_input.clear()
	self.inserted = True

    #callback for subscriber on "/ask_input" topic
    def return_input(self, data):
	while not self.inserted:
	    pass
	inputPub.publish(self.data_inserted)
	self.inserted = False


if __name__ == '__main__':

    try:
	#initialize node
	rospy.init_node("Interface")

	#initialize graphic interface
	app = QApplication(sys.argv)
	window = App()
	window.showFullScreen()

	#start video streaming 
	window.start_video()

	#show initial text and initial image
	window.show_image(IMAGES_PATH+"nao1.jpeg")
	starting_text = "Waiting for detect a person: I'm looking for a face and listening for the keyword 'HELLO'"
	window.show_text(starting_text)

	#luanch subscribers
	sub1 = rospy.Subscriber("/show_image", Image, window.show_image_callback)
	sub2 = rospy.Subscriber("/show_text", String, window.show_text_callback)
	sub3 = rospy.Subscriber("/ask_input", String, window.return_input)
	sys.exit(app.exec_())

    except:
	#disconnect subscribers
	sub1.unregister()
	sub2.unregister()
	sub3.unregister()
	sys.exit(0)
