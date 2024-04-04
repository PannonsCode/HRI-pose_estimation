#!/usr/bin/env python
import rospy
from std_msgs.msg import String
import time
import sys
import socket
import json
from robot_utilities.pubs_and_srvs import speechPub

#class to manage connection
class ASRServer():

	#initialization
    def __init__(self, port):

        # Create a TCP/IP socket
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

        self.sock.settimeout(3) # timeout when listening (exit with CTRL+C)

        # Bind the socket to the port
        server_address = ('', port)
        self.sock.bind(server_address)
        self.sock.listen(1)

        print "ASR Server running on port ", port, " ..."
		self.connection = None

    #accept connection
    def connect(self):
        connected = False

	#Wait for a connection
        while (not connected):
            try:
                self.connection, client_address = self.sock.accept()
                self.connection.settimeout(3)
                connected = True
                print 'ASR Server Connection from ', client_address
            except:
                pass


    def listen(self):

	#Receive the data in small chunks and retransmit it
	while True: 

	    #manage data reception
	    try:
		data = self.connection.recv(2048)
		data = data.strip()
	    except socket.timeout:
		data = "***"
	    except:
		self.cleanup()
		break

	    self.connection.send('ACK\n\r')

	    #deal with data
	    if (data!=None and data !="" and data!="***" and data[0]!='$' and data!='KEEP_AWAKE'):
		    if data!='REQ':
			s = json.loads(data)
			words = s['hypotheses'][0]['transcription'].lower()
			speechPub.publish(words)
			print("Message published")
		    if data=='REQ':
			self.connection.send('ACK\n\r')
	    elif (data == None or data==""):
		self.cleanup()
		break

    #free resourced
    def cleanup(self):
        print('ASR Server: Cleaning up resources and shutting down.')
	self.connection.close()
	self.sock.close()

#main loop
if __name__ == "__main__":

    #accept connection on this port
    default_port = 9002

    #manage connection
    server = ASRServer(default_port)

    #initialize node
    rospy.init_node('word_detector')

    #conncet and listening from socket
    server.connect()
    server.listen()

    #terminate program
    rospy.signal_shutdown("stop asr server")
    
