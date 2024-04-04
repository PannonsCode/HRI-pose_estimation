#!/usr/bin/env python
import rospy
import roslaunch
import sys
import subprocess
import threading
import time

#define launcher
launcher = roslaunch.scriptapi.ROSLaunch()

#define nodes to lauch
stateServiceNode = roslaunch.core.Node("robot_services", "robot_state_service.py", "robot_state")
dbServiceNode = roslaunch.core.Node("robot_services", "db_services.py", "db_services")
asrServerNode = roslaunch.core.Node("robot_scan", "asr_server.py", "asr_server")
faceDetectorNode = roslaunch.core.Node("robot_scan", "face_detector.py", "face_detector")
keywordDetectorNode = roslaunch.core.Node("robot_scan", "keyword_detector.py", "keyword_detector")
startInteractionNode = roslaunch.core.Node("interaction", "start_interaction.py", "start_interaction")
startSessionNode = roslaunch.core.Node("interaction", "start_session.py", "start_session")
interfaceNode = roslaunch.core.Node("interface", "interface.py", "human_robot_interface")

Nodes = [stateServiceNode, dbServiceNode, asrServerNode, interfaceNode,
	 faceDetectorNode, keywordDetectorNode, startInteractionNode, startSessionNode]

#check if a node is alive
def dead_node(node_name):
    result = subprocess.check_output(['rosnode', 'info', node_name], stderr=subprocess.STDOUT)
    return "unknown node" in result

#check a node in a parallel thread
def check_nodes(event):

    #main loop of the thread
    while True:
	time.sleep(15) #delay to let a better check of the node
	#resart nodes 
	if dead_node("/start_interaction"):
	    for node in Nodes[4:]:
		launcher.launch(node)

	if dead_node("/asr_server"):
	    launcher.launch(asrServerNode)

	if dead_node("/human_robot_interface") or event.is_set():
	    break

	if dead_node("/face_detector") and not dead_node("/keyword_detector"):
	    launcher.launch(faceDetectorNode)
	elif not dead_node("/face_detector") and dead_node("/keyword_detector"):
	    launcher.launch(keywordDetectorNode)

    return 0

#check a node in a parallel thread
def check_start_nodes(event):

    #main loop of the thread
    while True:
	time.sleep(20) #delay to let a better check of the node
	#resart nodes 
	if dead_node("/face_detector") and not dead_node("/keyword_detector"):
	    launcher.launch(faceDetectorNode)
	elif not dead_node("/face_detector") and dead_node("/keyword_detector"):
	    launcher.launch(keywordDetectorNode)
	elif (dead_node("/face_detector") and dead_node("/keyword_detector")) or event.is_set():
	    break
    return 0

#start all nodes
def start_nodes():
    #launch nodes
    for node in Nodes:
	launcher.launch(node)

#main loop
if __name__ == '__main__':

    try:
        #start launcher
        launcher.start()

        #create thread
	stop_event = threading.Event()
        thread = threading.Thread(target=check_nodes, args=(stop_event,))
        thread.daemon = True

        #thread2 = threading.Thread(target=check_start_nodes, args=(stop_event,))
        #thread2.daemon = True

        #init node
        rospy.init_node("launcher")
        start_nodes()

        #start the thread
        thread.start()
	#thread2.start()

	#terminate program
	thread.join()
	#thread2.join()
	sys.exit(0)

    except:
	#terminate in case of exception
	stop_event.set()
        sys.exit(0)
