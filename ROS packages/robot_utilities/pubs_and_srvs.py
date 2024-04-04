#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image
from custom_msgs.msg import *
from custom_srvs.srv import *

#define publishers
readyPub = rospy.Publisher("/status/ready_to_start", state, queue_size=1)
'''readyPub pulish the state with the goal to start the interaction, in particular
   the fields checked by the subscribers of this topic are face_detected and keyword_detected'''

speechPub = rospy.Publisher("/words_detected", String, queue_size=10)
'''speechPub pulish the messsages (strings) listened by the node 'asr_server.py' from pkg robot_scan '''

startSessionPub = rospy.Publisher("/status/start_session", state, queue_size = 1)
'''readyPub pulish the state with the goal to start the session, in particular
   the field checked by the subscribers of this topic is person_ready'''

endSessionPub = rospy.Publisher("/status/end_session", state, queue_size = 1)
'''readyPub pulish the state with the goal to start the session, in particular
   the field checked by the subscribers of this topic is person_ready''' 

textPub = rospy.Publisher("/show_text", String, queue_size=1)
'''textPub pulish texts which will be shown through the interface'''

inputReq = rospy.Publisher("/ask_input", String, queue_size=1)
'''inputReq publish a void string and it used only to activate the function which will save the input 
   from the bar on the interface after the click on Enter'''

inputPub = rospy.Publisher("/input", String, queue_size=1)
'''inputPub publish the input saved'''

imagePub = rospy.Publisher("/show_image", Image, queue_size = 1)
'''imagePub publish the image which will be shown through the interface'''

#define proxy servers
findPersonService = rospy.ServiceProxy("/find_person", findPerson)
'''connect to a service to query the database looking for the record corresponding to the inserted code,
   check only if the record is presente  and return True or False'''

recoverDataService = rospy.ServiceProxy("/recover_data", recoverData)
'''connect to a service to query the database looking for the record corresponding to the inserted code,
   return all the data retrived'''

storeDataService = rospy.ServiceProxy("/store_data", storeData)
'''connect to a service to insert a new record into the database'''

setStatusService = rospy.ServiceProxy("/set_robot_state", setStatus)
'''connect to a service to set the indicated field of the object Status'''

getStatusService = rospy.ServiceProxy("/get_robot_state", getStatus)
'''connect to a service to return the indicated field of the object Status'''

setPersonService = rospy.ServiceProxy("/set_robot_state/person", setPerson)
'''connect to a service to set the field 'person'of the object Status'''

#NOTES:
'''the services are implemented in the files 'db_services.py' and 'robot_state_service' from pkg robot_services;
   the object which services deal with is an istance of the class StatusClass in pkg robot_utilities '''
