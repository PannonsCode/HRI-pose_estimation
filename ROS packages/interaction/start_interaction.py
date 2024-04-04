#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from custom_msgs.msg import state, person
from custom_srvs.srv import *
import mysql.connector
from robot_utilities.textGenerationAPI import generate_text
from robot_utilities.pubs_and_srvs import *
import time
from robot_utilities.utils import convert_image

#check the right format of the code
def check_code(cf):

    if len(cf)!=16:
        return True
    else:
        return False

#function called by the subscriber: user recogition/registration insertin data - start session
def know_person_and_start_session(data):

    time.sleep(2)
    img = convert_image("nao2.jpeg", from_="disk", to_topic=True)
    imagePub.publish(img)

    #process executed only if a face and keyword detected
    if data.face_detected and data.keyword_detected:

        #print some messages on the interface (trhough a publisher)
        time.sleep(2)
        textPub.publish("Person detected, wait a moment...")
        prompt = "Memorize: you are a social robot and your task is to interact with the person in front of you, helping them to replicate some poses that you will show on the screen and to which you will give a rating. In reference to this: there is a person in front of you, explain your task to him and then ask him to enter his tax code to proceed with the recognition (or registration) and start the session as soon as possible. (All with max 100 words)"
        text = generate_text(prompt) 
        textPub.publish(text)

        #get the code from person
        inputReq.publish("")
        cf = rospy.wait_for_message("/input", String, timeout=10000.0)
        cf = cf.data

        #insert code again until the format is right
        while(check_code(cf)):
            textPub.publish("Wrong format, insert your code again")
            inputReq.publish("")
            cf = rospy.wait_for_message("/input", String, timeout=10000.0)
            cf = cf.data

        textPub.publish("Processing request...")
        time.sleep(2)

        #look if the person is already stored into the DB
        found = findPersonService(cf)

        #define a msg of type person
        p = person()
        
        #if person already stored, recover other data, otherwise store all data
        if found.response:

            #recover data already stored and inform user
            p = recoverDataService(cf).response
            textPub.publish("Data recovered")
            time.sleep(2)

        else:

            #Complete registration inserting all data
            textPub.publish("You are a new user: complete your registration inserting in the box below your data.")
            time.sleep(3)
            textPub.publish("Insert your name")
            p.code = cf
            inputReq.publish("")
            p.name = rospy.wait_for_message("/input", String, timeout=10000.0).data
            textPub.publish("Now insert your surname")
            inputReq.publish("")
            p.surname = rospy.wait_for_message("/input", String, timeout=10000.0).data
            textPub.publish("Finally insert your age")
            inputReq.publish("")
            p.age = int(rospy.wait_for_message("/input", String, timeout=10000.0).data)
            store = storeDataService(p)
            textPub.publish("You inserted: "+str(p)+". Registering your data...")
            time.sleep(3)

        #print message
        prompt = "Speak kindly in the first person: give only the welcome to {} and say the session is starting, wishing him good luck and fun and telling him to replicate the poses that are about to be shown.".format(p.name)
        text = generate_text(prompt)
        textPub.publish(text)
        time.sleep(20)

        #update state and publish it
        setStatusService("person_ready", True)
        setPersonService(p)
        res = getStatusService()
        startSessionPub.publish(res.robot_state)

    elif data.face_detected and not data.keyword_detected:
        textPub.publish("Face detected, waiting for the keyword...")

    elif not data.face_detected and data.keyword_detected:
        textPub.publish("Keyword detected, waiting for the face...")

#function called by the subscriber: session is ended and the node terminate
def end_session(data):

    if data.session_ended:
        #reset robot state to default values
        setStatusService("face_detected", False)
        setStatusService("keyword_detected", False)
        setStatusService("person_ready", False)
        setStatusService("session_ended", False)

        #reset interface
        img = convert_image("nao1.jpeg", from_="disk", to_topic=True)
        imagePub.publish(img)
        starting_text = "Waiting for detect a person: I'm looking for a face and listening for the keyword 'HELLO'"
        textPub.publish(starting_text)

        #shutdown node
        rospy.signal_shutdown("END INTERACTION")

#main loop
if __name__ == '__main__':

    #init node
    rospy.init_node('start_interaction')

    #define subscriber
    sub1 = rospy.Subscriber("/status/ready_to_start", state, know_person_and_start_session)
    sub2 = rospy.Subscriber("/status/end_session", state, end_session)
    
    #keeps python from exiting until this node is stopped
    rospy.spin()

    #disconnect subscribers
    sub1.unregister()
    sub2.unregister()
