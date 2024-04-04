#!/usr/bin/env python
import rospy
from custom_msgs.msg import person
from custom_srvs.srv import findPerson, recoverData, storeData
import mysql.connector

#parameters for connection to DB
HOST = 'localhost'
USER = 'mattia'
PW = 'mydb'
DB = 'Users'

#query the DB
def query_db(query, param = None):
    
    #connection
    conn = mysql.connector.connect(
		    host=HOST,
		    user = USER,
		    password = PW,
		    database = DB)
    cursor = conn.cursor()
    
    #manage the query if it contains parameters
    if param is None:
	cursor.execute(query)
    else:
	cursor.execute(query, param)

    #return data requested
    rows = cursor.fetchall()
    return rows

#insert a new record into the DB
def insert_into_db(query, param):
    try:
	#connection to the DB
	conn = mysql.connector.connect(
			host=HOST,
			user = USER,
			password = PW,
			database = DB)
	cursor = conn.cursor()
	cursor.execute(query, param)
	conn.commit()

    #return True if data correctly inserted, else return False
	return True

    except:
	return False

#service to check if person with that code is into the DB
def find_person(req):
    code = req.code
    print "search for "+ code
    sql_query = "SELECT * FROM people WHERE code = %(code)s"
    param = {"code":code}
    res = query_db(sql_query, param)
    if len(res) == 0:
	print "Person not registered"
	return False
    else:
	print "Person found"
	return True

#service to get all data of the person with that code from the DB 
def recover_data(req):
    code = req.code
    sql_query = "SELECT name, surname, age FROM people WHERE code = %(code)s"
    param = {"code":code}
    res = query_db(sql_query, param)
    p = person()
    p.code = code
    p.name = res[0][0]
    p.surname = res[0][1]
    p.age = res[0][2]
    print "Data recovered"
    return p

#service to insert new person into the DB
def store_data(req):
    sql_query = "INSERT INTO people (code, name, surname, age) VALUES (%(code)s,%(name)s,%(surname)s,%(age)s)"
    param = {"code":req.person.code,
             "name":req.person.name,
	     "surname":req.person.surname,
	     "age":req.person.age}
    res = insert_into_db(sql_query, param)
    print "Data stored"
    return res

#main loop
if __name__ == '__main__':
	
    #initailize node
    rospy.init_node("db_connection")
    
    #define services
    s1 = rospy.Service("/find_person", findPerson, find_person)
    s2 = rospy.Service("/recover_data", recoverData, recover_data)
    s3 = rospy.Service("/store_data", storeData, store_data)
    
    #keeps python from exiting until this node is stopped
    rospy.spin()
    
