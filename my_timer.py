#! /usr/bin/env python

import rospy
import time
from std_msgs.msg import String

start_time = 0
end_time = 0

start_time_recorded = 0
end_time_recorded = 0

command = "stop" #initial command

#timing are in seconds (s)
def timer ():
	global command, start_time, end_time, start_time_recorded, end_time_recorded
	
	if command == "move" and start_time_recorded == 0:
		start_time = time.time()
		print "Start Time: ", start_time

		start_time_recorded = 1
		end_time_recorded = 0

	elif command == "stop" and start_time_recorded == 1 and end_time_recorded == 0: 
		end_time = time.time()
		print "End Time: ", end_time
		
		print "Time elapsed: ", (end_time - start_time), "seconds"

		start_time_recorded = 0
		end_time_recorded = 1
		

def CheckCommand(msg):

	global command
	command = msg.data

if __name__ == '__main__':
	rospy.init_node('my_timer') 

	rate = rospy.Rate(10) #Go through loop 10 times per second. E.g. 10 Hz
	
	rospy.Subscriber('/my_command', String, CheckCommand)

	while not rospy.is_shutdown():	
		timer()
		rate.sleep()


