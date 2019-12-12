#! /usr/bin/env python

import rospy
from std_msgs.msg import String

command = "stop" #initial command
command_list = ["stop","move","close"]

def pub_command(command):
	pub = rospy.Publisher('/my_command', String, queue_size = 1)
	pub.publish(command)

def get_command ():
	global command
	
	valid_command = 0	
	
	while (valid_command != 1):
		
		print ("\n")
		print ("List of command: ", command_list) 
	
		command = str(raw_input("Enter command: "))
		command = command.lower()

		if command in command_list:
			valid_command = 1
		else:
			print "Invalid command. See list of command for valid commands.\n"

if __name__ == '__main__':
	rospy.init_node('my_command') 

	rate = rospy.Rate(10) #Go through loop 10 times per second. E.g. 10 Hz

	while not rospy.is_shutdown():	
		pub_command(command) 
		get_command()
		rate.sleep()


