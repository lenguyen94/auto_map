#! /usr/bin/env python

import rospy
from std_msgs.msg import Float32
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
import time

roll = pitch = yaw = 0.0
NAMESPACE = rospy.get_namespace()

def get_rotation (msg):
	global roll, pitch, yaw 
	orientation_quat =  msg.pose.pose.orientation
	
	orientation_list = [orientation_quat.x, orientation_quat.y, orientation_quat.z, orientation_quat.w]
	(roll, pitch, yaw) = euler_from_quaternion(orientation_list)
		
	#print yaw
	
	#Optional to put NAMESPACE for Publisher
	pub = rospy.Publisher((NAMESPACE+'my_euler_angle'), Float32, queue_size = 1)
	pub.publish(yaw)

#def callback (msg):
	#print msg.data

if __name__ == '__main__':
	rospy.init_node('my_quat_to_euler') 

	sub = rospy.Subscriber((NAMESPACE+'odom'), Odometry, get_rotation)
    
	#sub2 = rospy.Subscriber('/my_euler_angle', Float32, callback)
	rate = rospy.Rate(10) #Go through loop 10 times per second. E.g. 10 Hz

	while not rospy.is_shutdown():	 
		rate.sleep()


