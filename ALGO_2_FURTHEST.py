#! /usr/bin/env python

#Navigation Algorithm 2

import os
import rospy
import numpy as np
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32
from std_msgs.msg import String
from custom_message.msg import Float32List
import time

#Constants for Turtlebot3
PI = 3.142
DISTANCE_LIMIT = 0.3
TB3_VIEW_MIN = 156 #156 is 157 degrees. Point of view between 157 to 202 degrees.
TB3_VIEW_MAX = 201 #201 is 202 degrees. Point of view between 157 to 202 degrees.
NAMESPACE = rospy.get_namespace()

#Global variables for Turtlebot3
command = "stop"
prev_euler_angle = 0
prev_furthest_object_angle = 0

#Global flags for Turtlebot3 rotation
rotation_needed = rotation_amount = 0
rotation_direction = "LEFT" #default



class LaserSubs (object):

    def __init__(self):
        self.laser_ranges = []
        rospy.Subscriber((NAMESPACE+'scan'), LaserScan, self.LaserData)
    
    def LaserData(self,msg):
        self.laser_ranges = msg.ranges
        #print len(self.laser_ranges) #to check number of data in the range
        #print (self.laser_ranges[180])
    
    def read_laser_reading(self):
        return (self.laser_ranges)


class EulerSubs (object):

    def __init__(self):
        self.euler_angle = 0.0
        rospy.Subscriber((NAMESPACE+'my_euler_angle'), Float32, self.EulerAngle)
    
    def EulerAngle (self, msg):
        self.euler_angle = msg.data
        #print self.euler_angle
    
    def read_euler_angle(self):
        return self.euler_angle


class Data (object):

    def __init__ (self):
        self.laser_object = LaserSubs()
        self.euler_object = EulerSubs()
        
        self.nearest_object_dist = 0.0
        self.furthest_object_dist = 0.0
        
        self.nearest_object_angle = 180.0
        self.furthest_object_angle = 180.0
        
        self.curr_euler_angle = 0.0
    
    
    def Read_Distance (self):
        self.laser_range = self.laser_object.read_laser_reading()

        self.numpy_laser_range = np.array([self.laser_range]) #creates a numpy array
        
        #self.tb3_view_laser_range = self.numpy_laser_range[:, TB3_VIEW_MIN:(TB3_VIEW_MAX)] #inclusive start and end range [149:209]
        
        angle = 0
	
        #for x in np.nditer(self.tb3_view_laser_range, flags=['zerosize_ok']):
        for x in np.nditer(self.numpy_laser_range, flags=['zerosize_ok']):
            #For nearest_object
            if angle >= TB3_VIEW_MIN and angle <= TB3_VIEW_MAX:
                if (angle == TB3_VIEW_MIN): #for initial angle value, reinitialize nearest object distance and angle
                    if x == 0:
                        self.nearest_object_dist = 3.5
                        self.nearest_object_angle = angle
                    else:
                        self.nearest_object_dist = x
                        self.nearest_object_angle = angle
            
                else:
                    if x < self.nearest_object_dist and x != 0: #the smaller the value, the closer the object is
                        self.nearest_object_dist = x
                        self.nearest_object_angle = angle
        
            #For furthest_object
            #LiDar sensor returns 0 value for both (distance < 0.12m) and (distance > 3.5m)
            
            #Initialize a default case, just in case that all x is 0 reading.
            if angle == 0:
                self.furthest_object_dist = 0.12 #shortest distance LiDar can detect.
                self.furthest_object_angle = 359 #opposite direction from TB3
            
            if x > self.furthest_object_dist:
                self.furthest_object_dist = x
                self.furthest_object_angle = angle
        
            angle = angle + 1

    def Read_Angle (self):
       self.curr_euler_angle = self.euler_object.read_euler_angle()
       #print self.curr_euler_angle
        


#########################################################################
#
#  Walker() function implements the navigation algorithm for Turtlebot3
#
#########################################################################

def Walker (Data):
               
    global prev_euler_angle, prev_furthest_object_angle, rotation_needed, rotation_amount, rotation_direction
    
    curr_data = Data
    
    nearest_object_dist = curr_data.nearest_object_dist
    nearest_object_angle = curr_data.nearest_object_angle
    
    furthest_object_dist = curr_data.furthest_object_dist
    furthest_object_angle = curr_data.furthest_object_angle
    curr_euler_angle = curr_data.curr_euler_angle
    
    #Twist() initializes a Twist object
    twist = Twist() 

    #Optional to put NAMESPACE for Publisher
    pub = rospy.Publisher((NAMESPACE+'cmd_vel'), Twist, queue_size = 1)
	
    if command == "move":
 	if rotation_needed == 1:
		if rotation_direction == "LEFT":
		    twist.linear.x = 0.00
		    twist.angular.z = 1.0 #rotate counter clockwise
		    pub.publish(twist)
		else:
		    twist.linear.x = 0.00
		    twist.angular.z = -1.0 #rotate clockwise
		    pub.publish(twist)
	    
		curr_angle = curr_euler_angle
	       
		#print "Rotating " + rotation_direction + " " + str((rotation_amount*180)/PI) + " degrees"
		#print "Furthest object angle: " + str(prev_furthest_object_angle)
		
		if abs(curr_angle - prev_euler_angle) >= rotation_amount:
		    #print "Rotation completed"  
		    twist.linear.x = 0.00                                        
		    twist.angular.z = 0.0
		    pub.publish(twist)
		    
		    prev_euler_angle = curr_angle
		    rotation_needed = 0
		    rotation_amount = 0

    	else:
		#0 value means infinity distance ahead (> 3.5m) or distance lesser than 0.12m.
		#Always assume it is infinity distance right at the start as it will cut off at 0.3m in if-condition

		#print "Furthest Object Distance: ", furthest_object_dist, "m"

		if nearest_object_dist < DISTANCE_LIMIT and nearest_object_dist != 0 :
		    twist.linear.x = 0.00
		    twist.angular.z = 0.00
		    pub.publish(twist)

		    prev_euler_angle = curr_euler_angle #The current angle now will be the previous angle later
		    prev_furthest_object_angle = furthest_object_angle
		   
		    rotation_needed = 1
		
		    if furthest_object_angle <= 179: #179 is 180 degrees
		        rotation_direction = "RIGHT"
		        rotation_amount = (((180 - furthest_object_angle) * PI) / 180.0)

		    elif furthest_object_angle > 179:
		        rotation_direction = "LEFT"
		        rotation_amount = (((furthest_object_angle - 180) * PI) / 180.0)
		    
		    #else: can put an else as safety net

		else:
		    twist.linear.x = -0.1
		    twist.angular.z = 0.00
		    pub.publish(twist)
	    
    elif command == "stop":
		twist.linear.x = 0.00
		twist.angular.z = 0.00
		pub.publish(twist)
	
		#print command

    elif command == "close":
		twist.linear.x = 0.00
		twist.angular.z = 0.00
		pub.publish(twist)
	
		#rospy.on_shutdown("Shutting down program")
		rospy.signal_shutdown("Quit")


def CheckCommand(msg):

	global command
	command = msg.data


if __name__ == '__main__':

    rospy.init_node('my_walker', disable_signals=True)    
    data_object = Data() #initialize data_object
   
    #Note that Lidar spins at 5 revs/sec
    rate = rospy.Rate(20) #Go through loop 10 times per second. E.g. 10 Hz

    rospy.Subscriber('/my_command', String, CheckCommand)

    while not rospy.is_shutdown():
       data_object.Read_Distance()
       data_object.Read_Angle()
       Walker(data_object)	 
       rate.sleep()


