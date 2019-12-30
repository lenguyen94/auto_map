#! /usr/bin/env python

#Navigation Algorithm 1

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
prev_nearest_object_angle = 0

#Global flags for Turtlebot3 rotation
rotation_needed = rotation_amount = 0
rotation_direction = "LEFT" #default



class LaserSubs (object):

    def __init__(self):
        self.laser_ranges = []
        rospy.Subscriber((NAMESPACE+'scan'), LaserScan, self.LaserData)
        #rospy.spin() #will this work? No
    
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
        #rospy.spin() #will this work? No
    
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
        
        angle = 0
        
        #print self.tb3_view_laser_range.size
        
        for x in np.nditer(self.numpy_laser_range, flags=['zerosize_ok']):
            #For nearest_object
            if angle >= TB3_VIEW_MIN and angle <= TB3_VIEW_MAX:
                #for initial angle value, reinitialize nearest object distance and angle
                if (angle == TB3_VIEW_MIN):
                    if x == 0:
                        self.nearest_object_dist = 3.5
                        self.nearest_object_angle = angle
                    else:
                        self.nearest_object_dist = x
                        self.nearest_object_angle = angle
            
                else:
                    #the smaller the value, the closer the object is
                    if x < self.nearest_object_dist and x != 0:
                        self.nearest_object_dist = x
                        self.nearest_object_angle = angle

            #For furthest_object
            #LiDAR sensor returns 0 value for both (distance < 0.12m) and (distance > 3.5m)

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
               
    global prev_euler_angle, prev_nearest_object_angle, rotation_needed, rotation_amount, rotation_direction, prev_furthest_object_angle
    
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
   
    #print command
	
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
    #print "Nearest object angle: " + str(prev_nearest_object_angle)
		
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

            #print "Nearest Object Distance: ", nearest_object_dist, "m"

            if nearest_object_dist < DISTANCE_LIMIT and nearest_object_dist != 0 :
                twist.linear.x = 0.00
                twist.angular.z = 0.00
                pub.publish(twist)
                
                prev_euler_angle = curr_euler_angle #The current angle now will be the previous angle later
                prev_nearest_object_angle = nearest_object_angle
                prev_furthest_object_angle = furthest_object_angle
                
                rotation_needed = 1
                
                if nearest_object_angle <= 179: #179 is 180 degrees
                    rotation_direction = "LEFT"
                    rotation_amount = (((179 - nearest_object_angle) * PI) / 180.0)
                
                elif nearest_object_angle > 179:
                    rotation_direction = "RIGHT"
                    rotation_amount = (((nearest_object_angle - 179) * PI) / 180.0)
                
                #Ensure a minimum of 10 degrees rotation
                if rotation_amount < 0.174: #0.174 is around 10 degrees
                    rotatation_amount = 0.174
                
                turn_ok = 0
                while (turn_ok != 1):
                    turn_ok = TurnCheck(Data.laser_range, 179, 1)
                        
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


def TurnCheck (predict_range, predict_orientation, count):

    global rotation_amount

    numpy_predict_range = np.array([predict_range])

    if rotation_direction == "LEFT":
        #turn left, angle increase (e.g. 0 to 360)
        new_predict_orientation_centre = (predict_orientation + ((rotation_amount*180)/PI)) % 360
    else:
        #turn right, angle decrease (e.g. 360 to 0)
        new_predict_orientation_centre = abs(predict_orientation - ((rotation_amount*180)/PI)) % 360

    predict_TB3_VIEW_MIN = abs(new_predict_orientation_centre - 22.5) % 360

    predict_TB3_VIEW_MAX = (new_predict_orientation_centre + 22.5) % 360

    angle = 0
    predict_nearest_object_dist = 0
    predict_nearest_object_angle = 0

    for x in np.nditer(numpy_predict_range, flags=['zerosize_ok']):
        if angle >= predict_TB3_VIEW_MIN and angle <= predict_TB3_VIEW_MAX:
            #for initial angle value, reinitialize nearest object distance and angle
            if (angle == predict_TB3_VIEW_MIN):
                if x == 0:
                    predict_nearest_object_dist = 3.5
                    predict_nearest_object_angle = angle
                else:
                    predict_nearest_object_dist = x
                    predict_nearest_object_angle = angle
        
            else:
                #the smaller the value, the closer the object is
                if x < predict_nearest_object_dist and x != 0:
                    predict_nearest_object_dist = x
                    predict_nearest_object_angle = angle

        angle = angle + 1
        
        if angle > predict_TB3_VIEW_MAX:
        	####################################
        	## Two scenarios to stop branching:
        	##
        	##		1. No near object to avoid (predict_nearest_object_dist >= DISTANCE_LIMIT) 
        	##		2. Turn LEFT-LEFT or RIGHT-RIGHT
        	##

            if (predict_nearest_object_dist < DISTANCE_LIMIT and count < 5):
                if ((rotation_direction == "LEFT" and predict_nearest_object_angle > new_predict_orientation_centre) or 
                    (rotation_direction == "RIGHT" and predict_nearest_object_angle <= new_predict_orientation_centre)):
   					# LEFT-RIGHT or RIGHT-LEFT. Not ok
                    count = count + 1
                    TurnCheck(predict_range, new_predict_orientation_centre, count)

               	else: # means is LEFT-LEFT or RIGHT-RIGHT. ok
                    rotation_amount = rotation_amount * count
            else:
            	rotation_amount = rotation_amount * count
            	

            return 1

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


