#!/usr/bin/env python

import rospy
# import numpy as np
# from std_msgs.msg import String
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Twist
import math

roll = pitch = yaw = 0.0

def isnumber(value):
    try:
        int(value)
        return True
    except ValueError:
        return False


def rotatebot(rot_angle):
    # create Twist object
    twist = Twist()
    # set up Publisher to cmd_vel topic
    pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
    # set the update rate to 1 Hz
    rate = rospy.Rate(1)

    # get current yaw angle
    current_yaw = yaw
    # log the info
    rospy.loginfo(['Current: ' + str(current_yaw)])
    # calculate desired yaw
    desired_yaw = current_yaw + math.radians(rot_angle)
    rospy.loginfo(['Desired: ' + str(desired_yaw)])
    # set linear speed to zero so the TurtleBot rotates on the spot
    twist.linear.x = 0.0
    # check which direction we should rotate
    if(rot_angle>0):
        twist.angular.z = 1.0
    else:
        twist.angular.z = -1.0

    # rotate until yaw angle exceeds yaw+angle
    pub.publish(twist)

    # change twist object to stop rotation
    twist.angular.z = 0.0

    while(yaw < desired_yaw):
		rospy.loginfo(['Yaw: ' + str(yaw)])
        rate.sleep()

    # stop rotation
    pub.publish(twist)


def get_rotation (msg):
    orientation_quat =  msg.pose.pose.orientation
    orientation_list = [orientation_quat.x, orientation_quat.y, orientation_quat.z, orientation_quat.w]
    (roll, pitch, yaw) = euler_from_quaternion(orientation_list)


def listener():
    rate = rospy.Rate(1) # 10hz
    rospy.Subscriber('odom', Odometry, get_rotation)
    rate.sleep()
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()


def mover2():
    twist = Twist()
    pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
    rospy.init_node('mover', anonymous=True)
    rate = rospy.Rate(1) # 1 Hz

    while not rospy.is_shutdown():
        cmd_char = str(raw_input("Keys w/x -/+int s: "))

        if isnumber(cmd_char):
            # rotate by specified angle
            rotatebot(int(cmd_char))
        else:
            if cmd_char == 's':
                twist.linear.x = 0.0
                twist.angular.z = 0.0
            elif cmd_char == 'w':
                twist.linear.x = 0.1
                twist.angular.z = 0.0
            elif cmd_char == 'x':
                twist.linear.x = -0.1
                twist.angular.z = 0.0

            pub.publish(twist)

        rospy.loginfo(cmd_char)
        rate.sleep()

if __name__ == '__main__':
    try:
        mover2()
    except rospy.ROSInterruptException:
        pass
