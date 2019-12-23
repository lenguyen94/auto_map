#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Twist
import math
import cmath

roll = pitch = yaw = 0.0

def isnumber(value):
    try:
        int(value)
        return True
    except ValueError:
        return False


def get_rotation (msg):
    global roll, pitch, yaw
    orientation_quat =  msg.pose.pose.orientation
    orientation_list = [orientation_quat.x, orientation_quat.y, orientation_quat.z, orientation_quat.w]
    (roll, pitch, yaw) = euler_from_quaternion(orientation_list)
    yaw = yaw % (2*math.pi)


def rotatebot(rot_angle):
    # create Twist object
    twist = Twist()
    # set up Publisher to cmd_vel topic
    pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
    # set the update rate to 1 Hz
    rate = rospy.Rate(1)

# Use complex numbers with modulus 1 instead of angles. That would be z = cos(alpha) + i * sin(alpha).

# if (imag(desired / current) > 0.0)
#   current *= Complex(cos(0.1), sin(0.1));
# else
#   current *= Complex(cos(0.1), -sin(0.1));

# It's not just this piece of code: Pretty much anything you want to do with angles is easier with complex numbers.

    # get current yaw angle
    current_yaw = yaw
    # log the info
    rospy.loginfo(['Current: ' + str(math.degrees(current_yaw))])
    c_yaw = complex(math.cos(yaw),math.sin(yaw))
    # calculate desired yaw
    target_yaw = current_yaw + math.radians(rot_angle)
    c_target_yaw = complex(math.cos(target_yaw),math.sin(target_yaw))
    rospy.loginfo(['Desired: ' + str(math.degrees(cmath.phase(c_target_yaw)))])
    # divide the two complex numbers to get the change in direction
    c_change = c_target_yaw / c_yaw
    c_change_dir = c_change.imag / cmath.polar(c_change)
    # set linear speed to zero so the TurtleBot rotates on the spot
    twist.linear.x = 0.0
    # check which direction we should rotate
    twist.angular.z = c_change_dir * 0.1
    # rotate until yaw angle exceeds yaw+angle
    pub.publish(twist)

    c_dir_diff = c_change_dir    
    while(c_change_dir * c_dir_diff < 0):
        c_yaw = complex(math.cos(yaw),math.sin(yaw))
        rospy.loginfo(['While Yaw: ' + str(math.degrees(yaw))])
        c_change = c_target_yaw / c_yaw
        c_dir_diff = c_change.imag / cmath.polar(c_change)
        rate.sleep()

    rospy.loginfo(['End Yaw: ' + str(math.degrees(yaw))])
    twist.angular.z = 0.0
    # rotate until yaw angle exceeds yaw+angle
    pub.publish(twist)
    

def mover2():
    twist = Twist()
    pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
    rospy.init_node('mover', anonymous=True)
    rospy.Subscriber('odom', Odometry, get_rotation)
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
