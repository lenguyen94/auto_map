#!/usr/bin/env python

import rospy
import numpy as np
from nav_msgs.msg import OccupancyGrid

occ_bins = [-1, 0, 100, 101]

def callback(msg):
	# create numpy array
	occdata = np.array([msg.data])
	# compute histogram to identify percent of bins with -1
	occ_counts = np.histogram(occdata,occ_bins)
	# calculate total number of bins
	total_bins = msg.info.width * msg.info.height
	# log the info
	rospy.loginfo('% unmapped: %f', occ_counts[0][0]/ total_bins * 100)


def occupancy():
	# initialize node
	rospy.init_node('occupancy', anonymous=True)

	# set the update rate to 1 Hz
	rate = rospy.Rate(1) # 1 Hz

	# subscribe to map occupancy data
	rospy.Subscriber('map', OccupancyGrid, callback)
    
	# wait until it is time to run again
	rate.sleep()

	# spin() simply keeps python from exiting until this node is stopped
	rospy.spin()


if __name__ == '__main__':
    try:
	    occupancy()
    except  rospy.ROSInterruptException:
        pass
