#!/usr/bin/env python

import rospy
import numpy as np
from nav_msgs.msg import OccupancyGrid
import matplotlib.pyplot as plt
import tf

occ_bins = [-1, 0, 100, 101]
# counter = 0

def callback(msg):
    # create numpy array
    occdata = np.array([msg.data])
    # compute histogram to identify percent of bins with -1
    occ_counts = np.histogram(occdata,occ_bins)
    # calculate total number of bins
    total_bins = msg.info.width * msg.info.height
    # log the info
    rospy.loginfo('Unmapped: %i Unoccupied: %i Occupied: %i Total: %i', occ_counts[0][0], occ_counts[0][1], occ_counts[0][2], total_bins)

    listener = tf.TransformListener()

    # try:
    (trans,rot) = listener.lookupTransform('/map', '/odom', rospy.Time(0))
    rospy.loginfo(['Trans: ' + str(trans) + ' Rot: ' + str(rot)])

    # except (tf.LookupException):
        # pass

    # global counter
    # if counter % 10 == 0:
        # plt.cla()
    plt.imshow(occdata.reshape(msg.info.width,msg.info.height))
    plt.gca().invert_yaxis()
    plt.draw_all()
    plt.pause(0.00000000001)
        # callback.cla()
        # callback.fig.imshow(occdata)
        # callback.ax.axhline(callback.max, c='darkorange')
        # callback.fig.canvas.draw()

    # counter += 1


def occupancy():
    # initialize node
    rospy.init_node('occupancy', anonymous=True)

    # set the update rate to 1 Hz
    # rate = rospy.Rate(1) # 1 Hz
    # callback.fig =  plt.figure()
    # callback.ax = callback.fig.add_subplot(111)
    # callback.max = rospy.get_param('~kappa_max', 0.2)

    # subscribe to map occupancy data
    rospy.Subscriber('map', OccupancyGrid, callback)

    # plt.plot()
    # plt.ion()
    plt.show()
    # wait until it is time to run again
    # rate.sleep()

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()


if __name__ == '__main__':
    try:
        occupancy()
    except  rospy.ROSInterruptException:
        pass
