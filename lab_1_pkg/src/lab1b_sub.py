#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

def scan_callback(data):
    # data will contain the message information of type LaserScan, we can access and print that data as follows.
    rospy.loginfo('Received ctr:{}'.format(data.ranges[0]))

def scan_node():
    # Declare the node, and register it with a unique name
    rospy.init_node('scan_node', anonymous=True)
    # Define a subscriber that will catch the messages published by scan
    # Observe how the subscription has to match use some of the parameters defined for the publisher.
    rospy.Subscriber("/scan", LaserScan, scan_callback)
    # This node doesn't have to run all the time, but whenever a message is received, therefore, we can leave it spinning (waiting to wake up whenever a message is available).
    rospy.spin()


if __name__ == '__main__':
    try:
        scan_node()
    except rospy.ROSInterruptException:
        pass