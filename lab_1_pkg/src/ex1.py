#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist


dist = 100
def scan_callback(data):
    # data will contain the message information of type LaserScan, we can access and print that data as follows.
   dist = (data.ranges[0])
   # dist = 100
   print(dist)

def ex1_node():


    # Create a publisher object with Twist
    pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
    # Declare the node, and register it with a unique name
    rospy.init_node('ex1_node', anonymous=True)
    # Define the execution rate object (10Hz)
    rate = rospy.Rate(10)
    # This node doesn't have to run all the time, but whenever a message is received, therefore, we can leave it spinning (waiting to wake up whenever a message is available).
    # rospy.spin()
    while not rospy.is_shutdown():
        # Create message object with a specific type
        rospy.Subscriber("/scan", LaserScan, scan_callback)

        vel_msg = Twist()

        # Populate custom message object
        vel_msg.linear.x = 0.5 * (dist-1)
        # vel_msg.angular.z = 1
        # Log/trace information on console
        # rospy.loginfo('[my_control_node] Running')
        # Publish the data
        pub.publish(vel_msg)
        # Sleep the necessary amount of time to keep a 10Hz execution rate
        rate.sleep()


if __name__ == '__main__':
    try:
        ex1_node()
    except rospy.ROSInterruptException:
        pass