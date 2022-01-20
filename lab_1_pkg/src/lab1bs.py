#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist

def controller_node():
    # Create a publisher object with Twist
    pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
    # Declare the node, and register it with a unique name
    rospy.init_node('my_control_node', anonymous=True)
    # Define the execution rate object (10Hz)
    rate = rospy.Rate(10)

    '''
        This is the main node loop
    '''
    while not rospy.is_shutdown():
        # Create message object with a specific type
        vel_msg = Twist()

        # Populate custom message object
        vel_msg.linear.x = 1
        vel_msg.angular.z = 1
        # Log/trace information on console
        rospy.loginfo('[my_control_node] Running')
        # Publish the data
        pub.publish(vel_msg)
        # Sleep the necessary amount of time to keep a 10Hz execution rate
        rate.sleep()


if __name__ == '__main__':
    try:
        controller_node()
    except rospy.ROSInterruptException:
        pass