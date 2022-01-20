#!/usr/bin/env python3
import rospy
from lab_1_pkg.msg import CustomMessage
from std_msgs.msg import String


def my_tx_node():
    # Create a publisher object with the custom type
    pub = rospy.Publisher('tx_msg', CustomMessage, queue_size=10)
    # Declare the node, and register it with a unique name
    rospy.init_node('my_tx_node', anonymous=True)
    # Define the execution rate object (10Hz)
    rate = rospy.Rate(10)

    '''
        This is the main node loop
    '''
    ctr = 0
    while not rospy.is_shutdown():
        # Create message object with a specific type
        msg = CustomMessage()
        # Populate custom message object
        msg.ctr = ctr
        ctr = ctr + 1
        msg.text = 'Message from node my_tx_node'
        # Log/trace information on console
        rospy.loginfo('[my_tx_node] Running')
        # Publish the data
        pub.publish(msg)
        # Sleep the necessary amount of time to keep a 10Hz execution rate
        rate.sleep()


if __name__ == '__main__':
    try:
        my_tx_node()
    except rospy.ROSInterruptException:
        pass