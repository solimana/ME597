#!/usr/bin/env python3
import rospy
from lab_1_pkg.msg import CustomMessage
from lab_1_pkg.srv import CustomService, CustomServiceResponse


def my_rx_callback(data):
    # data will contain the message information of type CustomMessage, we can access and print that data as follows.
    rospy.loginfo('Received ctr:{}, and text:{}'.format(data.ctr,data.text))
    
def my_rx_server_callback(req):
    rospy.loginfo('service request received arg_1:{}'.format(req.arg_1))
    return CustomServiceResponse('service executed')

def my_rx_node():
    # Declare the node, and register it with a unique name
    rospy.init_node('my_rx_node', anonymous=True)
    # Define a subscriber that will catch the messages published by my_tx_node
    # Observe how the subscription has to match use some of the parameters defined for the publisher.
    rospy.Subscriber("my_tx_node", CustomMessage, my_rx_callback)
    # Define the callback to be executed when the service CustomService is requested
    s = rospy.Service('custom_service', CustomService, my_rx_server_callback)
    # This node doesn't have to run all the time, but whenever a message is received, therefore, we can leav it spinning (waiting to wake up whenever a message is available).
    rospy.spin()

if __name__ == '__main__':
    try:
        my_rx_node()
    except rospy.ROSInterruptException:
	    pass