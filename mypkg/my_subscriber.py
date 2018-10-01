#!/usr/bin/env python
#This line is very important


"""
Intro goes here
"""

import rospy
from geometry_msgs.msg import Twist

def callback(data):
    """
    Callback does something everytime a message is recieved by a subscriber.
    You define the logic after this line.
    :params:
    data: this is the data processed by the callback
    :return: returns nothing
    """
    print(data.linear)
    print(data.angular)

def subscribe(topic_name='/cmd_vel', callback_name=callback, node_name="mynode", message_object=Twist):
    """
    Subscriber subscribes to a topic and uses a callback to do something defined by you.
    :params:
    topic_name: The name of the topic you subscribe to goes here. Must be a string! 
    callback_name: Name of your callback function. Must be a func signature!
    node_name: Define your node name here. Must be a string!
    message_object: This is the message type specified a ROS message. Must be a message object!
    :return: Returns nothing 
    """
    rospy.init_node(node_name, anonymous=True)
    rospy.Subscriber(topic_name, message_object, callback_name)
    rospy.spin() # keeps the subscriber process from dying.

def main():
    """
    You can begin by modifying this function.
    """
    subscribe()

if __name__=="__main__":
    main()
