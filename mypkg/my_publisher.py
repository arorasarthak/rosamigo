#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist

def set_speed(x,wz):
    """
    This function sets your speed for the publisher. Use this function to implement your ideas.
    :params:
    x:This is the velocity along x-axis in m/sec. Limits: [-0.3,0.3]
    y:This is the velocity along y-axis in m/sec. Limits: [-0.3,0.3]
    wz: This is the angular velocity for turning (yaw) in rad/sec. Limits: [-1,1]
    :return: Returns a speed message
    """
    x = input("Enter the velocity along x-axis: ")
    wz = input("Enter the angular velocity for yaw/turn: ")
    speed = Twist()
    speed.linear.x = x
    speed.linear.y = 0
    speed.linear.z = 0
    speed.angular.x = 0
    speed.angular.y = 0
    speed.angular.z = wz
    return speed

def stop():
    rospy.init_node('stop_node', anonymous=True)
    stop_msg = set_speed(0,0)
    stop_pub = rospy.Publisher('/cmd_vel',Twist, queue_size=10)
    stop_pub.publish(stop_msg)

def publish_speed(speed_msg, node_name='my_speed_controller', topic='/cmd_vel', msg_type=Twist):
    """
    :params:
    speed_msg: this is the speed message coming from the set_speed function
    topic: this is the topic name you publish to
    msg_type: this is the message type
    node_name: this is the name of your node
    :return: returns nothing
    """

    #Initialize your node & publisher here.
    rospy.init_node(node_name, anonymous=True)
    pub = rospy.Publisher(topic, msg_type, queue_size=10)
    rate = rospy.Rate(10)
    print "Publishing to "+ topic + " at "+ str(10) +" hz."

    #Publish till your node is running
    while not rospy.is_shutdown():
        pub.publish(speed_msg)

def main():
    """
    This is the main function. Implement your logic here.
    :params: No params
    :return: returns nothing
    """
    msg = set_speed(0.1,0)
    publish_speed(msg)

if __name__=="__main__":
    main()
