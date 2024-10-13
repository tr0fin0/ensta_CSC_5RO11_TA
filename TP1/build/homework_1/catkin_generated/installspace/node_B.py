#!/usr/bin/env python3

import rospy
from std_msgs.msg import String

def callback(data):
    rospy.loginfo(f"Node B received: {data.data}")
    new_msg = data.data + "B"
    pub.publish(new_msg)

def listener():
    rospy.Subscriber('outgoing_A', String, callback)

if __name__ == '__main__':
    rospy.init_node('node_B', anonymous=True)
    pub = rospy.Publisher('outgoing_B', String, queue_size=10)
    listener()
    rospy.spin()
