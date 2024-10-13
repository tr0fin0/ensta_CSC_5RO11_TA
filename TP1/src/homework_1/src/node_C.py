#!/usr/bin/env python

import rospy
from std_msgs.msg import String

def callback(data):
    rospy.loginfo(f"Node C received: {data.data}")
    new_msg = data.data + "C"
    pub.publish(new_msg)

def listener():
    rospy.Subscriber('outgoing_B', String, callback)

if __name__ == '__main__':
    rospy.init_node('node_C', anonymous=True)
    pub = rospy.Publisher('outgoing_C', String, queue_size=10)
    listener()
    rospy.spin()
