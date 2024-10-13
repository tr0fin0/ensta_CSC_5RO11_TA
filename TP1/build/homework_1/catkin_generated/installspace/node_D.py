#!/usr/bin/env python3

import rospy
from std_msgs.msg import String

def callback(data):
    rospy.loginfo(f"Node D received: {data.data}")
    new_msg = data.data + "D"
    pub.publish(new_msg)

def listener():
    rospy.Subscriber('outgoing_C', String, callback)

if __name__ == '__main__':
    rospy.init_node('node_D', anonymous=True)
    pub = rospy.Publisher('incoming_A', String, queue_size=10)
    listener()
    rospy.spin()
