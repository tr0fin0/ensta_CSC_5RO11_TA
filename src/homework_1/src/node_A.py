#!/usr/bin/env python

import rospy
from std_msgs.msg import String

message = "secret"

def callback(data):
    rospy.loginfo(f"Node A received: {data.data}")

def talker():
    pub = rospy.Publisher('outgoing_A', String, queue_size=10)
    rate = rospy.Rate(0.5)  # 1 message every 2 seconds
    while not rospy.is_shutdown():
        pub.publish(message)
        rate.sleep()

def listener():
    rospy.Subscriber('incoming_A', String, callback)

if __name__ == '__main__':
    rospy.init_node('node_A', anonymous=True)
    listener()
    talker()
