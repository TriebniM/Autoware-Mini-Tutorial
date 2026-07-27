#!/usr/bin/env python3

import rospy
from std_msgs.msg import String

class Subscriber:
    def __init__(self):
        # Subscribers
        rospy.Subscriber('/message', String, self.message_callback)

    def message_callback(self, msg):
        print(msg.data)

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    rospy.init_node('subscriber')
    node = Subscriber()
    node.run()