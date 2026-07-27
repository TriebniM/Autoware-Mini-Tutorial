#!/usr/bin/env python3

import rospy
from std_msgs.msg import String

class Publisher:
    def __init__(self):
        # Internal variables
        self.message = rospy.get_param('~message', 'Hello World!')
        rate_hz = rospy.get_param('~rate', 1.0)

        # Publishers
        self.pub = rospy.Publisher('/message', String, queue_size=10)
        self.rate = rospy.Rate(rate_hz)

    def run(self):
        while not rospy.is_shutdown():
            self.pub.publish(self.message)
            self.rate.sleep()

if __name__ == '__main__':
    rospy.init_node('publisher')
    node = Publisher()
    node.run()