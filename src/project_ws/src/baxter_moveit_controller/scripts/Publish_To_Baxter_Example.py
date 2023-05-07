#!/usr/bin/env python

import rospy
from std_msgs.msg import String
import random

def talker():
    pub = rospy.Publisher('chatter', String, queue_size=10)
    count = 0

    rospy.init_node('tictactoe', anonymous = True)
    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        next_move = "1"
        rospy.loginfo(next_move)
        pub.publish(next_move)
        rate.sleep()
        count = count + 1
if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
