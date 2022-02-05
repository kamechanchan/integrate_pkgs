#!/usr/bin/env python3
from genpy import message
import rospy
import message_filters
from std_msgs.msg import String
from w2d_to_3d_ros.msg import shinya, hayasa

def callback(msg1, msg2):
    rospy.loginfo(msg1.data + str(msg2.count))

if __name__=='__main__':
    rospy.init_node('tutu')
    msg_sub_1 = message_filters.Subscriber('/shinya_1', shinya)
    msg_sub_2 = message_filters.Subscriber('/shinya_2', hayasa)
    #ts = message_filters.TimeSynchronizer(msg_sub_1, 10)
    #msg_sub_1.registerCallback(callback)
    ts = message_filters.ApproximateTimeSynchronizer([msg_sub_1, msg_sub_2], 10, 0.1, allow_headerless=True)
    ts.registerCallback(callback)
    rospy.spin()
