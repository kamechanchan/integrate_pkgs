#!/usr/bin/env python3
import rospy
import std_msgs
if __name__=='__main__':
    rospy.init_node("yese")
    pub = rospy.Publisher("shinya", std_msgs.msg.String)
    loop = rospy.Rate(10)
    data = std_msgs.msg.String()
    moto = std_msgs.msg.String()
    count = 0
    moto.data = "moji"
    while not rospy.is_shutdown():
        data.data = moto.data + str(count)
        rospy.loginfo(data.data)
        pub.publish(data)
        count = count + 1
        loop.sleep()    

