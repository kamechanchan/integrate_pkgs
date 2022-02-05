#!/usr/bin/env python3
import rospy
from gazebo_msgs.msg import *
import random
from rospy.timer import Rate
from tf.transformations import quaternion_from_euler
from geometry_msgs.msg import Pose
from denso_msgs.msg import object_kiriwake
import math
import numpy as np


class move_environment(object):
    def __init__(self):
        self.kaisuu = 0
        self.z_coordinamte = 0
        self.first_function()
        

    def first_function(self):
        self.model_state_pub = rospy.Publisher('/gazebo/set_model_state', ModelState, queue_size=1)
        self.occuluder_pub = rospy.Publisher("tsuchida_object_occuluder", object_kiriwake, queue_size=10)
        self.box_move()
        self.execute()

    def object_move(self):
        a_1 = 0.155
        a_4 = -0.155
        pose_data = ModelState()
        pose_list = []
        pose_data.model_name =  "HV8_0"
        loop = rospy.Rate(10)
        x = random.uniform(a_4, a_1) + 0.5
        y = random.uniform(a_4, a_1)
        z = self.z_coordinamte + 0.1
        roll = 0
        pitch = 0
        yaw = 0
        quat = quaternion_from_euler(roll, pitch, yaw)
        pose_data.pose.position.x = x
        pose_data.pose.position.y = y
        pose_data.pose.position.z = z
        pose_data.pose.orientation.x = quat[0]
        pose_data.pose.orientation.y = quat[1]
        pose_data.pose.orientation.z = quat[2]
        pose_data.pose.orientation.w = quat[3]
        pose_list.append(pose_data)
        for i in range(6):
            self.model_state_pub.publish(pose_list[0])
            loop.sleep()
    
    def execute(self):
        naibu_loop = rospy.Rate(30)
        count = 0
        while count < 1:
            self.object_move()
            print("topic_publish")
            naibu_loop.sleep()
            count = count + 1
            
        # self.object_move("HV8")
    def box_move(self):
        pose_list = []
        loop = rospy.Rate(2)
        pose_data = ModelState()
        pose_data.model_name = "object_box"
        pose_data.pose.position.x = 0.5
        pose_data.pose.position.y = 0
        pose_data.pose.position.z = self.z_coordinamte
        roll = 0
        pitch = 0
        yaw = 0
        quat = quaternion_from_euler(roll, pitch, yaw)
        pose_data.pose.orientation.x = quat[0]
        pose_data.pose.orientation.y = quat[1]
        pose_data.pose.orientation.z = quat[2]
        pose_data.pose.orientation.w = quat[3]
        pose_list.append(pose_data)
        for i in range(6):
            self.model_state_pub.publish(pose_list[0])
            loop.sleep()

if __name__=='__main__':
    rospy.init_node('random_state')
    ano = move_environment()
