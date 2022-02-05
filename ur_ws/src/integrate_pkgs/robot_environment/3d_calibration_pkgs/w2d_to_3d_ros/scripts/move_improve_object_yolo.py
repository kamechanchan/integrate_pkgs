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


def distance(x1, y1, x2, y2):
    return math.sqrt(float((x1 - x2)*(x1 - x2) + (y1 - y2)*(y1 - y2)))

def pose_is_ok(x, y, x_kako, y_kako, kyori, minus, plas):
    ok = True
    # print("tsuchida")
    if x < minus or x > plas or y < minus or y > plas:
        ok = False
    for i in range(len(x_kako)):
        # print(self.distance(x, y, x_kako[i], y_kako[i]))
        # print("x: " + str(x) + "  y: " + str(y) + "  x_kako" + str(x_kako[i]) + "  y_kako: " + str(y_kako[i]) + "  distance: " + str(distance(x, y, x_kako[i], y_kako[i])))
        if distance(x, y, x_kako[i], y_kako[i]) <= kyori:
            ok = False
    return ok

def limit_pose_is_ok(x, y, limit_minus, limit_plus):
    if x < limit_minus or x > limit_plus or y < limit_minus or y > limit_plus:
        return False
    else:
        return True
class annotation_environment(object):
    def __init__(self, model_name):
        cont = 0
        self.model_state_pub = rospy.Publisher('/gazebo/set_model_state', ModelState, queue_size=1)
        self.occuluder_pub = rospy.Publisher("tsuchida_object_occuluder", object_kiriwake, queue_size=10)
        self.kaisuu = 0
        self.occulution_object = object_kiriwake()
        self.box_move()
        self.decide_occuluder()
        self.model_name = model_name

        self.execute()
        

    def object_move(self):
        self.home_move()
        loop = rospy.Rate(100)
        object_name = object_kiriwake()
        # object_name.occuluder_pose[0].position.x
        
        com = 0
        pose_list = []
        a_1 = 0.155
        a_2 = 0.04
        a_3 = -0.04
        a_4 = -0.155
        # a_5 = -0.12
        
        limit_distance = 0.09
        a1 = np.arange(a_4, a_1, 0.099)
        a2 = np.arange(a_4, a_1, 0.099)
        youso = []
        for l1 in a1:
            for l2 in a2:
                youso.append([round(l1, 4), round(l2, 4)])
        random.shuffle(youso)
        for i in range(len(self.occulution_object.occuluder_object)):
            pose_data = ModelState()
            pose_data.model_name =  self.occulution_object.occuluder_object[i]
           
            x = youso[i][0]
            y = youso[i][1]   
            z = 1.1
            pose_object = geometry_msgs.msg.Pose()
            pose_object.position.x = x
            pose_object.position.y = y
            pose_object.position.z = z
            self.occulution_object.occuluder_pose.append(pose_object)
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
        x_kako = []
        y_kako = []
        for i in range(len(self.occulution_object.occuludy)):
            pose_data = ModelState()
            pose_data.model_name = self.occulution_object.occuludy[i]
            hanni = 0.025
            if i == 0:
                x_zure = random.uniform(-hanni, hanni)
                y_zure = random.uniform(-hanni, hanni)
                x = self.occulution_object.occuluder_pose[i].position.x + x_zure
                y = self.occulution_object.occuluder_pose[i].position.y + y_zure
            else:
                count = 0
                while True:
                    # count = count + 1
                    # print(count)
                    x_zure = random.uniform(-hanni, hanni)
                    y_zure = random.uniform(-hanni, hanni)
                    x = self.occulution_object.occuluder_pose[i].position.x + x_zure
                    y = self.occulution_object.occuluder_pose[i].position.y + y_zure
                    # if pose_is_ok(x, y, x_kako, y_kako, 0.09, a_4, a_1):
                    if limit_pose_is_ok(x, y, a_4, a_1):
                        break
            z = 1.02
            x_kako.append(x)
            y_kako.append(y)
            # pose_object = geometry_msgs.msg.Pose()
            # pose_object.position.x = x
            # pose_object.position.y = y
            # pose_object.position.z = z
            # self.occulution_object.occuluder_pose.append(pose_object)
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


        while com < 1:
            com = com + 1
            
            
            for i in range(len(pose_list)):
                # print(i)
                self.model_state_pub.publish(pose_list[len(pose_list) - 1 - i])
                # self.model_state_pub.publish(pose_list[i])
                self.occuluder_pub.publish(self.occulution_object)
                loop.sleep()
            #loop.sleep()
    
    def home_move(self):
        loop = rospy.Rate(100)
        object_name = object_kiriwake()
        com = 0
        pose_list = []
        a_1 = -12
        a_4 = -15
        # a_5 = -0.12
        x_kako = []
        y_kako = []
        limit_distance = 0.09
        for i in range(len(self.occulution_object.occuluder_object)):
            pose_data = ModelState()
            pose_data.model_name =  self.occulution_object.occuluder_object[i]
           
            x = random.uniform(a_4, a_1)
            y = random.uniform(a_4, a_1)
            z = random.uniform(a_4, a_1)
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

        for i in range(len(self.occulution_object.occuludy)):
            pose_data = ModelState()
            pose_data.model_name = self.occulution_object.occuludy[i]
            
            x = random.uniform(a_4, a_1)
            y = random.uniform(a_4, a_1)
            z = random.uniform(a_4, a_1)
          
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
        while com < 1:
            com = com + 1  
            for i in range(len(pose_list)):
                # print(i)
                self.model_state_pub.publish(pose_list[len(pose_list) - 1 - i])
                
                self.occuluder_pub.publish(self.occulution_object)
                loop.sleep()
            #loop.sleep()
    
    def execute(self):
        loop = rospy.Rate(10)
        naibu_loop = rospy.Rate(1)
        not_finish = rospy.get_param("not_finish", True)
        #count = 0
        while not rospy.is_shutdown():
            # not_finish = rospy.get_param("not_finish", True)
            # if not not_finish:
            #     break
            # hantei = rospy.get_param("move_is_ok", True)
            # if hantei:
            #     self.object_move()
                
            #     count = 0
            #     while count <= 2: 
            #         count = count + 1
            #         # print("move_count" + str(count))
            #         naibu_loop.sleep()
            #     rospy.set_param("write_is_ok", True)
            #     rospy.set_param("move_is_ok", False)
            #     count = 0    
            self.object_move()
            naibu_loop.sleep()
            
        # self.object_move("HV8")
    def box_move(self):
        pose_list = []
        loop = rospy.Rate(10)
        pose_data = ModelState()
        pose_data.model_name = "object_box"
        pose_data.pose.position.x = 0
        pose_data.pose.position.y = 0
        pose_data.pose.position.z = 1
        roll = 0
        pitch = 0
        yaw = 0
        quat = quaternion_from_euler(roll, pitch, yaw)
        pose_data.pose.orientation.x = quat[0]
        pose_data.pose.orientation.y = quat[1]
        pose_data.pose.orientation.z = quat[2]
        pose_data.pose.orientation.w = quat[3]
        pose_list.append(pose_data)
        for i in range(5):
            self.model_state_pub.publish(pose_list[0])
            loop.sleep()


    def decide_occuluder(self):
        self.occulution_object.occuluder_object.append("HV8_0")
        self.occulution_object.occuluder_mesh_topic_name.append("meshcloud_0")
        self.occulution_object.occuluder_object.append("HV8_1")
        self.occulution_object.occuluder_mesh_topic_name.append("meshcloud_1")
        self.occulution_object.occuluder_object.append("HV8_2")
        self.occulution_object.occuluder_mesh_topic_name.append("meshcloud_2")
        self.occulution_object.occuluder_object.append("HV8_3")
        self.occulution_object.occuluder_mesh_topic_name.append("meshcloud_3")
        self.occulution_object.occuluder_object.append("HV8_4")
        self.occulution_object.occuluder_mesh_topic_name.append("meshcloud_4")
        self.occulution_object.occuluder_object.append("HV8_5")
        self.occulution_object.occuluder_mesh_topic_name.append("meshcloud_5")
        self.occulution_object.occuluder_object.append("HV8_6")
        self.occulution_object.occuluder_mesh_topic_name.append("meshcloud_6")
        self.occulution_object.occuluder_object.append("HV8_7")
        self.occulution_object.occuluder_mesh_topic_name.append("meshcloud_7")
        self.occulution_object.occuluder_object.append("HV8_8")
        self.occulution_object.occuluder_mesh_topic_name.append("meshcloud_8")
        self.occulution_object.occuluder_object.append("HV8_9")
        self.occulution_object.occuluder_mesh_topic_name.append("meshcloud_9")
        self.occulution_object.occuluder_object.append("HV8_10")
        self.occulution_object.occuluder_mesh_topic_name.append("meshcloud_10")
        self.occulution_object.occuluder_object.append("HV8_11")
        self.occulution_object.occuluder_mesh_topic_name.append("meshcloud_11")
        # self.occulution_object.occuluder_object.append("HV8_12")
        # self.occulution_object.occuluder_object.append("HV8_13")


        self.occulution_object.occuludy.append("HV8_14")
        self.occulution_object.occuludy.append("HV8_15")
        self.occulution_object.occuludy.append("HV8_16")
        self.occulution_object.occuludy.append("HV8_17")
        self.occulution_object.occuludy.append("HV8_18")
        self.occulution_object.occuludy.append("HV8_19")
        self.occulution_object.occuludy.append("HV8_20")
        self.occulution_object.occuludy.append("HV8_21")
        self.occulution_object.occuludy.append("HV8_22")
        self.occulution_object.occuludy.append("HV8_23")
        self.occulution_object.occuludy.append("HV8_24")
    
   
    
    
    


            

if __name__=='__main__':
    rospy.init_node('random_state')
    model_name = rospy.get_param("~model_name", "HV8")
    annotation_environment(model_name)