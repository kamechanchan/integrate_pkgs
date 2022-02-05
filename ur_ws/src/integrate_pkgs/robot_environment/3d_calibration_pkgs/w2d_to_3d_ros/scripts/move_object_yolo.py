#!/usr/bin/env python3
import rospy
from gazebo_msgs.msg import *
import random
from rospy.timer import Rate
from tf.transformations import quaternion_from_euler
from geometry_msgs.msg import Pose

class annotation_environment(object):
    def __init__(self, model_name):
        cont = 0
        self.model_state_pub = rospy.Publisher('/gazebo/set_model_state', ModelState, queue_size=1)
        self.kaisuu = 0
        self.model_name = model_name
        self.execute()
        

    def object_move(self, name):
        loop = rospy.Rate(100)
        com = 0
        pose_list = []
        a_1 = 0.12
        a_2 = 0.06
        a_3 = 0
        a_4 = -0.06
        a_5 = -0.12
        for i in range(25):
                pose_data = ModelState()
                pose_data.model_name =  name + '_' + str(i)
                if i == 0:
                    pose_data.pose.position.x = a_1
                    pose_data.pose.position.y = a_1
                    pose_data.pose.position.z = 0.00
                elif i == 1:
                    pose_data.pose.position.x = a_1
                    pose_data.pose.position.y = a_2
                    pose_data.pose.position.z = 0.4
                elif i == 2:
                    pose_data.pose.position.x = a_1
                    pose_data.pose.position.y = a_3
                    pose_data.pose.position.z = 0.45
                elif i == 3:
                    pose_data.pose.position.x = a_1
                    pose_data.pose.position.y = a_4
                    pose_data.pose.position.z = 0.25
                elif i == 4:
                    pose_data.pose.position.x = a_1
                    pose_data.pose.position.y = a_5
                    pose_data.pose.position.z = 0.3
                elif i == 5:
                    pose_data.pose.position.x = a_2
                    pose_data.pose.position.y = a_1
                    pose_data.pose.position.z = 0.05
                elif i == 6:
                    pose_data.pose.position.x = a_2
                    pose_data.pose.position.y = a_2
                    pose_data.pose.position.z = 0.25
                elif i == 7:
                    pose_data.pose.position.x = a_2
                    pose_data.pose.position.y = a_3
                    pose_data.pose.position.z = 0.53
                elif i == 8:
                    pose_data.pose.position.x = a_2
                    pose_data.pose.position.y = a_4 
                    pose_data.pose.position.z = 0.15
                elif i == 9:
                    pose_data.pose.position.x = a_2
                    pose_data.pose.position.y = a_5
                    pose_data.pose.position.z = 0.46
                elif i == 10:
                    pose_data.pose.position.x = a_3
                    pose_data.pose.position.y = a_1
                    pose_data.pose.position.z = 0.38
                elif i == 11:
                    pose_data.pose.position.x = a_3
                    pose_data.pose.position.y = a_2
                    pose_data.pose.position.z = 0.29
                elif i == 12:
                    pose_data.pose.position.x = a_3
                    pose_data.pose.position.y = a_3
                    pose_data.pose.position.z = 0.43
                elif i == 13:
                    pose_data.pose.position.x = a_3
                    pose_data.pose.position.y = a_4
                    pose_data.pose.position.z = 0.19
                elif i == 14:
                    pose_data.pose.position.x = a_3
                    pose_data.pose.position.y = a_5
                    pose_data.pose.position.z = 0.36
                elif i == 15:
                    pose_data.pose.position.x = a_4
                    pose_data.pose.position.y = a_1
                    pose_data.pose.position.z = 0.50
                elif i == 16:
                    pose_data.pose.position.x = a_4
                    pose_data.pose.position.y = a_2
                    pose_data.pose.position.z = 0.42
                elif i == 17:
                    pose_data.pose.position.x = a_4
                    pose_data.pose.position.y = a_3
                    pose_data.pose.position.z = 0.32
                elif i == 18:
                    pose_data.pose.position.x = a_4
                    pose_data.pose.position.y = a_4
                    pose_data.pose.position.z = 0.20
                elif i == 19:
                    pose_data.pose.position.x = a_4
                    pose_data.pose.position.y = a_5
                    pose_data.pose.position.z = 0.16
                elif i == 20:
                    pose_data.pose.position.x = a_5
                    pose_data.pose.position.y = a_1
                    pose_data.pose.position.z = 0.06
                elif i == 21:
                    pose_data.pose.position.x = a_5
                    pose_data.pose.position.y = a_2
                    pose_data.pose.position.z = 0.36
                elif i == 22:
                    pose_data.pose.position.x = a_5
                    pose_data.pose.position.y = a_3
                    pose_data.pose.position.z = 0.26
                elif i == 23:
                    pose_data.pose.position.x = a_5
                    pose_data.pose.position.y = a_4
                    pose_data.pose.position.z = 0.16
                elif i == 24:
                    pose_data.pose.position.x = a_5 
                    pose_data.pose.position.y = a_5
                    pose_data.pose.position.z = 0.12



                roll = random.uniform(-1.2, 1.2)
                pitch = random.uniform(-1.2, 1.2)
                yaw = random.uniform(-0.01, 0.03)
                quat = quaternion_from_euler(roll, pitch, yaw)
                pose_data.pose.orientation.x = quat[0]
                pose_data.pose.orientation.y = quat[1]
                pose_data.pose.orientation.z = quat[2]
                pose_data.pose.orientation.w = quat[3]
                pose_list.append(pose_data)
        while com < 2:
            com = com + 1

            for i in range(25):
                self.model_state_pub.publish(pose_list[i])
                loop.sleep()
            #loop.sleep()

    def execute(self):
        loop = rospy.Rate(10)
        naibu_loop = rospy.Rate(3)
        not_finish = rospy.get_param("not_finish", True)
        #count = 0
        while not rospy.is_shutdown():
            not_finish = rospy.get_param("not_finish", True)
            if not not_finish:
                break
            hantei = rospy.get_param("move_is_ok", True)
            if hantei:
                self.object_move(self.model_name)

                count = 0
                while count <= 2: 
                    count = count + 1
                    # print("move_count" + str(count))
                    naibu_loop.sleep()
                rospy.set_param("write_is_ok", True)
                rospy.set_param("move_is_ok", False)
                count = 0    
            
        # self.object_move("HV8")
            

if __name__=='__main__':
    rospy.init_node('random_state')
    model_name = rospy.get_param("~model_name", "HV8")
    annotation_environment(model_name)