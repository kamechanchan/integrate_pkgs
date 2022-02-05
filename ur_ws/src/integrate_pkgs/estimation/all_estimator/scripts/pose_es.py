#!/usr/bin/python3
# -*- coding: utf-8 -*-
import sys
import os
from xml.etree.ElementTree import TreeBuilder
sys.path.append(os.path.join(os.path.dirname(__file__), '../../../networks/raugh_recognition/'))
sys.path.append(os.path.join(os.path.dirname(__file__), '../../../networks/raugh_recognition/pointnet_pose'))
sys.path.append(os.path.join(os.path.dirname(__file__), '../../../networks/'))
sys.path.append(os.path.join(os.path.dirname(__file__), '../../../networks/semantic_segmentation'))
sys.path.append(os.path.join(os.path.dirname(__file__), '../../../networks/semantic_segmentation/pointnet_semantic'))
from semantic_segmentation import pointnet_semantic

from raugh_recognition.pointnet_pose.options.test_options import TestOptions_raugh_recognition
from semantic_segmentation.pointnet_semantic.options.test_options import TestOptions_semantic_segmentation
from semantic_segmentation import pointnet_semantic
from raugh_recognition import pointnet_pose
from raugh_recognition.pointnet_pose import pointnet_exec
from semantic_segmentation.pointnet_semantic import semantic_exec
from common_function.cloud_util import *
from std_msgs.msg import Header
import numpy as np
import time
import open3d
from denso_srvs.srv import bounding, boundingResponse
from denso_msgs.msg import out_segmentation

from denso_srvs.srv import sensor_data_service, sensor_data_serviceResponse
from denso_srvs.srv import tf_quaternion_service, tf_quaternion_serviceRequest
from tf.transformations import *


# ROS
import rospy, rospkg
from geometry_msgs.msg import TransformStamped
import tf2_ros

from sensor_msgs.msg import PointCloud2
from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud


class DnnNode():
    def __init__(self):
        rospy.init_node("Pose_Estimation_server")
        rospy.loginfo("Ready.")
        rospack = rospkg.RosPack()
        # yes = TestOptions_semantic_segmentation()
        # self.opt = yes.test_parse()
        # self.opt.dataset_model = rospy.get_param("~object_name", "HV8")
        # self.package_path = rospack.get_path("all_estimator")
        # self.checkpoints_dir = rospy.get_param("~load_path", self.package_path + "/../networks/semantic_segmentation/pointnet_semantic/weights/semantic_demo.pth")
        # self.opt.checkpoints_dir = self.checkpoints_dir
        # self.instance_pub = rospy.Publisher("instance_pub", out_segmentation, queue_size=10)
        # self.opt.is_train = False
        # self.opt.is_estimate = True
        self.child_frame_id = rospy.get_param("~child_frame_id","estimated_tf")
        self.header_frame_id = rospy.get_param("~header_frame_id","photoneo_center_optical_frame")
        # self.header_frame_id = rospy.get_param("~header_frame_id","camera_depth_optical_frame")
        self.tf = tf2_ros.TransformBroadcaster()
        # self.tf_1 = tf2_ros.StaticTransformBroadcaster()
        
        self.object_name_stl = model_loader('random_original.pcd')
        self.stl_est = copy.deepcopy(self.object_name_stl)

        self.pub_GT = rospy.Publisher("Ground_Truth_cloud", out_segmentation, queue_size=1)
        self.pub_final = rospy.Publisher("final_raugh_estimated_cloud", PointCloud2, queue_size=1)
        # self.sub = rospy.Subscriber("/cloud_without_segmented", PointCloud2, self.maji)
        # self.model = pointnet_semantic.create_model(self.opt)
        yes = TestOptions_raugh_recognition()
        self.raugh_opt = yes.test_parse()
        self.raugh_opt.is_estimate = True
        self.package_path = rospack.get_path("all_estimator")
        self.raugh_opt.checkpoints_dir = rospy.get_param("~load_path2", self.package_path + "/weights/raugh_demo.pth")
        self.raugh_model = pointnet_pose.create_model(self.raugh_opt)
        
        self.resolution = rospy.get_param("~resolution2", 1024)

        self.output_pos_num = 3
        self.output_ori_num = 9

    # def run_service(self):
    #     service = rospy.Service("bounding_box", bounding, self.callback)

    # def callback(self, req):
        r = rospy.Rate(0.5)
        while True:
            self.start_callback = time.time()
            
            rospy.wait_for_service("message_service")
            resp1 = rospy.ServiceProxy("message_service", sensor_data_service)
            resp = resp1(True)
            # resp = sensor_data_serviceResponse()
            # resp.point_cloud2

            # data = np.array((resp.point_cloud2.x, resp.point_clouelf.out_num_pos)d2.y, resp.point_cloud2.z))
            data = np.zeros((len(resp.point_cloud2.x),3), float)
            print(len(resp.point_cloud2.x))
            for i in range(len(resp.point_cloud2.x)):
                data[i][0] = resp.point_cloud2.x[i]
                data[i][1] = resp.point_cloud2.y[i]
                data[i][2] = resp.point_cloud2.z[i]

            # res = boundingResponse()
            # res = sensor_data_serviceResponse()
            # data = np.array((req.x, req.y, req.z))
            # data = data.T
            # normalized_pcd, self.offset_data = getNormalizedPcd(data, self.opt.resolution)
            # normalized_pcd, offset = getNormalizedPcd(data, self.opt.resolution)
            # normalized_pcd += offset
            
            
            # segme, est_time, raugh_data = semantic_exec.pose_prediction(self.model, normalized_pcd, self.opt.resolution)
            # print(segme)
            # self.instance_pub.publish(segme)
            raugh_data = data

            # print(raugh_data.shape)
            # final_pcd, self.offset_data = getNormalizedPcd(raugh_data, self.resolution)
            final_pcd, offset2 = getNormalizedPcd(raugh_data, self.resolution)
            # final_pcd += offset
            print("*****")
            print(final_pcd.shape)
            print(offset2.shape)
            # for i in range(20):
            #     print("x: " + str(final_pcd[20*i][0]) +  "  z: " + str(final_pcd[20*i][2]))
            # final_pcd = final_pcd[np.newaxis, :, :]
            raugh_GT = out_segmentation()
            raugh_GT
            # print("************")
            # print(final_pcd.shape)
            for i in range(self.resolution):
                # raugh_GT.x.append(final_pcd[i, 0])
                # raugh_GT.y.append(final_pcd[i, 1])
                # raugh_GT.z.append(final_pcd[i, 2])
                raugh_GT.x.append(final_pcd[i, 0] + offset2[0][0])
                raugh_GT.y.append(final_pcd[i, 1] + offset2[0][1])
                raugh_GT.z.append(final_pcd[i, 2] + offset2[0][2])
            self.pub_GT.publish(raugh_GT)
            # for i in range(20):
            #     print("x: " + str(raugh_GT.x[20*i]) +  "  z: " + str(raugh_GT.z[20*i]))
            est_pose, est_time = pointnet_exec.pose_prediction(self.raugh_model, final_pcd, "integ_final_PointNet")
            final_est = TransformStamped()
            final_est.header.stamp = rospy.Time.now()
            final_est.header.frame_id = self.header_frame_id
            final_est.child_frame_id = self.child_frame_id
            # final_est.transform.translation = est_pose.pose.position
            # final_est.transform.rotation = est_pose.pose.orientation
            # final_est.transform.rotation.x = 0
            # final_est.transform.rotation.y = 0
            # final_est.transform.rotation.z = 0
            # final_est.transform.rotation.w = 1

            # q_rot = quaternion_from_euler(math.pi, 0, 0)
            # final_est.transform.rotation.x = q_rot[0]
            # final_est.transform.rotation.y = q_rot[1]
            # final_est.transform.rotation.z = q_rot[2]
            # final_est.transform.rotation.w = q_rot[3]

            final_est.transform.rotation = est_pose.pose.orientation

            final_est.transform.translation.x = est_pose.pose.position.x + offset2[0][0]
            final_est.transform.translation.y = est_pose.pose.position.y + offset2[0][1]
            final_est.transform.translation.z = est_pose.pose.position.z + offset2[0][2]
            # final_est.transform.translation.x = est_pose.pose.position.x
            # final_est.transform.translation.y = est_pose.pose.position.y
            # final_est.transform.translation.z = est_pose.pose.position.z 

            
            self.tf.sendTransform(final_est)


            # #tfのquaternionを変更
            # owari_est = TransformStamped()
            # owari_est = final_est
            # ittan = tf_quaternion_serviceRequest()
            # ittan.input_tf = owari_est.transform
            # rospy.wait_for_service("quaternion_service")
            # self.tf_service = rospy.ServiceProxy("quaternion_service", tf_quaternion_service)
            # out = self.tf_service(ittan)
            # owari_est.transform = out.output_tf
            # owari_est.child_frame_id = "owari"
            # owari_est.header.stamp = rospy.Time.now()
            # self.tf_1.sendTransform(owari_est)
                
            # print("x: " + str(est_pose.pose.position.x) + " z: "  + str(est_pose.pose.position.z))
            header = Header()
            header.frame_id = self.header_frame_id
            final_pcd = convertCloudFromOpen3dToRos(self.stl_est, header)
            est_cloud = do_transform_cloud(final_pcd, final_est)
            self.pub_final.publish(est_cloud)
            r.sleep()
            

if __name__ == "__main__":
     node = DnnNode()
