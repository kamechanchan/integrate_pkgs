#!/usr/bin/python3
# -*- coding: utf-8 -*-
import sys
import os
from typing import final
from xml.etree.ElementTree import TreeBuilder

from numpy.random.mtrand import random
sys.path.append(os.path.join(os.path.dirname(__file__), '../../../networks/raugh_recognition/'))
sys.path.append(os.path.join(os.path.dirname(__file__), '../../../networks/raugh_recognition/pointnet_pose'))
sys.path.append(os.path.join(os.path.dirname(__file__), '../../../networks/'))
sys.path.append(os.path.join(os.path.dirname(__file__), '../../../networks/semantic_segmentation'))
sys.path.append(os.path.join(os.path.dirname(__file__), '../../../networks/semantic_segmentation/pointnet_semantic'))
sys.path.append(os.path.join(os.path.dirname(__file__), '../../../networks/pcl_template_matting'))
sys.path.append(os.path.join(os.path.dirname(__file__), '../../../networks/pcl_template_matting/pointnet_temmat'))


from raugh_recognition.pointnet_pose.options.test_options import TestOptions_raugh_recognition
from semantic_segmentation.pointnet_semantic.options.test_options import TestOptions_semantic_segmentation
from pcl_template_matting.pointnet_temmat.options.test_options import TestOptions_temmat
from semantic_segmentation import pointnet_semantic
from raugh_recognition import pointnet_pose
from raugh_recognition.pointnet_pose import pointnet_exec
from pcl_template_matting import pointnet_temmat
from semantic_segmentation.pointnet_semantic import semantic_exec
from pcl_template_matting.pointnet_temmat import temmat_exec
from common_function.cloud_util import *
from std_msgs.msg import Header
import numpy as np
import time
from denso_srvs.srv import tf_quaternion_service, tf_quaternion_serviceRequest

# ROS
import rospy, rospkg
# from denso_srvs.srv import bounding, boundingResponse
from denso_srvs.srv import Acc_pcl_dnn, Acc_pcl_dnnResponse
from geometry_msgs.msg import TransformStamped
import tf2_ros
from denso_msgs.msg import out_segmentation
from sensor_msgs.msg import PointCloud2
from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud
from tf.transformations import quaternion_about_axis

class DnnNode():
    def __init__(self):
        rospy.init_node("Pose_Estimation_server")
        rospy.loginfo("Ready.")
        rospack = rospkg.RosPack()
        yes = TestOptions_semantic_segmentation()
        self.opt = yes.test_parse()
        self.opt.dataset_model = rospy.get_param("~object_name", "HV8")
        self.package_path = rospack.get_path("all_estimator")
        self.checkpoints_dir = rospy.get_param("~load_path", self.package_path + "/../networks/semantic_segmentation/pointnet_semantic/weights/semantic_demo.pth")
        self.opt.checkpoints_dir = self.checkpoints_dir
        self.instance_pub = rospy.Publisher("instance_pub", out_segmentation, queue_size=10)
        self.opt.is_train = False
        self.opt.is_estimate = True
        self.child_frame_id = rospy.get_param("~child_frame_id","estimated_tf")
        self.header_frame_id = rospy.get_param("~header_frame_id","photoneo_center_optical_frame")
        self.tf = tf2_ros.StaticTransformBroadcaster()
        self.tf_1 = tf2_ros.StaticTransformBroadcaster()
        
        self.object_name_stl = model_loader('random_original.pcd')
        self.stl_est = copy.deepcopy(self.object_name_stl)
        self.pub_GT = rospy.Publisher("Ground_Truth_cloud", out_segmentation, queue_size=1)
        self.pub_final = rospy.Publisher("final_raugh_estimated_cloud", PointCloud2, queue_size=1)
        self.model = pointnet_semantic.create_model(self.opt)
        yes = TestOptions_raugh_recognition()
        self.raugh_opt = yes.test_parse()
        self.raugh_opt.is_estimate = True
        self.raugh_opt.checkpoints_dir = rospy.get_param("~load_path2", self.package_path + "/../../networks/raugh_recognition/pointnet_pose/weights/raugh_demo.pth")
        self.raugh_model = pointnet_pose.create_model(self.raugh_opt)
        self.resolution = rospy.get_param("~resolution2", 1024)

        yes = TestOptions_temmat()
        self.temmat_opt = yes.test_parse()
        self.temmat_opt.is_estimate = True
        self.temmat_opt.checkpoints_dir = rospy.get_param("~load_path3", self.package_path + "/../../networks/pcl_template_matting/pointnet_temmat/weights/temmat_demo.pth")
        self.temmat_model = pointnet_temmat.create_model_temmat(self.temmat_opt)

        self.max_instance = rospy.get_param("~max_instance", 26)

        self.output_pos_num = 3
        self.output_ori_num = 9

    def run_service(self):
        print("service start")
        service = rospy.Service("pcl_dnn", Acc_pcl_dnn, self.callback)
        # print("goal")

    def callback(self, req):
        res = Acc_pcl_dnnResponse()
        r = rospy.Rate(10)

        self.single_extract(req)
        self.accuracy_calculate(req)
        
        est_pose, est_time = pointnet_exec.pose_prediction(self.raugh_model, self.final_pcd, "integ_final_PointNet")
        final_est = TransformStamped()
        
        final_est.header.frame_id = self.header_frame_id
        final_est.child_frame_id = self.child_frame_id
        final_est.transform.rotation = est_pose.pose.orientation
        final_est.transform.translation.x = est_pose.pose.position.x + self.offset[0][0]
        final_est.transform.translation.y = est_pose.pose.position.y + self.offset[0][1]
        final_est.transform.translation.z = est_pose.pose.position.z + self.offset[0][2]
        final_est.header.stamp = rospy.Time.now()
        self.tf.sendTransform(final_est)
        owari_est = TransformStamped()
        owari_est = final_est

        header = Header()
        header.frame_id = self.header_frame_id
        final_pcd = convertCloudFromOpen3dToRos(self.stl_est, header)
        est_cloud = do_transform_cloud(final_pcd, final_est)
        # print(est_cloud)
        self.pub_final.publish(est_cloud)
        res.kati = True
        return res

    def single_extract(self, req):
        temdata = -1000000
        temdata2 = -100000000
        temdata3 = -10000000
        keep_index = 0
        keep_index2 = 0
        keep_index3 = 0
        pcd_list = []
        choice = []
        cls_all = []
        offset_list = []
        offset_list2 = []

        for i in range(np.array(req.x).shape[0]):
            data = np.array((req.x[i].data, req.y[i].data, req.z[i].data))
            data = data.T

            normalized_pcd, offset = getNormalizedPcd(data, self.opt.resolution)

            segme, est_time, temmat_data = semantic_exec.pose_prediction(self.model, normalized_pcd, self.opt.resolution)
            # print("tanomzu")
            # print(np.array(segme).shape)
            self.instance_pub.publish(segme)

            temmat_pcd, offset2 = getNormalizedPcd(temmat_data, self.resolution)
            cls, confidence = temmat_exec.pose_prediction(self.temmat_model, temmat_pcd, self.temmat_opt)
            # print(offset2)
            cls_all.append(cls)
            choice.append(confidence)
            offset_list.append(offset)
            offset_list2.append(offset2)
            if cls[0] == 0 and temdata<confidence:
                temdata = confidence
                keep_index = i
            elif cls[0] == 0 and temdata2<confidence:
                temdata2 = confidence
                keep_index2 = i
            elif cls[0] == 0 and temdata3<confidence:
                temdata3 = confidence
                keep_index3 = i
            pcd_list.append(temmat_pcd)
        
        self.final_pcd = pcd_list[keep_index]
        self.offset = offset_list[keep_index]
        self.offset_second = offset_list2[keep_index]
        print("naruhodo")
        print(np.array(self.offset).shape)

        final_pcd2 = pcd_list[keep_index2]
        offset2 = offset_list[keep_index2]

        final_pcd3 = pcd_list[keep_index3]
        offset3 = offset_list[keep_index3]

        self.final_pcd_for_Acuracy = self.final_pcd + self.offset + self.offset_second
        
        raugh_GT = out_segmentation()
        for j in range(self.final_pcd.shape[0]):
            raugh_GT.x.append(self.final_pcd[j, 0] + self.offset[0][0])
            raugh_GT.y.append(self.final_pcd[j, 1] + self.offset[0][1])
            raugh_GT.z.append(self.final_pcd[j, 2] + self.offset[0][2])
        for j in range(final_pcd2.shape[0]):
            raugh_GT.x.append(final_pcd2[j, 0] + offset2[0][0])
            raugh_GT.y.append(final_pcd2[j, 1] + offset2[0][1])
            raugh_GT.z.append(final_pcd2[j, 2] + offset2[0][2])
        for j in range(final_pcd3.shape[0]):
            raugh_GT.x.append(final_pcd3[j, 0] + offset3[0][0])
            raugh_GT.y.append(final_pcd3[j, 1] + offset3[0][1])
            raugh_GT.z.append(final_pcd3[j, 2] + offset3[0][2])
        self.pub_GT.publish(raugh_GT)

    def accuracy_calculate(self, req):
        pre_data = []
        cnt_list = [0] * int(self.max_instance)
        print("iketa")
        print(np.array(cnt_list).shape)
        print(cnt_list[0])
        for i in range(self.final_pcd_for_Acuracy.shape[0]):
            # print(i)
            # print(self.final_pcd_for_Accuracy[i][0])
            for j in range(np.array(req.GT).shape[0]):
                # print(self.final_pcd_for_Accuracy[i][0])
                # print(req.GT[j].data[0])
                if self.final_pcd_for_Acuracy[i][0] == req.GT[j].data[0] and self.final_pcd_for_Acuracy[i][1] == req.GT[j].data[1] and self.final_pcd_for_Acuracy[i][2] == req.GT[j].data[2]:
                    # print("tabun")
                    pre_data.append(req.GT[j].data)
                    # print(req.GT[j].data[3])
                    cnt_list[int(req.GT[j].data[3])] += 1
        max_cnt = np.argmax(np.array(cnt_list))
        tp = 0
        fp = 0
        print(np.array(pre_data).shape)
        for i in range(np.array(pre_data).shape[0]):
            if pre_data[i][3] == max_cnt:
                tp += 1
            else:
                fp += 1
        iou = tp / (tp+fp)
        print("iou")
        print(iou)



if __name__ == "__main__":
    node = DnnNode()
    try:
        node.run_service()
        rospy.spin()
    except rospy.ROSInterruptException:
        sys.exit()