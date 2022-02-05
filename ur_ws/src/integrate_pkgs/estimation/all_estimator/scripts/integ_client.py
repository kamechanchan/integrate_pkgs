#!/usr/bin/env python3

from denso_srvs.srv import input_data
from sensor_msgs.msg import Image
import numpy as np
import cv2

from denso_msgs.msg import yolo_bridge
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Int32MultiArray
from denso_srvs.srv import img_bridge_pcl
from denso_srvs.srv import *
from denso_srvs.srv import input_data
from denso_srvs.srv import input_Bbox
from denso_srvs.srv import input_semseg
from denso_srvs.srv import input_occ
from denso_srvs.srv import input_bridge_multi
from denso_srvs.srv import bounding_multi
from sensor_msgs.msg import Image

import rospy

import time


class Client():
    def __init__(self):
        print("start")
        rospy.init_node("client_main", anonymous=True)
        self.pub_hat = rospy.Publisher("hatt", Image, 10)
        loop = rospy.Rate(1)
        while (1):
            self.client_main()
            

    def client_main(self):
        
        start = time.time()
        self.input_img()
        goal = time.time()
        print("***************1:input_img*****************" + str(goal-start))
        self.Bbox()
        box = time.time()
        print("*******************2:Bbox*****************" + str(box-goal))
        # self.SemSeg()
        # semseg = time.time()
        # print("semseg" + str(semseg-box))
        # self.occlusion()
        # occ = time.time()
        # print("occlusion" + str(occ-semseg))
        self.img_bridge_pcl()
        pcl_bridge = time.time()
        print("************3:2D_bridge_pcl***************" + str(pcl_bridge-box))
        # print("kamikmai")
        self.pcl_dnn()
        dnn = time.time()
        print("************4:pcl_dnn******************" + str(dnn-pcl_bridge))
        print("**********all_run_time:" + str(dnn-start))
        print("owari")
    
    # client function start
    def input_img(self):
        rospy.wait_for_service("ishiyama_input_data")
        data = None
        try:
            start = rospy.ServiceProxy("ishiyama_input_data", input_data)
            res = start(data)
            self.in_img = res.out_img
            
            self.pub_hat.publish(self.in_img)
            # print("tanomimasu")
            # print(self.in_img)
        except rospy.ServiceException:
            print("service call failed input_data")

    def Bbox(self):
        rospy.wait_for_service("input_Bbox")
        try:
            start = rospy.ServiceProxy("input_Bbox", input_Bbox)
            # print(self.in_img)
            res = start(self.in_img)
            self.Bbox_coordinate = res.out_data
            # self.occluder_Bbox = res.out_data
            self.out_img = res.output_img
        except rospy.ServiceException:
            print("service call failed input_Bbox")

    # def SemSeg(self):
    #     rospy.wait_for_service("input_SemSeg")
    #     try:
    #         start = rospy.ServiceProxy("input_SemSeg", input_semseg)
    #         res = start(self.Bbox_coordinate, self.out_img, self.in_img)
    #         self.mask_img = res.out_data
    #     except rospy.ServiceException:
    #         print("service call failed input_semseg")
    
    # def occlusion(self):
    #     rospy.wait_for_service("input_occ")
    #     try:
    #         start = rospy.ServiceProxy("input_occ", input_occ)
    #         res = start(self.mask_img, self.Bbox_coordinate, self.in_img)
    #         self.occluder_Bbox = res.Bbox_data_occluder
    #     except rospy.ServiceException:
    #         print("service call failed input_occ")

    def img_bridge_pcl(self):
        rospy.wait_for_service("input_bridge")
        try:
            print("atonobi")
            # print(self.Bbox_coordinate)
            start = rospy.ServiceProxy("input_bridge", input_bridge_multi)
            # res = start(self.occluder_Bbox, self.out_img, self.in_img)
            res = start(self.Bbox_coordinate, self.out_img, self.in_img)
            self.x = res.x
            self.y = res.y
            self.z = res.z
            # print("xyz")
            # print(np.array(res.x).shape)
            # print(np.array(res.y).shape)
            # print(np.array(res.z).shape)
            print("tuinihomeraretato")
            print(np.array(self.x[0]).shape)
            print(np.array(self.y[0]).shape)
            print(np.array(self.z[0]).shape)
            # self.instances = res.instances
            # print(self.x)
            # print(self.y)
            # print(self.z)
            # print(self.instances)
        except rospy.ServiceException:
            print("service call failed img_bridge_pcl")

    def pcl_dnn(self):
        rospy.wait_for_service("pcl_dnn")
        try:
            start = rospy.ServiceProxy("pcl_dnn", bounding_multi)
            # self.instances = [1, 2]
            res = start(self.x, self.y, self.z)
            # print("final:" + str(res.kati))
        except rospy.ServiceException:
            print("service call failed pcl_dnn")


if __name__ == "__main__":
    Client()
    # rospy.spin()