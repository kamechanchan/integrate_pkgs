#!/usr/bin/env python3

# from denso_srvs.srv import input_data
import cv_bridge
from sensor_msgs.msg import Image
import numpy as np
import cv2

from denso_msgs.msg import yolo_bridge
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import Int32MultiArray
from denso_srvs.srv import img_bridge_pcl
from denso_srvs.srv import *
from denso_srvs.srv import input_data
from denso_srvs.srv import input_Bbox
from denso_srvs.srv import input_semseg
from denso_srvs.srv import input_occ
# from denso_srvs.srv import input_bridge_multi
from denso_srvs.srv import Acc_bridge
from denso_srvs.srv import Acc_pcl_dnn
from sensor_msgs.msg import Image
from sensor_msgs.msg import PointCloud2

import rospy
import time
from tqdm import tqdm
import pcl
import h5py

from cv_bridge import CvBridge, CvBridgeError

# from Acc_read_hdf5 import Read_hdf5


class Client():
    def __init__(self):
        print("start")
        rospy.init_node("client_main", anonymous=True)
        self.num_data = rospy.get_param("~data_number", 1)
        loop = rospy.Rate(1)
        # while (1):
        self.client_main()
            

    def client_main(self):
        for i in tqdm(range(self.num_data)):
            read = Read_hdf5()
            # in_cloud, in_img, masks, self.rotation, self.translation = read.get_data(i+1)
            in_cloud, in_img, rotation, translation = read.get_data(i+1)
            # print("in_cloud")
            # print(type(in_cloud))
            # print(np.array(in_cloud).shape)
            # in_pcl = list(in_cloud)
            # print(type(in_pcl[0]))
            # self.sensor_pcl = PointCloud2()
            # self.sensor_pcl.data = in_pcl
            # print(self.sensor_pcl)
            # self.in_img = Image()
            # print("img")
            # print(type(in_img))
            # # in_img = np.array(in_img, dtype=np.uint64)
            # print(list(in_img))
            # self.in_img.data = list(in_img)

            # print(np.array(in_cloud).shape)
            
            self.x_pcl = in_cloud[:,0]
            self.y_pcl = in_cloud[:,1]
            self.z_pcl = in_cloud[:,2]

            self.rotation = []
            self.translation = []
            # rotation = list(rotation)
            # translation = list(translation)

            # try:
            if np.array(rotation).shape[0] == np.array(translation).shape[0]:
                for i in range(np.array(rotation).shape[0]):
                    t = []
                    # print(type(translation[1,0]))
                    # print(translation[1,0])
                    t.append(translation[i,0])
                    t.append(translation[i,1])
                    t.append(translation[i,2])
                    r = []
                    r.append(rotation[i,0])
                    r.append(rotation[i,1])
                    r.append(rotation[i,2])
                    r.append(rotation[i,3])
                    tran = Float64MultiArray(data=t)
                    rot = Float64MultiArray(data=r)
                    self.translation += [tran]
                    self.rotation += [rot]
            # print(np.array(self.translation).dtype)
            print(np.array(self.translation[0].data).dtype)
            # print(np.array(self.translation[0]).dtype)
            # except:
            #     print("error: rotation and translation are not same shape")
            #     print(np.array(rotation).shape[0])
            #     print(np.array(translation).shape[0])
            #     sys.exit(1)

            # self.x_pcl = np.squeeze(np.array(self.x_pcl))
            # self.y_pcl = np.squeeze(np.array(self.y_pcl))
            # self.z_pcl = np.squeeze(np.array(self.z_pcl))
            # print(type(self.x_pcl))
            
            self.in_cloud = []
            for i in range(np.array(in_cloud).shape[0]):
                # pre_cloud = in_cloud[i, :].data
                cloud = []
                cloud.append(in_cloud[i,0])
                # print("naisu")
                # print(in_cloud[i,0])
                # print(in_cloud[i,3].dtype)
                cloud.append(in_cloud[i,1])
                cloud.append(in_cloud[i,2])
                cloud.append(in_cloud[i,3])
                # cloud = Float32MultiArray(data=in_cloud[i,:].data)
                # self.in_cloud += cloud
                pre_cloud = Float32MultiArray(data=cloud)
                self.in_cloud += [pre_cloud]
                # self.in_cloud += [in_cloud[i,:]]
            # print(type(in_cloud[1,:]))
            # print(type(pre_cloud))

            # masks = np.array(masks)
            # masks = np.squeeze(masks)

            # print(np.array(in_cloud).shape)
            # print(masks.shape)

            # x_data = in_cloud[:,0].data
            # y_data = in_cloud[:,1].data
            # z_data = in_cloud[:,2].data
            # print("osososos")
            # print(np.array(x_data).shape)

            # self.GT_masks = np.array(x_data, y_data, z_data, masks)
            # self.GT_masks = np.concatenate([x_data, y_data, z_data, masks])
            # print("GT_masks")
            # print(self.GT_masks.shape)

            # print("fakf;af")
            # print(np.array(self.x_pcl).shape)
            # print(np.array(self.masks).shape)
            # print(masks.shape)
            # print(np.array(self.rotation).shape)
            # print(np.array(self.translation).shape)
            
            # x_max = 0
            # x_min = 0
            # y_max = 0
            # y_min = 0
            # z_max = 0
            # z_min = 0
            # for j in range(np.array(self.x_pcl).shape[0]):
            #     if x_max > self.x_pcl[j]:
            #         x_max = self.x_pcl[j]
            #     if y_max > self.y_pcl[j]:
            #         y_max = self.y_pcl[j]
            #     if z_max > self.z_pcl[j]:
            #         z_max = self.z_pcl[j]
            #     if x_min < self.x_pcl[j]:
            #         x_min = self.x_pcl[j]
            #     if y_min < self.y_pcl[j]:
            #         y_min = self.y_pcl[j]
            #     if z_min < self.z_pcl[j]:
            #         z_min = self.z_pcl[j]
            # print("hajimaruze")
            # print(x_max)
            # print(x_min)
            # print(y_max)
            # print(y_min)
            # print(z_max)
            # print(z_min)


            # self.x_pcl = in_cloud[:,1]
            # self.y_pcl = in_cloud[:,0]
            # self.z_pcl = in_cloud[:,2]

            try:
                in_img = np.array(in_img)
                # print(in_img.dtype)
                bridge = CvBridge()
                self.in_img = bridge.cv2_to_imgmsg(in_img, "bgr8")
                # print(self.in_img.header)
            except CvBridgeError as e:
                print(e)
                return e
            # sensor_pcl = pcl
            # pub = rospy.Publisher("kamisama_IMG", Image, queue_size=10)
            # pub.publish(self.in_img)
            start = time.time()
            self.Bbox()
            goal = time.time()
            self.img_bridge_pcl()
            # box = time.time()
            self.pcl_dnn()
            # dnn = time.time()
            # rospy.spin()

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

    def img_bridge_pcl(self):
        rospy.wait_for_service("input_bridge")
        try:
            print("atonobi")
            # print(self.Bbox_coordinate)
            start = rospy.ServiceProxy("input_bridge", Acc_bridge)
            # print("na~ru")
            # res = start(self.occluder_Bbox, self.out_img, self.in_img)
            # print(np.array(self.x_pcl).shape)
            # print(np.array(self.y_pcl).shape)
            # print(np.array(self.z_pcl).shape)
            res = start(self.Bbox_coordinate, self.out_img, self.in_img, self.x_pcl, self.y_pcl, self.z_pcl)
            self.x = res.x
            self.y = res.y
            self.z = res.z
            # print("xyz")
            # print(np.array(res.x).shape)
            # print(np.array(res.y).shape)
            # print(np.array(res.z).shape)
            # print("tuinihomeraretato")
            # print(np.array(self.x).shape)
            # print(np.array(self.y).shape)
            # print(np.array(self.z).shape)
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
            print("start")
            # print(type(self.x))
            # print(type(self.y))
            # print(np.array(self.z).shape)
            # print(type(self.in_cloud))
            # print(np.array(self.in_cloud).dtype)
            print(type(self.translation))
            print(np.array(self.translation).shape)
            print(np.array(self.rotation).shape)
            print(np.array(self.translation[0]).dtype)
            print(np.array(self.rotation).dtype)
            # print(np.array(self.x[0]))
            start = rospy.ServiceProxy("pcl_dnn", Acc_pcl_dnn)
            # self.instances = [1, 2]
            # res = start(self.x, self.y, self.z, self.GT_masks, self.translation, self.rotation)
            res = start(self.x, self.y, self.z, self.in_cloud, self.translation, self.rotation)
            # print("final:" + str(res.kati))
        except rospy.ServiceException:
            print("service call failed pcl_dnn")

class Read_hdf5():
    def __init__(self):
        # rospy.init_node("main", anonymous=True)
        # rospy.init_node("ikerukane", anonymous=True)
        self.data_path = rospy.get_param("~data_path", "/home/ericlab/hdf5_file/Acc_accuracy/")
        self.data_name = rospy.get_param("~data_name", "instance_tsuchida_1_4_2_1.hdf5")
        data = self.data_path + self.data_name
        print(data)
        self.hdf5_file = h5py.File(data, "r")

    def get_data(self, index):
        in_cloud = self.hdf5_file["data_" + str(index)]["Points"]
        in_img = self.hdf5_file["data_" + str(index)]["img"]
        # masks = self.hdf5_file["data_" + str(index)]["masks"]
        rotation = self.hdf5_file["data_" + str(index)]["rotation"]
        translation = self.hdf5_file["data_" + str(index)]["translation"]

        # return in_cloud, in_img, masks, rotation, translation
        return in_cloud, in_img, rotation, translation

if __name__ == "__main__":
    Client()
    # rospy.spin()