#!/usr/bin/env python3

import rospy
from denso_srvs.srv import img_bridge_pcl
from denso_srvs.srv import *
# from denso_srvs.srv import SemSeg_Occ

import os
os.environ['CUDA_VISIBLE_DEVICES'] = '0'
import sys
import json
import cv2
import numpy as np
from PIL import Image
# import pycocotools.mask as maskUtils

sys.path.append(os.path.join(os.path.dirname(__file__), "../../../networks/Deocclusion/"))

# from matting_api import Matting
import utils
import inference as infer
# from datasets import COCOADataset, read_COCOA
from demo_utils import *

from torchvision import transforms

from cv_bridge import CvBridge

import time


class Occlusion():
    def __init__(self):
        rospy.init_node("DeOcclusion", anonymous=True)
        # self.out_pub = rospy.Publisher("Occlusion_output", Image, queue_size=10)
        print("ok")
        rospy.Service("input_occ", input_occ, self.data_transformation)
        rospy.spin()
    
    def data_transformation(self, req):
        start = time.time()
        print("service success!")
        self.input_ori = req.input_img
        self.mask = req.out_data
        self.mask = np.array(self.mask)
        self.Bbox_coordinate = req.Bbox_data
        self.bridge = CvBridge()
        self.input_img = self.bridge.imgmsg_to_cv2(self.input_ori)
        to_cv2 = time.time()
        print("tocv2"+str(to_cv2-start))
        # input_transform = transform.Compose([
        # transform.ToTensor(),
        # transform.Normalize([.485, .456, .406], [.229, .224, .225])])
        input_transform = transforms.ToTensor()(self.input_img)
        self.input = input_transform.unsqueeze(0)
        print("Mr_tsuchida")

        final_out_img = []
        print(np.array(self.mask).shape)
        self.out_img = np.zeros((np.array(self.mask).shape[0], self.input_img.shape[0], self.input_img.shape[1]), dtype=np.uint8)
        syokika_moromoro = time.time()
        print("moromoro"+str(syokika_moromoro-to_cv2))
        for i in range(np.array(self.mask).shape[0]):
            out_img = np.zeros((self.input_img.shape[0], self.input_img.shape[1], 1), dtype=np.uint8)
            for j in range(self.input_img.shape[0]):
                # print(self.mask[i].data[0])
                # print(self.mask[i].data[2])
                for k in range(self.input_img.shape[1]):
                    # print(self.mask[i].data[1])
                    # print(self.mask[i].data[3])
                    if self.mask[i].img_new_1[j].img_new[k] == 255:
                        for n in range(3):
                            # self.out_img[j][k][n] = self.input_img[j][k][n]
                            self.out_img[i][j][k] = 1
                            out_img[j][k][0] = 255
                    # else :
                    # else:
                    #     for n in range(3):
                    #         self.out_img[i][j][k] = 0
            im = time.time()
            cv2.imwrite("/home/ericlab/Desktop/ishiyama/zatsumuyou/sorokyukei" + str(i) + ".jpg", out_img)
            write = time.time()
            # print("imwrite:"+str(write-im))
            # print(i)
        combine = time.time()
        print("combine"+str(combine-syokika_moromoro))
        res = input_occResponse()
        res.Bbox_data_occluder = self.Bbox_coordinate
        itou = time.time()
        self.Net()
        net = time.time()
        print("net"+str(net-itou))
        print("zentai"+str(net-start))
        # self.Net(self.args)
        return res

    def Net(self):
        kokokai = time.time()
        print("sokjakfj;a")
        phase = 'val'
        # PCNet-M
        exp = '/home/ericlab/ros_package/integrate_ws/src/integrate_pkgs/networks/Deocclusion/yaml'
        config_file = exp + '/config.yaml'
        load_model = '/home/ericlab/ros_package/integrate_ws/src/integrate_pkgs/networks/Deocclusion/weights/COCOA_pcnet_m.pth.tar'
        pcnetm = DemoPCNetM(config_file, load_model)
        kokodesukai = time.time()
        print("netm"+str(kokodesukai-kokokai))
        # np.set_printoptions(threshold=np.inf)
        # print(self.out_img)
        order_matrix = infer.infer_order(pcnetm.model, self.input_img, self.out_img, self.Bbox_coordinate, use_rgb=pcnetm.use_rgb, th=0.1, dilate_kernel=0,input_size=256, min_input_size=16, interp='nearest', debug_info=False)
        infer_o = time.time()
        print("infer"+str(infer_o-kokodesukai))
        ind = None
        print("soukisenkou")
        # plt.figure(figsize=(10,5))
        # plt.subplot(121)
        # pos = draw_graph(order_matrix, ind=ind)
        # plt.subplot(122)
        # plt.show()

if __name__ == "__main__":
    start = Occlusion()