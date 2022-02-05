#!/usr/bin/env python3

from xml.etree.ElementTree import TreeBuilder
import cv2
from numpy.lib.index_tricks import ravel_multi_index
import cv_bridge
import rospy
from denso_srvs.srv import img_bridge_pcl
# from denso_srvs.srv import SemSeg_Occ
from denso_srvs.srv import *
from rospy.impl.tcpros_base import start_tcpros_server
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import Image

import os,sys

sys.path.append(os.path.join(os.path.dirname(__file__), "../../../networks/SemSeg_2D/"))

import numpy as np

import torch
import torchvision.transforms as transform

import encoding.utils as utils

# from PIL import Image
from cv_bridge import CvBridge

from encoding.nn import BatchNorm
from encoding.datasets import datasets
from encoding.models import get_model, get_segmentation_model, MultiEvalModule

from option import Options

from std_msgs.msg import Int32MultiArray

from denso_msgs.msg import new_1_array, new_array

import time


class to_SemSegNet():
    def __init__(self):
        option = Options()
        # option.parser.add_argument('--input-path', type=str, required=True, help='path to read input image')
        option.parser.add_argument('--save-path', type=str, required=True, help='path to save output image')
        self.args = option.parse()

        rospy.init_node("SemSeg_2d", anonymous=True)
        self.out_pub = rospy.Publisher("SemSeg_output", Image, queue_size=10)
        print("ok")
        # start = time.time()
        rospy.Service("input_SemSeg", input_semseg, self.data_transformation)
        # goal = time.time()
        # print("zentai:" + str(goal-start))
        rospy.spin()
        

    def data_transformation(self, req):
        start = time.time()
        print("service success!")
        self.input_ori = req.input_img
        self.Bbox_coordinate_ori = req.Bbox_data
        self.output_img = req.output_img
        # print(np.array(self.Bbox_coordinate[0]).shape)
        self.Bbox_coordinate = np.array(self.Bbox_coordinate_ori)
        self.bridge = CvBridge()
        self.input_img = self.bridge.imgmsg_to_cv2(self.input_ori)
        to_cv2 = time.time()
        print("to_cv2:" + str(to_cv2-start))
        input_transform = transform.Compose([
        transform.ToTensor(),
        transform.Normalize([.485, .456, .406], [.229, .224, .225])])
        # input_transform = transform.Compose([
        # transform.ToTensor()])
        self.input = input_transform(self.input_img).unsqueeze(0)
        tensor = time.time()
        print("tensor:" + str(tensor-to_cv2))
        self.Net(self.args)
        net = time.time()
        print("net:" + str(net-tensor))
        self.Bbox_SemSeg_combine()
        combine = time.time()
        print("box_semseg_combine:" + str(combine-net))
        print("zentai:" + str(combine-start)) 
        res = input_semsegResponse()
        res.out_data = self.out_img
        return res

    def Net(self, args):
        start = time.time()
        if args.model_zoo is not None:
                model = get_model(args.model_zoo, pretrained=True)
        else:
            model = get_segmentation_model(args.model, dataset = args.dataset,
                                        backbone = args.backbone, dilated = args.dilated,
                                        lateral = args.lateral, jpu = args.jpu, aux = args.aux,
                                        se_loss = args.se_loss, norm_layer = BatchNorm,
                                        base_size = args.base_size, crop_size = args.crop_size)
            # resuming checkpoint
            if args.resume is None or not os.path.isfile(args.resume):
                raise RuntimeError("=> no checkpoint found at '{}'" .format(args.resume))
            hu_n = time.time()
            checkpoint = torch.load(args.resume)
            load = time.time()
            print("kokoda"+str(load-start))
            print("naruhodo"+str(load-hu_n))
            print(type(model))
            # strict=False, so that it is compatible with old pytorch saved models
            model.load_state_dict(checkpoint['state_dict'], strict=False)
            print("=> loaded checkpoint '{}' (epoch {})".format(args.resume, checkpoint['epoch']))
            state = time.time()
            print("state"+str(state-load))

        # print(model)
        scales = [0.5, 0.75, 1.0, 1.25, 1.5, 1.75, 2.0, 2.25] if args.dataset == 'citys' else \
            [0.5, 0.75, 1.0, 1.25, 1.5, 1.75]
        if not args.ms:
            scales = [1.0]
        num_classes = datasets[args.dataset.lower()].NUM_CLASS
        evaluator = MultiEvalModule(model, num_classes, scales=scales, flip=args.ms).cuda()
        evaluator.eval()
        kokoka = time.time()
        print("kokoka"+str(kokoka-state))

        with torch.no_grad():
            output = evaluator.parallel_forward(self.input)[0]
            net = time.time()
            print("net"+str(net-kokoka))
            predict = torch.max(output, 1)[1].cpu().numpy()
        mask = utils.get_mask_pallete(predict, args.dataset)
        mask.save(args.save_path)
        save_mask = time.time()
        print("mask"+str(save_mask-net))
        predict = predict.astype(np.uint8)
        self.predict = np.reshape(predict, (predict.shape[1],predict.shape[2],predict.shape[0]))
        # print(predict.shape)
        # print(predict.dtype)
        out_img = self.bridge.cv2_to_imgmsg(self.predict)
        # print(out_img)
        self.out_pub.publish(out_img)
        savesama = time.time()
        print("iijanai"+str(savesama-save_mask))
        print("iijanai1"+str(savesama-kokoka))
        print("iijanai"+str(savesama-start))

    def Bbox_SemSeg_combine(self):
        # print(out_img.shape)
        # print(out_img)
        # print(self.predict[1][2][0])
        final_out_img = []
        # print(self.Bbox_coordinate[0])
        # print(self.Bbox_coordinate[0].data[0])

        # for i in range(self.Bbox_coordinate.shape[0]):

            # out_img = np.zeros((self.predict.shape[0], self.predict.shape[1], self.predict.shape[2]), dtype=np.uint8)
            # print(out_img.shape)
            # out_img = list(out_img.item())
            # out_img = out_img.tolist()
        print("kami")
        naruho = time.time()
        out_img = []
        # print("***")
        for i in range(self.Bbox_coordinate.shape[0]):
            img_0_0 = []
            for ii in range(self.predict.shape[0]):
                img_0 = new_array(img_new=[0] * self.predict.shape[1])
                    # if ii == 8:
                        # print(np.array(img_0.img_new).shape)
                        # pass
                    # print(np.array(img_0)

                    # img_0_0.append(img_0)
                img_0_0 += [img_0]
            # print("11111")
            # print(np.array(img_0.img_new).shape)
            # print(np.array(img_0_0).shape)
            img_1 = new_1_array(img_new_1=img_0_0)
                # img_1 = new_array(img_new=[0] * self.predict.shape[1])
                # img_1.img_new = [[0] * self.predict.shape[1] for i in range(self.predict.shape[0]) for j in range(self.predict.shape[2])]
                # img_1.img_new = [0] * self.predict.shape[1]
                # out_img.img_new_1 = [0] * self.predict.shape[0]
                # out11 = []
                # out11 += img_1.img_new
                # out_img = new_1_array(img_new_1=out11)
                
            # out_img += [[img_1] for i in range(self.Bbox_coordinate.shape[0])]
            out_img += [img_1]
        # [[0] * self.predict.shape[1] for i in range(self.predict.shape[0])]
            # out_img = new_1_array(img_new_1=out_img)

            # out_img.img_new_1 += [img_1]
            # print(self.predict.shape[0])
            # print(self.predict.shape[1])
        print(np.array(img_1.img_new_1[0].img_new).shape)
        print(self.input_img.shape)
        print(np.array(out_img).shape)
            # print(np.array(out_img).shape)
        kokosuka = time.time()
        print("uzasugi"+str(kokosuka-naruho))
        for i in range(self.Bbox_coordinate.shape[0]):
            for j in range(self.Bbox_coordinate[i].data[0], self.Bbox_coordinate[i].data[2]):
                # print(self.Bbox_coordinate[i].data[0])
                # print(self.Bbox_coordinate[i].data[2])
                for k in range(self.Bbox_coordinate[i].data[1], self.Bbox_coordinate[i].data[3]):
                    # print(self.Bbox_coordinate[i].data[1])
                    # print(self.Bbox_coordinate[i].data[3])
                    if self.predict[k][j] != 1000:
                        # print("af;")
                        # out_img.img_new_1[0][k][j] = 255
                        out_img[i].img_new_1[k].img_new[j] = 255
            # cv2.imwrite("/home/ericlab/Desktop/ishiyama/zatsumuyou/kamiyo" + str(i) + ".jpg", out_img)
            # out_img = Int32MultiArray(data=out_img)
            
            # from denso_msgs.msg import new_1_array
            # final_out_img = new_1_array()
            # final_out_img += [out_img]
            # true_final_out_img = []
            # true_final_out_img.append(final_out_img)
            # final_out_img += [out_img]

        # print("tanomzu")
        # print(self.input_img.shape)
        # for i in range(self.Bbox_coordinate.shape[0]):
        #     out_img = np.zeros((self.input_img.shape[0], self.input_img.shape[1], self.input_img.shape[2]))
        #     for j in range(self.input_img.shape[0]):
        #         # print(self.Bbox_coordinate[i].data[0])
        #         # print(self.Bbox_coordinate[i].data[2])
        #         for k in range(self.input_img.shape[1]):
        #             # print(self.Bbox_coordinate[i].data[1])
        #             # print(self.Bbox_coordinate[i].data[3])
        #             if final_out_img[i][j][k][0] == 255:
        #                 for n in range(3):
        #                     out_img[j][k][n] = self.input_img[j][k][n]
        #             else :
        #                 for n in range(3):
        #                     out_img[j][k][n] = 0
        #     cv2.imwrite("/home/ericlab/Desktop/ishiyama/zatsumuyou/hikaku" + str(i) + ".jpg", out_img)
        
        kokokai = time.time()
        print("kokoha1byoukann"+str(kokokai-kokosuka))
        print("finish")
        print(np.array(out_img).shape)
        self.out_img = out_img
        # print(np.array(final_out_img[0].data).shape)
        # final_out_img = np.array(final_out_img).squeeze()
        # final_out_img = list(final_out_img)
        # print(final_out_img.shape)
        # print(type(self.input_ori))

        # rospy.wait_for_service("Semseg_to_occ")
        # print("kami")
        # try:
        #     print("kanpekidesu")
        #     to_occ = rospy.ServiceProxy("Semseg_to_occ", img_bridge_pcl)
        #     print("juubunnsugimasu")
        #     # complete = to_occ(self.Bbox_coordinate, self.output_img, self.input_ori)
        #     complete = to_occ(out_img, self.Bbox_coordinate_ori, self.output_img, self.input_ori)
        #     print(complete)
        # except rospy.ServiceException:
        #     print("SemSeg to Occ service call failed")

if __name__ == "__main__":
    go = to_SemSegNet()