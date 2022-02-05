#!/usr/bin/env python3

from logging import info
import sys
import os

from numpy.lib.arraypad import pad
sys.path.append(os.path.join(os.path.dirname(__file__), '../../../networks'))
# sys.path.append(os.path.join(os.path.dirname(__file__), '../../networks/yolov3/layer'))
from yolov3.YOLO import YOLOv3
from cv2 import BRISK
from numpy.core.fromnumeric import trace
import torch
import rospy
from denso_srvs.srv import input_data
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from torchvision import transforms 
import numpy as np
import cv2
from torchvision import ops as ops
from PIL import ImageDraw, ImageFont
from matplotlib import pyplot as plt, rcParams
import yaml
import rospkg
import PIL
# from .msg import yolo_bridge
from denso_msgs.msg import yolo_bridge
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Int32MultiArray
from denso_srvs.srv import img_bridge_pcl
from denso_srvs.srv import *
from denso_srvs.srv import input_Bbox

import time


def postprocess(outputs, conf_threshold, iou_threshold, pad_infos):
    decoded = []
    for output, *pad_info in zip(outputs, *pad_infos):
        # 矩形の形式を変換する。 (YOLO format -> Pascal VOC format)
        output[:, :4] = yolo_to_pascalvoc(output[:, :4])

        # フィルタリングする。
        output = filter_boxes(output, conf_threshold, iou_threshold)

        # letterbox 処理を行う前の座標に変換する。
        if len(output):
            output[:, :4] = decode_bboxes(output[:, :4], pad_info)

        # デコードする。
        decoded.append(output)

    return decoded

def yolo_to_pascalvoc(bboxes):
    cx, cy, w, h = torch.chunk(bboxes, 4, dim=1)
    # print("majikami")
    # print(cx.shape)
    # print(cy.shape)
    # print(w.shape)
    # print(h.shape)

    x1, y1 = cx - w / 2, cy - h / 2
    x2, y2 = cx + w / 2, cy + h / 2

    bboxes = torch.cat((x1, y1, x2, y2), dim=1)
    # print(bboxes.shape)

    return bboxes

def filter_boxes(output, conf_threshold, iou_threshold):
    # 閾値以上の箇所を探す。
    keep_rows, keep_cols = (
        (output[:, 5:] * output[:, 4:5] >= conf_threshold).nonzero().T
    )
    if not keep_rows.nelement():
        return []

    conf_filtered = torch.cat(
        (
            output[keep_rows, :5],
            output[keep_rows, 5 + keep_cols].unsqueeze(1),
            keep_cols.float().unsqueeze(1),
        ),
        1,
    )

    # Non Maximum Suppression を適用する。
    nms_filtered = []
    detected_classes = conf_filtered[:, 6].unique()
    for c in detected_classes:
        detections_class = conf_filtered[conf_filtered[:, 6] == c]
        keep_indices = ops.nms(
            detections_class[:, :4],
            detections_class[:, 4] * detections_class[:, 5],
            iou_threshold,
        )
        detections_class = detections_class[keep_indices]

        nms_filtered.append(detections_class)

    nms_filtered = torch.cat(nms_filtered)

    return nms_filtered

def decode_bboxes(bboxes, info_img):
    # print(np.array(info_img).shape)
    scale_x, scale_y, dx, dy = info_img

    bboxes -= torch.stack([dx, dy, dx, dy])
    bboxes /= torch.stack([scale_x, scale_y, scale_x, scale_y])

    return bboxes

def get_text_color(color):
    value = color[0] * 0.299 + color[1] * 0.587 + color[2] * 0.114

    return "black" if value > 128 else "white"


class Bbox:
    def __init__(self):
        self.img_path = "/home/ericlab/tsuchida/ishi.jpg"
        print(self.img_path)
        # image_topic_name = rospy.get_param("~image_topic", "/photoneo_center/sensor/image_color")
        rospy.init_node("client", anonymous=True)
        print("ok")
        self.img_size = 416
        self.gpu_id = 0
        # self.opt = TestOptions().parse()
        self.conf_threshold = 0.5
        self.nms_threshold = 0.45
        # self.class_names = "HV8"
        rospack = rospkg.RosPack()
        load_path = rospy.get_param("~load_path", "networks/yolov3/weights/yolov3_000200.pth")
        network_path = rospack.get_path("all_estimator") + "/../../"
        self.font_path = network_path + "networks/yolov3/font/ipag.ttc"
        self.save_path = network_path + "networks/yolov3/output/output.jpg"
        self.config_path = network_path + "networks/yolov3/config/yolov3_denso.yaml"
        # self.load_path = network_path + "networks/yolov3/weights/yolov3_000200.pth"
        self.load_path = network_path + load_path
        self.con_path = network_path + "networks/yolov3/config"
        self.arch = "YOLO"
        self.device_set()     
        
        with open(self.config_path) as f:
            self.config = yaml.safe_load(f)
        # print(self.config["model"]["class_names"])
        class_path = os.path.join(self.con_path, self.config["model"]["class_names"])
        with open(class_path) as f:
            
            self.class_names = [x.strip() for x in f.read().splitlines()]
        self.create_model()
        # rospy.Subscriber(image_topic_name, Image, self.callback)
        # rospy.wait_for_service("ishiyama_input_data")
        # data = None
        # try:
        #     tem_data = rospy.ServiceProxy("ishiyama_input_data", input_data)
        #     out_data = tem_data(data)
        #     self.input = self.data_transformation(out_data.out_img)
        #     To_Yolo(self.input, self.img_path)
        #     # self.pub.publish(out_data.out_img)
        #     # print(out_data.out_img)
        # except rospy.ServiceException:
        #     print("service call failed")
        # rospy.spin()

        # start = time.time()
        rospy.Service("input_Bbox", input_Bbox, self.tem_data)
        # goal = time.time()
        rospy.spin()

    def callback(self, data):
        print("kati")
        # rospy.wait_for_service("ishiyama_input_data")
        # try:
        #     tem_data = rospy.ServiceProxy("ishiyama_input_data", input_data)
        #     out_data = tem_data(data)
        #     self.input = self.data_transformation(out_data.out_img)
        #     To_Yolo(self.input, self.img_path)
        #     # self.pub.publish(out_data.out_img)
        #     # print(out_data.out_img)
        # except rospy.ServiceException:
        #     print("service call failed")
    
    def data_transformation(self, data):
        
        try:
            bridge = CvBridge()
            # print("tanomuze")
            # print(data)
            out_data = bridge.imgmsg_to_cv2(data, "bgr8")
            print("save")
            cv2.imwrite(self.img_path, out_data)
            # out_data = transforms.ToTensor()(out_data)
            return out_data
        except CvBridgeError as e:
            print(e)
            return e

    def tem_data(self, req):
        start = time.time()
        print("Bbox")
        data = req.in_img
        in_data = self.data_transformation(data)
        haiyo = time.time()
        print("data_trans" + str(haiyo-start))
        self.To_Yolo(in_data)
        yolo = time.time()
        print("yolo:" + str(yolo-haiyo))
        res = input_BboxResponse()
        res.out_data = self.out_data
        res.output_img = self.output_img
        print("zentai:" + str(yolo-start))
        # res.input_img = self.input_img

        return res


    def To_Yolo(self, input):
        start = time.time()
        # self.in_pub = rospy.Publisher("ishiyama_input", Image, queue_size=10)
        # self.out_pub = rospy.Publisher("ishiyama_YOLO", yolo_bridge, queue_size=10)
        self.yolo_out_pub = rospy.Publisher("YOLO_output", Image, queue_size=10)
       

        self.input = input
        print("naru")
        start2 = time.time()
        print("start"+str(start2-start))
        
        dev = time.time()
        print("device"+str(dev-start2))
        self.data_for_yolo()
        data = time.time()
        print("data"+str(data-dev))
        self.est_net()
        net = time.time()
        print("net"+str(net-data))
        self.result()
        result = time.time()
        print("result"+str(result-net))
    
    def data_for_yolo(self, jitter=0, random_placing=False):
        # bridge = CvBridge()
        # in_img = bridge.cv2_to_imgmsg(self.input)
        # self.in_pub.publish(in_img)
        
        # print("inputsama")
        # print(type(self.input))
        org_h, org_w, _ = self.input.shape
        first = time.time()
        if jitter:
            dw = jitter * org_w
            dh = jitter * org_h
            new_aspect = (org_w + np.random.uniform(low=-dw, high=dw)) / (
                org_h + np.random.uniform(low=-dh, high=dh)
            )
        else:
            new_aspect = org_w / org_h
        # second = time.time()
        # print("first"+ str(second-first))

        if new_aspect < 1:
            new_w = int(self.img_size * new_aspect)
            new_h = self.img_size
        else:
            new_w = self.img_size
            new_h = int(self.img_size / new_aspect)
        # third = time.time()
        # print("second"+str(third-second))

        if random_placing:
            dx = int(np.random.uniform(self.img_size - new_w))
            dy = int(np.random.uniform(self.img_size - new_h))
        else:
            dx = (self.img_size - new_w) // 2
            dy = (self.img_size - new_h) // 2
        # fourth = time.time()
        # print("third"+str(fourth-third))

        img = cv2.resize(self.input, (new_w, new_h))
        pad_img = np.full((self.img_size, self.img_size, 3), 127, dtype=np.uint8)
        # kokoda = time.time()
        # print("koko"+str(kokoda-fourth))
        pad_img[dy : dy + new_h, dx : dx + new_w, :] = img
        # kokodesuka = time.time()
        # print("kokodesuka"+str(kokodesuka-kokoda))
        pad_img = transforms.ToTensor()(pad_img)
        # kokodesua = time.time()
        # print("tanomuze"+str(kokodesua-kokodesuka))
        # pad_img = np.array(pad_img)
        # pad_img = pad_img[np.newaxis, :,:,:]
        pad_img = pad_img.unsqueeze(0)
        tanomuze = time.time()
        # print("tanomuyo"+str(tanomuze-kokodesua))
        # print("kokosuka")
        # print(pad_img.shape)
        # print(self.device)
        self.pad_img = pad_img.to(self.device)
        
        # maji = time.time()
        # print("majisuka"+str(maji-tanomuze))
        # self.pad_img = pad_img

        # karume = time.time()
        # print("kokohatabunkarui"+str(karume-fourth))

        scale_x = np.float32(new_w / org_w)
        scale_y = np.float32(new_h / org_h)
        pad_info = (scale_x, scale_y, dx, dy)
        pad_infos = []
        for x in pad_info:
            y = torch.from_numpy(np.array([x], dtype=np.float32))
            pad_infos.append(y)
        # ayasime = time.time()
        # print("ayasii"+str(ayasime-karume))
        # pad_info = np.array(pad_info)
        # pad_info = torch.from_numpy(pad_info.astype(np.float32))
        # pad_info = [torch.from_numpy(x.astype(np.float32)) for x in np.array(pad_info)]
        # print("douiukotoda")
        # print(np.array(pad_infos).shape)
        self.pad_info = [x.to(self.device) for x in pad_infos]
        # kokodesuka = time.time()
        # print("koko?"+str(kokodesuka-ayasime))

    def device_set(self):
        if self.gpu_id >=0 and torch.cuda.is_available():
            self.device = torch.device("cuda", self.gpu_id)
            # print("gpu")
        else:
            # print("cpu")
            self.device = torch.device("cpu")

    def est_net(self):
        load = time.time()
        
        load_owari = time.time()
        print("load"+str(load_owari-load))
        # self.model = model.net.to(self.device).eval()
        # print(self.pad_img.shape)
        with torch.no_grad():
            print("tanomuze************")
            print(self.pad_img.shape)
            outputs = self.model(self.pad_img)
            self.outputs = postprocess(outputs, self.conf_threshold, self.nms_threshold, self.pad_info)
            # self.detections = self.output_to_dict(outputs, self.class_names) 

    def create_model(self):
        if self.arch =="YOLO":
            self.model = YOLOv3(self.config["model"])
            # print(self.load_path)
            state = torch.load(self.load_path, self.device)
            
            self.model.load_state_dict(state["model_state_dict"])
            print(f"state_dict format weights file loaded. {self.load_path}")
            self.model = self.model.to(self.device).eval()

    def result(self):
        # self.detections = self.output_to_dict(self.outputs, self.class_names) 
        # img = PIL.Image.open(self.img_path)

        img = cv2.cvtColor(self.input, cv2.COLOR_BGR2RGB)
        img = PIL.Image.fromarray(img)
        # msg_data = yolo_bridge()
        self.out_data = []
        # detection = []
        for x in self.outputs:
            # cluster_num = x.shape[0]
            # box_coor = np.zeros((cluster_num, 4), dtype=np.Float32)
            # box_coor = []
            # box_final_coor = []
            for i, (x1, y1, x2, y2, obj_conf, class_conf, label) in enumerate(x):
                box = {
                    "confidence": float(obj_conf * class_conf),
                    "class_id": int(label),
                    "class_name": self.class_names[int(label)],
                    "x1": float(x1),
                    "y1": float(y1),
                    "x2": float(x2),
                    "y2": float(y2),
                }

                x1 = int(np.clip(box["x1"], 0, img.size[0] - 1))
                y1 = int(np.clip(box["y1"], 0, img.size[1] - 1))
                x2 = int(np.clip(box["x2"], 0, img.size[0] - 1))
                y2 = int(np.clip(box["y2"], 0, img.size[1] - 1))
                caption = box["class_name"]

                draw = ImageDraw.Draw(img, mode="RGBA")
                # 色を作成する。
                cmap = plt.cm.get_cmap("hsv", len(self.class_names))
                # フォントを作成する。
                fontsize = max(3, int(0.01 * min(img.size)))
                font = ImageFont.truetype(self.font_path, size=fontsize)

                color = tuple(cmap(box["class_id"], bytes=True))
                # 矩形を描画する。
                draw.rectangle((x1, y1, x2, y2), outline=color, width=3)
                # ラベルを描画する。
                text_size = draw.textsize(caption, font=font)
                text_origin = np.array([x1, y1])
                text_color = get_text_color(color)

                draw.rectangle(
                    [tuple(text_origin), tuple(text_origin + text_size - 1)], fill=color
                )
                draw.text(text_origin, caption, fill=text_color, font=font)

                img_3 = np.array(img)
                # print(img_3.shape)
                img_3 = cv2.cvtColor(img_3, cv2.COLOR_RGB2BGR)
                # cv2.imshow("img_3", img_3)
                # if cv2.waitKey(1) & 0xff == ord("q"):
                #     break
                # box_coor = [x1, y1, x2, y2]
                
                # np.append(box_coor,x1)
                # np.append(box_coor,y1)
                # np.append(box_coor,x2)
                # np.append(box_coor,y2)
                
                box_coor = []
                box_coor.append(x1)
                box_coor.append(y1)
                box_coor.append(x2)
                box_coor.append(y2)
                
                box_final = Int32MultiArray(data=box_coor)
                # msg_data.out_data += [box_final]
                # print("houhou")
                # print(np.array(box_final).shape)
                self.out_data += [box_final]

                # print(box_final_coor)
                
                # for j, name in enumerate(box_coor):
                #     box_final_coor[i][j] = name
                
                # box_final_coor[0].append(x1)
                # print("ifajjf;")
                # print(x1)
                # print(i)
                # box_final_coor[i][0] = x1
                # box_final_coor[i][1] = y1
                # box_final_coor[i][2] 

                # detection.append(bbox)
            # for box in self.detection:
        #     print(
        #         f"{box['class_name']} {box['confidence']:.0%} "
        #         f"({box['x1']:.0f}, {box['y1']:.0f}, {box['x2']:.0f}, {box['y2']:.0f})"
        #     )
        # 検出結果を画像に描画して、保存する。
        
        # self.draw_boxes(img, self.detection)
        # img.save(self.save_path)
        # print(box_final_coor)
        bridge = CvBridge()
        # msg_data.input_img = bridge.cv2_to_imgmsg(self.input)
        # msg_data.output_img = bridge.cv2_to_imgmsg(img_3)
        # out_data = Float32MultiArray(data=out_data)
        # print("hajimaruyo")
        # print(type(self.input))
        # print(self.input.dtype)
        # print(img_3.shape)
        # cv2.imwrite("/home/ericlab/tsuchida/tsuchida.jpg", img_3)
        self.input_img = bridge.cv2_to_imgmsg(self.input)
        self.output_img = bridge.cv2_to_imgmsg(img_3)
        # print(len(box_coor))
        # box_coor = np.array(box_coor).reshape(cluster_num, 4)
        # box_final_coor = box_coor.tolist()
        # print(type(box_final_coor))
        # print(np.array(box_final_coor).shape)
        # print(box)
        # for i in range(cluster_num):
        #     tem_data = Float32MultiArray(data=box_final_coor(i))
        #     msg_data.out_data += [tem_data]
        # msg_data.out_data = Float32MultiArray(data=box_final_coor)
        # print(msg_data)
        # print(np.array(msg_data.out_data).shape)

        # self.out_pub.publish(msg_data)
        
        # self.yolo_out_pub.publish(msg_data.output_img)
        # print(self.output_img)
        self.yolo_out_pub.publish(self.output_img)

        print(type(self.out_data))
        print(np.array(self.out_data[0]).dtype)
        print(np.array(self.out_data).shape)
        print(self.out_data[0].data)
        # print(type(input_img))
        # print(type(output_img))

        # rospy.wait_for_service("Bbox_to_SemSeg")
        # try:
        #     to_SemSeg = rospy.ServiceProxy("Bbox_to_SemSeg", Bbox_to_SemSeg)
        #     complete = to_SemSeg(out_data, input_img, output_img)
        #     print("complete:" + str(complete))
        # except rospy.ServiceException:
        #     print("service call failed")

        # print("ishiy")

if __name__ == "__main__":
    Bbox()