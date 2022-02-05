#!/usr/bin/env python3
import torch
import torch.nn as nn
from .YOLO import YOLOv3


class Yolo_box:
    def __init__(self, opt):
        self.opt = opt
        if self.gpu_id >=0 and torch.cuda.is_available():
            self.device = torch.device("cuda", self.gpu_id)
            # print("gpu")
        else:
            # print("cpu")
            self.device = torch.device("cpu")

    
    def define_network(self):
        net = YOLOv3(self.config["model"])
        net = net.to(self.device)
        return net
        
            

