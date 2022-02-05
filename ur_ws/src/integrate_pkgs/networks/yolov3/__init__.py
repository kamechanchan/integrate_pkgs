#!/usr/bin/env python3
from .yolov3 import Yolo_box
from torch import nn


def create_model(opt):
    #model = nn.DataParallel(opt)
    #cudnn.benchmark = False
    model = Yolo_box(opt)
    
    return model