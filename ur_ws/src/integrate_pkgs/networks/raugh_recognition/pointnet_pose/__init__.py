#!/usr/bin/env python3
from .pose_estimate import EstimatorModel
from torch import nn


def create_model(opt):
    #model = nn.DataParallel(opt)
    #cudnn.benchmark = False
    model = EstimatorModel(opt)
    
    return model