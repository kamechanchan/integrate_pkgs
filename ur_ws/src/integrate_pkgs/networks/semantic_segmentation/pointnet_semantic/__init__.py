#!/usr/bin/env python3
from .pointnet_semantic import SemanticModel
from torch import nn


def create_model(opt):
    #model = nn.DataParallel(opt)
    #cudnn.benchmark = False
    model = SemanticModel(opt)
    
    return model