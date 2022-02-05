#!/usr/bin/env python3
from __future__ import print_function
import torch
import numpy as np
import os


def mkdir(path):
    if not os.path.exists(path):
        os.makedirs(path)


def print_network(net):
    print("============ Network initialized ===========")
    num_params = 0
    for param in net.parameters():
        num_params += param.numel()
    print('[Network] Total numpber of parameters : %.3f M' % (num_params / 1e6))
    print("============================================")

def batch_quat_to_rotmat(q, out=None):
    
    batchsize = q.size(0)

    if out is None:
        out = torch.FloatTensor(batchsize, 3, 3)

    # 2 / squared quaternion 2-norm
    s = 2/torch.sum(q.pow(2), 1)

    # coefficients of the Hamilton product of the quaternion with itself
    h = torch.bmm(q.unsqueeze(2), q.unsqueeze(1))

    out[:, 0, 0] = 1 - (h[:, 2, 2] + h[:, 3, 3]).mul(s)
    out[:, 0, 1] = (h[:, 1, 2] - h[:, 3, 0]).mul(s)
    out[:, 0, 2] = (h[:, 1, 3] + h[:, 2, 0]).mul(s)

    out[:, 1, 0] = (h[:, 1, 2] + h[:, 3, 0]).mul(s)
    out[:, 1, 1] = 1 - (h[:, 1, 1] + h[:, 3, 3]).mul(s)
    out[:, 1, 2] = (h[:, 2, 3] - h[:, 1, 0]).mul(s)

    out[:, 2, 0] = (h[:, 1, 3] - h[:, 2, 0]).mul(s)
    out[:, 2, 1] = (h[:, 2, 3] + h[:, 1, 0]).mul(s)
    out[:, 2, 2] = 1 - (h[:, 1, 1] + h[:, 2, 2]).mul(s)

    return out
