#!/usr/bin/env python
# -*- coding: utf-8 -*-
from sys import pycache_prefix
import torch
import torch.utils.data as data
import numpy as np
import pickle
import os
class BaseDataset(data.Dataset):
    def __init__(self, opt):
        super(BaseDataset, self).__init__()
        self.opt = opt
        self.mean = 0
        self.std = 1
        self.ninput_channels = None
        self.arch = opt.arch
        #self.root = opt.dataroot
        self.dataset_model = opt.dataset_model
        self.dir = opt.dataroot
        self.dataroot_swich=opt.dataroot_swich
        # self.dataroot=os.path.join(self.dir,self.dataroot_swich)
        self.dataroot=os.path.join(self.dir)
        self.resolution = opt.resolution
        self.size = opt.max_dataset_size
        self.len_size = 0
        self.dataset_number = opt.dataset_number
        self.hdf5_data = None
        for i in range(self.dataset_number):
            self.len_size = self.len_size + self.size[i]


def collate_fn(batch):
    meta = {}
    keys = batch[0].keys()
    for key in keys:
        meta.update({key:np.array([d[key] for d in batch])})

    return meta

def collate_fn_original(batch):
    meta_x = []
    meta_y = []
    for sample in batch:
        meta_x.append(sample[0])
        meta_y.append(torch.FloatTensor(sample[1]))
    meta_x = np.array(meta_x)
    meta_x = torch.from_numpy(meta_x.astype(np.float32))
    return meta_x, meta_y
