#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import os
import sys
sys.path.append(os.path.join(os.path.dirname(__file__), '../options'))
import torch
import os, csv, random, sys
import numpy as np
from tqdm import tqdm
from options.test_options import TestOptions_temmat
from base_dataset import BaseDataset
from pcd_loader import PCD_Loader

class PoseData(BaseDataset):
    def __init__(self, opt):
        BaseDataset.__init__(self, opt)
        self.resolution = opt.resolution
        self.hdf5_data = PCD_Loader(self.dataroot, self.dataset_model, self.size, self.dataset_number, opt)
        self.hdf5_data.load_hdf5()

    def __getitem__(self, index):
        meta = {}
        try:
            x_data, y_data = self.hdf5_data.get_pcd_data(index, self.resolution)
            meta["x_data"] = x_data
            meta["y_data"] = y_data
            return meta
            
        except:
            print("pose_estimate_data.py: Error! Cloud not load hdf5_data")
            sys.exit(1)


    def __len__(self):
        return self.len_size
        

if __name__ == "__main__":
    opt = TrainOptions().parse()
    loader = PoseData(opt)
