#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import os
import sys
sys.path.append(os.path.join(os.path.dirname(__file__), '../../utils'))
import numpy as np
from tqdm import tqdm
from base_loader import Base_Loader
from common_function.cloud_util import *
import time
import h5py


class PCD_Loader(Base_Loader):
    def __init__(self, dir_name, dataset_model, dataset_size, dataset_number, opt):
        super(PCD_Loader, self).__init__(dir_name, dataset_model, dataset_size, dataset_number)
        self.dataset_mode = opt.dataset_mode
        self.concat_dataset_model = '+'.join(dataset_model)

    def load_hdf5(self):
        for i in range(self.dataset_number):
            path = self.find_h5py_filenames(self.dir)[i] #get file_name
            dir_path = self.dir+"/"+path #get path
            self.hdf5_file = h5py.File(dir_path, "r")

            print("Start loading datasets !!")
            for n in tqdm(range(0, self.dataset_size[i])):
                pcl_data = self.hdf5_file["data_" + str(n + 1)]['pcl'][()]
                pose_data = self.hdf5_file["data_" + str(n + 1)]['pose'][()]
                pose_data = self.conv_quat2mat(pose_data)
                self.x_data.append(pcl_data)
                self.y_data.append(pose_data)

    def get_pcd_data(self, index, resolution):
        pcd_data = self.x_data[index]
        
        x_data, pcd_offset = getNormalizedPcd(pcd_data, resolution)
        y_data = self.y_data[index]
        y_pos = y_data[0:3] - pcd_offset[0]
        y_rot = y_data[3:]
        y_data = np.concatenate([y_pos, y_rot])

        return x_data, y_data

    def get_voxel_data(self, index, resolution):
        channel = 1
        pcd_data = self.x_data[index]
        pose_data = self.y_data[index]
        min_p, max_p, diff_max = doMinMax(pcd_data)
        voxel = np.zeros((resolution, resolution, resolution), dtype="float32")
        binary_data = runMakeVoxelBinary(pcd_data, resolution)
        voxel = binary_data.reshape(resolution, resolution, resolution)
        voxel = voxel[np.newaxis, :]

        norm_pose_data = np.zeros((12), dtype="float32")
        norm_pose_data[0] = (pose_data[0] - min_p[0]) / diff_max
        norm_pose_data[1] = (pose_data[1] - min_p[1]) / diff_max
        norm_pose_data[2] = (pose_data[2] - min_p[2]) / diff_max
        norm_pose_data[3] = pose_data[3]
        norm_pose_data[4] = pose_data[4]
        norm_pose_data[5] = pose_data[5]
        norm_pose_data[6] = pose_data[6]
        norm_pose_data[7] = pose_data[7]
        norm_pose_data[8] = pose_data[8]
        norm_pose_data[9] = pose_data[9]
        norm_pose_data[10] = pose_data[10]
        norm_pose_data[11] = pose_data[11]

        return voxel, norm_pose_data

    def get_voxel(self, index):
        voxel_data = self.x_data[index]
        pose_data = self.y_data[index]
        print(pose_data)
        return voxel_data, pose_data

if __name__ == "__main__":
    loader = PCD_Loader("../../datasets/", "HV7", 200)
    loader.load_hdf5()

    size = loader.dataset_size
    op_time = 0.0
    for i in range(size):
        print(i)
        s_time = time.time()
        voxel, pose = loader.get_voxel_data(i, 50)
        e_time = time.time()
        lap_time = e_time - s_time
        op_time += lap_time
