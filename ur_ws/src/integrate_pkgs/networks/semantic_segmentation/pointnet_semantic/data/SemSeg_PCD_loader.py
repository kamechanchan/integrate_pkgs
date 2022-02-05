from __future__ import print_function
from numpy.core.fromnumeric import size
from tqdm import tqdm
import h5py
import os
import sys
sys.path.append(os.path.join(os.path.dirname(__file__), '../../utils'))
import numpy as np
from base_loader import Base_Loader
import h5py
from common_function.cloud_util import *


class SemSeg_PCD_Loader(Base_Loader):
    def __init__(self, dir_name, dataset_model, dataset_size, dataset_number, opt):
        super(SemSeg_PCD_Loader, self).__init__(dir_name, dataset_model, dataset_size, dataset_number)
        self.instance_number = opt.instance_number
        self.dataset_mode = opt.dataset_mode
        # self.dataset_model = dataset_model
        self.concat_dataset_model = '+'.join(dataset_model)
        
    def load_hdf5(self):
        for i in range(self.dataset_number):
            path = self.find_h5py_filenames(self.dir)[i] #get file_name
            dir_path = self.dir+"/"+path #get path
            # print("*******************start*********************")
            # print(dir_pathassee)
            self.hdf5_file = h5py.File(dir_path, "r")

            print("Start loading datasets !!")
            for n in tqdm(range(0, self.dataset_size[i])):
                pcl_data = self.hdf5_file["data_" + str(n + 1)]['Points'][()]
                mask_data = self.hdf5_file["data_" + str(n + 1)]['masks'][()]
                prepare_data = np.hstack([pcl_data, mask_data])
                self.x_data.append(prepare_data)

    def get_pcd_data(self, index, resolution):
        pcd_data = self.x_data[index]
        # print("pcd_data")
        # print(pcd_data)
        original_data, pcd_offset = getNormalizedPcd_seg(pcd_data, resolution)
        x_data = original_data[:,:3]
        y_data = original_data[:,3]

        # print("y_data")
        # print(type(x_data))

        # pcl_visu = pcl.PointCloud(x_data)
        # pcd_dir = "/home/ericlab/DENSO_results/August/pcl_visu/train_input/"+self.dataset_mode+"/"+self.concat_dataset_model
        # util.mkdir(pcd_dir)
        # pcl.save(pcl_visu, pcd_dir+"/result"+str(index)+".pcd")

        cnt_a = 0
        cnt_b = 0
        for i in range(8192):
            if y_data[i] == 0:
                cnt_a+=1
            elif y_data[i] == 1:
                cnt_b+=1
        # print("0:"+str(cnt_a))
        # print("1:"+str(cnt_b))

        return x_data, y_data