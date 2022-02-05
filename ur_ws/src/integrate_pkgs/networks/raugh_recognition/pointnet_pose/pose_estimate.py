#!/usr/bin/env python3
from numpy.core.fromnumeric import size
import torch
import numpy as np
from os.path import join
from common_function.util import print_network
from common_function.util import mkdir
import h5py
from POINTNET import *
import os, sys
class EstimatorModel:
    def __init__(self, opt):
        self.opt = opt
        self.process_swich = opt.process_swich
        self.checkpoints_dir = opt.checkpoints_dir
        self.local_checkpoints_dir=opt.local_checkpoints_dir
        self.dataset_model = self.opt.dataset_model
        self.concat_dataset_model = '+'.join(self.opt.dataset_model)
        self.arch = opt.arch
        self.checkpoints_human_swich = opt.checkpoints_human_swich
        # self.checkpoints_process_swich = opt.checkpoints_process_swich
        self.dataset_mode = opt.dataset_mode
        save_dir = join(self.checkpoints_dir, self.dataset_mode, self.checkpoints_human_swich, self.arch, self.concat_dataset_model)
        self.save_dir = os.path.splitext(save_dir)[0]
        local_save_dir=join(self.local_checkpoints_dir, self.dataset_mode, self.checkpoints_human_swich, self.arch, self.concat_dataset_model)
        self.local_save_dir = os.path.splitext(local_save_dir)[0]
        self.gpu_ids = opt.gpu_ids
        # self.device = torch.device('cuda:{}'.format(self.gpu_ids[0])) if self.gpu_ids else torch.device('cpu')
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        self.is_train = self.opt.is_train
        self.instance_number_manual = opt.instance_number-1
        self.instance_number = opt.instance_number
        self.dataset_mode = opt.dataset_mode
        self.is_estimate = opt.is_estimate
        self.net = self.define_network()
        self.criterion = self.define_loss().to(self.device)
        if self.is_train and not self.is_estimate:
            self.net.train(self.is_train)
            self.optimizer = torch.optim.Adam(self.net.parameters(), lr=self.opt.lr)
            print_network(self.net)
        elif not self.is_train and not self.is_estimate:
            self.load_network()
        else:
            self.load_network_estimator()
        
    

    def define_network(self):
        net = PointNet_Pose(3, 9)
        net = net.to(self.device)
        return net


    def define_loss(self):
        loss = nn.MSELoss()
        return loss
    
    def get_centroid(self, data):
        x_mean = torch.mean(data, axis=1)
        x_mean = x_mean.unsqueeze(1)
        self.position_offset = x_mean
        data = data - x_mean
        return data


    def set_input(self, data):
        if self.opt.phase == "train":
            x_data = torch.from_numpy(data["x_data"].astype(np.float32)) 
            y_data = torch.from_numpy(data["y_data"].astype(np.float32))
            x_data = x_data.transpose(2, 1)
            self.x_data, self.y_data = x_data.to(self.device), y_data.to(self.device)

        elif self.opt.phase == "test":
            x_data = torch.from_numpy(data)
            x_data = x_data.float()
            #x_data = self.get_centroid(x_data)
            x_data = x_data.transpose(2, 1)
            self.x_data = x_data.to(self.device)

    def set_input_acc(self, data):
        x_data = torch.from_numpy(data["x_data"].astype(np.float32)) 
        if self.dataset_mode == "pose_estimation":
            y_data = torch.from_numpy(data["y_data"].astype(np.float32))
            x_data = x_data.transpose(2, 1)
        self.x_data, self.y_data = x_data.to(self.device), y_data.to(self.device)

    def set_input_segmentation(self, data):
        x_data = data["x_data"]
        y_data = data["y_data"]
        sizes = data["sizes"]
        if self.opt.phase == "train":
            # x_data = np.array(meta_x)
            # y_data = np.array(meta_y)
            x_data = torch.from_numpy(x_data.astype(np.float32))
            y_data = torch.from_numpy(y_data.astype(np.float32))
            sizes = torch.from_numpy(sizes.astype(np.int32))
            x_data = x_data.transpose(2, 1)
            self.x_data, self.y_data, self.sizes = x_data.to(self.device), y_data.to(self.device), sizes.to(self.device)
        elif self.opt.phase == "test":
            # x_data = np.array(meta_x)
            x_data = torch.from_numpy(x_data)
            x_data = x_data.transpose(2, 1)
            self.x_data = x_data.to(self.device)


    def train_step(self):
        self.net.train()
        self.optimizer.zero_grad()
        # pred = self.net(self.x_data)
        pred = self.net(self.x_data)
        self.loss = self.criterion(pred, self.y_data)
        self.loss.backward()
        self.optimizer.step()
        # return self.loss.item() * self.x_data.size(0)
        return self.loss.item()


    def val_step(self):
        self.net.eval()
        if self.process_swich == "raugh_recognition":
            pred = self.net(self.x_data)
            self.loss = self.criterion(pred, self.y_data)
        # return self.loss.item() * self.x_data.size(0)
        return self.loss.item()


    def test_step(self):
        # print(self.x_data.shape)
        # for i in range(4):
        #     print(str(self.x_data[0][0][i]) +  " " + str(self.x_data[0][1][i]) +  " " + str(self.x_data[0][2][i]))
        #     print("")
        print("raugh_recognition_input_pcl_number:" + str(self.x_data.shape))
        pred = self.net(self.x_data)

        pred = pred.to('cpu').detach().numpy().copy()
        # print("****")
        # print(type(pred))
        # print(pred.shape)
        # for i in range(4):
        #     print(str(pred[0][0]) +  " " + str(pred[0][1]) +  " " + str(pred[0][2]) +  " " + str(pred[0][3]) +  " " + str(pred[0][5]))
        #     print(str(pred[0][6]) +  " " + str(pred[0][7]) +  " " + str(pred[0][8]) +  " " + str(pred[0][9]) +  " " + str(pred[0][10]) +  " " + str(pred[0][11]))
        #     print("")
        return pred


    def acc_step(self):
        pred = self.net(self.x_data)
        # print("p")
        # print(pred.shape)
        pred = pred.to('cpu').detach().numpy().copy()
        # print("pred")
        # print(pred.shape)
      
        return pred


    def load_network(self):
        save_filename = "latest_net.pth"
        load_path = join(self.save_dir, save_filename)
        net = self.net

        if isinstance(net, torch.nn.DataParallel):
            net = net.module
        
        # print("loading the model from %s" % load_path)
        state_dict = torch.load(load_path, self.device)
        if hasattr(state_dict, "_metadata"):
            del state_dict._metadata
        net.load_state_dict(state_dict,strict=False)


    def load_network_estimator(self):
        save_filename = self.checkpoints_dir
        print(save_filename)
        load_path = save_filename
        net = self.net

        if isinstance(net, torch.nn.DataParallel):
            net = net.module
        print("loading the model from %s" % load_path)
        state_dict = torch.load(load_path, self.device)
        if hasattr(state_dict, "_metadata"):
            del state_dict._metadata
        net.load_state_dict(state_dict,strict=False)

    def save_network(self, which_epoch):
        save_filename= "%s_net.pth" % (which_epoch)
        self.save_dir = os.path.splitext(self.save_dir)[0]
        if not os.path.exists(self.save_dir):
            mkdir(self.save_dir)
        save_path = join(self.save_dir, save_filename)
        self.local_save_dir = os.path.splitext(self.local_save_dir)[0]
        if not os.path.exists(self.local_save_dir):
            mkdir(self.local_save_dir)
        local_save_path=join(self.local_save_dir,save_filename)
        
        if len(self.gpu_ids) > 0 and torch.cuda.is_available():
            torch.save(self.net.cpu().state_dict(), save_path)
            torch.save(self.net.cpu().state_dict(), local_save_path)
            self.net.cuda(self.gpu_ids[0])
        else:
            torch.save(self.net.cpu().state_dict(), save_path)

    def progress_save_pcd(self, opt, epoch, index):
        pred = self.net(self.x_data)
        pred = pred.to('cpu').detach().numpy().copy()
        self.x_data = self.x_data.to('cpu').detach().numpy().copy()
        self.y_data = self.y_data.to('cpu').detach().numpy().copy()
        self.concat_dataset_model = '+'.join(opt.dataset_model)
        import datetime
        dt_now = datetime.datetime.now()
        local_save_dir=join(self.local_checkpoints_dir, self.dataset_mode, self.concat_dataset_model)
        pcd_dir = os.path.splitext(local_save_dir)[0]
        jikan = str(dt_now.day) + "_" + str(dt_now.hour) + "_" + str(dt_now.minute) + "_" + str(dt_now.second)
        pcd_dir = join(pcd_dir, jikan)
        
        # pcd_dir = "/home/ericlab/DENSO_results/August/pcl_visu/progress_output/"+opt.dataset_mode+"/"+self.concat_dataset_model +"/"+str(epoch)
        mkdir(pcd_dir)
        result_h5 = h5py.File(pcd_dir + "/result" + str(epoch) + ".hdf5", "a")
        data = result_h5.create_group("data_" + str(index))
        data.create_dataset("Points", data=self.x_data, compression="lzf")
        data.create_dataset("est", data=pred, compression="lzf")
        data.create_dataset("ground_truth", data=self.y_data, compression="lzf")
        result_h5.flush()
       
       
        return pred