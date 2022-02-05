#!/usr/bin/env python3
from numpy.core.fromnumeric import size
import torch
import numpy as np
from os.path import join
from common_function.util import print_network
from common_function.util import mkdir
from POINTNET_SEMANTIC import *
from semantic_loss import Semantic_Loss
import os

import h5py

class SemanticModel:
    def __init__(self, opt):
        self.opt = opt
        self.process_swich = opt.process_swich
        self.checkpoints_dir = opt.checkpoints_dir
        self.local_checkpoints_dir=opt.local_checkpoints_dir
        self.dataset_model = self.opt.dataset_model
        self.concat_dataset_model = '+'.join(self.opt.dataset_model)
        self.arch = opt.arch
        self.checkpoints_human_swich = opt.checkpoints_human_swich
        self.dataset_mode = opt.dataset_mode
        save_dir = join(self.checkpoints_dir, self.dataset_mode, self.checkpoints_human_swich, self.arch, self.concat_dataset_model)
        self.save_dir = os.path.splitext(save_dir)[0]
        self.gpu_ids = opt.gpu_ids
        local_save_dir=join(self.local_checkpoints_dir, self.dataset_mode, self.checkpoints_human_swich, self.arch, self.concat_dataset_model)
        self.local_save_dir = os.path.splitext(local_save_dir)[0]
        # self.device = torch.device('cuda:{}'.format(self.gpu_ids[0])) if self.gpu_ids else torch.device('cpu')
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        self.is_train = self.opt.is_train
        self.instance_number_manual = opt.instance_number-1
        self.instance_number = opt.instance_number
        self.dataset_mode = opt.dataset_mode
        self.is_estimate = opt.is_estimate
        self.net = self.define_network(opt)
        self.criterion = self.define_loss().to(self.device)
        if self.is_train and not self.is_estimate:
            self.net.train(self.is_train)
            self.optimizer = torch.optim.Adam(self.net.parameters(), lr=self.opt.lr)
            print_network(self.net)
        elif not self.is_train and not self.is_estimate:
            self.load_network()
        else:
            self.load_network_estimator()

    def define_network(self, opt):
        net = PointNet_Semantic_Segmentation(opt.semantic_number)
        net = net.to(self.device)
        return net


    def define_loss(self):
        loss = Semantic_Loss()
        return loss

    def set_input(self, data):
        if self.opt.phase == "train":
            x_data = torch.from_numpy(data["x_data"].astype(np.float32)) 
            y_data = torch.from_numpy(data["y_data"].astype(np.int64))
            x_data = x_data.transpose(2, 1)

            self.x_data, self.y_data = x_data.to(self.device), y_data.to(self.device)

        elif self.opt.phase == "test":
            # print("tanomuze")
            # print(data.dtype)
            x_data = torch.from_numpy(data)
            x_data = x_data.float()
            x_data = x_data.transpose(2, 1)

            self.x_data = x_data.to(self.device)

    def set_input_acc(self, data):
        x_data = torch.from_numpy(data["x_data"].astype(np.float32)) 
        y_data = torch.from_numpy(data["y_data"].astype(np.int64))
        x_data = x_data.transpose(2, 1)

        self.x_data, self.y_data = x_data.to(self.device), y_data.to(self.device)

    def set_input_segmentation(self, data):
        x_data = data["x_data"]
        y_data = data["y_data"]
        # sizes = data["sizes"]
        if self.opt.phase == "train":
            x_data = torch.from_numpy(x_data.astype(np.float32))
            y_data = torch.from_numpy(y_data.astype(np.float32))
            # sizes = torch.from_numpy(sizes.astype(np.int32))
            x_data = x_data.transpose(2, 1)
            # self.x_data, self.y_data, self.sizes = x_data.to(self.device), y_data.to(self.device), sizes.to(self.device)
            self.x_data, self.y_data = x_data.to(self.device), y_data.to(self.device)
        elif self.opt.phase == "test":
            x_data = torch.from_numpy(x_data)
            x_data = x_data.transpose(2, 1)
            self.x_data = x_data.to(self.device)


    def train_step(self):
        self.net.train()
        self.optimizer.zero_grad()
        pred, trans_feat = self.net(self.x_data)
        # print(type(trans_feat))
        # print(trans_feat.shape)
        # print(self.y_data.shape)
        # print(pred.shape)
        self.loss = self.criterion(pred, self.y_data, trans_feat)
        self.loss.backward()
        self.optimizer.step()
        return self.loss.item()


    def val_step(self):
        self.net.eval()
        pred, trans_feat = self.net(self.x_data)
        self.loss = self.criterion(pred, self.y_data, trans_feat)
        return self.loss.item()

    def test_step(self):
        print("SemSeg_input_pcl_number:" + str(self.x_data.shape))
        pred, trans = self.net(self.x_data)
        # print("pred_")
        # print(pred.contiguous().cpu().data.max(2)[1])
        pred = pred.contiguous().cpu().data.max(2)[1].numpy()
       
        return pred


    def acc_step(self):
        pred, trans = self.net(self.x_data)
        pred = pred.contiguous().cpu().data.max(2)[1].numpy()
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
        load_path = save_filename
        net = self.net

        if isinstance(net, torch.nn.DataParallel):
            net = net.module
        print("loading the model from %s" % load_path)
        # state_dict = torch.load(load_path, map_location=str(self.device))
       
        state_dict = torch.load(load_path, self.device)
        if hasattr(state_dict, "_metadata"):
            del state_dict._metadata
        net.load_state_dict(state_dict,strict=False)
        # net.load_state_dict(state_dict["model_state_dict"])

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
            # torch.save(self.net.module.cpu().state_dict(), save_path)
            # torch.save(self.net.module.cpu().state_dict(), local_save_path)
            torch.save(self.net.cpu().state_dict(), save_path)
            torch.save(self.net.cpu().state_dict(), local_save_path)
            self.net.cuda(self.gpu_ids[0])
        else:
            torch.save(self.net.cpu().state_dict(), save_path)

    def progress_save_pcd(self, opt, epoch, index):
        pred, trans = self.net(self.x_data)
        pred = pred.contiguous().cpu().data.max(2)[1].numpy()
        self.concat_dataset_model = '+'.join(opt.dataset_model)
        import datetime
        dt_now = datetime.datetime.now()
        local_save_dir=join(self.local_checkpoints_dir, self.dataset_mode, self.concat_dataset_model)
        pcd_dir = os.path.splitext(local_save_dir)[0]
        jikan = str(dt_now.day) + "_" + str(dt_now.hour) + "_" + str(dt_now.minute) + "_" + str(dt_now.second)
        pcd_dir = join(pcd_dir, jikan)
        mkdir(pcd_dir)
        result_h5 = h5py.File(pcd_dir + "/result" + str(epoch) + ".hdf5", "a")
        data = result_h5.create_group("data_" + str(index))
        data.create_dataset("Points", data=self.x_data, compression="lzf")
        data.create_dataset("est", data=pred, compression="lzf")
        data.create_dataset("ground_truth", data=self.y_data, compression="lzf")
        result_h5.flush()
        return pred