#!/usr/bin/enc python3
import torch
import torch.nn as nn
import torch.optim as optim

import torch.nn.functional as F
from collections import defaultdict
import yaml
from torch.autograd import Variable
import numpy as np


class PointNet_global_feat(nn.Module):
    def __init__(self, num_points=1024):
        super(PointNet_global_feat, self).__init__()

        self.num_points = num_points
        
        self.conv1 = torch.nn.Conv1d(3, 128, 1)
        self.conv2 = torch.nn.Conv1d(128, 256, 1)
        self.conv3 = torch.nn.Conv1d(256, self.num_points, 1)
        # self.max_pool = nn.MaxPool1d(self.num_points)

        self.bn1 = nn.BatchNorm1d(128)
        self.bn2 = nn.BatchNorm1d(256)
        self.bn3 = nn.BatchNorm1d(1024)

    def forward(self, x):
        x = F.relu(self.bn1(self.conv1(x)))
        x = F.relu(self.bn2(self.conv2(x)))
        x = F.relu(self.bn3(self.conv3(x)))
        # print("net:")
        # print(x.shape)
        x = torch.max(x, 2, keepdim=True)[0] #getting max data of each ch  (※:[0] is role for getting maxed data([1] is index))
        # print(x.shape)
        # x = self.max_pool(x)
        x = x.view(-1, 1024)

        return x

class PointNet_Pose(nn.Module):
    def __init__(self, out_num_pos, out_num_rot):
        super(PointNet_Pose, self).__init__()
        self.out_num_pos = out_num_pos
        self.out_num_rot = out_num_rot
        # print("num_pos")
        # print(self.out_num_pos)
        # print(self.out_num_rot)

        self.pointnet_vanila_global_feat = PointNet_global_feat(num_points=1024)
        self.fc_pos1 = nn.Linear(1024, 512)
        self.fc_pos2 = nn.Linear(512, 256)
        self.fc_pos3 = nn.Linear(256, 128)
        self.fc_pos = nn.Linear(128, self.out_num_pos)
        self.fc_rot1 = nn.Linear(1024, 512)
        self.fc_rot2 = nn.Linear(512, 256)
        self.fc_rot3 = nn.Linear(256, 128)
        self.fc_rot = nn.Linear(128, self.out_num_rot)

        self.bn1 = nn.BatchNorm1d(512)
        self.bn2 = nn.BatchNorm1d(256)
        self.bn3 = nn.BatchNorm1d(128)
        self.relu = nn.ReLU()
        self.tanh = nn.Tanh()
        self.dropout = nn.Dropout(0.3)

    def forward(self, normalized_points):
        # print("forword")
        # print(normalized_points.shape)
        global_feat = self.pointnet_vanila_global_feat(normalized_points)
        
        h1 = self.relu(self.fc_pos1(global_feat))
        h1 = self.relu(self.fc_pos2(h1))
        h1 = self.relu(self.fc_pos3(h1))
        h1 = self.fc_pos(h1)
        # print("h1")
        # print(h1.shape)

        h2 = self.relu(self.fc_rot1(global_feat))
        h2 = self.relu(self.fc_rot2(h2))
        h2 = self.relu(self.fc_rot3(h2))
        h2 = self.fc_rot(h2)
        # print("h1")
        # print(h2.shape)

        y = torch.cat([h1, h2], axis=1)
        return y


class PointNet_temmat(nn.Module):
    def __init__(self, class_num):
        super(PointNet_temmat, self).__init__()
        self.class_num = class_num
        # print("num_pos")
        # print(self.out_num_pos)
        # print(self.out_num_rot)

        self.pointnet_vanila_global_feat = PointNetEncoder()
        self.fc1 = nn.Linear(1024, 512)
        self.fc2 = nn.Linear(512, 256)
        self.fc3 = nn.Linear(256, 128)
        self.fc_out = nn.Linear(128, self.class_num)

        self.bn1 = nn.BatchNorm1d(512)
        self.bn2 = nn.BatchNorm1d(256)
        self.bn3 = nn.BatchNorm1d(128)
        self.relu = nn.ReLU()
        self.tanh = nn.Tanh()
        self.dropout = nn.Dropout(0.3)

    def forward(self, normalized_points):
        # print("forword")
        # print(normalized_points.shape)
        global_feat, trans, trans_feat = self.pointnet_vanila_global_feat(normalized_points)
        
        h1 = self.relu(self.fc1(global_feat))
        h1 = self.relu(self.fc2(h1))
        h1 = self.relu(self.fc3(h1))
        h1 = self.fc_out(h1)
        x = F.log_softmax(h1, dim=1)
        # x = F.softmax(h1, dim=1)
        # print("h1")
        # print(h1.shape)

        # y = torch.cat([h1, h2], axis=1)
        return x, trans_feat


class PointNetEncoder(nn.Module):
    def __init__(self, global_feat=True, feature_transform=True, channel=3):
        super(PointNetEncoder, self).__init__()
        self.stn = STN3d(channel)
        self.conv1 = torch.nn.Conv1d(channel, 64, 1)
        self.conv2 = torch.nn.Conv1d(64, 128, 1)
        self.conv3 = torch.nn.Conv1d(128, 1024, 1)
        self.bn1 = nn.BatchNorm1d(64)
        self.bn2 = nn.BatchNorm1d(128)
        self.bn3 = nn.BatchNorm1d(1024)
        self.global_feat = global_feat
        self.feature_transform = feature_transform
        if self.feature_transform:
            self.fstn = STNkd(k=64)

    def forward(self, x):
        B, D, N = x.size()
        trans = self.stn(x)
        x = x.transpose(2, 1)
        if D > 3:
            feature = x[:, :, 3:]
            x = x[:, :, :3]
        x = torch.bmm(x, trans)
        if D > 3:
            x = torch.cat([x, feature], dim=2)
        x = x.transpose(2, 1)
        # x = F.relu(self.bn1(self.conv1(x)))
        x = F.relu(self.conv1(x))

        if self.feature_transform:
            trans_feat = self.fstn(x)
            x = x.transpose(2, 1)
            x = torch.bmm(x, trans_feat)
            x = x.transpose(2, 1)
        else:
            trans_feat = None

        pointfeat = x
        # x = F.relu(self.bn2(self.conv2(x)))
        x = F.relu(self.conv2(x))
        # x = self.bn3(self.conv3(x))
        x = self.conv3(x)
        x = torch.max(x, 2, keepdim=True)[0]
        x = x.view(-1, 1024)
        if self.global_feat:
            return x, trans, trans_feat
        else:
            x = x.view(-1, 1024, 1).repeat(1, 1, N)
            return torch.cat([x, pointfeat], 1), trans, trans_feat


class STN3d(nn.Module):
    def __init__(self, channel):
        super(STN3d, self).__init__()
        self.conv1 = torch.nn.Conv1d(channel, 64, 1)
        self.conv2 = torch.nn.Conv1d(64, 128, 1)
        self.conv3 = torch.nn.Conv1d(128, 1024, 1)
        self.fc1 = nn.Linear(1024, 512)
        self.fc2 = nn.Linear(512, 256)
        self.fc3 = nn.Linear(256, 9)
        self.relu = nn.ReLU()

        self.bn1 = nn.BatchNorm1d(64)
        self.bn2 = nn.BatchNorm1d(128)
        self.bn3 = nn.BatchNorm1d(1024)
        self.bn4 = nn.BatchNorm1d(512)
        self.bn5 = nn.BatchNorm1d(256)

    def forward(self, x):
        batchsize = x.size()[0]
        # x = F.relu(self.bn1(self.conv1(x)))
        # x = F.relu(self.bn2(self.conv2(x)))
        # x = F.relu(self.bn3(self.conv3(x)))
        x = F.relu(self.conv1(x))
        x = F.relu(self.conv2(x))
        x = F.relu(self.conv3(x))
        x = torch.max(x, 2, keepdim=True)[0]
        x = x.view(-1, 1024)

        # x = F.relu(self.bn4(self.fc1(x)))
        # x = F.relu(self.bn5(self.fc2(x)))
        x = F.relu(self.fc1(x))
        x = F.relu(self.fc2(x))
        x = self.fc3(x)

        iden = Variable(torch.from_numpy(np.array([1, 0, 0, 0, 1, 0, 0, 0, 1]).astype(np.float32))).view(1, 9).repeat(
            batchsize, 1)
        if x.is_cuda:
            iden = iden.cuda()
        x = x + iden
        x = x.view(-1, 3, 3)
        return x


class STNkd(nn.Module):
    def __init__(self, k=64):
        super(STNkd, self).__init__()
        self.conv1 = torch.nn.Conv1d(k, 64, 1)
        self.conv2 = torch.nn.Conv1d(64, 128, 1)
        self.conv3 = torch.nn.Conv1d(128, 1024, 1)
        self.fc1 = nn.Linear(1024, 512)
        self.fc2 = nn.Linear(512, 256)
        self.fc3 = nn.Linear(256, k * k)
        self.relu = nn.ReLU()

        self.bn1 = nn.BatchNorm1d(64)
        self.bn2 = nn.BatchNorm1d(128)
        self.bn3 = nn.BatchNorm1d(1024)
        self.bn4 = nn.BatchNorm1d(512)
        self.bn5 = nn.BatchNorm1d(256)

        self.k = k

    def forward(self, x):
        batchsize = x.size()[0]
        # x = F.relu(self.bn1(self.conv1(x)))
        # x = F.relu(self.bn2(self.conv2(x)))
        # x = F.relu(self.bn3(self.conv3(x)))
        x = F.relu(self.conv1(x))
        x = F.relu(self.conv2(x))
        x = F.relu(self.conv3(x))
        x = torch.max(x, 2, keepdim=True)[0]
        x = x.view(-1, 1024)

        # x = F.relu(self.bn4(self.fc1(x)))
        # x = F.relu(self.bn5(self.fc2(x)))
        x = F.relu(self.fc1(x))
        x = F.relu(self.fc2(x))
        x = self.fc3(x)

        iden = Variable(torch.from_numpy(np.eye(self.k).flatten().astype(np.float32))).view(1, self.k * self.k).repeat(
            batchsize, 1)
        if x.is_cuda:
            iden = iden.cuda()
        x = x + iden
        x = x.view(-1, self.k, self.k)
        return x