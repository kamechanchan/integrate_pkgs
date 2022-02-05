#!/usr/bin/enc python3
import torch
import torch.nn as nn
import torch.optim as optim

import torch.nn.functional as F
from collections import defaultdict
import yaml

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



