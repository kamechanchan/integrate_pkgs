#!/usr/bin/env python3
import torch
import torch.nn as nn

class Semantic_Loss(nn.Module):
    def __init__(self, mat_diff_loss_scale=0.001, weight=None, reduction="mean"):
        super(Semantic_Loss, self).__init__()
        self.nll = nn.NLLLoss(weight, reduction=reduction)
        self.mat_diff_loss_scale = mat_diff_loss_scale

    def forward(self, pred, target, trans_feat):
        batch_size = pred.shape[0]
        num_points = pred.shape[1]
        pred = pred.view(batch_size * num_points, -1)
        target = target.view(batch_size * num_points)
        target = target.long()
        # target = torch.tensor(target, dtype=torch.long)
        # print(target.dtype)
        loss = self.nll(pred, target)
        return loss

    def feature_transform_reguliarzer(trans):
        d = trans.size()[1]
        I = torch.eye(d)[None, :, :]
        if trans.is_cuda:
            I = I.cuda()
        loss = torch.mean(torch.norm(torch.bmm(trans, trans.transpose(2, 1)) - I, dim=(1, 2)))
        return loss