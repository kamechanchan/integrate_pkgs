#!/usr/bin/env python3
import sys
import os

from numpy.core.fromnumeric import mean
sys.path.append(os.path.join(os.path.dirname(__file__), '../../../trainer'))
sys.path.append(os.path.join(os.path.dirname(__file__), '../../../utils'))
import time
import numpy as np
from scipy.spatial.transform import Rotation
from geometry_msgs.msg import PoseStamped
# from raugh_recognition.pointnet_pose import create_model
from pcl_template_matting.pointnet_temmat import create_model_temmat


def estimation(model, data):
    time_sta = time.time()
    # print("estimation")
    # print(model)
    # print(data.shape)
    model.set_input(data)
    pred_choice, pred_confi  = model.test_step()
    time_end = time.time()
    # return pred_choice, pred_confi (time_end - time_sta)
    return pred_choice, pred_confi

    
def pose_prediction(opt, data, arg):
    n_data = len(data)
    row = 3
    col = n_data // row
    x = data[np.newaxis, :, :]
    y_pre, pred_confi = estimation(opt, x)
    # pred_choice = y_pre.data.max(1)[1]
    return y_pre, pred_confi

def run_test(opt, dataset):
    opt.serial_batches = True
    val_loss = 0.0
    model = create_model_temmat(opt)
    mean_correct = []
    for i, data in enumerate(dataset):
        time_sta = time.time()

        model.set_input(data)
        loss, correct  = model.val_step()
        mean_correct.append(correct)
        time_end = time.time()

        val_loss += loss
    return val_loss, mean_correct

def run_progress_savetest(opt, dataset, epoch):
    opt.serial_batches = True
    val_loss = 0.0
    model = create_model_temmat(opt)

    for i, data in enumerate(dataset):
        time_sta = time.time()

        model.set_input(data)
        model.progress_save_pcd(opt, epoch, i)
        time_end = time.time()

    return

def run_segmentation_test(opt, dataset):

    opt.serial_batches = True
    val_loss = 0.0
    model = create_model(opt)

    for i, data in enumerate(dataset):
        time_sta = time.time()

        model.set_input_segmentation(data)
        loss = model.val_step()
        time_end = time.time()

        val_loss += loss
    return val_loss