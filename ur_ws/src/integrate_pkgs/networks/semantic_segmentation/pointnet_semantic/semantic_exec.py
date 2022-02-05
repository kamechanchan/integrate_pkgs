#!/usr/bin/env python3
import sys
import os
import numpy as np
import time
from denso_msgs.msg import out_segmentation
from semantic_segmentation.pointnet_semantic import create_model

def estimation(model, data):
    time_sta = time.time()
    # print(data.shape)
    model.set_input(data)
    pred = model.test_step()
    time_end = time.time()
    return pred, (time_end - time_sta)

def pose_prediction(opt, data, resolution):
    # print(type(data))
    n_data = len(data)
    row = 3
    col = n_data // row
    x = data[np.newaxis, :, :]
    y_pre = estimation(opt, x)
    y = np.squeeze(y_pre[0])
   
    est_time = y_pre[1]
    msg_out = out_segmentation()
    raugh_data = []
    for i in range(resolution):
        msg_out.x.append(x[0][i][0])
        msg_out.y.append(x[0][i][1])
        msg_out.z.append(x[0][i][2])
        msg_out.instance.append(y[i])
        # print(y[i])
        if y[i] == 0:
            raugh_data.append(x[0, i, :])
            # print("*************")
    raugh_data = np.array(raugh_data)
    # raugh_data = raugh_data[np.newaxis, : , :]

    return (msg_out, est_time, raugh_data)


def run_test(opt, dataset):
    
    opt.serial_batches = True
    val_loss = 0.0
    model = create_model(opt)

    for i, data in enumerate(dataset):
        time_sta = time.time()

        model.set_input(data)
        loss = model.val_step()
        time_end = time.time()

        val_loss += loss
    return val_loss

def run_progress_savetest(opt, dataset, epoch):
    opt.serial_batches = True
    val_loss = 0.0
    model = create_model(opt)

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