#!/usr/bin/env python3
import sys
import os
sys.path.append(os.path.join(os.path.dirname(__file__), '../../../trainer'))
sys.path.append(os.path.join(os.path.dirname(__file__), '../../../utils'))
import time
import numpy as np
from scipy.spatial.transform import Rotation
from geometry_msgs.msg import PoseStamped
from raugh_recognition.pointnet_pose import create_model


def estimation(model, data):
    time_sta = time.time()
    # print("estimation")
    # print(model)
    # print(data.shape)
    model.set_input(data)
    pred = model.test_step()
    time_end = time.time()
    return pred, (time_end - time_sta)

    
def pose_prediction(opt, data, arg):
    if arg == "PointNet":
        n_data = len(data)
        row = 3
        col = n_data // row
        x = np.reshape(np.array(data), (col, row))[np.newaxis, :, :]
    elif arg == "integ_final_PointNet":
        # x = getNormalizedPcd(data, 1024)
        x = data[np.newaxis, :, :]
    # for i in range(20):
    #     print("x: " + str(x[0][0]) +  "  z: " + str(x[0][2]))
    y_pre = estimation(opt, x)
    
    
    # print(y_pre.shape)
    for pre in y_pre:
        # print("****")
        # print(pre.shape)
        y = pre[0]
        # for i in range(4):
        #         print(str(pre[0][0]) +  " " + str(pre[0][1]) +  " " + str(pre[0][2]) +  " " + str(pre[0][3]) +  " " + str(pre[0][4]))
        #         print(str(pre[0][6]) +  " " + str(pre[0][7]) +  " " + str(pre[0][8]) +  " " + str(pre[0][9]) +  " " + str(pre[0][10]) +  " " + str(pre[0][11]))
        #         print("")
        break
        
    # y = np.squeeze(y_pre[0])
    
    # for i in range(20):
    #     print("x: " + str(y[0]) +  "  z: " + str(y[2]))
    est_time = y_pre[1]
    y_pos = y[0:3]
    rot = Rotation.from_matrix(y[3:12].reshape(3, 3))
    y_euler = rot.as_quat()
    y = np.r_[y_pos, y_euler]
    

    est_pose = PoseStamped()
    est_pose.pose.position.x = y[0]
    est_pose.pose.position.y = y[1]
    est_pose.pose.position.z = y[2]
    est_pose.pose.orientation.x = y[3]
    est_pose.pose.orientation.y = y[4]
    est_pose.pose.orientation.z = y[5]
    est_pose.pose.orientation.w = y[6]

    return (est_pose, est_time)


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