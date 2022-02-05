#! /usr/bin/env python3
# -*- coding: utf-8 -*-
import sys, os
sys.path.append(os.path.join(os.path.dirname(__file__), '../../'))
sys.path.append(os.path.join(os.path.dirname(__file__), '../'))
sys.path.append(os.path.join(os.path.dirname(__file__), './'))
sys.path.append(os.path.join(os.path.dirname(__file__), './data'))
sys.path.append(os.path.join(os.path.dirname(__file__), './options'))
import numpy as np
from tqdm import tqdm
import torch, numpy, random

from options.train_options import TrainOptions_semanctic_segmentation
from options.test_options import TestOptions_semantic_segmentation
from data import *
from semantic_segmentation.pointnet_semantic import create_model
from semantic_exec import *
from pointnet_semantic import *
from common_function.writer import Writer
import matplotlib.pyplot as plt

if __name__ == '__main__':
    print("------------------current main directory------------------")
    print(__file__)

    opt = TrainOptions_semanctic_segmentation().train_parse()
    opt_v = TestOptions_semantic_segmentation().test_parse()
    print(opt.arch)
    train_dataset, val_dataset = TrainValDataset(opt)
    train_dataset = TrainDataLoader(train_dataset, opt)
    val_dataset = ValDataLoader(val_dataset, opt)

    train_dataset_size = len(train_dataset)
    val_dataset_size = len(val_dataset)

    print("#training data = %d" % train_dataset_size)
    print("#val data = %d" % val_dataset_size)

    model = create_model(opt)
    writer = Writer(opt)
    total_steps = 0
    count = 0
    loss_plot_y = []
    plot_x = []
   
    for epoch in range(opt.epoch_count, opt.num_epoch + 1):
        epoch_start_time = time.time()
        iter_data_time = time.time()
        epoch_iter = 0
        train_loss = 0.0

        for i, data in enumerate(train_dataset):
            iter_start_time = time.time()
            if total_steps % opt.print_freq == 0:
                t_data = iter_start_time - iter_data_time

            total_steps += opt.batch_size * opt.gpu_num
            epoch_iter += opt.batch_size * opt.gpu_num

            # print("***data****")
            # print(data)
            model.set_input_segmentation(data)
            t_loss = model.train_step()
            train_loss += t_loss
            loss_plot_y.append(t_loss)
            count = count + 1
            plot_x.append(count)
            # print("********************GOAL*************************")
            # break

            if total_steps % opt.print_freq == 0:
                t = (time.time() - iter_start_time / opt.batch_size)
                writer.print_current_losses("train", epoch, epoch_iter, t_loss, t, t_data)

            if i % opt.save_latest_freq == 0:
                print("saving the latest model (epoch %d, total_steps %d)" % (epoch, total_steps))
                model.save_network("latest")

            iter_data_time = time.time()
        # break
        if epoch % opt.save_epoch_freq == 0:
            print("saving the model at the end of epoch %d, iter %d" % (epoch, total_steps))
            model.save_network("latest")
            model.save_network(epoch)


        if epoch % opt.run_test_freq == 0:
            val_loss = run_segmentation_test(opt_v, val_dataset)
            writer.print_current_losses("val", epoch, epoch_iter, t_loss, t, t_data)

        if epoch % opt.run_test_freq == 0:
            train_loss /= train_dataset_size
            val_loss /= val_dataset_size
            print("epoch: {}, train_loss: {:.3}" ", val_loss: {:.3}".format(epoch, train_loss, val_loss))
            writer.plot_loss(epoch, train_loss, val_loss)
        writer.close()
    plt.plot(plot_x, loss_plot_y)
    plt.grid()
    plot_file = opt.checkpoints_dir + "/" + opt.dataset_mode + opt.checkpoints_human_swich + "/" + opt.arch + "/" + opt.dataset_model + "/loss_plot.png"
    plt.savefig(plot_file)
    print(plot_file)
    

