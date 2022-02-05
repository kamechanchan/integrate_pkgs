#!/usr/bin/env bash

python3 /home/ericlab/ros_package/integrate_ws/src/integrate_pkgs/networks/raugh_recognition/pointnet_pose/train.py \
--dataroot /home/ericlab/Downloads \
--dataset_model HV8_size_20000_range_pi_4.hdf5 \
--checkpoints_dir /home/ericlab/Documents/checkpoints \
--resolution 1024 \
--phase train \
--dataset_mode pose_estimation \
--batch_size 8 \
--num_epoch 200 \
--max_dataset_size 20000 \
--arch PointNet_Pose \
--print_freq 10 \
--save_latest_freq 20000 \
--save_epoch_freq 5 \
--run_test_freq 1 \
--gpu_ids 0 \
--gpu_num 1 \
--num_threads 0 \
--serial_batches False \
--verbose_plot True \
--lr 0.0001 \
--checkpoints_human_swich tsuchida_raugh \
--dataroot_swich tsuchida \
--local_checkpoints_dir /home/ericlab/DENSO_results/raugh_recognition/checkpoint \
--tensorboardX_results_directory /home/ericlab/ros_package/integrate_ws/src/networks/raugh_recognition/pointnet_pose/tensorboardX \
--tensorboardX_results_directory_switch tsuchida_raugh/0628 \