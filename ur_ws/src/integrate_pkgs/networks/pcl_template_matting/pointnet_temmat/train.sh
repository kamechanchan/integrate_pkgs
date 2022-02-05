#!/usr/bin/env bash

python3 /home/ericlab/ros_package/integrate_ws/src/integrate_pkgs/networks/pcl_template_matting/pointnet_temmat/train.py \
--dataroot /home/ericlab/hdf5_data/temmat \
--dataset_model for_temmat_raugh_changed_HV8_size_5000_range_pi_2.hdf5 for_temmat_occlution_changed_occulution_kiriwake_11_18_4987_1.hdf5 \
--checkpoints_dir /home/ericlab/DENSO_results/August \
--resolution 1024 \
--phase train \
--dataset_mode temmat \
--batch_size 8 \
--num_epoch 200 \
--max_dataset_size 5000 4987 \
--arch PointNet_Pose \
--print_freq 10 \
--save_latest_freq 500 \
--save_epoch_freq 5 \
--run_test_freq 1 \
--gpu_ids 0 \
--gpu_num 1 \
--num_threads 0 \
--serial_batches False \
--verbose_plot True \
--lr 0.0001 \
--checkpoints_human_swich ishiyama \
--dataroot_swich ishiyama \
--local_checkpoints_dir /home/ericlab/DENSO_results/August/temmat/checkpoint \
--tensorboardX_results_directory /home/ericlab/ros_package/integrate_ws/src/networks/raugh_recognition/pointnet_pose/tensorboardX \
--tensorboardX_results_directory_switch ishiyama_temmat/1123 \