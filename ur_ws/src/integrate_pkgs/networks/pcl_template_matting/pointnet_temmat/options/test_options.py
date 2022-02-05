#!/usr/bin/env python3
import torch 
import argparse

class TestOptions_temmat():
    def __init__(self):
        pass

    def initialize(self):
        self.dataset_number = 2
        self.parser = argparse.ArgumentParser(formatter_class=argparse.ArgumentDefaultsHelpFormatter)
        self.parser.add_argument("--results_dir", type=str, default="../../weights")
        self.parser.add_argument("--phase", type=str, default="test", help="train, val, test")
        self.parser.add_argument("--which_epoch", type=str, default="latest")
        self.parser.add_argument('--main_directory',type=str,default=__file__)
        self.parser.add_argument('--dataset_mode', choices={"instance_segmentation", "pose_estimation","semantic_segmentation", "temmat"}, default='pose_estimation')
        self.parser.add_argument('--dataset_model', type=str, nargs=self.dataset_number, default='HV8')
        self.parser.add_argument('--max_dataset_size', type=int, nargs=self.dataset_number, default=float("inf"), help='Maximum num of samples per epoch')
        self.parser.add_argument('--process_swich', type=str, choices={"raugh_recognition", "object_segment"}, default="raugh_recognition")
        self.parser.add_argument('--batch_size', type=int, default=1)
        self.parser.add_argument('--num_epoch', type=int, default=150)
        self.parser.add_argument('--arch', type=str, default="PointNet_Pose")
        self.parser.add_argument('--resolution', type=int, default=1024)
        self.parser.add_argument('--gpu_ids', type=str, default='-1')
        self.parser.add_argument('--gpu_num', type=int, default=1)
        self.parser.add_argument('--num_threads', type=int, default=3)
        self.parser.add_argument('--checkpoints_dir', type=str, default="/home/ericlab/Desktop/ishiyama/Yolo_saikou/weights/yolo_simulator.pth")
        self.parser.add_argument('--serial_batches', action='store_true', help='if true, takes meshs in order')
        self.parser.add_argument('--export_folder', type=str, default='')
        self.parser.add_argument('--checkpoints_human_swich',type=str,default='ishiyama')
        self.parser.add_argument('--dataroot_swich',type=str,default='front')
        self.parser.add_argument('--local_checkpoints_dir',type=str,default='/home/ericlab/DENSO/raugh_recognition/checkpoint')
        self.parser.add_argument('--local_export_folder', type=str, default='')
        self.parser.add_argument('--tensorboardX_results_directory',type=str,default="/home/ericlab/ros_package/denso_ws/src/denso_run/denso_pkgs/pose_estimator_pkg/trainer/tensorboardX/")
        self.parser.add_argument('--tensorboardX_results_directory_switch',type=str,default="ishiyama")
        self.parser.add_argument('--dataset_number', type=int, default=self.dataset_number)
        self.parser.add_argument('--lr', type=float, default=0.001, help='initial learning rate of adam')
        self.parser.add_argument('--is_train', type=bool, default=False)
        self.parser.add_argument('--is_estimate', type=bool, default=False)
        # for instance-segmentation
        self.parser.add_argument('--embedded_size', type=int, default=32)
        self.parser.add_argument('--delta_d', type=float, default=1.5)
        self.parser.add_argument('--delta_v', type=float, default=0.5)
        self.parser.add_argument('--instance_number', type=int, default=8)
        # self.parser.add_argument('--checkpoints_process_swich',type=str,default='raugh_recognition')
        self.parser.add_argument('--semantic_number',type=int,default=3)


    def test_parse(self):
        self.initialize()
        self.opt, unknown = self.parser.parse_known_args()
        str_ids = self.opt.gpu_ids.split(',')
        self.opt.gpu_ids = []
        self.concat_dataset_model = '+'.join(self.opt.dataset_model)
        for str_id in str_ids:
            id = int(str_id)
            if id>= 0:
                self.opt.gpu_ids.append(id)

        if len(self.opt.gpu_ids) > 0:
            # torch.cuda.set_device(self.opt.gpu_ids[0])
            # torch.device("cuda" if torch.cuda.is_available() else "cpu")
            pass
        args = vars(self.opt)
        return self.opt





