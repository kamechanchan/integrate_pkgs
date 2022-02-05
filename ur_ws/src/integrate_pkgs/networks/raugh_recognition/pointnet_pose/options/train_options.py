#!/usr/bin/env python3
import argparse

class TrainOptions_raugh_recognition():
    def __init__(self):
        pass
    
    def initialize(self):
        self.dataset_number = 1;
        self.parser = argparse.ArgumentParser(formatter_class=argparse.ArgumentDefaultsHelpFormatter)
        self.parser.add_argument('--dataroot', required=True)
        self.parser.add_argument('--print_freq', type=int, default=10, help='frequency of showing training results on console')
        self.parser.add_argument('--save_latest_freq', type=int, default=250, help='frequency of saving the latest results')
        self.parser.add_argument('--save_epoch_freq', type=int, default=1, help='frequency of saving checkpoints at the end of epochs')
        self.parser.add_argument('--run_test_freq', type=int, default=1, help='frequency of running test in training script')
        self.parser.add_argument('--continue_train', action="store_true", help='continue training: load the latest model')
        self.parser.add_argument('--epoch_count', type=int, default= 1, help='starting epoch count, we save the model by <epoch_count>, <epoch_count>+<save_latest_freq>, ...')
        self.parser.add_argument('--phase', type=str, default="train", help='train, val or trainval')
        self.parser.add_argument('--which_epoch', type=str, default="latest", help='which epoch to load? set to latest to use latest cached model')
        self.parser.add_argument('--niter', type=int, default=100, help='of iter at starting learning rate')
        self.parser.add_argument('--niter_decay', type=int, default=500, help='of iter to lineary decay learing rate to zero')
        self.parser.add_argument('--beta1', type=float, default=0.9, help='momentum term of adam')
        self.parser.add_argument('--lr_policy', type=str, default="lambda", help='learning rate policy: lambda|step|plateau')
        self.parser.add_argument('--lr_decay_iters', type=int, default=50, help='multiply by a gamma every lr_decay_iters iterations')

        # for ternsorboard visualization
        self.parser.add_argument('--no_vis', action='store_true', help='will not use tensorboard')
        self.parser.add_argument('--verbose_plot', action='store_true', help='plots network weights, etc.', default=True)
        self.parser.add_argument('--main_directory',type=str,default=__file__)
        self.parser.add_argument('--dataset_mode', choices={"instance_segmentation", "pose_estimation","semantic_segmentation"}, default='pose_estimation')
        self.parser.add_argument('--dataset_model', nargs=self.dataset_number, type=str, default='tsuchida_1000.hdf5')
        self.parser.add_argument('--max_dataset_size', nargs=self.dataset_number, type=int, default=float("inf"), help='Maximum num of samples per epoch')
        self.parser.add_argument('--process_swich', type=str, choices={"raugh_recognition", "object_segment"}, default="object_segment")
        self.parser.add_argument('--batch_size', type=int, default=8)
        self.parser.add_argument('--num_epoch', type=int, default=150)
        self.parser.add_argument('--arch', type=str, default="YOLO")
        self.parser.add_argument('--resolution', type=int, default=1024)
        self.parser.add_argument('--gpu_ids', type=str, default='')
        self.parser.add_argument('--gpu_num', type=int, default=1)
        self.parser.add_argument('--num_threads', type=int, default=3)
        self.parser.add_argument('--checkpoints_dir', type=str, default="/home/ericlab/Desktop/ishiyama/Yolo_saikou/weights/yolo_simulator.pth")
        self.parser.add_argument('--serial_batches', action='store_true', help='if true, takes meshs in order', default=True)
        self.parser.add_argument('--export_folder', type=str, default='')
        self.parser.add_argument('--checkpoints_human_swich',type=str,default='ishiyama')
        self.parser.add_argument('--dataroot_swich',type=str,default='front')
        self.parser.add_argument('--local_checkpoints_dir',type=str,default='/home/ericlab/DENSO/raugh_recognition/checkpoint')
        self.parser.add_argument('--local_export_folder', type=str, default='')
        self.parser.add_argument('--tensorboardX_results_directory',type=str,default="/home/ericlab/ros_package/denso_ws/src/denso_run/denso_pkgs/pose_estimator_pkg/trainer/tensorboardX/")
        self.parser.add_argument('--tensorboardX_results_directory_switch',type=str,default="ishiyama")
        self.parser.add_argument('--dataset_number', type=int, default=self.dataset_number)
        self.parser.add_argument('--lr', type=float, default=0.001, help='initial learning rate of adam')
        self.parser.add_argument('--is_train', type=bool, default=True)
        self.parser.add_argument('--is_estimate', type=bool, default=False)
        # for instance-segmentation
        self.parser.add_argument('--embedded_size', type=int, default=32)
        self.parser.add_argument('--delta_d', type=float, default=1.5)
        self.parser.add_argument('--delta_v', type=float, default=0.5)
        self.parser.add_argument('--instance_number', type=int, default=8)
        # self.parser.add_argument('--checkpoints_process_swich',type=str,default='raugh_recognition')
        self.parser.add_argument('--semantic_number',type=int,default=3)
       

        
        #self.is_train = True

    def train_parse(self):
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


