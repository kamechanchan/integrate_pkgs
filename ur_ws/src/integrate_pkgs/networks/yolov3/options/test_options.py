#!/usr/bin/env python3
import torch 
import argparse
import rospkg

class TestOptions_raugh_recognition():
    def __init__(self):
        pass

    def initialize(self):
        self.dataset_number = 1;
        rospack = rospkg.RosPack()
        network_path = rospack.get_path("all_estimator") + "/../"
        self.parser = argparse.ArgumentParser(formatter_class=argparse.ArgumentDefaultsHelpFormatter)
        self.parser.add_argument("--font_path", type=str, default=network_path + "networks/yolov3/font/ipag.ttc")
        self.parser.add_argument("--save_path", type=str, default=network_path + "networks/yolov3/output/output.jpg")
        self.parser.add_argument("--config_path", type=str, default=network_path + "networks/yolov3/config/yolov3_denso.yaml")
        self.parser.add_argument("--load_path", type=str, default=network_path + "networks/yolov3/weights/yolo_simulator.pth")
        self.parser.add_argument("--config_dir_path", type=str, default=network_path + "networks/yolov3/config")
        self.parser.add_argument('--gpu_ids', type=str, default='0')
        self.parser.add_argument('--num_threshold', type=float, default=0.45)
        self.parser.add_argument('--conf_threshold', type=float, default=0.5)
        self.parser.add_argument('--img_size', type=int, default=416)


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





