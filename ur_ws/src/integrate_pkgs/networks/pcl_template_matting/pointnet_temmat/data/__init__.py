
from os import PRIO_USER
import torch.utils.data
from torch.utils.data.dataset import Subset
from data.base_dataset import collate_fn, collate_fn_original
import sys
from pose_estimate_data import PoseData

def TrainValDataset(opt):
    print(opt.dataroot)
    dataset = PoseData(opt)
    # print(dataset.size)
    # print(dataset)
    n_samples = len(dataset)
    train_size = int(n_samples * 0.95)

    subset1_indices = list(range(0, train_size))
    subset2_indices = list(range(train_size, n_samples))

    subset1 = Subset(dataset, subset1_indices) #set train_data and index(対応付け)
    subset2 = Subset(dataset, subset2_indices)
    # print("dataset_dataset")
    # print(type(subset1))
    # print(type(subset2))
    return subset1, subset2


def EstimateDataset(opt):
    
    dataset = PoseData(opt)
    n_samples = len(dataset)
    subset_indices = list(range(0,n_samples))
    subset = Subset(dataset, subset_indices)
    
    return subset


class TrainDataLoader:
    def __init__(self, dataset, opt):
        self.opt = opt
        self.dataset= dataset
        self.batch_size = opt.batch_size * opt.gpu_num
        # print("ishiyama")
        # print(opt.batch_size)
        # print(opt.gpu_num)
        
        # if self.opt.dataset_mode == "pose_estimation":
        self.dataloader = torch.utils.data.DataLoader(
                self.dataset,
                batch_size=self.batch_size,
                shuffle=True,
                num_workers=int(opt.num_threads),
                collate_fn=collate_fn)
        # elif self.opt.dataset_mode == "instance_segmentation":
        #     self.dataloader = torch.utils.data.DataLoader(
        #             self.dataset,
        #             batch_size=self.batch_size,
        #             shuffle=True,
        #             num_workers=int(opt.num_threads),
        #             collate_fn=collate_fn)
        # else :
        #     print("opt.dataset_mode problem")
        #     sys.exit(1)

        self.len_size = 0
        for i in range(self.opt.dataset_number):
            self.len_size = self.len_size + self.opt.max_dataset_size[i]

    def __len__(self):
        return min(len(self.dataset), self.len_size)


    def __iter__(self):
        for i, data in enumerate(self.dataloader):
            if i * self.opt.batch_size >= self.len_size:
                break
            yield data


class ValDataLoader:
    def __init__(self, dataset, opt):
        self.opt = opt
        self.dataset= dataset
        self.batch_size = opt.batch_size * opt.gpu_num
        # print("ishiyama")
        # print(opt.batch_size)
        # print(opt.gpu_num)
        # print(self.batch_size)

        # if self.opt.dataset_mode == "pose_estimation":
        self.dataloader = torch.utils.data.DataLoader(
                self.dataset,
                batch_size=self.batch_size,
                shuffle=False,
                num_workers=int(opt.num_threads),
                collate_fn=collate_fn)
        # elif self.opt.dataset_mode == "instance_segmentation":
        #     self.dataloader = torch.utils.data.DataLoader(
        #             self.dataset,
        #             batch_size=self.batch_size,
        #             shuffle=False,
        #             num_workers=int(opt.num_threads),
        #             collate_fn=collate_fn)
        # else :
        #     print("opt.dataset_mode problem")
        #     sys.exit(1)

        self.len_size = 0
        for i in range(self.opt.dataset_number):
            self.len_size = self.len_size + self.opt.max_dataset_size[i]

    def __len__(self):
        return min(len(self.dataset), self.len_size)


    def __iter__(self):
        for i, data in enumerate(self.dataloader):
            if i * self.opt.batch_size >= self.len_size:
                break
            yield data
