from typing_extensions import Annotated
import h5py
import rospy


class Read_hdf5():
    def __init__(self):
        # rospy.init_node("main", anonymous=True)
        self.data_path = rospy.get_param("~data_path", "/home/ericlab/hdf5_file/Acc_accuracy/")
        self.data_name = rospy.get_aram("~data_name", "instance_tsuchida_12_31_100_1.hdf5")
        data = self.data_path + self.data_name
        self.hdf5_file = h5py.File(data, "r")

    def get_data(self, index):
        in_cloud = self.hdf5_file["data_" + str(index)]["Points"]
        in_img = self.hdf5_file["data_" + str(index)]["img"]
        masks = self.hdf5_file["data_" + str(index)]["masks"]
        rotation = self.hdf5_file["data_" + str(index)]["rotation"]
        translation = self.hdf5_file["data_" + str(index)]["translation"]

        return in_cloud, in_img, masks, rotation, translation
