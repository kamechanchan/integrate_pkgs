#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os, sys, copy
from numpy.lib.twodim_base import mask_indices
from open3d import *
import numpy as np
from ctypes import *
import pcl

import rospy, rospkg, geometry_msgs, tf
from std_msgs.msg import Header
from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs.point_cloud2 as pc2


FIELDS_XYZ = [
    PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
    PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
    PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
]

FIELDS_XYZRGB = FIELDS_XYZ + \
        [PointField(name='rgb', offset=12, datatype=PointField.UINT32, count=1)]

BIT_MOVE_16 = 2**16
BIT_MOVE_8 = 2**8

convert_rgbUint32_to_tuple = lambda rgb_uint32: (
        (rgb_uint32 & 0x00ff0000) >> 16, (rgb_uint32 & 0x0000ff00) >> 8, (rgb_uint32 & 0x000000ff)
)

convert_rgbFloat_to_tuple = lambda rgb_float: convert_rgbUint32_to_tuple(
        int(cast(pointer(c_float(rgb_float)), POINTER(c_uint32)).contents.value)
)

def convertCloudFromOpen3dToRos(open3d_cloud, frame_header):
    frame_header.stamp = rospy.Time.now()

    points = np.asarray(open3d_cloud.points)
    if not open3d_cloud.colors:
        fields = FIELDS_XYZ
        cloud_data = points.astype(np.float32)
        POINTS = cloud_data.tolist()
    else:
        fields = FIELDS_XYZRGB
        colors = np.floor(np.asarray(open3d_cloud.colors)*255)
        colors = colors[:,0] * BIT_MOVE_16 + colors[:, 1] * BIT_MOVE_8 + colors[:, 2]
        POINTS = np.c_[points, colors]

    return pc2.create_cloud(frame_header, fields, POINTS)


def convertCloudFromRosToOpen3d(ros_cloud):
    field_names=[field.name for field in ros_cloud.fields]
    cloud_data = list(pc2.read_points(ros_cloud, skip_nans=True, field_names = field_names))

    open3d_cloud = open3d.geometry.PointCloud()
    if len(cloud_data)==0:
        # point("Converting an empty cloud")
        return None

    if "rgb" in field_names:
        IDX_RGB_IN_FIELD=3

        xyz = [(x, y, z) for x, y, z, rgb in cloud_data]

        if type(cloud_data[0][IDX_RGB_IN_FIELD])==float:
            rgb = [convert_rgbFloat_to_tuple(rgb) for x, y, z, rgb in cloud_data ]
        else:
            rgb = [convert_rgbUint32_to_tuple(rgb) for x, y, z, rgb in cloud_data ]

        open3d_cloud.points = open3d.utility.Vector3dVector(np.array(xyz))
        open3d_cloud.colors = open3d.utility.Vector3dVector(np.array(rgb)/255.0)
    else:
        xyz = [(x, y, z) for x, y, z in cloud_data]
        open3d_cloud.points = open3d.utility.Vector3dVector(np.array(xyz))

    return open3d_cloud

def doMinMax(cloud):
    p = np.asarray(cloud)
    min_p = np.amin(p, axis=0)
    max_p = np.amax(p, axis=0)
    diff_max = np.amax(np.amax(p) - np.amin(p))

    max_x = max_p[0]
    max_y = max_p[1]
    max_z = max_p[2]

    min_x = min_p[0]
    min_y = min_p[1]
    min_z = min_p[2]

    diff_x = max_x - min_x
    diff_y = max_y - min_y
    diff_z = max_z - min_z

    diff_max = 0.0
    diff_max = diff_x
    if (diff_max < diff_y):
        diff_max = diff_y
    elif (diff_max < diff_z):
        diff_max = diff_z

    return min_p, max_p, diff_max


def doMinMax_inference(cloud):
    p = np.asarray(cloud)
    min_p = np.amin(p, axis=0)
    max_p = np.amax(p, axis=0)
    diff_max = np.amax(np.amax(p) - np.amin(p))
    voxel_offset_x = 0.01
    voxel_offset_y = 0.01
    voxel_offset_z = 0.01

    max_x = max_p[0] + voxel_offset_x
    max_y = max_p[1] + voxel_offset_y
    max_z = max_p[2] + voxel_offset_z
    
    min_x = min_p[0] - voxel_offset_x
    min_y = min_p[1] - voxel_offset_y
    min_z = min_p[2] - voxel_offset_z

    diff_x = max_x - min_x
    diff_y = max_y - min_y
    diff_z = max_z - min_z

    diff_max = 0.0
    diff_max = diff_x
    if (diff_max < diff_y):
        diff_max = diff_y
    elif (diff_max < diff_z):
        diff_max = diff_z

    return min_p, max_p, diff_max





def makeVoxel(cloud, min_p, diff_max, voxel_resolution):
    leaf = round(1.0 / voxel_resolution, 5)
    p = np.asarray(cloud)
    normalized_p = ((p - min_p) / diff_max).astype('float32')
    normalized_p = np.where(normalized_p >= 1.0, 1.0, normalized_p).astype("float32")
    normal_p = pcl.PointCloud(normalized_p)
    sor = normal_p.make_voxel_grid_filter()
    sor.set_leaf_size(leaf, leaf, leaf)
    voxel_filtered = sor.filter()
    return voxel_filtered

def makeBinaryData(cloud, voxel_resolution):
    leaf = round(1.0 / voxel_resolution, 5)
    binary_voxel = np.zeros(voxel_resolution *voxel_resolution * voxel_resolution)

    for i in range(cloud.size):
        point_x =  int(np.asarray(cloud)[i][0] / leaf)
        point_y = int(np.asarray(cloud)[i][1] / leaf)
        point_z = int(np.asarray(cloud)[i][2] / leaf)

        index = voxel_resolution * voxel_resolution * point_z + voxel_resolution * point_y + point_x
        if index <= voxel_resolution ** 3 -1:
            binary_voxel[index] = 1

    return binary_voxel


def runMakeVoxelBinary(cloud, resolution=64):
    min_p, max_p, diff_max = doMinMax(cloud)
    voxel_filter = makeVoxel(cloud, min_p, diff_max, resolution)
    return makeBinaryData(voxel_filter, resolution)


# def crop_geometry(data):
#     print("Demo for manual geometry cropping")
#     print("1) Press 'Y' twice to align geometry with positive direction of y-axis")
#     print("2) Press 'K' to lock screen and to switch to selection mode")
#     print("3) Drag for rectangle selection,")
#     print("   or use ctrl + left click for polygon selection")
#     print("4) Press 'C' to get a selected geometry and to save it")
#     print("5) Press 'F' to switch to freeview mode")

#     draw_geometries_with_editing([data])
#     return data

# def get_cropped(path):
#     obj = read_point_cloud(path)
#     return obj


def stl2cloud(model):
    rospack = rospkg.RosPack()
    package_path = rospack.get_path("denso_descriptions")
    stl_file = os.path.join(package_path, "object_description", "meshes", "STL", model)
    mesh = io.read_triangle_mesh(stl_file)
    pcd = geometry.PointCloud()
    pcd.points = mesh.vertices
    return pcd


def model_loader(model):
    rospack = rospkg.RosPack()
    package_path = rospack.get_path("all_estimator")
    pcd_file = os.path.join(package_path, "pcd", model)
    print(pcd_file)
    pcd = io.read_point_cloud(pcd_file)
    return pcd


def stl2PointCloud(model):
    rospack = rospkg.RosPack()
    package_path = rospack.get_path("denso_descriptions")
    stl_file = os.path.join(package_path, "object_description", "meshes", "STL", model)
    mesh = io.read_triangle_mesh(stl_file)
    pcd = mesh.sample_points_poisson_disk(10000)
    return pcd





def getNormalizedPcd(np_cloud, resolution):
    """
    cloud_start=pcl.PointCloud(cloud_data)
    pre_cloud = cloud_start.make_voxel_grid_filter()
    pcl.VoxelGridFilter.set_leaf_size(pre_cloud, 0.0035, 0.0035, 0.0035)
    cloud_filter=pcl.VoxelGridFilter.filter(pre_cloud)
    np_cloud=np.array(cloud_filter)
    """ 
    pcd_offset = np.expand_dims(np.mean(np_cloud, axis=0), 0)
    # print("zettaikoko")
    # print(pcd_offset)
    pcd_data = np_cloud - pcd_offset  #original
    # pcd_data = np_cloud 
    #pcd_data = np.asarray(np_cloud)  #improve
    choice_index = np.arange(pcd_data.shape[0])
    choice = np.random.choice(choice_index, resolution)
    normalized_pcd = pcd_data[choice, :]
    #new_pcd = pcl.PointCloud(np.array(normalized_pcd, np.float32))
    #pcl.save(new_pcd, "/home/ericlab/random_original.pcd")
    #pcl.save(new_pcd, '/home/ericlab/random_improve.pcd')
    # return normalized_pcd, pcd_offset[0]
    return normalized_pcd, pcd_offset

def getNormalizedPcd_seg(np_cloud, resolution):
    # print("*********pcl******************")
    # cloud_start=pcl.PointCloud(cloud_data[:,:3])
    # pre_cloud = cloud_start.make_voxel_grid_filter()
    # pcl.VoxelGridFilter.set_leaf_size(pre_cloud, 0.0035, 0.0035, 0.0035)
    # cloud_filter=pcl.VoxelGridFilter.filter(pre_cloud)
    # np_cloud=np.array(cloud_filter)
    
    # cnt_a = 0
    # cnt_b = 0
    # cnt_c = 0
    # cnt_d = 0
    # cnt_e = 0
    # cnt_f = 0
    # cnt_g = 0
    # cnt_l = 0
    # for i in range(300000) :
    #     if np_cloud[i,0] == 1:
    #         cnt_a+=1
    #     elif np_cloud[i,1] == 1:
    #         cnt_b+=1
    #     elif np_cloud[i,2] == 1:
    #         cnt_c+=1
    #     elif np_cloud[i,3] == 1:
    #         cnt_d+=1
    #     elif np_cloud[i,4] == 1:
    #         cnt_e+=1
    #     elif np_cloud[i,5] == 1:
    #         cnt_f+=1
    #     elif np_cloud[i,6] == 1:
    #         cnt_g+=1
    #     elif np_cloud[i,7] == 1:
    #         cnt_l+=1
    # print("******************cnt********************")        
    # print("cnt_a")
    # print(cnt_a)
    # print("cnt_b")
    # print(cnt_b)
    # print("cnt_c")
    # print(cnt_c)
    # print("cnt_d")
    # print(cnt_d)
    # print("cnt_e")
    # print(cnt_e)
    # print("cnt_f")
    # print(cnt_f)
    # print("cnt_g")
    # print(cnt_g)
    # print("cnt_l")
    # print(cnt_l)

    pcd_offset = np.expand_dims(np.mean(np_cloud[:,:3], axis=0), 0)
    pre_pcd_data = np_cloud[:,:3] - pcd_offset  #original
    # pcd_offset = np.expand_dims(np.mean(np_cloud, axis=0), 0)
    # pre_pcd_data = np_cloud - pcd_offset  #original
    mask_data = np_cloud[:,3]
    # mask_data = cloud_data[:,3]
    mask_data = np.expand_dims(mask_data, 1)
    pcd_data = np.hstack([pre_pcd_data, mask_data])
    #pcd_data = np.asarray(np_cloud)  #improve
    choice_index = np.arange(pcd_data.shape[0])
    choice = np.random.choice(choice_index, resolution)
    normalized_pcd = pcd_data[choice, :]
    #new_pcd = pcl.PointCloud(np.array(normalized_pcd, np.float32))
    #pcl.save(new_pcd, "/home/ericlab/random_original.pcd")
    #pcl.save(new_pcd, '/home/ericlab/random_improve.pcd')
    return normalized_pcd, pcd_offset[0]


# def p2pICP(source, target, trans_init, threshold=0.01):
#     reg_p2p = registration.registration_icp(source, target, threshold, trans_init, registration.TransformationEstimationPointToPoint())
#     return reg_p2p.transformation


# def p2planeICP(source, target, trans_init, threshold=0.02):
#     reg_p2p = registration.registration_icp(source, target, threshold, trans_init, registration.TransformationEstimationPointToPlane())
#     return reg_p2p.transformation


# def draw_registration_result(source, target, transformation):
#     source_temp = copy.deepcopy(source)
#     target_temp = copy.deepcopy(target)
#     source_temp.paint_uniform_color([1, 0.706, 0])
#     target_temp.paint_uniform_color([0, 0.651, 0.929])
#     source_temp.transform(transformation)
#     visualization.draw_geometries([source_temp, target_temp])


# Transform to homogeneous matrix
def transform2homogeneousM(tfobj):
    # Quat to euler sxyz とあるが， XYZWの順番で良い。ちょっとわかりにくくないか？
    tfeul= tf.transformations.euler_from_quaternion([tfobj.rotation.x,tfobj.rotation.y,tfobj.rotation.z,tfobj.rotation.w],axes='sxyz')
    # 並進量の記述
    tftrans = [ tfobj.translation.x,tfobj.translation.y,tfobj.translation.z]
    tfobjM = tf.transformations.compose_matrix(angles=tfeul,translate=tftrans)
    # return
    return  tfobjM

def homogeneous2transform(Mat):
    scale, shear, angles, trans, persp = tf.transformations.decompose_matrix(Mat)
    quat = tf.transformations.quaternion_from_euler(angles[0],angles[1],angles[2])
    tfobj = geometry_msgs.msg.TransformStamped()
    tfobj.transform.rotation.x = quat[0]
    tfobj.transform.rotation.y = quat[1]
    tfobj.transform.rotation.z = quat[2]
    tfobj.transform.rotation.w = quat[3]
    tfobj.transform.translation.x = trans[0]
    tfobj.transform.translation.y = trans[1]
    tfobj.transform.translation.z = trans[2]
    return tfobj


def homogeneous2pose(Mat):
    scale, shear, angles, trans, persp = tf.transformations.decompose_matrix(Mat)
    quat = tf.transformations.quaternion_from_euler(angles[0],angles[1],angles[2])
    poseobj = geometry_msgs.msg.Pose()
    poseobj.orientation.x = quat[0]
    poseobj.orientation.y = quat[1]
    poseobj.orientation.z = quat[2]
    poseobj.orientation.w = quat[3]
    poseobj.position.x = trans[0]
    poseobj.position.y = trans[1]
    poseobj.position.z = trans[2]
    return poseobj


# Transform diff tf1 to 2

def transform_diff(tf1,tf2):
    tf1M = transform2homogeneousM(tf1)
    tf2M = transform2homogeneousM(tf2)
    return  homogeneous2transform(tf2M.dot(tf.transformations.inverse_matrix(tf1M)))


# Pose version も作る
def pose2homogeneousM(poseobj):
    try:
        # Quat to euler sxyz とあるが， XYZWの順番で良い。ちょっとわかりにくくないか？
        tfeul= tf.transformations.euler_from_quaternion([poseobj.orientation.x,poseobj.orientation.y,poseobj.orientation.z,poseobj.orientation.w],axes='sxyz')
        # 並進量の記述
        tftrans = [ poseobj.position.x,poseobj.position.y,poseobj.position.z]
        poseobjM = tf.transformations.compose_matrix(angles=tfeul,translate=tftrans)
        return poseobjM
    except:
        print("Input must be a pose object!")


def homogeneous2pose(Mat):
    scale, shear, angles, trans, persp = tf.transformations.decompose_matrix(Mat)
    quat = tf.transformations.quaternion_from_euler(angles[0],angles[1],angles[2])
    poseobj = geometry_msgs.msg.Pose()
    poseobj.orientation.x = quat[0]
    poseobj.orientation.y = quat[1]
    poseobj.orientation.z = quat[2]
    poseobj.orientation.w = quat[3]
    poseobj.position.x = trans[0]
    poseobj.position.y = trans[1]
    poseobj.position.z = trans[2]
    return poseobj

def pose_diff(p1,p2):
    p1M = pose2homogeneousM(p1)
    p2M = pose2homogeneousM(p2)
    return  homogeneous2pose(p2M.dot(tf.transformations.inverse_matrix(p1M)))
