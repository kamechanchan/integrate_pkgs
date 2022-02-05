#!/usr/bin/env python3

from denso_srvs.srv import sensor_data_service, sensor_data_serviceResponse
import rospy
import open3d

import numpy as np


rospy.init_node("message_receive")
rospy.wait_for_service("message_service")
resp1 = rospy.ServiceProxy("message_service", sensor_data_service)
resp = resp1(True)
# resp = sensor_data_serviceResponse()
resp.point_cloud2

# data = np.array((resp.point_cloud2.x, resp.point_cloud2.y, resp.point_cloud2.z))
data = np.zeros((len(resp.point_cloud2.x),3), float)
print(len(resp.point_cloud2.x))
for i in range(len(resp.point_cloud2.x)):
    data[i][0] = resp.point_cloud2.x[i]
    data[i][1] = resp.point_cloud2.y[i]
    data[i][2] = resp.point_cloud2.z[i]



pointcloud = open3d.geometry.PointCloud()
pointcloud.points = open3d.utility.Vector3dVector(data)
open3d.visualization.draw_geometries([pointcloud])


