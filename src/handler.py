#!/usr/bin/env python

import rospy
import math
import pcl
import ros_numpy as rnp
import numpy as np
from sensor_msgs.msg import Image
from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs.point_cloud2 as pc2

import open3d as o3d
from open3d_ros_helper import open3d_ros_helper as orh
import ctypes
import struct

# from cv_bridge import CvBridge, CvBridgeError
# bridge = CvBridge()

class DyanmicHandler():
    def __init__(self):
        self.publisher = rospy.Publisher("/visualization", PointCloud2, queue_size=2)

        rospy.Subscriber("/camera/depth/color/points", PointCloud2, self.pointclout_handler )
        rospy.spin()
        self.o3dpc = None

    def pointclout_handler(self, point_cloud):
        print("handler")
        # -- distance detect
        pc_np = rnp.point_cloud2.pointcloud2_to_xyz_array(point_cloud,remove_nans=True)
        dist_arr = np.zeros(shape=(pc_np.shape[0], 1))
        for i in range(pc_np.shape[0]):
            distance = math.sqrt(pc_np[i, 0] ** 2 + pc_np[i, 1] ** 2 + pc_np[i, 2] ** 2)
            dist_arr[i, 0] = distance

        # print(np.amin(distance_array))
        # print("Min distance:" + str(np.argmin(dist_arr)))
        # print("Min distance:" + str(np.amin(dist_arr)))
        # print("Max distance:" + str(np.argmax(dist_arr)))
        # print("Max distance:" + str(np.amax(dist_arr)))
        # --

        # cloud = pc2.read_points(point_cloud, skip_nans=True )
        # data  = list(cloud)
        #
        # xyz = np.empty((len(data), 3))
        # rgb = np.empty((len(data), 3))
        #
        # for x in data:
        #     test = x[3]
        #     s = struct.pack('>f', test)
        #     i = struct.unpack('>l', s)[0]
        #     pack = ctypes.c_uint32(i).value
        #     r = (pack & 0x00FF0000) >> 16
        #     g = (pack & 0x0000FF00) >> 8
        #     b = (pack & 0x000000FF)
        #     xyz = np.append(xyz, [[x[0], x[1], x[2]]], axis=0)
        #     rgb = np.append(rgb, [[r, g, b]], axis=0)
        #
        # pclObj = o3d.geometry.PointCloud()
        # pclObj.points = o3d.utility.Vector3dVector(xyz)
        # pclObj.colors = o3d.utility.Vector3dVector(rgb)
        #
        # o3d.visualization.draw_geometries([pclObj],
        #                                   zoom=0.3412,
        #                                   front=[0.4257, -0.2125, -0.8795],
        #                                   lookat=[2.6172, 2.0475, 1.532],
        #                                   up=[-0.0694, -0.9768, 0.2024])

        se3 = np.eye(4)
        ros_transform = orh.se3_to_transform(se3)
        o3dpc = orh.rospc_to_o3dpc(point_cloud)

        # o3d.visualization.draw_geometries([o3dpc],
        #                                   zoom=0.3412,
        #                                   front=[0.4257, -0.2125, -0.8795],
        #                                   lookat=[2.6172, 2.0475, 1.532],
        #                                   up=[-0.0694, -0.9768, 0.2024])

        #rospc = orh.rospc_to_o3dpc(o3dpc) # cause error

        # pc = rnp.numpify(point_cloud)
        # points = np.zeros((pc.shape[0], 3))
        # points[:, 0] = pc['x']
        # points[:, 1] = pc['y']
        # points[:, 2] = pc['z']
        # p = pcl.PointCloud(np.array(points, dtype=np.float32))

        downpcd = o3dpc.voxel_down_sample(voxel_size=0.05)
        # o3d.visualization.draw_geometries([downpcd],
        #                                   zoom=0.3412,
        #                                   front=[0.4257, -0.2125, -0.8795],
        #                                   lookat=[2.6172, 2.0475, 1.532],
        #                                   up=[-0.0694, -0.9768, 0.2024])

        rospc = orh.rospc_to_o3dpc(downpcd)


        # publish to rviz
        self.publisher.publish(rospc)

        # convert to message
        #rospc = orh.rospc_to_o3dpc(o3dpc)


if __name__ == '__main__':
    rospy.init_node('listener', anonymous=True)
    try:
        detect = DyanmicHandler()
    except KeyboardInterrupt:
        print('closed')