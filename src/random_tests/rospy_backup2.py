#!/usr/bin/env python3

import rospy
import pcl
import numpy as np
import ctypes
import struct
import math
import sensor_msgs.point_cloud2 as pc2

from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
from random import randint
import ros_numpy as rnp

import matplotlib.pyplot as plt

class ObsDetector():
    feed_width = 640
    feed_height= 480

    def __init__(self):
        self.fig = plt.figure()
        self.ax = plt.axes(projection='3d')
        self.ax.set_xlabel('X axis')
        self.ax.set_ylabel('Y axis')
        self.ax.set_zlabel('Z axis')

        rospy.Subscriber("/camera/depth/color/points", PointCloud2, self.point_cloud_handler)
        # plt.show()
        rospy.spin()

    def point_cloud_handler(self, point_cloud):
        print('here')

        pc_np = rnp.point_cloud2.pointcloud2_to_xyz_array(point_cloud,remove_nans=True)
        distance_array = np.zeros(shape=(pc_np.shape[0],1))
        for i in range(pc_np.shape[0]): 
            distance = math.sqrt(pc_np[i,0]**2 + pc_np[i,1]**2 + pc_np[i,2]**2)
            distance_array[i,0] = distance

        # print(np.amin(distance_array))
        print("Minimum distance index is: " + str(np.argmin(distance_array)))
        print("Maximum distance index is: " + str(np.amax(distance_array)))

        mean = np.mean(pc_np[:,2])
        sd = np.std(pc_np[:,2])
        data_ground = pc_np[(pc_np[:,2] < mean + 1.5*sd) & (pc_np[:,2] > mean - 1.5*sd)]
        data_wo_ground = pc_np[(pc_np[:,2] > mean + 1.5*sd) | (pc_np[:,2] < mean - 1.5*sd)]
        
        print(data_ground.shape, data_wo_ground.shape)

        #self.ax.scatter(data_wo_ground[0], data_wo_ground[1], data_wo_ground[2])
        
        # j=0
        # while j <= 10: 
        #     self.ax.scatter(pc_np[:,0], pc_np[:,1], pc_np[:,2])
        # time.sleep(20)

        #print(distance_array.shape)
        #print(pc_np[1,0])
        print("Listening END. Size is: " + str(pc_np.shape))

if __name__ == '__main__':
    rospy.init_node('listener', anonymous=True)
    try:
        detect = ObsDetector()
    except KeyboardInterrupt:
        print('closing')