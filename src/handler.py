#!/usr/bin/env python

import rospy
import math
import ros_numpy as rnp
import numpy as np
from sensor_msgs.msg import Image
from sensor_msgs.msg import PointCloud2, PointField

class DyanmicHandler():
    def __init__(self):
        rospy.Subscriber("/camera/depth/color/points", PointCloud2, self.pointclout_handler )
        rospy.spin()

    def pointclout_handler(self, point_cloud):
        print("handler")
        pc_np = rnp.point_cloud2.pointcloud2_to_xyz_array(point_cloud,remove_nans=True)
        dist_arr = np.zeros(shape=(pc_np.shape[0], 1))
        for i in range(pc_np.shape[0]):
            distance = math.sqrt(pc_np[i, 0] ** 2 + pc_np[i, 1] ** 2 + pc_np[i, 2] ** 2)
            dist_arr[i, 0] = distance

        # print(np.amin(distance_array))
        print("Min distance:" + str(np.argmin(dist_arr)))
        print("Min distance:" + str(np.amin(dist_arr)))
        print("Max distance:" + str(np.argmax(dist_arr)))
        print("Max distance:" + str(np.amax(dist_arr)))


if __name__ == '__main__':
    rospy.init_node('listener', anonymous=True)
    try:
        detect = DyanmicHandler()
    except KeyboardInterrupt:
        print('closed')