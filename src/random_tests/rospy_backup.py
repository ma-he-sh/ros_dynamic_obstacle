#!/usr/bin/env python3

import rospy
import pcl
import numpy as np
import ctypes
import struct
import sensor_msgs.point_cloud2 as pc2

from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
from random import randint

class ObsDetector():
    feed_width = 640
    feed_height= 480

    def __init__(self):
        rospy.Subscriber("/camera/depth/color/points", PointCloud2, self.point_cloud_handler)

    def point_cloud_handler(self, point_cloud):
        print('here')
        points = []
        lim = 8
        for i in range(lim):
            for j in range(lim):
                for k in range(lim):
                    x = float(i) / lim
                    y = float(j) / lim
                    z = float(k) / lim
                    r = int(x * 255.0)
                    g = int(y * 255.0)
                    b = int(z * 255.0)
                    a = 255
                    print (r, g, b, a)
                    rgb = struct.unpack('I', struct.pack('BBBB', b, g, r, a))[0]
                    print (hex(rgb))
                    pt = [x, y, z, rgb]
                    points.append(pt)

                    

        # fields = [PointField('x', 0, PointField.FLOAT32, 1),
        #         PointField('y', 4, PointField.FLOAT32, 1),
        #         PointField('z', 8, PointField.FLOAT32, 1),
        #         # PointField('rgb', 12, PointField.UINT32, 1),
        #         PointField('rgba', 12, PointField.UINT32, 1),
        #         ]

        # header = Header()
        # header.frame_id = "map"
        # cloud = pc2.create_cloud(header, fields, points)


if __name__ == '__main__':
    rospy.init_node('listener', anonymous=True)
    try:
        detect = ObsDetector()
        rospy.spin()
    except KeyboardInterrupt:
        print('closing')