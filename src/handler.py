#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
import sensor_msgs.point_cloud2 as pc2

class DyanmicHandler():
    def __init__(self):
        rospy.Subscriber("/camera/depth/color/points", PointCloud2, self.pointclout_handler )
        rospy.spin()

    def pointclout_handler(self, point_cloud):
        print("here")
        print(point_cloud)

if __name__ == '__main__':
    rospy.init_node('listener', anonymous=True)
    try:
        detect = DyanmicHandler()
    except KeyboardInterrupt:
        print('closed')