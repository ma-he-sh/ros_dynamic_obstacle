#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2, PointField, Image
from cv_bridge import CvBridge, CvBridgeError

bridge = CvBridge()

class ObsDetector():
    feed_width = 640
    feed_height= 480

    def __init__(self):
        rospy.Subscriber("/camera/depth/image_rect_raw", Image, self.point_cloud_handler)
        rospy.spin()

    def show_image(self, img):
      cv2.imshow("Image Window", img)
      cv2.waitKey(3)

    def point_cloud_handler(self, point_cloud):
        print('here')

        try:
            cv_image = bridge.imgmsg_to_cv2(point_cloud, "passthrough")
        except CvBridgeError as e:
            rospy.logerr("CvBridge Error: {0}".format(e))

        depth_array = np.array(cv_image, dtype=np.float32)
        depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(cv_image, alpha=0.03), cv2.COLORMAP_JET)

        self.show_image(depth_colormap)



        # for x in pts:
        #     test = x[3]

        #     s = struct.pack('>f' ,test)
        #     i = struct.unpack('>l',s)[0]

        #     pack = ctypes.c_uint32(i).value
        #     r = (pack & 0x00FF0000)>> 16
        #     g = (pack & 0x0000FF00)>> 8
        #     b = (pack & 0x000000FF)
            
        #     xyz = np.append(xyz,[[x[0],x[1],x[2]]], axis = 0)
        #     rgb = np.append(rgb,[[r,g,b]], axis = 0)

if __name__ == '__main__':
    rospy.init_node('listener', anonymous=True)
    try:
        detect = ObsDetector()
    except KeyboardInterrupt:
        print('closing')