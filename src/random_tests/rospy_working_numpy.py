# #!/usr/bin/env python2.7
# # Import ROS libraries and messages
# import rospy
# from sensor_msgs.msg import Image

# # Import OpenCV libraries and tools
# import cv2
# from cv_bridge import CvBridge, CvBridgeError

# # Initialize the ROS Node named 'opencv_example', allow multiple nodes to be run with this name
# rospy.init_node('opencv_example', anonymous=True)

# # Print "Hello ROS!" to the Terminal and to a ROS Log file located in ~/.ros/log/loghash/*.log
# rospy.loginfo("Hello ROS!")

# # Initialize the CvBridge class
# bridge = CvBridge()

# # Define a function to show the image in an OpenCV Window
# def show_image(img):
#     cv2.imshow("Image Window", img)
#     cv2.waitKey(3)

# # Define a callback for the Image message
# def image_callback(img_msg):
#     # log some info about the image topic
#     rospy.loginfo(img_msg.header)

#     # Try to convert the ROS Image message to a CV2 Image
#     try:
#         cv_image = bridge.imgmsg_to_cv2(img_msg, "passthrough")
#     except CvBridgeError as e:
#         rospy.logerr("CvBridge Error: {0}".format(e))

#     # Flip the image 90deg
#     cv_image = cv2.transpose(cv_image)
#     cv_image = cv2.flip(cv_image,1)

#     # Show the converted image
#     show_image(cv_image)

# # Initalize a subscriber to the "/camera/rgb/image_raw" topic with the function "image_callback" as a callback
# sub_image = rospy.Subscriber("/camera/rgb/image_raw", Image, image_callback)

# # Initialize an OpenCV Window named "Image Window"
# cv2.namedWindow("Image Window", 1)

# # Loop to keep the program from shutting down unless ROS is shut down, or CTRL+C is pressed
# while not rospy.is_shutdown():
#     rospy.spin()


import rospy
import pcl
from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
import numpy as np
import ros_numpy
import cv2
import struct
import ctypes
from cv_bridge import CvBridge, CvBridgeError
from PIL import Image as im

bridge = CvBridge()

def show_image(img):
    cv2.imshow("Image Window", img)
    cv2.waitKey(3)

def points_to_image(xs, ys, ps, img_size):
    coords = np.stack((ys, xs))
    print(coords)
    abs_coords = np.ravel_multi_index(coords, img_size)
    img = np.bincount(abs_coords, weights=ps, minlength=img_size[0]*img_size[1])
    img = img.reshape(img_size)

def callback(ros_point_cloud):
    print('here-->>')

    xyz = np.array([[0,0,0]])
    rgb = np.array([[0,0,0]])
    gen = pc2.read_points(ros_point_cloud, skip_nans=True)
    int_data = list(gen)

    for x in int_data:
        test = x[3] 
        # cast float32 to int so that bitwise operations are possible
        s = struct.pack('>f' ,test)
        i = struct.unpack('>l',s)[0]
        # you can get back the float value by the inverse operations
        pack = ctypes.c_uint32(i).value
        r = (pack & 0x00FF0000)>> 16
        g = (pack & 0x0000FF00)>> 8
        b = (pack & 0x000000FF)
        # prints r,g,b values in the 0-255 range
                    # x,y,z can be retrieved from the x[0],x[1],x[2]
        xyz = np.append(xyz,[[x[0],x[1],x[2]]], axis = 0)
        rgb = np.append(rgb,[[r,g,b]], axis = 0)

        # imgdata = im.fromarray(rgb)
        # imgdata.save("image.png")
        #png = np.resize(rgb, (480, 640, 3))
        imgdata = im.fromarray(rgb)
        if imgdata.mode != 'RGB':
            imgdata = imgdata.convert('RGB')
        imgdata.save("image.png")


        #new_rgb = np.dstack(rgb)

        #show_image(new_rgb)

    # try:
    #     cv_image = bridge.imgmsg_to_cv2(ros_point_cloud, "passthrough")
    # except CvBridgeError as e:
    #     rospy.logerr("CvBridge Error: {0}".format(e))

    # # Flip the image 90deg
    # cv_image = cv2.transpose(cv_image)
    # cv_image = cv2.flip(cv_image,1)

    # # Show the converted image
    # show_image(cv_image)

    # pc = ros_numpy.numpify(ros_point_cloud)
    # points=np.zeros((pc.shape[0],3))
    # points[:,0]=pc['x']
    # points[:,1]=pc['y']
    # points[:,2]=pc['z']
    
    # print(pc['x'])

    # img = points_to_image( pc['x'], pc['y'], pc['z'], (180, 240) )
    # print(img)

rospy.init_node('listener', anonymous=True)
rospy.Subscriber("/camera/depth/color/points", PointCloud2, callback)
cv2.namedWindow("Image Window", 1)

rospy.spin()