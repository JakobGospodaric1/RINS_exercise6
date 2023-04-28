#!/usr/bin/env python3
# Import ROS libraries and messages
import rospy

# import the PointCloud2 message
from sensor_msgs.msg import PointCloud2, Image

import message_filters

# Import OpenCV libraries and tools
import cv2
from cv_bridge import CvBridge, CvBridgeError

# Initialize the CvBridge class
bridge = CvBridge()

cv2.startWindowThread()

def find_rings_callback(rgb_msg, depth_msg):
    #print timestamp
    print(rgb_msg.header.stamp)

    try:
        rgb_image = bridge.imgmsg_to_cv2(rgb_msg, "passthrough")
    except CvBridgeError as e:
        print(e)

    try:
        depth_image = bridge.imgmsg_to_cv2(depth_msg, "passthrough")
    except CvBridgeError as e:
        print(e)

    if depth_image is None or rgb_image is None:
        return
    
    cv2.imshow("RGB Image", rgb_image)
    cv2.waitKey(50)

if __name__ == '__main__':
    # Initialize the ROS node
    rospy.init_node('program', anonymous=True)

    # Message filering for synchronizing the rgb and depth images
    rgm_sub = message_filters.Subscriber("/camera/rgb/image_raw", Image)
    depth_sub = message_filters.Subscriber("/camera/depth/image_raw", Image)

    synchronizer = message_filters.ApproximateTimeSynchronizer([rgm_sub, depth_sub], 10, 0.1)
    synchronizer.registerCallback(find_rings_callback)

    # Spin
    rospy.spin()