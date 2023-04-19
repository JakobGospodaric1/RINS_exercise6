#!/usr/bin/env python3
# Import ROS libraries and messages
import rospy

# import the PointCloud2 message
from sensor_msgs.msg import PointCloud2

def callback_cylinder(msg):
    # Print the point cloud
    rospy.loginfo(msg.row_step)

if __name__ == '__main__':
    # Initialize the ROS node
    rospy.init_node('program', anonymous=True)

    # Create a ROS subscriber with the desired topic and message type
    sub = rospy.Subscriber("/cylinder", PointCloud2, callback_cylinder)

    # Spin
    rospy.spin()