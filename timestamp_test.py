#!/usr/bin/env python
import rospy
from sensor_msgs.msg import PointCloud2

def callback(data):
    timestamp = data.header.stamp.to_sec()
    rospy.loginfo(timestamp)

def listener():
    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber("/zoe/velodyne_points", PointCloud2, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()
