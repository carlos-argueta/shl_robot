#!/usr/bin/env python

import rospy

import message_filters

from sensor_msgs.msg import Image, CameraInfo
from sensor_msgs.msg import PointCloud2

import numpy as np
import ros_numpy

import cv2
from cv_bridge import CvBridge

from datetime import datetime

import os

def pointcloud2_to_xyz(cloud_msg, file_path):
    """Convert a ROS PointCloud2 message to a XYZ file.

    Parameters
    ----------
    cloud_msg : sensor_msgs.PointCloud2
        The ROS PointCloud2 message to convert.
    file_path : str
        The file path to save the XYZ file.

    Returns
    -------
    bool
        True if the conversion was successful, False otherwise.
    """
    # Convert the PointCloud2 message to a NumPy array
    cloud_arr = ros_numpy.numpify(cloud_msg)

    # Extract the points from the NumPy array
    points = cloud_arr[['x', 'y', 'z']].view(np.float32).reshape(-1, 3)

    # Create the XYZ file
    with open(file_path, 'w') as f:
        # Write the points to the file
        for point in points:
            f.write('{} {} {}\n'.format(point[0], point[1], point[2]))

    return True

def sync_callback(lidar_data, camera_data, camera_info_sub):
  # The callback processing the set of topics that arrived at approximately the same time
  print("In sync!")
  print("Lidar",lidar_data.header.stamp,datetime.utcfromtimestamp(lidar_data.header.stamp.to_sec()))
  print("Zed",camera_data.header.stamp,datetime.utcfromtimestamp(camera_data.header.stamp.to_sec()))

  # Save the timestamps of the pointcloud and camera messages
  pointcloud_timestamp = lidar_data.header.stamp
  camera_timestamp = camera_data.header.stamp

  # Find the folder name based on the timestamp of either the pointcloud or camera message
  folder_timestamp = min(pointcloud_timestamp, camera_timestamp)
  folder_name = '{}_{}'.format(pointcloud_timestamp.secs, camera_timestamp.secs)

  # Create a folder to save the synchronized data
  if not os.path.exists(folder_name):
      os.makedirs(folder_name)

  # Save the synchronized data in the created folder
  # Convert the PointCloud2 message to an XYZ file
  success = pointcloud2_to_xyz(lidar_data, os.path.join(folder_name, 'pointcloud.xyz'),)

  if success:
    print('PointCloud2 message successfully saved to XYZ file!')
  else:
    print('Error saving PointCloud2 message to XYZ file.')

  cv_image = CvBridge().imgmsg_to_cv2(camera_data, 'bgr8')
  cv2.imwrite(os.path.join(folder_name, 'image.jpg'), cv_image)


def lidar_callback(data):
  print("Lidar",data.header.stamp,datetime.utcfromtimestamp(data.header.stamp.to_sec()))

def camera_callback(data):
  print("Zed",data.header.stamp,datetime.utcfromtimestamp(data.header.stamp.to_sec()))
  
if __name__ == '__main__':
    try:
      rospy.init_node('lidar_and_camera_synchronizer', anonymous=True)

      lidar_sub = message_filters.Subscriber('/rslidar_points', PointCloud2)
      image_sub = message_filters.Subscriber('/zed/rgb/image_rect_color', Image)
      camera_info_sub = message_filters.Subscriber('/zed/rgb/camera_info', CameraInfo)

      rospy.Subscriber('/rslidar_points', PointCloud2, lidar_callback)
      rospy.Subscriber('/zed/rgb/image_rect_color', Image, camera_callback)

      ts = message_filters.ApproximateTimeSynchronizer([lidar_sub, image_sub, camera_info_sub], queue_size = 10, slop = 0.05, allow_headerless=True)
      ts.registerCallback(sync_callback)
      
      rospy.spin()
    except rospy.ROSInterruptException:
      print("Bye!")