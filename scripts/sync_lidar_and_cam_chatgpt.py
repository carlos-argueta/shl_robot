# First, let's import the necessary libraries
import rospy
import os
from sensor_msgs.msg import PointCloud2, Image
from cv_bridge import CvBridge
import cv2
from message_filters import TimeSynchronizer, Subscriber

# Next, let's define the callback function for the synchronized pointcloud and camera data
def sync_callback(pointcloud_data, camera_data):
    print("Sync")
    # Save the timestamps of the pointcloud and camera messages
    pointcloud_timestamp = pointcloud_data.header.stamp
    camera_timestamp = camera_data.header.stamp

    # Find the folder name based on the timestamp of either the pointcloud or camera message
    folder_timestamp = min(pointcloud_timestamp, camera_timestamp)
    folder_name = '{}_{}'.format(pointcloud_timestamp.secs, camera_timestamp.secs)

    # Create a folder to save the synchronized data
    if not os.path.exists(folder_name):
        os.makedirs(folder_name)

    # Save the synchronized data in the created folder
    with open(os.path.join(folder_name, 'pointcloud.txt'), 'w') as f:
        f.write(str(pointcloud_data.data))
    cv_image = CvBridge().imgmsg_to_cv2(camera_data, 'bgr8')
    cv2.imwrite(os.path.join(folder_name, 'image.jpg'), cv_image)

# Now, let's create the ROS node
rospy.init_node('pointcloud_camera_synchronizer')

# Subscribe to the pointcloud and camera topics using the TimeSynchronizer
pointcloud_sub = Subscriber('/rslidar_points', PointCloud2)
camera_sub = Subscriber('/zed/rgb/image_rect_color', Image)
sync =  ApproximateTimeSynchronizer([pointcloud_sub, camera_sub], 10, 0.1)
sync.registerCallback(sync_callback)

# Spin to keep the node running
rospy.spin()
