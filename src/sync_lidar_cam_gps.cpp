#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <sensor_msgs/Image.h>
#include "sensor_msgs/NavSatFix.h"
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/PointCloud2.h>


#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>

#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <fstream>
#include <sstream>    

#include <boost/filesystem.hpp>
                                       
static const std::string OPENCV_WINDOW = "Image window";

#include <ros/package.h>

using namespace sensor_msgs;
using namespace message_filters;
using namespace std;


std::string root_output_dir, lidar_output_dir, camera_output_dir, gnss_output_dir;
bool debug;

// This is passing the values by reference, not a pointer. It is a cpp thing.
void callback(const ImageConstPtr& image, const CameraInfoConstPtr& camara_info, const PointCloud2ConstPtr& point_cloud, const NavSatFixConstPtr& fix)
{
  printf("%s\n", "All babies in sync!" );

  // Convert ROS PointCloud2 to PCL point cloud
  pcl::PointCloud<pcl::PointXYZI> cloud;
  pcl::fromROSMsg(*point_cloud, cloud);

  // Create a date string from the point cloud's timestamp to use in the file name of the saved data
  const int output_size = 100;
  char output[output_size];
  std::time_t raw_time = static_cast<time_t>(point_cloud->header.stamp.sec);
  struct tm* timeinfo = localtime(&raw_time);
  std::strftime(output, output_size, "lidar_%Y_%m_%d_%H_%M_%S", timeinfo);

  // Creates a string containing the millisencods to be added to the previously created date string
  std::stringstream ss; 
  ss << std::setw(9) << std::setfill('0') << point_cloud->header.stamp.nsec;  
  const size_t fractional_second_digits = 4;
  
  // Combine all of the pieces to get the output file name
  std::string output_file = lidar_output_dir + "/" + std::string(output) + "." + ss.str().substr(0, fractional_second_digits)+".pcd";

  // Save the point cloud as a PCD file
  pcl::io::savePCDFileASCII (output_file, cloud);
  printf("%s\n", output_file.c_str() );

  // Convert the ROS image to an OpenCV image
  cv_bridge::CvImagePtr cv_ptr;
  try{
    cv_ptr = cv_bridge::toCvCopy(image, sensor_msgs::image_encodings::BGR8);
  }catch (cv_bridge::Exception& e){
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  // Update GUI Window
  if (debug){
    cv::imshow(OPENCV_WINDOW, cv_ptr->image);
    cv::waitKey(3);
  }

  // Create the filename for the image
  output_file = camera_output_dir + "/" + std::string(output) + "." + ss.str().substr(0, fractional_second_digits)+".jpg";
  
  // Save the image
  cv::imwrite(output_file, cv_ptr->image);

  // Open a file for the gnss data
  output_file = gnss_output_dir + "/" + std::string(output) + "." + ss.str().substr(0, fractional_second_digits)+".txt";
  
  ofstream myfile;
  myfile.open (output_file.c_str());
  myfile << fix->latitude << endl << fix->longitude << endl << fix->altitude;
  myfile.close();
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "lidar_and_cam_synchronizer");

  ros::NodeHandle nh("~");

  // Get the paremeters 
  nh.param("root_output_dir", root_output_dir, ros::package::getPath("shl_robot")+"/data/lidar"); 
  //nh.param("camera_output_dir", camera_output_dir, ros::package::getPath("lidar_test")+"/data/camera");
  nh.param("debug", debug, false);

  // Create the subscribers
  message_filters::Subscriber<Image> image_sub(nh, "/zed/rgb/image_rect_color", 1);
  message_filters::Subscriber<CameraInfo> camera_info_sub(nh, "/zed/rgb/camera_info", 1);
  message_filters::Subscriber<PointCloud2> point_cloud_sub(nh, "/rslidar_points", 1);
  message_filters::Subscriber<NavSatFix> gnss_sub(nh, "/ublox/fix", 1);

  // Create the synchronizer
  typedef sync_policies::ApproximateTime<Image, CameraInfo, PointCloud2, NavSatFix> MySyncPolicy;
  MySyncPolicy sp = MySyncPolicy(10);
  sp.setAgePenalty(0.5);
  // ApproximateTime takes a queue size as its constructor argument, hence MySyncPolicy(10)
  Synchronizer<MySyncPolicy> sync(MySyncPolicy(100), image_sub, camera_info_sub, point_cloud_sub, gnss_sub);
  sync.registerCallback(boost::bind(&callback, _1, _2, _3, _4));

  // Create the folder for the current run
  const int output_size = 100;
  char output[output_size];
  std::time_t raw_time = static_cast<time_t>(ros::Time::now().sec);
  struct tm* timeinfo = localtime(&raw_time);
  std::strftime(output, output_size, "run_%Y_%m_%d_%H_%M_%S", timeinfo);

  // Combine all of the pieces to get the output folder for this run
  lidar_output_dir = root_output_dir + "/" + std::string(output)+"/pcd";
  camera_output_dir = root_output_dir + "/" + std::string(output)+"/jpg";
  gnss_output_dir = root_output_dir + "/" + std::string(output)+"/gnss";

  boost::filesystem::create_directories(lidar_output_dir);
  boost::filesystem::create_directories(camera_output_dir);
  boost::filesystem::create_directories(gnss_output_dir);

  std::cout << lidar_output_dir << std::endl;

  ros::spin();

  return 0;
}