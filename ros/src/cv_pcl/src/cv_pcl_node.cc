#include <iostream>
#include <opencv2/opencv.hpp>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
typedef pcl::PointXYZI PointType;

void callback(const sensor_msgs::CompressedImageConstPtr &msg) {
  cv::Mat img = cv::imdecode(cv::Mat(msg->data), 1);
  cv::imshow("2", img);
  cv::waitKey(2);
}
void callback_pcl(const sensor_msgs::PointCloud2ConstPtr &msg) {}
int main(int argc, char **argv) {
  ros::init(argc, argv, "node");
  ros::NodeHandle nh;
//   ros::Subscriber sub =
//       nh.subscribe("/sensor/hugo1/image1/compressed", 10, callback);
//   ros::Subscriber sub_pcl =
//       nh.subscribe("/sensor/velodyne/points", 10, callback_pcl);
  cv::Mat im1 = cv::Mat::zeros(3, 3, CV_8UC1);
  cv::Mat im2 = cv::Mat::ones(3, 3, CV_8UC1);
  im1.at<unsigned char>(1, 2) = 12;
  cv::Mat mask1 = im1 > 0;

//  pcl::visualization::PCLVisualizer viewer("viewer");
//  viewer.addCoordinateSystem(3.0, "coor");
//  viewer.initCameraParameters();
//  // viewer.setCameraPosition(0.0, 0.0, 100.0, 0.0, -1.0, 0.0);
//  viewer.setCameraPosition(0.0, 0.0, 100.0, 0.0, 1.0, 0.0);

  std::string bag_file = argv[1];
  std::string point_topic = "/sensor/velodyne/points";
  std::string img_topic = "/sensor/hugo1/image1/segdet_track/compressed";
  rosbag::Bag bag;
  bag.open(bag_file, rosbag::bagmode::Read);
  rosbag::View points_view(bag, rosbag::TopicQuery(point_topic));
  rosbag::View::iterator pt_view_it = points_view.begin();
  rosbag::View img_view(bag, rosbag::TopicQuery(img_topic));
  rosbag::View::iterator img_view_it = img_view.begin();
  while(ros::ok() && img_view_it != img_view.end()){
    auto img_msg = img_view_it->instantiate<sensor_msgs::CompressedImage>();
    auto img = cv::imdecode(cv::Mat(img_msg->data), 1);
    cv::Mat mask = img == 16;
		std::cout << "tyep is " << mask.channels() << std::endl;
    cv::imshow("tt", mask);
    cv::waitKey(0);
  }

  // while (pt_view_it != points_view.end()) {
  //   auto pt_msg = pt_view_it->instantiate<sensor_msgs::PointCloud2>();
  //   pcl::PointCloud<PointType>::Ptr source(new pcl::PointCloud<PointType>);
  //   pcl::PointCloud<pcl::PointXYZRGB>::Ptr rgb(new pcl::PointCloud<pcl::PointXYZRGB>);
  //   pcl::fromROSMsg(*pt_msg, *source);
  //   rgb->points.resize(source->points.size());
  //   pcl::PointXYZRGB temp;
  //   for(auto pt : source->points){
  //       temp.x = pt.x;
  //       temp.y = pt.y;
  //       temp.z = pt.z;

  //       temp.r = 255;
  //       temp.g = 0;
  //       temp.b = 0;
  //       rgb->points.push_back(temp);
  //   }

  //   viewer.removePointCloud("cloud");
  //   viewer.addPointCloud(rgb, "cloud");
  //   viewer.setPointCloudRenderingProperties(
  //       pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "cloud");
  //   viewer.spinOnce(10);
  //   pt_view_it++;
  // }
//   while (ros::ok()) {
//     ros::spinOnce();
//   }
return 0;
}
