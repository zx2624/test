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
#include <nav_msgs/Odometry.h>
typedef pcl::PointXYZI PointType;
#define VIEWER


int main(int argc, char **argv) {
  ros::init(argc, argv, "node");
  ros::NodeHandle nh;
//   ros::Subscriber sub =
//       nh.subscribe("/sensor/hugo1/image1/compressed", 10, callback);
//   ros::Subscriber sub_pcl =
//       nh.subscribe("/sensor/velodyne/points", 10, callback_pcl);
  // cv::Mat im1 = cv::Mat::zeros(3, 3, CV_8UC1);
  // cv::Mat im2 = cv::Mat::ones(3, 3, CV_8UC1);
  // im1.at<unsigned char>(1, 2) = 12;
  // cv::Mat mask1 = im1 > 0;
  int a = 0;
#ifdef VIEWER
//  pcl::visualization::PCLVisualizer viewer("viewer");
//  viewer.addCoordinateSystem(3.0, "coor");
//  viewer.initCameraParameters();
//  // viewer.setCameraPosition(0.0, 0.0, 100.0, 0.0, -1.0, 0.0);
//  viewer.setCameraPosition(0.0, 0.0, 100.0, 0.0, 1.0, 0.0);
#endif //VIEWER
  std::cout << "bag: " << argv[1] << std::endl;
  std::cout << "topic: "  << argv[2] << std::endl;
  std::string bag_file = argv[1];
  std::string point_topic = "/sensor/velodyne/points/label_5_test_2/new";
  std::string img_topic = "/sensor/hugo1/image1/segdet_track/compressed";
  std::string img_raw = "/sensor/hugo1/image1/compressed";
  std::string odom_topic = argv[2];

  rosbag::Bag bag;
  bag.open(bag_file, rosbag::bagmode::Read);

  std::vector<nav_msgs::Odometry> vec_nav;
  rosbag::View points_view(bag, rosbag::TopicQuery(point_topic));
  rosbag::View odom_view(bag, rosbag::TopicQuery(odom_topic));
  rosbag::View::iterator pt_view_it = points_view.begin();
  rosbag::View::iterator odom_view_it = odom_view.begin();
  while (ros::ok() && odom_view_it != odom_view.end())
  {
    auto odom_msg = odom_view_it->instantiate<nav_msgs::Odometry>();
nav_msgs::Odometry msg_write;
msg_write.header = odom_msg->header;
msg_write.header.frame_id = "velodyne";
msg_write.pose = odom_msg->pose;
msg_write.pose.pose.position.x=0;
msg_write.pose.pose.position.y=0;
msg_write.pose.pose.position.z=0;

vec_nav.push_back(msg_write);
std::cout << msg_write.header.stamp << std::endl;
    // for(auto pt : source->points){
    //     temp.x = pt.x;
    //     temp.y = pt.y;
    //     temp.z = pt.z;

    //     temp.r = 255;
    //     temp.g = 0;
    //     temp.b = 0;
    //     rgb->points.push_back(temp);
    // }

    // viewer.removePointCloud("cloud");
    // viewer.addPointCloud(rgb, "cloud");
    // viewer.setPointCloudRenderingProperties(
    //     pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "cloud");
    // viewer.spinOnce(10);
    odom_view_it++;
  }
  bag.close();
  bag.open(bag_file, rosbag::BagMode::Append);
  std::cout <<"--" << std::endl;
  for(auto msg : vec_nav){
    bag.write("/check_odom_3", msg.header.stamp, msg);
  }
  bag.close();
  // rosbag::View img_view(bag, rosbag::TopicQuery(img_topic));
  // rosbag::View::iterator img_view_it = img_view.begin();
  //   rosbag::View img_view_raw(bag, rosbag::TopicQuery(img_raw));
  // rosbag::View::iterator img_view_it_raw = img_view_raw.begin();
  // while(ros::ok() && img_view_it != img_view.end()){
  //   auto img_msg = img_view_it->instantiate<sensor_msgs::CompressedImage>();
  //   auto img = cv::imdecode(cv::Mat(img_msg->data), 1);
  //   auto raw_msg = img_view_it_raw->instantiate<sensor_msgs::CompressedImage>();
  //   auto raw_img = cv::imdecode(cv::Mat(raw_msg->data), 1);
  //   auto raw_stamp = raw_msg->header.stamp.toSec();
  //   auto img_stamp = img_msg->header.stamp.toSec();
  //   while(raw_stamp - img_stamp > 0.001){
  //     img_view_it++;
  //     img_msg = img_view_it->instantiate<sensor_msgs::CompressedImage>();
  //     img_stamp = img_msg->header.stamp.toSec();

  //   }
  //   while(img_stamp - raw_stamp > 0.001){
  //     img_view_it_raw++;
  //     raw_msg = img_view_it_raw->instantiate<sensor_msgs::CompressedImage>();
  //     raw_stamp = raw_msg->header.stamp.toSec();
  //   }

  //   cv::Mat mask = img == 16;
  //   for(int i = 0; i < raw_img.rows; ++i){
  //     for(int j = 0; j < raw_img.cols; ++j){
  //       auto val = img.ptr<cv::Vec3b>(i)[j][0];
  //       if(val == 16){
  //         raw_img.ptr<cv::Vec3b>(i)[j][0] = 255;
  //         raw_img.ptr<cv::Vec3b>(i)[j][1] = 0;
  //         raw_img.ptr<cv::Vec3b>(i)[j][2] = 0;
  //       }

  //     }
  //   }
  //   cv::imshow("tt", raw_img);
  //   cv::waitKey(2);
  //   img_view_it++;
  //   img_view_it_raw++;
  // }
#ifdef VIEWER
  // while (ros::ok() && pt_view_it != points_view.end()) {
  //   auto pt_msg = pt_view_it->instantiate<sensor_msgs::PointCloud2>();
  //   pcl::PointCloud<pcl::Label>::Ptr source(new pcl::PointCloud<pcl::Label>);
  //   pcl::PointCloud<pcl::PointXYZRGB>::Ptr rgb(new pcl::PointCloud<pcl::PointXYZRGB>);
  //   pcl::fromROSMsg(*pt_msg, *source);
  //   rgb->points.resize(source->points.size());
  //   pcl::PointXYZRGB temp;
  //   std::cout << source->size() << std::endl;
  //   // for(auto pt : source->points){
  //   //     temp.x = pt.x;
  //   //     temp.y = pt.y;
  //   //     temp.z = pt.z;

  //   //     temp.r = 255;
  //   //     temp.g = 0;
  //   //     temp.b = 0;
  //   //     rgb->points.push_back(temp);
  //   // }

  //   // viewer.removePointCloud("cloud");
  //   // viewer.addPointCloud(rgb, "cloud");
  //   // viewer.setPointCloudRenderingProperties(
  //   //     pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "cloud");
  //   // viewer.spinOnce(10);
  //   pt_view_it++;
  // }
  #endif //VIEWER
//   while (ros::ok()) {
//     ros::spinOnce();
//   }
return 0;
}
