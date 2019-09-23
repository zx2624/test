#include <iostream>
#include <opencv2/opencv.hpp>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <tf/tf.h>

#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <nav_msgs/Odometry.h>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
typedef pcl::PointXYZI PointType;
#define VIEWER

int main(int argc, char **argv)
{
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
  std::cout << "topic: " << argv[2] << std::endl;
  std::string bag_file = argv[1];
  std::string point_topic = "/sensor/velodyne/points/label_5_test_2/new";
  std::string img_topic = "/sensor/hugo1/image1/segdet_track/compressed";
  std::string img_raw = "/sensor/hugo1/image1/compressed";
  std::string odom_topic = argv[2];

  rosbag::Bag bag;
  bag.open(bag_file, rosbag::bagmode::Read);

  std::vector<nav_msgs::Odometry> vec_rpy;
  std::vector<nav_msgs::Odometry> vec_wrong;
  rosbag::View points_view(bag, rosbag::TopicQuery(point_topic));
  rosbag::View odom_view(bag, rosbag::TopicQuery(odom_topic));
  rosbag::View::iterator pt_view_it = points_view.begin();
  rosbag::View::iterator odom_view_it = odom_view.begin();
  bool is_first = true;
  double init_x, init_y, init_z;
  while (ros::ok() && odom_view_it != odom_view.end())
  {
    auto odom_msg = odom_view_it->instantiate<nav_msgs::Odometry>();
    nav_msgs::Odometry msg_rpy;
    msg_rpy.header = odom_msg->header;
    msg_rpy.header.frame_id = "velodyne";
    Eigen::Quaterniond quat;
    quat.x() = odom_msg->pose.pose.orientation.x;
    quat.y() = odom_msg->pose.pose.orientation.y;
    quat.z() = odom_msg->pose.pose.orientation.z;
    quat.w() = odom_msg->pose.pose.orientation.w;
    Eigen::Vector3d rpy;
    rpy = quat.matrix().eulerAngles(2, 1, 0);
    // std::cout << "msg is " << quat.coeffs() << std::endl;
    Eigen::Matrix3d mat_rpy =
        Eigen::AngleAxisd(rpy(0), Eigen::Vector3d::UnitZ()) *
        Eigen::AngleAxisd(rpy(1), Eigen::Vector3d::UnitX()) *
        Eigen::AngleAxisd(rpy(2), Eigen::Vector3d::UnitY()).matrix();
    Eigen::Quaterniond q_rpy(mat_rpy);

    Eigen::Matrix3d mat_wrong =
        Eigen::AngleAxisd(rpy(0), Eigen::Vector3d::UnitZ()) *
        Eigen::AngleAxisd(rpy(1), Eigen::Vector3d::UnitY()) *
        Eigen::AngleAxisd(rpy(2), Eigen::Vector3d::UnitX()).matrix();
    Eigen::Quaterniond q_wrong(mat_wrong);
    std::cout << "wrong coeffs : " << q_wrong.coeffs() << std::endl;


    msg_rpy.pose.pose.orientation.x = q_rpy.x();
    msg_rpy.pose.pose.orientation.y = q_rpy.y();
    msg_rpy.pose.pose.orientation.z = q_rpy.z();
    msg_rpy.pose.pose.orientation.w = q_rpy.w();

    msg_rpy.pose.pose.position.x = 0;
    msg_rpy.pose.pose.position.y = 0;
    msg_rpy.pose.pose.position.z = 0;

    vec_rpy.push_back(msg_rpy);

    msg_rpy.pose.pose.orientation.x = q_wrong.x();
    msg_rpy.pose.pose.orientation.y = q_wrong.y();
    msg_rpy.pose.pose.orientation.z = q_wrong.z();
    msg_rpy.pose.pose.orientation.w = q_wrong.w();


    vec_wrong.push_back(msg_rpy);

    std::cout << msg_rpy.header.stamp << std::endl;
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
  std::cout << "--" << std::endl;
  for (int i = 0; i < vec_rpy.size(); ++i)
  {
    auto msg_rpy = vec_rpy[i];
    auto msg_wrong = vec_wrong[i];
    // bag.write("/odom_rpy_check", msg_rpy.header.stamp, msg_rpy);
    bag.write("/odom_rpy_check", msg_rpy.header.stamp, msg_wrong);
  
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
  //0.0128760916	0.0212256319	3.4143890528

  // std::cout << "q_after norm " << q_after.norm() << std::endl;
  //   tf::Matrix3x3(tf::Quaternion(q_after.x(), q_after.y(),
  //                              q_after.z(),
  //                              q_after.w()))
  //     .getRPY(rpy_raw(0), rpy_raw(1), rpy_raw(2));
  // std::cout << "the rpy--------------------- is \n" << rpy_raw << std::endl;
  //   Eigen::Vector3d x(1,0,0);

  // Eigen::Quaterniond q_z(Eigen::AngleAxisd(-3.141592653 / 4, Eigen::Vector3d(0,0,1)));
  // std::cout << q_z * x<< std::endl;
  // Eigen::Quaterniond q_z2(Eigen::AngleAxisd(-3.141592653 / 4, Eigen::Vector3d(0,0,1)));
  // std::cout << (q_z*q_z2 ).matrix()<< std::endl;
  // std::cout << (q_z*q_z2 ).toRotationMatrix()<< std::endl;

  // std::cout << q_z2.inverse()*q_z.inverse()*z_ax << std::endl;

  //   ros::Subscriber sub =
  //       nh.subscribe("/sensor/hugo1/image1/compressed", 10, callback);
  //   ros::Subscriber sub_pcl =
  //       nh.subscribe("/sensor/velodyne/points", 10, callback_pcl);
  //   cv::Mat im1 = cv::Mat::zeros(3, 3, CV_8UC1);
  //   cv::Mat im2 = cv::Mat::ones(3, 3, CV_8UC1);
  //   im1.at<unsigned char>(1, 2) = 12;
  //   cv::Mat mask1 = im1 > 0;

  //  pcl::visualization::PCLVisualizer viewer("viewer");
  //  viewer.addCoordinateSystem(3.0, "coor");
  //  viewer.initCameraParameters();
  //  // viewer.setCameraPosition(0.0, 0.0, 100.0, 0.0, -1.0, 0.0);
  //  viewer.setCameraPosition(0.0, 0.0, 100.0, 0.0, 1.0, 0.0);

  //   std::string bag_file = argv[1];
  //   std::string point_topic = argv[2];
  //   std::string img_topic = "/sensor/hugo1/image1/segdet_track/compressed";
  //   rosbag::Bag bag;
  //   bag.open(bag_file, rosbag::bagmode::Read);
  //   rosbag::View points_view(bag, rosbag::TopicQuery(point_topic));
  //   rosbag::View::iterator pt_view_it = points_view.begin();
  //   rosbag::View img_view(bag, rosbag::TopicQuery(img_topic));
  //   rosbag::View::iterator img_view_it = img_view.begin();
  //   // while(ros::ok() && img_view_it != img_view.end()){
  //   //   auto img_msg = img_view_it->instantiate<sensor_msgs::CompressedImage>();
  //   //   auto img = cv::imdecode(cv::Mat(img_msg->data), 1);
  //   //   cv::Mat mask = img == 16;
  // 	// 	std::cout << "tyep is " << mask.channels() << std::endl;
  //   //   cv::imshow("tt", mask);
  //   //   cv::waitKey(2);
  //   //   img_view_it++;
  //   // }

  //   while (pt_view_it != points_view.end()&& ros::ok()) {
  //     auto pt_msg = pt_view_it->instantiate<sensor_msgs::PointCloud2>();
  //     pcl::PointCloud<PointType>::Ptr source(new pcl::PointCloud<PointType>);
  //     pcl::PointCloud<pcl::PointXYZRGB>::Ptr rgb(new pcl::PointCloud<pcl::PointXYZRGB>);
  //     pcl::fromROSMsg(*pt_msg, *source);
  //     rgb->points.resize(source->points.size());
  //     pcl::PointXYZRGB temp;
  //     for(auto pt : source->points){
  //         temp.x = pt.x;
  //         temp.y = pt.y;
  //         temp.z = pt.z;

  //         temp.r = 255;
  //         temp.g = 0;
  //         temp.b = 0;
  //         rgb->points.push_back(temp);
  //     }

  //     viewer.removePointCloud("cloud");
  //     viewer.addPointCloud(rgb, "cloud");
  //     viewer.setPointCloudRenderingProperties(
  //         pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "cloud");
  //     viewer.spinOnce(10);
  //     pt_view_it++;
  //   }
  //   while (ros::ok()) {
  //     ros::spinOnce();
  //   }
  return 0;
}
