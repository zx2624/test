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
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
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
//0.0128760916	0.0212256319	3.4143890528

  double yaw = 1.4143890528, pitch = 0.0212256319, roll = 0.0128760916;
  //0104
  // double yaw = 1.4330470610715276, pitch =  -1.493158152010737, roll = 0.1380770137450042;
  Eigen::Vector3d rpy_raw;
// z y x
  Eigen::Matrix3d mat =
      Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *
      Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
      Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX()).matrix();
  std::cout << mat << std::endl;
  std::cout << "==============\n";
    Eigen::Matrix3d mat_pry =
          Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
    Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX())*
      Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()).matrix();
std::cout << mat_pry << std::endl;
  std::cout << "==============\n";
std::cout << mat_pry-mat << std::endl;

// z x y
    // Eigen::Matrix3d mat =
    //   Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *
    //   Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX()) *
    //   Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()).matrix();
//y x z
    // Eigen::Matrix3d mat =
    //   Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
    //   Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX()) *
    //   Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()).matrix();
  std::cout << "eigen =======================" << std::endl;
  std::cout << mat.eulerAngles(2, 1, 0) << "\n";
  std::cout << mat_pry.eulerAngles(2, 1, 0) << std::endl;
  // std::cout << mat.eulerAngles(2, 0, 1) << "\neuler\n";

  // Eigen::Matrix3d mat =
  //     Eigen::AngleAxisd(yaw, Eigen::Vector3d(1, 2, 3)).matrix();
  Eigen::Quaterniond quat(mat);
  // quat.normalize();
  // std::cout << "==" << quat.norm() << std::endl;
  // std::cout << quat.x() << " " << quat.y() << " " << quat.z() << std::endl;
  tf::Matrix3x3(tf::Quaternion(quat.x(), quat.y(),
                               quat.z(),
                               quat.w()))
      .getRPY(rpy_raw(0), rpy_raw(1), rpy_raw(2));
  std::cout << "tf ================= \n" << rpy_raw << std::endl;
  std::cout << "tf ================= \n"  << std::endl;


  Eigen::Vector3d z_ax(0,0,1);
std::cout << quat.inverse()*z_ax << " inverse"<< std::endl;
std::cout << quat*z_ax << " inverse"<< std::endl;
Eigen::Quaterniond deltaq(Eigen::AngleAxisd(-yaw, z_ax));
std::cout << deltaq.norm() << " is the norm "<< std::endl;
  Eigen::Quaterniond q_after =( deltaq*quat);
  std::cout << (q_after.matrix()).eulerAngles(2,1, 0) * 57.32<< std::endl;
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
