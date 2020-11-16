#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_types.h>
#include <iostream>
#include <regex>
#include <string>
#include "pcl/filters/voxel_grid.h"
#include "pcl/point_types.h"

bool getAllBagFilesPath(std::string bag_dir,
                        std::vector<std::string>& vec_bag_files_path) {
  // check if bag file path is right
  DIR* dp;
  struct dirent* dirp;
  if ((dp = opendir(bag_dir.c_str())) == NULL) {
    std::cout << "Can not open " << bag_dir << std::endl;
    return false;
  }
  if (bag_dir[bag_dir.size() - 1] != '/') {
    bag_dir += '/';
  }

  std::regex reg("(.*)(.ply)");
  // Record each bag's start time
  std::vector<std::string> vec_bag_path_temp;

  uint64_t idx = 0;
  while ((dirp = readdir(dp)) != NULL) {
    // DT_RRE means is regular file
    if (dirp->d_type == DT_REG) {
      if (std::regex_match(dirp->d_name, reg)) {
        std::string bag_name(dirp->d_name);
        std::string all_path = bag_dir + bag_name;
        // std::cout << "the name is " << bag_name << std::endl;
        vec_bag_files_path.push_back(bag_dir + bag_name);
        // std::cout << "the dir " << bag_dir << std::endl;
        // size_t pos_start = bag_name.find_last_of("_");
        // size_t pos_end = bag_name.find_last_of(".");

        // if (pos_start == -1 || pos_end == -1) {
        // 	std::cerr << "bag_name : " << bag_name << "is not valid!\n";
        // 	return false;
        // }

        // size_t num = std::stoi(bag_name.substr(pos_start + 1,
        // 	pos_end - pos_start - 1));
        // //std::cout << "num: " << num << std::endl;

        // pos_start = bag_name.find_last_of("/");
        // pos_end = bag_name.find_last_of("_");

        // //std::string pre = bag_name.substr(pos_start + 1);
        // std::string pre = bag_name.substr(pos_start + 1, pos_end - pos_start
        // - 1);
        // //std::cout << "pre: " << pre << std::endl;

        // vec_start_time_idx.push_back(TimeIdx(num, idx++, pre));
        // vec_bag_path_temp.push_back(all_path);
      }
    }
  }
}
int main() {
  pcl::PointCloud<pcl::PointXYZRGBL>::Ptr cloud(
      new pcl::PointCloud<pcl::PointXYZRGBL>);
  pcl::PLYReader reader;
  std::string path =
      "/media/horizon_ad/c46ecf39-923d-4758-89d0-26c1d08ae988/"
      "siyuan/map_data/BeiJing/20190220/sensor_20190220-112430_/"
      "used/merge/divide_pcd_label_5_test_2_new_fusion_merge_1016/";
  std::vector<std::string> files;
  getAllBagFilesPath(path, files);
  for (auto file : files) {
      pcl::PointCloud<pcl::PointXYZRGBL>::Ptr cloud_onebag(
      new pcl::PointCloud<pcl::PointXYZRGBL>);
    std::vector<std::string> str_vec;
    boost::split(str_vec, file, boost::is_any_of("/"));
    std::string name = str_vec[str_vec.size() - 1];
    if (name != "20190306-123139_4.bin.ply") continue;
    std::cout << file << std::endl;
    // str_vec.clear();
    // boost::split(str_vec, name, boost::is_any_of("_"));
    // std::string path = str_vec[0];

    if (reader.read(file,
                    *cloud_onebag) == -1)  //* load the file
    {
      PCL_ERROR("Couldn't read file\n");
      system("PAUSE");
      return (-1);
    }
    *cloud = *cloud + *cloud_onebag;
  }
  std::cout << cloud->size() << " ============== " << std::endl;
  pcl::KdTreeFLANN<pcl::PointXYZRGBL> kdtree;
  kdtree.setInputCloud(cloud);
  std::vector<float> ptvec = {15937.187, 2014.556, -10.740};
  pcl::PointXYZRGBL pt;
  pt.x = ptvec[0];
  pt.y = ptvec[1];
  pt.z = ptvec[2];
  int k = 5;
  std::vector<int> pointIdxRadiusSearch(k);
  std::vector<float> pointRadiusSquaredDistance(k);
  kdtree.nearestKSearch(pt, k, pointIdxRadiusSearch,
                        pointRadiusSquaredDistance);
  for (auto dis : pointRadiusSquaredDistance) std::cout << dis << " ";

  int size = std::pow(2, 28);
  std::cout << "++++++++++++++" << size << std::endl;
  cloud->points.resize(size);
  for (int i = 0; i < size; ++i) {
    pcl::PointXYZRGBL pt(2, 3, 4,5);
    pt.x = 1;
    pt.y = i % 1000000;
    if(i % 100000 == 0){
      std::cout << "======" << std::endl;
    }
    pt.z = 0;
    cloud->points[i] = pt;
  }
  pcl::io::savePCDFileBinary(
     "test.pcd",
      *cloud);
  std::cout << "reading ..." << std::endl;
  pcl::io::loadPCDFile("test.pcd", *cloud);
  std::cout << "--------" << cloud->size() << std::endl;
  return 0;
}