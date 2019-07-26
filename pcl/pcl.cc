#include "pcl/filters/voxel_grid.h"
#include "pcl/point_types.h"


int main()
{
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(
      new pcl::PointCloud<pcl::PointXYZRGB>);

  cloud->width    = 50000;
  cloud->height   = 1;
  cloud->is_dense = false;
  cloud->points.resize(cloud->width * cloud->height);

  for (size_t i = 0; i < cloud->points.size(); ++i)
  {
    cloud->points[i].x = 1024 * rand() / (RAND_MAX + 1.0f);
    cloud->points[i].y = 1024 * rand() / (RAND_MAX + 1.0f);
    cloud->points[i].z = 1024 * rand() / (RAND_MAX + 1.0f);

    cloud->points[i].r = 255 * rand() / (RAND_MAX + 1.0f);
    cloud->points[i].g = 255 * rand() / (RAND_MAX + 1.0f);
    cloud->points[i].b = 255 * rand() / (RAND_MAX + 1.0f);
    cloud->points[i].a = 0;
  }

  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloudA(
      new pcl::PointCloud<pcl::PointXYZRGBA>());

  pcl::copyPointCloud(*cloud, *cloudA);

  pcl::VoxelGrid<pcl::PointXYZRGBA> grid;
  grid.setLeafSize(0.01f, 0.01f, 0.01f);
  grid.setInputCloud(cloudA);
  grid.filter(*cloudA);
  return 0;
}