#include <boost/python/extract.hpp>
#include <boost/python/import.hpp>
#include <boost/python/list.hpp>
#include <boost/python/numpy.hpp>
#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include <iostream>
using namespace boost::python;
class Foo {
  // blah blah...
public:
  Foo(int a) { 
    numpy::initialize();
  }
  boost::python::object bar(boost::python::object obj) {
    std::cout << "bar ..." << std::endl;
    std::string str =
        boost::python::extract<std::string>(obj.attr("root_path"));
    std::cout << "root path" << str << std::endl;
    str = boost::python::extract<std::string>(obj.attr("data_type"));
    std::cout << "data type" << str << std::endl;
    boost::python::object file = boost::python::import("label_cloud");
    boost::python::object LabeledPointCloud = extract<object>(file.attr("LabeledPointCloud"));
    object newcloud = LabeledPointCloud();
    newcloud.attr("root_path") = "newpath";
    return newcloud;
  }
  void DumpCloud(boost::python::numpy::ndarray data) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(
        new pcl::PointCloud<pcl::PointXYZ>);
    const tuple shape = extract<tuple>(data.attr("shape"));
    int h = extract<int>(shape[0]);
    int w = extract<int>(shape[1]);
    for (int i = 0; i < h; ++i) {
      float a = extract<float>(data[make_tuple(i, 0)]);
      float b = extract<float>(data[make_tuple(i, 1)]);
      float c = extract<float>(data[make_tuple(i, 2)]);
      pcl::PointXYZ pt(a, b, c);
      cloud->push_back(pt);
    }
    pcl::io::savePCDFileASCII("tmp.pcd", *cloud);
  }
};

std::string duh() { return "duh!"; }