#include <boost/python/list.hpp>
#include <boost/python/import.hpp>
#include <boost/python/extract.hpp>
#include <iostream>
using namespace boost::python;
class Foo {
  // blah blah...
public:
  Foo(int a, int b, std::string c...) {
    std::cout << "8888888888constructor" << std::endl;
  }
  boost::python::list bar(std::string d) {
    std::cout << "bar ..." << std::endl;
    boost::python::list list;
    list.append(d);
    list.append(d);
    list.append(d);
    list.append(d);
    boost::python::object obj = boost::python::import("t");
    std::string b = boost::python::extract<std::string>(obj.attr("b"));
    std::cout << "b is " << b << std::endl;
    return list;
  }
};

std::string duh() { return "duh!"; }