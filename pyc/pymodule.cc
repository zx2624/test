#include "pymodule.h"  
#include <boost/python.hpp>

// your stuff

// use this name space so that you can call the following function
using namespace boost::python;

BOOST_PYTHON_MODULE(foo)  // foo will be the name you import in python
{
    class_<Foo>("Foo",   
                 init<int>())  // constructor args
        .def("bar", &Foo::bar)  // member function
        .def("DumpCloud", &Foo::DumpCloud)
    ;
    
    def("duh", duh);  // other functions
}