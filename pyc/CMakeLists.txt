cmake_minimum_required(VERSION 2.8.3)
project(pyc)

FIND_PACKAGE(PythonInterp  REQUIRED)
FIND_PACKAGE(PythonLibs  REQUIRED)
FIND_PACKAGE(Boost COMPONENTS python numpy)
set(PCL_DIR /usr/local/share/pcl-1.8)
find_package( PCL REQUIRED )

INCLUDE_DIRECTORIES(${Boost_INCLUDE_DIRS} ${PYTHON_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS})
message(================ ${PYTHON_INCLUDE_DIRS})
LINK_LIBRARIES(${Boost_LIBRARIES} ${PYTHON_LIBRARIES} ${PCL_LIBRARIES})
message(====pcl======  ${PCL_LIBRARIES})
message(==========  ${PYTHON_LIBRARIES})
message(=====boost=====  ${Boost_LIBRARIES})

add_library(foo SHARED
    pymodule.cc)

# set prefix otherwise the output will be libfoo.so instead of foo.so
set_target_properties(foo PROPERTIES PREFIX "")

target_link_libraries(foo
    ${Boost_LIBRARIES}
    ${PYTHON_LIBRARIES}
    ${PCL_LIBRARIES}
    )