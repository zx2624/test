cmake_minimum_required(VERSION 2.8.3)
project(landmark_localization)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")


find_package(OpenCV 2 REQUIRED)
find_package(Eigen3 REQUIRED)
INCLUDE_DIRECTORIES(${Eigen_INCLUDE_DIRS})
message("========================",${EIGEN3_INCLUDE_DIR})
add_executable(tt test.cc)
target_link_libraries(tt ${OpenCV_LIBRARIES})
message("=====================" ${OpenCV_LIBRARIES})


