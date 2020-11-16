cmake_minimum_required(VERSION 2.8.3)
project(landmark_localization)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -g")
SET(CMAKE_BUILD_TYPE "Debug")  # 定义编译类型
SET(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -O0 -Wall -g2 -ggdb") # 定义Debug编译参数
add_library(testlib SHARED testlib.cc)
find_package(OpenCV  REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Qt5 COMPONENTS Test Concurrent Gui Widgets OpenGL)
find_package(Ceres)
####pcl
set(PCL_DIR "/usr/lib/x86_64-linux-gnu/cmake/pcl")
find_package( PCL REQUIRED )


find_package( GTSAM REQUIRED )

INCLUDE_DIRECTORIES(${Eigen_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS} ${GTSAM_INCLUDE_DIR})

add_executable(tt test.cc)
target_link_libraries(tt ${OpenCV_LIBRARIES} testlib ${PCL_LIBRARIES} ${CERES_LIBRARIES} -lgtsam)

add_library(prior_pose_cali SHARED  prior_pose_cali.cc)
target_link_libraries(prior_pose_cali gtsam)
add_executable(hand_eye_gtsam hand_eye_gtsam.cc)
target_link_libraries(hand_eye_gtsam ${OpenCV_LIBRARIES}  ${PCL_LIBRARIES} ${CERES_LIBRARIES} -lgtsam prior_pose_cali)

add_executable(can_imu_handeye can_imu_handeye_simu.cc)
target_link_libraries(can_imu_handeye ${OpenCV_LIBRARIES} testlib ${PCL_LIBRARIES} ${CERES_LIBRARIES})

add_executable(from_lidar from_lidar.cc)
target_link_libraries(from_lidar yaml-cpp ${OpenCV_LIBRARIES} jsoncpp)

add_executable(from_front from_front.cc)
target_link_libraries(from_front yaml-cpp ${OpenCV_LIBRARIES} jsoncpp)

