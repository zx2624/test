cmake_minimum_required(VERSION 3.5)
project(filter_test)
set(PCL_DIR "/usr/lib/x86_64-linux-gnu/cmake/pcl")
find_package( PCL REQUIRED )
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")


include_directories(${PROJECT_NAME} SYSTEM PUBLIC
                          ${PCL_INCLUDE_DIRS}
                          )
message("===========${PCL_LIBRARY_DIRS}\n")                          
add_executable(pcl pcl.cc)
target_link_libraries(pcl ${PCL_LIBRARIES})