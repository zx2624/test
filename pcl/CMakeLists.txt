cmake_minimum_required(VERSION 3.5)
project(filter_test)
find_package( PCL REQUIRED )
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")


include_directories(${PROJECT_NAME} SYSTEM PUBLIC
                          ${PCL_INCLUDE_DIRS}
                          )
message("===========${PCL_LIBRARY_DIRS}\n")                          
foreach(lib ${PCL_LIBRARIES})
    set(name "lib${lib}.so")
    message("=== ${name}===\n")
    
endforeach(lib ${PCL_LIBRARIES})


