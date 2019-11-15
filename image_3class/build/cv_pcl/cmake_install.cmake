# Install script for directory: /media/horizon_ad/c46ecf39-923d-4758-89d0-26c1d08ae988/zx/test/image_3class/src/cv_pcl

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/media/horizon_ad/c46ecf39-923d-4758-89d0-26c1d08ae988/zx/test/image_3class/install")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/media/horizon_ad/c46ecf39-923d-4758-89d0-26c1d08ae988/zx/test/image_3class/build/cv_pcl/catkin_generated/installspace/cv_pcl.pc")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/cv_pcl/cmake" TYPE FILE FILES
    "/media/horizon_ad/c46ecf39-923d-4758-89d0-26c1d08ae988/zx/test/image_3class/build/cv_pcl/catkin_generated/installspace/cv_pclConfig.cmake"
    "/media/horizon_ad/c46ecf39-923d-4758-89d0-26c1d08ae988/zx/test/image_3class/build/cv_pcl/catkin_generated/installspace/cv_pclConfig-version.cmake"
    )
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/cv_pcl" TYPE FILE FILES "/media/horizon_ad/c46ecf39-923d-4758-89d0-26c1d08ae988/zx/test/image_3class/src/cv_pcl/package.xml")
endif()

