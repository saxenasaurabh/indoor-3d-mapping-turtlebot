cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
project(stitch3d)

include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

# link_directories(/usr/lib)

# MESSAGE( STATUS "PCL_LIBRARY_DIRS: " ${PCL_LIBRARY_DIRS} )
# MESSAGE( STATUS "OpenCV_DEFINITIONS : " ${OpenCV_DEFINITIONS})
# MESSAGE( STATUS "OpenCV_LIBS : " ${OpenCV_LIBS})

if (DEFINED BUILD_GRABBER)
  find_package(PCL 1.7 REQUIRED)
  find_package(OpenCV REQUIRED)
  include_directories(${PCL_INCLUDE_DIRS})
  FIND_PACKAGE(Qt4 REQUIRED)
  INCLUDE(${QT_USE_FILE})
  FIND_PACKAGE(VTK)
  INCLUDE(${VTK_USE_FILE})
  link_directories(${PCL_LIBRARY_DIRS} ${OpenCV_LIBS} ${PROJECT_SOURCE_DIR}/lib)
  add_definitions(${PCL_DEFINITIONS} ${QT_DEFINITIONS})
  link_directories(/opt/ros/groovy/lib)
  
  add_executable (mainGUI src/mainGUI.cpp src/pairwiseRegistration.cpp)
  target_link_libraries (mainGUI listener tf ${PCL_LIBRARIES} ${OpenCV_LIBS})
  
  add_executable (main src/main.cpp src/pairwiseRegistration.cpp)
  target_link_libraries (main listener tf ${PCL_LIBRARIES} ${OpenCV_LIBS})
  
  add_executable (capture3d src/capture3d.cpp)
  target_link_libraries (capture3d listener tf ${PCL_LIBRARIES})
  
  add_executable (mergeClouds src/mergeClouds.cpp)
  target_link_libraries (mergeClouds ${PCL_LIBRARIES})
  
  add_executable (clusterProfiling src/clusterProfiling.cpp)
  target_link_libraries (clusterProfiling ${PCL_LIBRARIES})
else ()
  rosbuild_init()
  rosbuild_add_library(listener src/listener.cpp src/listener.h)
endif ()