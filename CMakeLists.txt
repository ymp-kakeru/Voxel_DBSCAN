cmake_minimum_required(VERSION 3.5.1)
project(Voxel_DBSCAN)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_DEBUG -O1)
#set(VTK_DIR )

find_package(PCL 1.7 REQUIRED COMPONENTS
  core
  filters
  io
  segmentation
  pcl_conversions
  octree
)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  sensor_msgs
  std_msgs
  pcl_ros
  tf
  tf_conversions
)

find_package(VTK REQUIRED)

catkin_package(
)

include_directories(include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

link_directories ( 
  ${PCL_LIBRARY_DIRS}
)

add_definitions (
  ${PCL_DEFINITIONS}
)

include(${VTK_USE_FILE})

set(LIBS
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

add_executable(dbscan src/voxel_dbscan.cpp)
add_executable(test_db src/test_dbscan.cpp)

target_link_libraries(dbscan ${LIBS})
target_link_libraries(test_db ${LIBS})
