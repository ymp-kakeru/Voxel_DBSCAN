cmake_minimum_required(VERSION 3.5.1)
project(Voxel_DBSCAN)

set(CMAKE_CXX_STANDARD 14)
#set(-DCMAKE_BUILD_TYPE=Debug)
#set(VTK_DIR )

find_package(PCL 1.8 REQUIRED COMPONENTS
  filters
  io
  segmentation
  pcl_conversions
  octree
  visualization
)

#[[find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  sensor_msgs
  std_msgs
  tf
  tf_conversions
)]]

find_package(VTK 6 REQUIRED
  vtkRenderingCore
  vtkhdf5
)

#catkin_package()

include_directories(include
#  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${VTK_INCLUDE_DIRS}
)

link_directories ( 
  ${PCL_LIBRARY_DIRS}
  ${VTK_LIBRARY_DIRS}
)

add_definitions (
  ${PCL_DEFINITIONS}
  ${VTK_DEFINITIONS}
)

include(${VTK_USE_FILE})

set(LIBS
#  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${VTK_LIBRARIES}
)

add_executable(dbscan src/voxel_dbscan.cpp)
add_executable(test_db src/test_dbscan.cpp)
add_executable(octree_viewer src/octree_viewer.cpp)


target_link_libraries(dbscan ${LIBS})
target_link_libraries(test_db ${LIBS})
target_link_libraries(octree_viewer ${LIBS})
