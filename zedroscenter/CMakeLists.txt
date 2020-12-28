cmake_minimum_required(VERSION 2.8.3)
project(zedroscenter)

add_compile_options(-std=c++14 -g -O3) #(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  cv_bridge
  image_transport
  pcl_ros
  sensor_msgs
)

find_package(OpenCV REQUIRED)
include_directories(${OPENCV_INCLUDE_DIR})
find_package(CUDA ${ZED_CUDA_VERSION} EXACT REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES zedroscenter
  CATKIN_DEPENDS roscpp rospy std_msgs
  DEPENDS system_lib
)


include_directories ("/home/user/catkin_ws/opencv-3.4.0/include")
include_directories("/usr/local/zed/include")
include_directories( include ${catkin_INCLUDE_DIRS})# ${Boost_INCLUDE_DIRS} ${GSTREAMER_INCLUDE_DIRS})
include_directories(${Eigen_INCLUDE_DIRS})
link_directories(${catkin_LIBRARY_DIRS})# ${Eigen_LIBRARY_DIRS})

include_directories(${CUDA_INCLUDE_DIRS})
include_directories(${ZED_INCLUDE_DIRS})
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)

#link_directories("/usr/local/zed/lib")
link_directories(${ZED_LIBRARY_DIR})
link_directories(${OpenCV_LIBRARY_DIRS})
link_directories(${CUDA_LIBRARY_DIRS})

include_directories ("${PROJECT_SOURCE_DIR}/include/zedroscenter")

#add_library(${PROJECT_NAME} src/${PROJECT_NAME}/zedroscenter_node.cpp)

add_executable(zedroscenter_node src/zedroscenter_node.cpp src/centroid-tracking.cpp) # src/SaveDepth.cpp)
target_link_libraries(zedroscenter_node ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${execName} ${ZED_LIBS})# ${Eigen_LIBRARIES} )
add_dependencies(zedroscenter_node ${LIBS} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} ${catkin_LIBRARIES})#  ${Eigen_LIBRARIES})

if (LINK_SHARED_ZED)
    SET(ZED_LIBS ${ZED_LIBRARIES} ${CUDA_CUDA_LIBRARY} ${CUDA_CUDART_LIBRARY})
else()
    SET(ZED_LIBS ${ZED_STATIC_LIBRARIES} ${CUDA_CUDA_LIBRARY} ${CUDA_LIBRARY})
endif()




