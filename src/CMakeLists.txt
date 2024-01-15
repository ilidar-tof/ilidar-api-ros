cmake_minimum_required(VERSION 2.8.3)
project(ilidar)
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14")
#add_definitions(-std=c++11) # Use C++11

find_package(OpenCV)
include_directories(${OpenCV_INCLUDE_DIRS})

find_package(catkin REQUIRED COMPONENTS
  rosconsole
  roscpp
  cv_bridge
  pcl_ros
  pcl_conversions
  sensor_msgs
  image_transport
)

set(ILIDAR_SRC_PATH "./src/")

FILE(GLOB ILIDAR_SRC 
  "${ILIDAR_SRC_PATH}/*.cpp"
  "${ILIDAR_SRC_PATH}/*.c"
)

catkin_package(
  INCLUDE_DIRS
  CATKIN_DEPENDS roscpp
                 pcl_conversions
                 pcl_ros
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${PROJECT_SOURCE_DIR}/src
)

add_executable(ilidar-roscore ilidar-ros.cpp  ${ILIDAR_SRC})

target_link_libraries(ilidar-roscore
   ${catkin_LIBRARIES}
   ${OpenCV_LIBRARIES}
)

install(TARGETS ilidar-roscore
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch sdk
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS
)

