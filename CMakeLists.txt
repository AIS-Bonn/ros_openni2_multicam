cmake_minimum_required(VERSION 2.8)

project(ros_openni2_multicam)

find_package(catkin REQUIRED COMPONENTS roscpp nodelet image_transport pcl_ros)
find_package(Boost COMPONENTS thread)
find_package(OpenCV REQUIRED) #TODO change this to optional

set(OPENNI2_SDK_PATH "" CACHE STRING "Path to OpenNI SDK")
find_path(OPENNI2_ROS_INCLUDE_DIRS OpenNI.h HINTS ENV OPENNI2_INCLUDE "${OPENNI2_SDK_PATH}/Include" "/usr/local/include" "/usr/include")
find_library(OPENNI2_ROS_LIBRARIES OpenNI2  HINTS ENV OPENNI2_REDIST  "${OPENNI2_SDK_PATH}/Bin/x64-Release" "${OPENNI2_SDK_PATH}/Redist" "/usr/local/lib" "/usr/lib")

catkin_package()

if(OPENNI2_ROS_INCLUDE_DIRS AND OPENNI2_ROS_LIBRARIES)
  include_directories(${OPENNI2_ROS_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})

  add_library(ros_openni2_multicam
    src/openni_driver.cpp
    src/camera_handler.cpp
    src/image_mask.cpp
    )

  target_link_libraries(ros_openni2_multicam
    ${catkin_LIBRARIES}
    ${OPENNI2_ROS_LIBRARIES}
    ${Boost_LIBRARIES}
    ${OpenCV_LIBRARIES}
    )
else()
  message(SEND_ERROR "Could not find OpenNI SDK!")
endif()
