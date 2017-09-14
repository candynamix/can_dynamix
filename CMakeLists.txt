cmake_minimum_required(VERSION 2.8.3)
project(can_dynamix)



find_package(catkin REQUIRED COMPONENTS
  roscpp
  genmsg
  cv_bridge 
  image_transport
)


find_package(OpenCV REQUIRED)
message(STATUS "OpenCV VERSION: ${OpenCV_VERSION}")
message(STATUS "OpenCV Components: ${OpenCV_LIB_COMPONENTS}")
if(OpenCV_VERSION VERSION_LESS "3.0" OR TARGET opencv_optflow)
  set(OPENCV_HAVE_OPTFLOW TRUE)
endif()
# Supporting CompressedImage in cv_bridge has been started from 1.11.9 (2015-11-29)
# note : hydro 1.10.18, indigo : 1.11.13 ...
# https://github.com/ros-perception/vision_opencv/pull/70
if(cv_bridge_VERSION VERSION_LESS "1.11.9")
  add_definitions("-DCV_BRIDGE_COMPRESSED_IMAGE_IS_NOT_SUPPORTED")
endif()
# Supporting CvtColorForDisplay in cv_bridge has been started from 1.11.9 (2015-11-29)
if(cv_bridge_VERSION VERSION_LESS "1.11.9")
  add_definitions("-DCV_BRIDGE_CVT_COLOR_FOR_DISPLAY_IS_NOT_SUPPORTED")
endif()
# Supporting CvtColorForDisplayOptions in cv_bridge has been started from 1.11.13 (2016-07-11)
if(cv_bridge_VERSION VERSION_LESS "1.11.13")
  add_definitions("-DCV_BRIDGE_CVT_COLOR_FOR_DISPLAY_OPTION_IS_NOT_SUPPORTED")
endif()



add_message_files(DIRECTORY msg
  FILES
  FindredMsg.msg
  BlockMsg.msg
)

add_service_files(DIRECTORY srv
  FILES
  VelSrv.srv
)

generate_messages(
  DEPENDENCIES
)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES can_dynamix
#  CATKIN_DEPENDS can_driver roscpp
#  DEPENDS system_lib
)



include_directories(
 include
  ${catkin_INCLUDE_DIRS} 
  ${OpenCV_INCLUDE_DIRS}
)




add_executable(can_video_load src/can_video_load.cpp)
target_link_libraries(can_video_load ${catkin_LIBRARIES}  ${OpenCV_LIBRARIES})
#add_dependencies(can_video_load can_dynamix_generate_messages_cpp)


add_executable(can_driving_node src/can_driving_node.cpp)
target_link_libraries(can_driving_node ${catkin_LIBRARIES}  ${OpenCV_LIBRARIES})
add_dependencies(can_driving_node can_dynamix_generate_messages_cpp)


add_executable(can_findred src/can_findred.cpp)
target_link_libraries(can_findred ${catkin_LIBRARIES}  ${OpenCV_LIBRARIES})
add_dependencies(can_findred can_dynamix_generate_messages_cpp)


add_executable(can_control src/can_control_node.cpp)
target_link_libraries(can_control ${catkin_LIBRARIES}   ${OpenCV_LIBRARIES})
add_dependencies(can_control can_dynamix_generate_messages_cpp)


add_executable(can_blockbar src/can_blockbar.cpp include/can_dynamix/Object.cpp)
target_link_libraries(can_blockbar ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
add_dependencies(can_blockbar can_dynamix_generate_messages_cpp)

add_executable(can_lidar src/can_lidar_node.cpp )
target_link_libraries(can_lidar ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
add_dependencies(can_lidar can_dynamix_generate_messages_cpp)


