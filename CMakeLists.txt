cmake_minimum_required(VERSION 3.0.2)
set(CMAKE_CXX_STANDARD 17)

project( data_streamer_node )

find_package( catkin REQUIRED COMPONENTS
  roscpp
  roscpp_serialization
  rosbag_storage
  message_generation
  rio_control_node )

find_package( Boost REQUIRED COMPONENTS
  system
  regex )

SET_PROPERTY(GLOBAL PROPERTY TARGET_SUPPORTS_SHARED_LIBS TRUE)

include_directories(
 ${catkin_INCLUDE_DIRS}
 include/
 lib/
 lib/asio/include
 lib/ros_msg_parser/3rdparty/
 lib/nodejs/http_parser/
 lib/ros_msg_parser/include/ros_msg_parser/
 lib/ros_msg_parser/include/
 ${CMAKE_SOURCE_DIR}/../../../third_party_libs/CKfmt/include
 )

catkin_package( )


file(GLOB ros_msg_parser_SRC ./lib/ros_msg_parser/src/*.cpp)


message( STATUS ${ros_msg_parser_SRC})


set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)

add_executable( data_streamer_node
  src/node.cpp
  src/webserver.cpp
  src/data_handler.cpp
  src/data_callbacks.cpp
  lib/nodejs/http_parser/http_parser.c
  ${ros_msg_parser_SRC})


target_link_libraries(data_streamer_node ${catkin_LIBRARIES})
target_link_libraries(data_streamer_node Threads::Threads)

if(ROBOT_ARCHITECTURE_NATIVE)
  target_link_libraries(data_streamer_node ${CMAKE_SOURCE_DIR}/../../../third_party_libs/CKfmt/build/native/libfmt.a)
  target_link_libraries(data_streamer_node ${CMAKE_SOURCE_DIR}/../../../third_party_libs/CKso_5/build/native/libso_s.5.5.24.3.a)
endif(ROBOT_ARCHITECTURE_NATIVE)

if(ROBOT_ARCHITECTURE_AARCH64)
  target_link_libraries(data_streamer_node ${CMAKE_SOURCE_DIR}/../../../third_party_libs/CKfmt/build/aarch64/libfmt.a)
  target_link_libraries(data_streamer_node ${CMAKE_SOURCE_DIR}/../../../third_party_libs/CKso_5/build/aarch64/libso_s.5.5.24.3.a)
endif(ROBOT_ARCHITECTURE_AARCH64)


add_dependencies(data_streamer_node rio_control_node)
