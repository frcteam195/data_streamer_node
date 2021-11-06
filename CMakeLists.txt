cmake_minimum_required(VERSION 3.0.2)
set(CMAKE_CXX_STANDARD 17)

project( data_streamer_node )

find_package(catkin REQUIRED COMPONENTS roscpp message_generation rio_control_node ros_msg_parser)
find_package(Boost REQUIRED COMPONENTS system regex )

add_subdirectory(lib/fmt)
add_subdirectory(lib/nodejs/http_parser)
add_subdirectory(lib/restinio)
add_subdirectory(lib/json_dto)
add_subdirectory(lib/so_5/)
add_subdirectory(lib/ros_msg_parser/)


include_directories(
 ${catkin_INCLUDE_DIRS}
 include/
 lib/
 lib/asio/include
 lib/rapidjson/include
 lib/fmt/include
 lib/nodejs/http_parser/
 lib/ros_msg_parser/include/ros_msg_parser/
 )

catkin_package()

message("CATKIN INCLUDES:${catkin_INCLUDE_DIRS}")

set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)

add_executable( data_streamer_node
  src/node.cpp
  src/webserver.cpp
  src/data_handler.cpp
  src/data_callbacks.cpp
  lib/nodejs/http_parser/http_parser.c )


target_link_libraries(data_streamer_node ${catkin_LIBRARIES})
target_link_libraries(data_streamer_node Threads::Threads)
target_link_libraries(data_streamer_node fmt)
target_link_libraries(data_streamer_node so)
target_link_libraries(data_streamer_node ros_msg_parser)

add_dependencies(data_streamer_node rio_control_node)
