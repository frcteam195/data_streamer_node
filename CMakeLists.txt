cmake_minimum_required(VERSION 3.0.2)

set( PROJECT_NAME data_streamer_node )
project( ${PROJECT_NAME} )

find_package(catkin REQUIRED COMPONENTS roscpp message_generation)

include_directories(
 ${catkin_INCLUDE_DIRS}
)

message("CATKIN INCLUDES:${catkin_INCLUDE_DIRS}")

set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)

add_executable(  ${PROJECT_NAME}  src/node.cpp )
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})
target_link_libraries(${PROJECT_NAME} Threads::Threads)
