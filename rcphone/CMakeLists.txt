cmake_minimum_required(VERSION 2.8.3)
project(rcphone)

find_package(Boost REQUIRED COMPONENTS date_time thread)
find_package(catkin REQUIRED COMPONENTS message_generation rostime roscpp rosconsole roscpp_serialization std_msgs)
 add_message_files(
   FILES
   rpydata.msg
zdata.msg
switchdata.msg
 )
include_directories(${catkin_INCLUDE_DIRS})
link_directories(${catkin_LIBRARY_DIRS})

generate_messages(DEPENDENCIES std_msgs)

catkin_package(CATKIN_DEPENDS message_runtime std_msgs)



include_directories(${catkin_INCLUDE_DIRS})
add_executable(listener listener.cpp)
add_dependencies(listener rcphone_generate_messages_cpp)
target_link_libraries(listener ${catkin_LIBRARIES})
install(TARGETS listener
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

