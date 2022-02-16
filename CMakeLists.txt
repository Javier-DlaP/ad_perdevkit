cmake_minimum_required(VERSION 2.8.3)
project(ad_perdevkit)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  std_msgs
  geometry_msgs
  vision_msgs
)

set(CMAKE_CXX_FLAGS "-std=c++11 -O2 -Wall ${CMAKE_CXX_FLAGS}")

add_message_files(FILES
  GT_3D_Object.msg
  GT_3D_Object_list.msg
)

generate_messages(DEPENDENCIES
  std_msgs
  geometry_msgs
  vision_msgs
)

catkin_package(CATKIN_DEPENDS
  message_runtime
  std_msgs
  geometry_msgs
  vision_msgs
)

include_directories(
 ~/t4ac_ws/devel/include
  ${catkin_INCLUDE_DIRS}
)