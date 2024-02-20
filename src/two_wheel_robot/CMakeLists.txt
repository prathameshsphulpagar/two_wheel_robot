cmake_minimum_required(VERSION 3.0.2)
##project(3wheel_robot)
project(two_wheel_robot)
## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  message_generation
)

catkin_package()
include_directories(${catkin_INCLUDE_DIRS})


add_executable(lidar_client1_executable src/client_cpp.cpp)
target_link_libraries(lidar_client1_executable ${catkin_LIBRARIES})

add_executable(node_restart_executable src/node_restart_cpp.cpp)
target_link_libraries(node_restart_executable ${catkin_LIBRARIES})

add_executable(lidar_client_front_mid_executable src/client_front_mid_cpp.cpp)
target_link_libraries(lidar_client_front_mid_executable ${catkin_LIBRARIES})

add_executable(lidar_client_front_top_executable src/client_front_top_cpp.cpp)
target_link_libraries(lidar_client_front_top_executable ${catkin_LIBRARIES})

add_executable(lidar_client_left_executable src/client_left_cpp.cpp)
target_link_libraries(lidar_client_left_executable ${catkin_LIBRARIES})

add_executable(lidar_client_right_executable src/client_right_cpp.cpp)
target_link_libraries(lidar_client_right_executable ${catkin_LIBRARIES})

add_executable(lidar_client_back_executable src/client_back_cpp.cpp)
target_link_libraries(lidar_client_back_executable ${catkin_LIBRARIES})

add_executable(teleop_keyboard_executable src/teleop_keyboard_cpp.cpp)
target_link_libraries(teleop_keyboard_executable ${catkin_LIBRARIES})

add_executable(normal_encoder_executable src/normal_encoder_cpp.cpp)
target_link_libraries(normal_encoder_executable ${catkin_LIBRARIES})

add_executable(absolute_encoder_executable src/absolute_encoder_cpp.cpp)
target_link_libraries(absolute_encoder_executable ${catkin_LIBRARIES})

add_executable(sensing_stoping_executable src/sensing_stoping_cpp.cpp)
target_link_libraries(sensing_stoping_executable ${catkin_LIBRARIES})

add_executable(combine_six_lidar_executable src/combine_six_lidar_cpp.cpp)
target_link_libraries(combine_six_lidar_executable ${catkin_LIBRARIES})

add_executable(wall_following_executable src/wall_following_cpp.cpp)
target_link_libraries(wall_following_executable ${catkin_LIBRARIES})

add_executable(storage_executable src/storage_cpp.cpp)
target_link_libraries(storage_executable ${catkin_LIBRARIES})
