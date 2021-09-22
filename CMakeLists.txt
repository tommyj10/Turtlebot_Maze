# define cmake version
cmake_minimum_required(VERSION 2.8.3)

# define package name
project(wallfollower)

# define package requirements for the node
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  geometry_msgs
  message_generation
)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
# )

## Generate added messages and services with any dependencies listed here
generate_messages(
   DEPENDENCIES
   std_msgs
   sensor_msgs
   geometry_msgs

)

# define catkin package dependencies
catkin_package(
CATKIN_DEPENDS message_runtime

)

# include any directories necessary to build
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(wallfollower src/main.cpp src/VelocityController.cpp src/LaserScanner.cpp src/IMU.cpp src/TurtleController.cpp)
target_link_libraries(wallfollower ${catkin_LIBRARIES})

