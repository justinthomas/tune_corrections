cmake_minimum_required(VERSION 2.8.3)
project(tune_corrections)

# set default build type
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE RelWithDebInfo)
endif()

find_package(catkin REQUIRED COMPONENTS roscpp quadrotor_msgs controllers_manager null_tracker std_msgs cmake_modules)
find_package(Eigen REQUIRED)

include_directories(include ${catkin_INCLUDE_DIRS} ${EIGEN_INCLUDE_DIR})

catkin_package(
  INCLUDE_DIRS
  LIBRARIES
  CATKIN_DEPENDS roscpp quadrotor_msgs controllers_manager null_tracker std_msgs 
  DEPENDS Eigen)

add_executable(tune_corrections src/state_control.cpp)
add_dependencies(tune_corrections ${catkin_EXPORTED_TARGETS})
target_link_libraries(tune_corrections ${catkin_LIBRARIES})
