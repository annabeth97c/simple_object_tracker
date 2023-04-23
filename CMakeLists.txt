cmake_minimum_required(VERSION 3.6.0)
project(simple_object_tracker)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} --coverage -fprofile-arcs -ftest-coverage")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} --coverage")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslaunch
  rospy
  geometry_msgs
  std_msgs
  tf
)

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

add_message_files(
  FILES
  Object.msg
  ObjectList.msg
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  geometry_msgs
  std_msgs
)

catkin_package(
 CATKIN_DEPENDS roscpp roslaunch rospy geometry_msgs std_msgs tf
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PROJDIR}/include
)

find_package(roslint REQUIRED)
roslint_cpp()

add_executable(simple_object_tracker
  src/tracker_main.cpp
  src/tracker.cpp
)
add_dependencies(simple_object_tracker simple_object_tracker_generate_messages_cpp)
target_link_libraries(simple_object_tracker
  ${catkin_LIBRARIES}
  ${EIGEN_LIBRARIES}
)

install(TARGETS simple_object_tracker
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# install(DIRECTORY launch
#  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

# install(DIRECTORY configs
# DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

if(CATKIN_ENABLE_TESTING)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fprofile-arcs -ftest-coverage")

  find_package(rostest REQUIRED)
  
  # Add gtest-based tests
  catkin_add_gtest(simple_object_tracker_test test/test_tracker.cpp src/tracker.cpp)
  target_link_libraries(simple_object_tracker_test ${catkin_LIBRARIES})
  
endif()