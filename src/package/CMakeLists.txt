cmake_minimum_required(VERSION 3.0.2)
project(package)

find_package(Eigen3 REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  message_generation
  std_msgs
)


## Generate services in the 'srv' folder
add_service_files(
  FILES
  coordinates.srv
#   Service1.srv
#   Service2.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES  std_msgs  
)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES motion_planner
CATKIN_DEPENDS roscpp std_msgs message_runtime rospy
#  DEPENDS system_lib
)


## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
)

#test1
#add_executable(test1 src/ur5Functions.cpp src/tests/test1.cpp)
#target_link_libraries(test1 ${catkin_LIBRARIES} ${Eigen_LIBRARIES})

#testInvDiffKin
#add_executable(testInvDiffKin src/ur5Functions.cpp src/tests/testInvDiffKin.cpp)
#target_link_libraries(testInvDiffKin ${catkin_LIBRARIES} ${Eigen_LIBRARIES})

# testObject
add_executable(testObject src/motion_planner/ur5Object.cpp src/tests/testObject.cpp)
target_link_libraries(testObject ${catkin_LIBRARIES} ${Eigen_LIBRARIES})

# test_node
add_executable(test_node src/tests/test_node.cpp)
target_link_libraries(test_node ${catkin_LIBRARIES})

add_executable(testJointStatePublisher src/motion_planner/jointStatePublisher.cpp src/tests/testJointStatePublisher.cpp)
target_link_libraries(testJointStatePublisher ${catkin_LIBRARIES})

add_executable(testUr5Movement src/motion_planner/ur5Object.cpp src/motion_planner/jointStatePublisher.cpp src/tests/testUr5Movement.cpp)
target_link_libraries(testUr5Movement ${catkin_LIBRARIES} ${Eigen_LIBRARIES})
