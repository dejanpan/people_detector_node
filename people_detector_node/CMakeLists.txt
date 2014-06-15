cmake_minimum_required(VERSION 2.8.3)
project(people_detector_node)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin COMPONENTS roscpp std_msgs sensor_msgs)


include(ExternalProject)
ExternalProject_Add(people_detector
  GIT_REPOSITORY https://github.com/dejanpan/people_detector_2d_laser 
  BUILD_IN_SOURCE 1
  CONFIGURE_COMMAND ""
  BUILD_COMMAND make
  INSTALL_COMMAND "")

MESSAGE( STATUS "CMAKE_CURRENT_BINARY_DIR: " ${CMAKE_CURRENT_BINARY_DIR} )

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${CMAKE_CURRENT_BINARY_DIR}/people_detector-prefix/src/people_detector/PeopleDetector
  ${CMAKE_CURRENT_BINARY_DIR}/people_detector-prefix/src/people_detector/features
  ${CMAKE_CURRENT_BINARY_DIR}/people_detector-prefix/src/people_detector/common
  ${CMAKE_CURRENT_BINARY_DIR}/people_detector-prefix/src/people_detector/linalg
)

link_directories(
#  ${CMAKE_CURRENT_BINARY_DIR}
  ${CMAKE_CURRENT_BINARY_DIR}/people_detector-prefix/src/people_detector/PeopleDetector
  ${CMAKE_CURRENT_BINARY_DIR}/people_detector-prefix/src/people_detector/features
  ${CMAKE_CURRENT_BINARY_DIR}/people_detector-prefix/src/people_detector/common
  ${CMAKE_CURRENT_BINARY_DIR}/people_detector-prefix/src/people_detector/linalg
   )

add_executable(people_detector_node src/people_detector_node.cpp)
add_dependencies(people_detector_node people_detector)
target_link_libraries(people_detector_node
${catkin_LIBRARIES} libcommon.a libfeatures.a liblinalg.a libPeopleDetector.a
 # ${catkin_LIBRARIES} 
 # ${CMAKE_CURRENT_BINARY_DIR}/people_detector-prefix/src/people_detector/features/libfeatures.a 
 # ${CMAKE_CURRENT_BINARY_DIR}/people_detector-prefix/src/people_detector/common/libcommon.a 
 # ${CMAKE_CURRENT_BINARY_DIR}/people_detector-prefix/src/people_detector/PeopleDetector/libPeopleDetector.a
  )
