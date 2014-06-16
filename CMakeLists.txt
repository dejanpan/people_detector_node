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
target_link_libraries(people_detector_node
  ${catkin_LIBRARIES}
  #needed to be able to combine static and shared libraries
  "-Wl,--whole-archive" 
  libcommon.a
  libfeatures.a
  liblinalg.a
  libPeopleDetector.a 
  "-Wl,--no-whole-archive"
  gsl gslcblas m
  )
add_dependencies(people_detector_node people_detector)
set_target_properties(people_detector_node
    PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY "${CATKIN_DEVEL_PREFIX}/lib/people_detector_node"
)

add_executable(logger src/logger.cpp)
target_link_libraries(logger
  ${catkin_LIBRARIES}
  )
set_target_properties(logger
  PROPERTIES
  RUNTIME_OUTPUT_DIRECTORY "${CATKIN_DEVEL_PREFIX}/lib/people_detector_node"
  )