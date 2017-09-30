cmake_minimum_required(VERSION 2.8.3)
set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")
project(detect_place)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  geometry_msgs
)
find_package(Boost REQUIRED COMPONENTS system thread)
catkin_package(
	LIBRARIES ${PROJECT_NAME}
    CATKIN_DEPENDS std_msgs sensor_msgs geometry_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
include_directories(... ${Boost_INCLUDE_DIRS})

add_executable(detect_place_node src/detect_place.cpp)
add_dependencies(detect_place_node ${PROJECT_NAME})
target_link_libraries(detect_place_node ${catkin_LIBRARIES})


install(TARGETS detect_place_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS
)
