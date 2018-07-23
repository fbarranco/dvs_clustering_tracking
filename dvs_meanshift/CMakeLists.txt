cmake_minimum_required(VERSION 2.8.3)
project(dvs_meanshift)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  dvs_msgs
  image_transport
  pcl_ros
  roscpp
  rospy
  sensor_msgs
  std_msgs
)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED)
find_package(OpenCV REQUIRED)
find_package(PCL 1.7 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES dvs_meanshift
  CATKIN_DEPENDS cv_bridge dvs_msgs image_transport pcl_ros roscpp rospy sensor_msgs std_msgs
  DEPENDS OpenCV boost
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(dvs_meanshift ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
# add_executable(dvs_meanshift_node src/dvs_meanshift_node.cpp)

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(dvs_meanshift_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
# target_link_libraries(dvs_meanshift_node
#   ${catkin_LIBRARIES}
# )

add_executable(dvs_meanshift src/meanshift.cpp src/meanshift_node.cpp src/color_meanshift.cpp src/segment.cpp src/image_reconst.cpp)
target_link_libraries(dvs_meanshift ${catkin_LIBRARIES} ${PCL_COMMON_LIBRARIES} ${PCL_IO_LIBRARIES} ${OpenCV_LIBS})
add_dependencies(dvs_meanshift ${catkin_EXPORTED_TARGETS} )
