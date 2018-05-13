cmake_minimum_required(VERSION 2.8.3)
project(drone_app)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Add warnings
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Werror")
set(OpenCV_DIR /opt/ros/kinetic/share/OpenCV-3.3.1)
## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  drone_app_msgs
  image_transport
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

find_package(gazebo REQUIRED)

include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})
add_compile_options(${GAZEBO_CXX_FLAGS})

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()


################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES drone_app
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
 include
 ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library

add_library(${PROJECT_NAME}
  src/BBox.cpp
  src/Pose.cpp
  src/Drone.cpp
  src/DronePool.cpp
  src/Math.cpp
  src/RegulatorUtil.cpp
  src/GaussErrorGenerator.cpp
  src/StatisticalErrorGenerator.cpp
)

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide

add_executable(controller
	src/controller.cpp
)

add_executable(gazeboDetector
	src/gazeboDetector.cpp
)

add_executable(gazeboImageSender
	src/gazeboImageSender.cpp
)

## Add manual tests here

add_executable(commandsTest
	test/manual/commandsTest.cpp
)

add_executable(gazeboTest
    test/manual/gazeboTest.cpp
)

add_executable(errorGaussTest
    test/manual/errorGaussTest.cpp
)

add_executable(errorStatisticalTest
    test/manual/errorStatisticalTest.cpp
)


## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above

add_dependencies(controller drone_app_msgs_generate_messages_cpp)
add_dependencies(gazeboDetector drone_app_msgs_generate_messages_cpp)

## Specify libraries to link a library or executable target against

target_link_libraries(controller
    ${catkin_LIBRARIES}
    ${PROJECT_NAME}
)

target_link_libraries(gazeboDetector
    ${PROJECT_NAME}
    ${catkin_LIBRARIES}
    ${GAZEBO_LIBRARIES}
    pthread
)

target_link_libraries(gazeboImageSender
    ${PROJECT_NAME}
    ${catkin_LIBRARIES}
    ${GAZEBO_LIBRARIES}
    pthread
)

## Specify libraries for manual tests here

target_link_libraries(commandsTest
    ${catkin_LIBRARIES}
)

target_link_libraries(gazeboTest
    ${PROJECT_NAME}
    ${catkin_LIBRARIES}
    ${GAZEBO_LIBRARIES}
    pthread
)

target_link_libraries(errorGaussTest
    ${PROJECT_NAME}
    ${catkin_LIBRARIES}
    ${GAZEBO_LIBRARIES}
    pthread
)

target_link_libraries(errorStatisticalTest
    ${PROJECT_NAME}
    ${catkin_LIBRARIES}
    ${GAZEBO_LIBRARIES}
    pthread
)

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries

catkin_add_gtest(BBoxTest test/BBoxTest.cpp)
catkin_add_gtest(PoseTest test/PoseTest.cpp)
catkin_add_gtest(DroneTest test/DroneTest.cpp)
catkin_add_gtest(DronePoolTest test/DronePoolTest.cpp)
catkin_add_gtest(RegulatorUtilTest test/RegulatorUtilTest.cpp)
catkin_add_gtest(MathUtilTest test/MathUtilTest.cpp)

if(TARGET BBoxTest)
  target_link_libraries(BBoxTest ${PROJECT_NAME})
  target_link_libraries(BBoxTest ${catkin_LIBRARIES})
endif()

if(TARGET PoseTest)
  target_link_libraries(PoseTest ${PROJECT_NAME})
  target_link_libraries(PoseTest ${catkin_LIBRARIES})
endif()

if(TARGET DroneTest)
  target_link_libraries(DroneTest ${PROJECT_NAME})
  target_link_libraries(DroneTest ${catkin_LIBRARIES})
endif()

if(TARGET DronePoolTest)
  target_link_libraries(DronePoolTest ${PROJECT_NAME})
  target_link_libraries(DronePoolTest ${catkin_LIBRARIES})
endif()

if(TARGET RegulatorUtilTest)
  target_link_libraries(RegulatorUtilTest ${PROJECT_NAME})
  target_link_libraries(RegulatorUtilTest ${catkin_LIBRARIES})
endif()

if(TARGET MathUtilTest)
  target_link_libraries(MathUtilTest ${PROJECT_NAME})
  target_link_libraries(MathUtilTest ${catkin_LIBRARIES})
endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
# catkin_package()
