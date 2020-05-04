cmake_minimum_required(VERSION 2.8.3)
project(autopark)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a exec_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a exec_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#  DEPENDENCIES
#  std_msgs
#)

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a exec_depend tag for "dynamic_reconfigure"
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
  INCLUDE_DIRS include
#  LIBRARIES autopark
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
add_library(autoparking
  include/autopark/autoparking.h
  src/autoparking.cpp
)
target_link_libraries(autoparking ${catkin_LIBRARIES})
add_dependencies(autoparking ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


add_executable(controller_parking src/controller/controller_parking.cpp)
target_link_libraries(controller_parking ${catkin_LIBRARIES})

add_executable(controller_motor src/controller/controller_motor.cpp)
target_link_libraries(controller_motor ${catkin_LIBRARIES})

add_executable(controller_steering src/controller/controller_steering.cpp)
target_link_libraries(controller_steering ${catkin_LIBRARIES})

add_executable(sensor_encoder src/sensor/sensor_encoder.cpp)
target_link_libraries(sensor_encoder ${catkin_LIBRARIES})

add_executable(sensor_apa_lf src/sensor/sensor_apa_lf.cpp)
target_link_libraries(sensor_apa_lf ${catkin_LIBRARIES})

add_executable(sensor_apa_lb src/sensor/sensor_apa_lb.cpp)
target_link_libraries(sensor_apa_lb ${catkin_LIBRARIES})

add_executable(sensor_apa_rf src/sensor/sensor_apa_rf.cpp)
target_link_libraries(sensor_apa_rf ${catkin_LIBRARIES})

add_executable(sensor_apa_rb src/sensor/sensor_apa_rb.cpp)
target_link_libraries(sensor_apa_rb ${catkin_LIBRARIES})

add_executable(sensor_upa_fl src/sensor/sensor_upa_fl.cpp)
target_link_libraries(sensor_upa_fl ${catkin_LIBRARIES})

add_executable(sensor_upa_fcl src/sensor/sensor_upa_fcl.cpp)
target_link_libraries(sensor_upa_fcl ${catkin_LIBRARIES})

add_executable(sensor_upa_fcr src/sensor/sensor_upa_fcr.cpp)
target_link_libraries(sensor_upa_fcr ${catkin_LIBRARIES})

add_executable(sensor_upa_fr src/sensor/sensor_upa_fr.cpp)
target_link_libraries(sensor_upa_fr ${catkin_LIBRARIES})

add_executable(sensor_upa_bl src/sensor/sensor_upa_bl.cpp)
target_link_libraries(sensor_upa_bl ${catkin_LIBRARIES})

add_executable(sensor_upa_bcl src/sensor/sensor_upa_bcl.cpp)
target_link_libraries(sensor_upa_bcl ${catkin_LIBRARIES})

add_executable(sensor_upa_bcr src/sensor/sensor_upa_bcr.cpp)
target_link_libraries(sensor_upa_bcr ${catkin_LIBRARIES})

add_executable(sensor_upa_br src/sensor/sensor_upa_br.cpp)
target_link_libraries(sensor_upa_br ${catkin_LIBRARIES})

add_executable(search_parking_space_lf src/search_parking_space_lf.cpp)
target_link_libraries(search_parking_space_lf autoparking ${catkin_LIBRARIES})

add_executable(search_parking_space_lb src/search_parking_space_lb.cpp)
target_link_libraries(search_parking_space_lb autoparking ${catkin_LIBRARIES})

add_executable(search_parking_space_rf src/search_parking_space_rf.cpp)
target_link_libraries(search_parking_space_rf autoparking ${catkin_LIBRARIES})

add_executable(search_parking_space_rb src/search_parking_space_rb.cpp)
target_link_libraries(search_parking_space_rb autoparking ${catkin_LIBRARIES})


## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
# add_executable(${PROJECT_NAME}_node src/autopark_node.cpp)

## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
# target_link_libraries(${PROJECT_NAME}_node
#   ${catkin_LIBRARIES}
# )

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

## Mark executables for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_executables.html
# install(TARGETS ${PROJECT_NAME}_node
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark libraries for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_libraries.html
# install(TARGETS ${PROJECT_NAME}
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
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
# catkin_add_gtest(${PROJECT_NAME}-test test/test_autopark.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
